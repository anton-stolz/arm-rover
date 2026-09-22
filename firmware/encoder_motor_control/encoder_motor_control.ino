// ============================================================
//  FINAL: Encoder + PI + Motorsteuerung + binäres Protokoll
//  - Active-low relays: OFF = HIGH, ON = LOW
//  - Safe relay init: pins go HIGH as early as possible
//  - Raw bumper ADC values always in telemetry
//  - Simplified per-wheel state machine (no cross-wheel sync)
// ============================================================

#include "calib_constants.h"
#include <avr/pgmspace.h>
#include <math.h>

// ============================================================
//  FORWARD DECLARATIONS 
// ============================================================
struct WheelController;
void motorOff(WheelController* wc);
void motorSetDuty(WheelController* wc, uint8_t duty);
void motorSetDir(WheelController* wc, int8_t dir);
void updateMotorController(WheelController* wc, float dt_ms);

// ---------------- Pins ----------------
const int PIN_ENC1_S1 = A0;   // links
const int PIN_ENC1_S2 = A1;
const int PIN_ENC2_S1 = A2;   // rechts
const int PIN_ENC2_S2 = A3;

const int PIN_BUMPER_LEFT = A4;
const int PIN_BUMPER_RIGHT = A5;

struct Motor {
    uint8_t pwm, rlyA, rlyB;
    const char* name;
};

Motor LEFT_MOTOR  = { 11, 10, 9, "L" };
Motor RIGHT_MOTOR = { 3, 4, 5, "R" };

// ================= SIGN CALIBRATION =================
const int8_t LEFT_MOTOR_SIGN   = -1;
const int8_t RIGHT_MOTOR_SIGN  = -1;
const int8_t LEFT_ENCODER_SIGN = 1;
const int8_t RIGHT_ENCODER_SIGN = 1;
// ====================================================

// Active-low relay module:
// LOW  -> relay ON
// HIGH -> relay OFF
const bool RELAY_ON  = LOW;
const bool RELAY_OFF = HIGH;

const uint8_t MOTOR_MIN_DUTY = 55;      // Deadzone
const uint8_t MOTOR_MAX_DUTY = 255;
const float MAX_TARGET_DEG_S = 720.0;

float KP = 0.15;
float KI = 0.8;
const float INTEGRAL_LIMIT = 200.0;

// Combined relay switching delay (back-EMF decay + relay settle)
const unsigned long RELAY_SWITCH_MS = 150;

const unsigned long CMD_WATCHDOG_MS = 500;
unsigned long last_valid_cmd_ms = 0;

// ---------------- Encoder ----------------
struct EncoderState {
    const char* name;
    float o1, o2, a1, a2, delta;
    const float* lut;
    const float* lissajous_s1;
    const float* lissajous_s2;
    int safe_angle_idx;
    bool safe_zone_verified;
    float unwrapped_phase;
    float last_raw_angle;
    float display_offset;
    float continuous_angle_deg;
    bool first_update;
    float last_continuous_angle;
    float velocity_deg_s;
};

EncoderState enc1 = {
    "Enc1", ENC1_O1, ENC1_O2, ENC1_A1, ENC1_A2, ENC1_DELTA,
    ENC1_LUT, ENC1_LISSAJOUS_S1, ENC1_LISSAJOUS_S2, ENC1_SAFE_ANGLE_IDX,
    false, 0.0, 0.0, 0.0, 0.0, true, 0.0, 0.0
};

EncoderState enc2 = {
    "Enc2", ENC2_O1, ENC2_O2, ENC2_A1, ENC2_A2, ENC2_DELTA,
    ENC2_LUT, ENC2_LISSAJOUS_S1, ENC2_LISSAJOUS_S2, ENC2_SAFE_ANGLE_IDX,
    false, 0.0, 0.0, 0.0, 0.0, true, 0.0, 0.0
};

int safeAnalogRead(int pin) {
    analogRead(pin);       // dummy read to settle ADC mux
    return analogRead(pin);
}

float get_raw_phase(EncoderState* enc, int s1, int s2) {
    float x = (s1 - enc->o1) / enc->a1;
    float y = (s2 - enc->o2) / enc->a2;
    float sin_d = sin(enc->delta);
    float cos_d = cos(enc->delta);
    float y_corr = (y - x * sin_d) / cos_d;
    return atan2(y_corr, x);
}

float read_progmem_float(const float* array, int index) {
    float val;
    memcpy_P(&val, &array[index], sizeof(float));
    return val;
}

void init_encoder_start(EncoderState* enc, int pin_s1, int pin_s2) {
    float err_0 = 0, err_2pi = 0;
    int valid0 = 0, valid2pi = 0;

    for (int i = 0; i < 20; i++) {
        int s1 = safeAnalogRead(pin_s1);
        int s2 = safeAnalogRead(pin_s2);
        float ang = get_raw_phase(enc, s1, s2);

        float d0 = fmod(ang, 4.0 * PI);
        if (d0 < 0) d0 += 4.0 * PI;
        int i0 = (int)((d0 / (4.0 * PI)) * 720) % 720;

        float d2 = fmod(ang + 2.0 * PI, 4.0 * PI);
        if (d2 < 0) d2 += 4.0 * PI;
        int i2 = (int)((d2 / (4.0 * PI)) * 720) % 720;

        float e1 = read_progmem_float(enc->lissajous_s1, i0);
        float e2 = read_progmem_float(enc->lissajous_s2, i0);
        if (e1 > -0.5 && e2 > -0.5) {
            err_0 += (e1 - s1) * (e1 - s1) + (e2 - s2) * (e2 - s2);
            valid0++;
        }

        float f1 = read_progmem_float(enc->lissajous_s1, i2);
        float f2 = read_progmem_float(enc->lissajous_s2, i2);
        if (f1 > -0.5 && f2 > -0.5) {
            err_2pi += (f1 - s1) * (f1 - s1) + (f2 - s2) * (f2 - s2);
            valid2pi++;
        }

        delay(2);
    }

    enc->unwrapped_phase = (valid0 > 0 && valid2pi > 0 && err_2pi < err_0) ? 2.0 * PI : 0.0;
    enc->last_raw_angle = get_raw_phase(enc, safeAnalogRead(pin_s1), safeAnalogRead(pin_s2));
    enc->display_offset = 0.0;
    enc->safe_zone_verified = false;
    enc->continuous_angle_deg = 0.0;
    enc->last_continuous_angle = 0.0;
    enc->velocity_deg_s = 0.0;
    enc->first_update = true;
}

void update_encoder(EncoderState* enc, int s1, int s2, float dt) {
    float ang = get_raw_phase(enc, s1, s2);
    float diff = ang - enc->last_raw_angle;

    if (diff > PI) enc->unwrapped_phase -= 2.0 * PI;
    else if (diff < -PI) enc->unwrapped_phase += 2.0 * PI;

    enc->last_raw_angle = ang;

    float d_phase = fmod(ang + enc->unwrapped_phase, 4.0 * PI);
    if (d_phase < 0) d_phase += 4.0 * PI;
    int idx = (int)((d_phase / (4.0 * PI)) * 720) % 720;

    if (!enc->safe_zone_verified) {
        int cur = idx % 360;
        int dist = abs(cur - enc->safe_angle_idx);
        if (dist > 180) dist = 360 - dist;

        if (dist <= 10) {
            float e1 = read_progmem_float(enc->lissajous_s1, cur);
            float e2 = read_progmem_float(enc->lissajous_s2, cur);
            float f1 = read_progmem_float(enc->lissajous_s1, cur + 360);
            float f2 = read_progmem_float(enc->lissajous_s2, cur + 360);

            float err0 = (e1 - s1) * (e1 - s1) + (e2 - s2) * (e2 - s2);
            float err2 = (f1 - s1) * (f1 - s1) + (f2 - s2) * (f2 - s2);

            bool cur_is_0 = (idx < 360);
            bool true_is_0 = (err0 <= err2);

            if (cur_is_0 != true_is_0) {
                if (cur_is_0) {
                    enc->unwrapped_phase += 2.0 * PI;
                    enc->display_offset -= 180.0;
                } else {
                    enc->unwrapped_phase -= 2.0 * PI;
                    enc->display_offset += 180.0;
                }
            }

            enc->safe_zone_verified = true;

            d_phase = fmod(ang + enc->unwrapped_phase, 4.0 * PI);
            if (d_phase < 0) d_phase += 4.0 * PI;
            idx = (int)((d_phase / (4.0 * PI)) * 720) % 720;
        }
    }

    int wraps = (int)floor((ang + enc->unwrapped_phase) / (4.0 * PI));
    float bin_float = (d_phase / (4.0 * PI)) * 720.0;
    int idx1 = ((int)bin_float) % 720;
    int idx2 = (idx1 + 1) % 720;
    float frac = bin_float - (int)bin_float;

    float lut1 = read_progmem_float(enc->lut, idx1);
    float lut2 = read_progmem_float(enc->lut, idx2);

    float lut_diff = lut2 - lut1;
    if (lut_diff < -PI) lut_diff += 2.0 * PI;
    if (lut_diff > PI) lut_diff -= 2.0 * PI;

    float lut_val = lut1 + frac * lut_diff;

    enc->continuous_angle_deg = ((wraps * 2.0 * PI) + lut_val) * (180.0 / PI) + enc->display_offset;

    if (enc->first_update) {
        enc->last_continuous_angle = enc->continuous_angle_deg;
        enc->first_update = false;
    } else if (dt > 0.0) {
        float raw_vel = (enc->continuous_angle_deg - enc->last_continuous_angle) / dt;
        enc->velocity_deg_s = 0.2 * raw_vel + 0.8 * enc->velocity_deg_s;
        enc->last_continuous_angle = enc->continuous_angle_deg;
    }
}

// ---------------- Motor State Machine ----------------
// Simplified: 3 states, no cross-wheel synchronization.
//   IDLE      – motor off, relays don't matter
//   SWITCHING – PWM off, relays being set, waiting for settle
//   RUNNING   – PI controller active
enum MotorState {
    MOTOR_IDLE,
    MOTOR_SWITCHING,
    MOTOR_RUNNING
};

struct WheelController {
    Motor* motor;
    EncoderState* encoder;
    int8_t motor_sign;
    int8_t encoder_sign;
    float target_deg_s;
    float measured_deg_s;
    float kp, ki, integral;
    uint8_t pwm_duty;
    int8_t active_dir;      // current relay direction (1, -1, or 0=off)
    int8_t desired_dir;     // direction we want to switch to
    MotorState state;
    unsigned long switch_start_ms;
};

WheelController left_ctrl = {
    &LEFT_MOTOR,
    &enc1,
    LEFT_MOTOR_SIGN,
    LEFT_ENCODER_SIGN,
    0, 0,
    KP, KI,
    0,
    0, 0,
    MOTOR_IDLE,
    0
};

WheelController right_ctrl = {
    &RIGHT_MOTOR,
    &enc2,
    RIGHT_MOTOR_SIGN,
    RIGHT_ENCODER_SIGN,
    0, 0,
    KP, KI,
    0,
    0, 0,
    MOTOR_IDLE,
    0
};

// ---------------- Relay / PWM init ----------------
void relayInitPin(uint8_t pin) {
    // Safe init for active-low relay:
    // set HIGH before making the pin an output.
    digitalWrite(pin, RELAY_OFF);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, RELAY_OFF);
}

void pwmInitPin(uint8_t pin) {
    // Inverted PWM in this project:
    // analogWrite(255) means OFF.
    digitalWrite(pin, HIGH);
    pinMode(pin, OUTPUT);
    analogWrite(pin, 255);
}

void relaySet(uint8_t pin, bool on) {
    digitalWrite(pin, on ? RELAY_ON : RELAY_OFF);
}

void motorOff(WheelController* wc) {
    analogWrite(wc->motor->pwm, 255);
}

void motorSetDuty(WheelController* wc, uint8_t duty) {
    wc->pwm_duty = duty;
    analogWrite(wc->motor->pwm, 255 - duty);
}

void motorSetDir(WheelController* wc, int8_t dir) {
    relaySet(wc->motor->rlyA, dir == 1);
    relaySet(wc->motor->rlyB, dir == -1);
    wc->active_dir = dir;
}

// Compute the relay direction needed for a given target speed.
int8_t targetToRelayDir(WheelController* wc, float target) {
    if (target == 0.0) return 0;
    int8_t s = (target > 0) ? 1 : -1;
    return (int8_t)(s * wc->motor_sign);
}

void updateMotorController(WheelController* wc, float dt_ms) {
    unsigned long now_ms = millis();
    wc->measured_deg_s = wc->encoder->velocity_deg_s * wc->encoder_sign;

    int8_t needed_dir = targetToRelayDir(wc, wc->target_deg_s);

    switch (wc->state) {
        case MOTOR_IDLE:
            if (wc->target_deg_s == 0.0) {
                // Stay idle.
                break;
            }
            // Need to start moving.
            wc->integral = 0.0;
            if (needed_dir == wc->active_dir && wc->active_dir != 0) {
                // Relays already in the right position, go straight to running.
                wc->state = MOTOR_RUNNING;
            } else {
                // Need to set/change relays.
                motorOff(wc);
                wc->pwm_duty = 0;
                wc->desired_dir = needed_dir;
                motorSetDir(wc, needed_dir);
                wc->switch_start_ms = now_ms;
                wc->state = MOTOR_SWITCHING;
            }
            break;

        case MOTOR_SWITCHING:
            // While switching, accept updated direction without restarting timer
            // (only restart if the desired direction actually changed).
            if (wc->target_deg_s == 0.0) {
                // Target went to zero while switching — just go idle.
                motorOff(wc);
                wc->pwm_duty = 0;
                wc->state = MOTOR_IDLE;
                break;
            }
            if (needed_dir != wc->desired_dir) {
                // Direction changed during switch — update relays and restart timer.
                wc->desired_dir = needed_dir;
                motorSetDir(wc, needed_dir);
                wc->switch_start_ms = now_ms;
            }
            if (now_ms - wc->switch_start_ms >= RELAY_SWITCH_MS) {
                wc->integral = 0.0;
                wc->state = MOTOR_RUNNING;
            }
            break;

        case MOTOR_RUNNING: {
            if (wc->target_deg_s == 0.0) {
                motorOff(wc);
                wc->integral = 0.0;
                wc->pwm_duty = 0;
                wc->state = MOTOR_IDLE;
                break;
            }

            // Check if direction change is needed.
            if (needed_dir != wc->active_dir) {
                motorOff(wc);
                wc->pwm_duty = 0;
                wc->integral = 0.0;
                wc->desired_dir = needed_dir;
                motorSetDir(wc, needed_dir);
                wc->switch_start_ms = now_ms;
                wc->state = MOTOR_SWITCHING;
                break;
            }

            // PI control.
            float target_mag = fabs(wc->target_deg_s);
            float s = (wc->target_deg_s > 0) ? 1.0 : -1.0;
            float measured_mag = wc->measured_deg_s * s;

            float ff = MOTOR_MIN_DUTY
                     + (target_mag / MAX_TARGET_DEG_S) * (MOTOR_MAX_DUTY - MOTOR_MIN_DUTY);

            float error = target_mag - measured_mag;
            wc->integral += error * dt_ms * 0.001;
            wc->integral = constrain(wc->integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

            float out = ff + wc->kp * error + wc->ki * wc->integral;
            motorSetDuty(wc, (uint8_t)constrain(out, 0.0, 255.0));
            break;
        }
    }
}

// ---------------- Binary Protocol ----------------
struct __attribute__((packed)) CmdPacket {
    uint8_t magic1, magic2;
    int16_t left_dps10, right_dps10;
    uint8_t checksum;
};

struct __attribute__((packed)) TelemetryPacket {
    uint8_t magic1, magic2;
    uint32_t time_ms;
    float left_angle_deg, right_angle_deg;
    float left_vel_deg_s, right_vel_deg_s;
    int16_t left_target_dps10, right_target_dps10;
    uint8_t left_pwm, right_pwm;
    uint8_t state_left, state_right;
    uint16_t raw_bumper_left;
    uint16_t raw_bumper_right;
    uint8_t checksum;
};

uint8_t calcChecksum(const uint8_t* data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len - 1; i++) {
        sum += data[i];
    }
    return sum;
}

uint8_t cmd_buffer[sizeof(CmdPacket)];
size_t cmd_pos = 0;

void processSerial() {
    while (Serial.available()) {
        uint8_t b = Serial.read();

        if (cmd_pos == 0) {
            if (b == 0xAA) {
                cmd_buffer[0] = b;
                cmd_pos = 1;
            }
        } else {
            if (b == 0xAA && cmd_pos == 1) {
                cmd_buffer[0] = b;
                cmd_pos = 1;
            } else {
                cmd_buffer[cmd_pos++] = b;

                if (cmd_pos >= sizeof(CmdPacket)) {
                    CmdPacket* cmd = (CmdPacket*)cmd_buffer;

                    if (cmd->magic2 == 0x55 &&
                        cmd->checksum == calcChecksum(cmd_buffer, sizeof(CmdPacket))) {

                        left_ctrl.target_deg_s =
                            constrain(cmd->left_dps10 / 10.0, -MAX_TARGET_DEG_S, MAX_TARGET_DEG_S);

                        right_ctrl.target_deg_s =
                            constrain(cmd->right_dps10 / 10.0, -MAX_TARGET_DEG_S, MAX_TARGET_DEG_S);

                        last_valid_cmd_ms = millis();
                    }

                    cmd_pos = 0;
                }
            }
        }
    }
}

unsigned long last_telemetry_ms = 0;

void sendTelemetry() {
    unsigned long now = millis();

    if (now - last_telemetry_ms < 20) return;
    last_telemetry_ms = now;

    TelemetryPacket pkt;

    pkt.magic1 = 0xAB;
    pkt.magic2 = 0xCD;
    pkt.time_ms = now;

    pkt.left_angle_deg  = enc1.continuous_angle_deg;
    pkt.right_angle_deg = enc2.continuous_angle_deg;

    pkt.left_vel_deg_s  = left_ctrl.measured_deg_s;
    pkt.right_vel_deg_s = right_ctrl.measured_deg_s;

    pkt.left_target_dps10  = (int16_t)(left_ctrl.target_deg_s * 10.0);
    pkt.right_target_dps10 = (int16_t)(right_ctrl.target_deg_s * 10.0);

    pkt.left_pwm  = left_ctrl.pwm_duty;
    pkt.right_pwm = right_ctrl.pwm_duty;

    pkt.state_left  = left_ctrl.state;
    pkt.state_right = right_ctrl.state;

    pkt.raw_bumper_left  = (uint16_t)safeAnalogRead(PIN_BUMPER_LEFT);
    pkt.raw_bumper_right = (uint16_t)safeAnalogRead(PIN_BUMPER_RIGHT);

    pkt.checksum = calcChecksum((uint8_t*)&pkt, sizeof(pkt));

    Serial.write((uint8_t*)&pkt, sizeof(pkt));
}

// ---------------- Setup / Loop ----------------
unsigned long last_time = 0;

void setup() {
    Serial.begin(115200);

    // PWM pins first, in OFF state.
    pwmInitPin(LEFT_MOTOR.pwm);
    pwmInitPin(RIGHT_MOTOR.pwm);

    // Relay pins safe init: active-low relays off = HIGH.
    relayInitPin(LEFT_MOTOR.rlyA);
    relayInitPin(LEFT_MOTOR.rlyB);
    relayInitPin(RIGHT_MOTOR.rlyA);
    relayInitPin(RIGHT_MOTOR.rlyB);

    // Explicitly turn everything off.
    motorOff(&left_ctrl);
    motorOff(&right_ctrl);

    relaySet(LEFT_MOTOR.rlyA, false);
    relaySet(LEFT_MOTOR.rlyB, false);
    relaySet(RIGHT_MOTOR.rlyA, false);
    relaySet(RIGHT_MOTOR.rlyB, false);

    // Bumper analog inputs.
    pinMode(PIN_BUMPER_LEFT, INPUT);
    pinMode(PIN_BUMPER_RIGHT, INPUT);

    init_encoder_start(&enc1, PIN_ENC1_S1, PIN_ENC1_S2);
    init_encoder_start(&enc2, PIN_ENC2_S1, PIN_ENC2_S2);

    last_time = micros();
    last_valid_cmd_ms = millis();
}

void loop() {
    unsigned long now = micros();
    float dt_ms = (now - last_time) / 1000.0;
    last_time = now;

    processSerial();

    if (millis() - last_valid_cmd_ms > CMD_WATCHDOG_MS) {
        left_ctrl.target_deg_s = 0.0;
        right_ctrl.target_deg_s = 0.0;
    }

    update_encoder(
        &enc1,
        safeAnalogRead(PIN_ENC1_S1),
        safeAnalogRead(PIN_ENC1_S2),
        dt_ms * 0.001
    );

    update_encoder(
        &enc2,
        safeAnalogRead(PIN_ENC2_S1),
        safeAnalogRead(PIN_ENC2_S2),
        dt_ms * 0.001
    );

    updateMotorController(&left_ctrl, dt_ms);
    updateMotorController(&right_ctrl, dt_ms);

    sendTelemetry();
}