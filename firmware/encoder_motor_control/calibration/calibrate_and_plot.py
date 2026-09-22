import csv
import math
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.optimize import minimize

# This script performs the Ultimate Synthesis calibration:
# 1. 5-Parameter Ellipse Fitting (Nelder-Mead optimization)
# 2. 4Pi Kinematic Velocity LUT generation (to remove physical magnet ripple)

def read_data(f):
    with open(f, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader) # skip header
        data = []
        for row in reader:
            try:
                data.append((float(row[0]), float(row[1]), float(row[2]), float(row[3]), float(row[4])))
            except ValueError:
                pass
    return data

def insert_nans(t, y):
    """Inserts NaNs into arrays when the angle wraps, so the plot doesn't draw vertical lines."""
    t_new, y_new = [], []
    for i in range(len(y)):
        if i > 0 and abs(y[i] - y[i-1]) > 180:
            t_new.append(np.nan)
            y_new.append(np.nan)
        t_new.append(t[i])
        y_new.append(y[i])
    return t_new, y_new

class EncoderSolver:
    def __init__(self, s1_idx, s2_idx):
        self.s1_idx = s1_idx
        self.s2_idx = s2_idx
        self.is_active = False
        
        # 5-param math constants
        self.o1 = 0
        self.o2 = 0
        self.a1 = 1
        self.a2 = 1
        self.delta = 0
        
        self.lut_2d = None
        self.lut = None

    def get_phase(self, v1, v2):
        x = (v1 - self.o1) / self.a1
        y = (v2 - self.o2) / self.a2
        sin_d = math.sin(self.delta)
        cos_d = math.cos(self.delta)
        y_corr = (y - x * sin_d) / cos_d
        return math.atan2(y_corr, x)

    def calibrate(self, data, lut_size=720):
        min1 = min(d[self.s1_idx] for d in data)
        max1 = max(d[self.s1_idx] for d in data)
        if max1 == 0 and min1 == 0: return
        self.is_active = True
        
        # --- 1. 5-Parameter Math ---
        s1 = np.array([d[self.s1_idx] for d in data])
        s2 = np.array([d[self.s2_idx] for d in data])
        def obj_func(params):
            o1, o2, a1, a2, delta = params
            x = (s1 - o1) / a1
            y = (s2 - o2) / a2
            sin_d = np.sin(delta)
            cos_d = np.cos(delta)
            y_corr = (y - x * sin_d) / cos_d
            r = np.sqrt(x**2 + y_corr**2)
            return np.sum((r - 1.0)**2)
            
        p0 = [(min1+max1)/2, (min(s2)+max(s2))/2, (max1-min1)/2, (max(s2)-min(s2))/2, 0.0]
        res = minimize(obj_func, p0, method='Nelder-Mead')
        self.o1, self.o2, self.a1, self.a2, self.delta = res.x
        
        # Build Lissajous LUT for phase disambiguation
        lut_s1 = [[] for _ in range(720)]
        lut_s2 = [[] for _ in range(720)]
        distorted = []
        times = []
        unwrapped = 0
        last_angle = None
        
        for d in data:
            v1 = d[self.s1_idx]
            v2 = d[self.s2_idx]
            ang = self.get_phase(v1, v2)
            if last_angle is not None:
                diff = ang - last_angle
                if diff > math.pi: unwrapped -= 2 * math.pi
                elif diff < -math.pi: unwrapped += 2 * math.pi
            last_angle = ang
            distorted.append(ang + unwrapped)
            times.append(d[0])
            
            d_phase = (ang + unwrapped) % (4 * math.pi)
            if d_phase < 0: d_phase += 4 * math.pi
            idx = int((d_phase / (4 * math.pi)) * 720) % 720
            lut_s1[idx].append(v1)
            lut_s2[idx].append(v2)
            
        final_s1, final_s2 = [], []
        for i in range(720):
            if len(lut_s1[i]) > 0:
                final_s1.append(sum(lut_s1[i])/len(lut_s1[i]))
                final_s2.append(sum(lut_s2[i])/len(lut_s2[i]))
            else:
                final_s1.append(None)
                final_s2.append(None)
                
        # Fixed Interpolation for missing Lissajous data
        valid_idx = [i for i, x in enumerate(final_s1) if x is not None]
        if len(valid_idx) > 0:
            for i in range(720):
                if final_s1[i] is None:
                    next_i = (i + 1) % 720
                    while final_s1[next_i] is None: next_i = (next_i + 1) % 720
                    prev_i = (i - 1) % 720
                    while final_s1[prev_i] is None: prev_i = (prev_i - 1) % 720
                    
                    d1 = (i - prev_i) % 720
                    d2 = (next_i - i) % 720
                    w1 = d2 / (d1 + d2)
                    w2 = d1 / (d1 + d2)
                    final_s1[i] = w1 * final_s1[prev_i] + w2 * final_s1[next_i]
                    final_s2[i] = w1 * final_s2[prev_i] + w2 * final_s2[next_i]
                
        self.lut_2d = list(zip(final_s1, final_s2))

        # --- 2. 4Pi Kinematic Velocity LUT ---
        dt = np.diff(times)
        dt[dt == 0] = 0.001
        speed = np.diff(distorted) / dt

        w = 1001
        pad = np.pad(speed, (w//2, w//2), mode='edge')
        macro_speed = np.convolve(pad, np.ones(w)/w, mode='valid')

        safe_speed = np.copy(speed)
        safe_speed[np.abs(safe_speed) < 0.1] = np.sign(safe_speed[np.abs(safe_speed) < 0.1]) * 0.1 + 0.001
        derivative = macro_speed / safe_speed
        derivative = np.clip(derivative, 0.3, 3.0)

        phases = np.array(distorted[:-1]) % (4 * math.pi)
        phases[phases < 0] += 4 * math.pi

        derivative_bins = [[] for _ in range(lut_size)]
        for i in range(len(phases)):
            idx = int((phases[i] / (4 * math.pi)) * lut_size) % lut_size
            derivative_bins[idx].append(derivative[i])

        avg_derivative = []
        for i in range(lut_size):
            if len(derivative_bins[i]) > 0:
                avg_derivative.append(np.mean(derivative_bins[i]))
            else:
                avg_derivative.append(None)

        # Fixed interpolation for missing derivative data
        fixed_derivative = list(avg_derivative)
        valid_idx = [i for i, x in enumerate(avg_derivative) if x is not None]
        if len(valid_idx) > 0:
            for i in range(lut_size):
                if avg_derivative[i] is None:
                    next_i = (i + 1) % lut_size
                    while avg_derivative[next_i] is None: next_i = (next_i + 1) % lut_size
                    prev_i = (i - 1) % lut_size
                    while avg_derivative[prev_i] is None: prev_i = (prev_i - 1) % lut_size
                    
                    d1 = (i - prev_i) % lut_size
                    d2 = (next_i - i) % lut_size
                    w1 = d2 / (d1 + d2)
                    w2 = d1 / (d1 + d2)
                    fixed_derivative[i] = w1 * avg_derivative[prev_i] + w2 * avg_derivative[next_i]
        avg_derivative = fixed_derivative

        w_d = 51
        pad_d = avg_derivative[-w_d//2:] + avg_derivative + avg_derivative[:w_d//2]
        smooth_derivative = np.convolve(pad_d, np.ones(w_d)/w_d, mode='valid')

        lut = np.zeros(lut_size)
        d_phase_step = (4 * math.pi) / lut_size
        for i in range(1, lut_size):
            lut[i] = lut[i-1] + smooth_derivative[i-1] * d_phase_step

        total_integral = lut[-1] + smooth_derivative[-1] * d_phase_step
        lut = lut * (2 * math.pi / total_integral)
        self.lut = lut.tolist()

    def startup_guess(self, chunk):
        """Resolves the 180-degree physical ambiguity by pattern matching against the Lissajous LUT."""
        err_A = 0
        err_B = 0
        for s1, s2 in chunk:
            ang = self.get_phase(s1, s2)
            
            # Test unwrapped = 0
            d_phase_0 = ang % (4 * math.pi)
            if d_phase_0 < 0: d_phase_0 += 4 * math.pi
            idx_0 = int((d_phase_0 / (4 * math.pi)) * len(self.lut_2d)) % len(self.lut_2d)
            if self.lut_2d[idx_0][0] is not None:
                err_A += (self.lut_2d[idx_0][0] - s1)**2 + (self.lut_2d[idx_0][1] - s2)**2
                
            # Test unwrapped = 2 * math.pi
            d_phase_2pi = (ang + 2 * math.pi) % (4 * math.pi)
            if d_phase_2pi < 0: d_phase_2pi += 4 * math.pi
            idx_2pi = int((d_phase_2pi / (4 * math.pi)) * len(self.lut_2d)) % len(self.lut_2d)
            if self.lut_2d[idx_2pi][0] is not None:
                err_B += (self.lut_2d[idx_2pi][0] - s1)**2 + (self.lut_2d[idx_2pi][1] - s2)**2
                
        return 0 if err_A < err_B else 2 * math.pi

    def track(self, data):
        if not self.is_active: return []
        
        chunk = [(d[self.s1_idx], d[self.s2_idx]) for d in data[:min(10, len(data))]]
        unwrapped = self.startup_guess(chunk)
        
        last_angle = None
        results = []
        
        for d in data:
            v1 = d[self.s1_idx]
            v2 = d[self.s2_idx]
            ang = self.get_phase(v1, v2)
            
            if last_angle is not None:
                diff = ang - last_angle
                if diff > math.pi: unwrapped -= 2 * math.pi
                elif diff < -math.pi: unwrapped += 2 * math.pi
            last_angle = ang
            
            d_phase = (ang + unwrapped) % (4 * math.pi)
            if d_phase < 0: d_phase += 4 * math.pi
            bin_float = (d_phase / (4 * math.pi)) * len(self.lut)
            idx1 = int(bin_float) % len(self.lut)
            idx2 = (idx1 + 1) % len(self.lut)
            frac = bin_float - int(bin_float)
            
            lut1 = self.lut[idx1]
            lut2 = self.lut[idx2]
            
            lut_diff = lut2 - lut1
            if lut_diff < -math.pi: lut_diff += 2 * math.pi
            if lut_diff > math.pi: lut_diff -= 2 * math.pi
            
            lut_val = lut1 + frac * lut_diff
            
            wraps = int((ang + unwrapped) / (4*math.pi))
            if (ang + unwrapped) < 0 and (ang + unwrapped) % (4*math.pi) != 0: wraps -= 1
            
            true_angle_rad = (wraps * 2 * math.pi) + lut_val
            
            # Convert to degrees and wrap to 0-360 for plotting
            wrapped_deg = (true_angle_rad % (2 * math.pi)) * (180.0 / math.pi)
            
            # Also compute naive (No LUT) angle for comparison
            naive_wrapped_deg = ((d_phase / 2.0) % (2 * math.pi)) * (180.0 / math.pi)
            
            results.append((d[0], v1, v2, naive_wrapped_deg, wrapped_deg))
            
        return results

def plot_encoder(fname, enc_name, res, enc_solver):
    times = [r[0] for r in res]
    zoom_t_limit = min(times[-1], times[0] + 10.0)
    
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle(f"{fname} - {enc_name} (Top: No LUT, Bottom: With LUT)")
    
    for row, use_lut in enumerate([False, True]):
        # Full Duration Plot
        ax1 = axes[row][0]
        ax1.set_title(f"Full Duration ({'With LUT' if use_lut else 'No LUT'})")
        ax1.plot(times, [r[1] for r in res], color='tab:blue', alpha=0.5)
        ax1.plot(times, [r[2] for r in res], color='tab:orange', alpha=0.5)
        ax2 = ax1.twinx()
        angles = [r[4] if use_lut else r[3] for r in res]
        t_nan, ang_nan = insert_nans(times, angles)
        ax2.plot(t_nan, ang_nan, color='tab:red', linewidth=1.5)
        ax2.set_ylim(-10, 370)
        
        # Zoomed Plot
        ax3 = axes[row][1]
        ax3.set_title(f"Zoomed ({'With LUT' if use_lut else 'No LUT'})")
        z_idx = np.array(times) <= zoom_t_limit
        z_times = np.array(times)[z_idx]
        ax3.plot(z_times, np.array([r[1] for r in res])[z_idx], color='tab:blue', alpha=0.8)
        ax3.plot(z_times, np.array([r[2] for r in res])[z_idx], color='tab:orange', alpha=0.8)
        ax4 = ax3.twinx()
        z_ang = np.array(angles)[z_idx]
        zt_nan, zang_nan = insert_nans(z_times, z_ang)
        ax4.plot(zt_nan, zang_nan, color='tab:red', linewidth=2.0)
        ax4.set_ylim(-10, 370)
        
        # Lissajous Plot
        ax5 = axes[row][2]
        ax5.set_title("Lissajous")
        if enc_solver.lut_2d:
            ax5.plot([p[0] for p in enc_solver.lut_2d], [p[1] for p in enc_solver.lut_2d], color='lightgray', linewidth=4)
        ax5.scatter([r[1] for r in res], [r[2] for r in res], c=times, cmap='viridis', s=10, alpha=0.7)
        
    plt.tight_layout()
    plt.savefig(f"{fname.replace('.csv', '')}_{enc_name.lower().replace(' ', '')}_compare.png", dpi=150)
    plt.close()

def export_c_header(enc1, enc2, filename="calib_constants.h"):
    with open(filename, "w") as f:
        f.write("// Auto-generated Calibration Constants for Magnetic Encoders\n")
        f.write("#ifndef CALIB_CONSTANTS_H\n")
        f.write("#define CALIB_CONSTANTS_H\n\n")
        f.write("#include <avr/pgmspace.h>\n\n")
        
        for i, enc in enumerate([enc1, enc2]):
            idx = i + 1
            f.write(f"// === ENCODER {idx} ===\n")
            f.write(f"const float ENC{idx}_O1 = {enc.o1:.3f};\n")
            f.write(f"const float ENC{idx}_O2 = {enc.o2:.3f};\n")
            f.write(f"const float ENC{idx}_A1 = {enc.a1:.3f};\n")
            f.write(f"const float ENC{idx}_A2 = {enc.a2:.3f};\n")
            f.write(f"const float ENC{idx}_DELTA = {enc.delta:.5f};\n")
            
            # Calculate the safe zone index
            max_dist = -1
            best_i = -1
            for k in range(360):
                if enc.lut_2d[k][0] is not None and enc.lut_2d[k+360][0] is not None:
                    dx = enc.lut_2d[k][0] - enc.lut_2d[k+360][0]
                    dy = enc.lut_2d[k][1] - enc.lut_2d[k+360][1]
                    dist = math.sqrt(dx**2 + dy**2)
                    if dist > max_dist:
                        max_dist = dist
                        best_i = k
            f.write(f"const int ENC{idx}_SAFE_ANGLE_IDX = {best_i};\n")
            
            f.write(f"const float ENC{idx}_LUT[720] PROGMEM = {{\n    ")
            for j, val in enumerate(enc.lut):
                f.write(f"{val:.5f}, ")
                if (j + 1) % 10 == 0:
                    f.write("\n    ")
            f.write("\n};\n\n")
            
            f.write(f"const float ENC{idx}_LISSAJOUS_S1[720] PROGMEM = {{\n    ")
            for j, val in enumerate(enc.lut_2d):
                v = val[0] if val[0] is not None else -1.0
                f.write(f"{v:.2f}, ")
                if (j + 1) % 10 == 0:
                    f.write("\n    ")
            f.write("\n};\n\n")
            
            f.write(f"const float ENC{idx}_LISSAJOUS_S2[720] PROGMEM = {{\n    ")
            for j, val in enumerate(enc.lut_2d):
                v = val[1] if val[1] is not None else -1.0
                f.write(f"{v:.2f}, ")
                if (j + 1) % 10 == 0:
                    f.write("\n    ")
            f.write("\n};\n\n")

        f.write("#endif // CALIB_CONSTANTS_H\n")

def main():
    # Only process the slow, high-quality data file
    data_file = 'data/aufnahme_20260824_010724.csv'
    
    if not os.path.exists(data_file):
        print(f"Error: Could not find {data_file}")
        return
        
    print(f"Reading {data_file}...")
    data = read_data(data_file)
    
    enc1 = EncoderSolver(1, 2)
    enc2 = EncoderSolver(3, 4)
    
    print("Calibrating encoders...")
    enc1.calibrate(data)
    enc2.calibrate(data)
    
    print("Tracking data...")
    res1 = enc1.track(data)
    res2 = enc2.track(data)
    
    print("Generating plots...")
    if res1:
        plot_encoder(os.path.basename(data_file), "Encoder 1", res1, enc1)
    if res2:
        plot_encoder(os.path.basename(data_file), "Encoder 2", res2, enc2)
    print("Exporting Arduino header file...")
    export_c_header(enc1, enc2, filename="encoder_decoder/calib_constants.h")
        
    print("Done! Plots and calib_constants.h have been saved.")

if __name__ == "__main__":
    main()
