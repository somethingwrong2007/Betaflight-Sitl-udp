#!/usr/bin/env python3
"""Quantify the vibration content of the LOCAL gyro feed from sitl-burst.log.

The burst recorder (src/sitl_local.c) writes one line per FC loop period:

  t_us gyroR gyroP gyroY pR pP pY iR iP iY dR dP dY fR fP fY sumR sumP sumY
  m0 m1 m2 m3 attR attP attY modes armFlags pidDeltaUs gyroDeltaUs
  rpm0 rpm1 rpm2 rpm3 notch1 notch2 notch3 gyroADCfR gyroADCfP gyroADCfY

  gyroR/P/Y   raw int16 written into the virtual gyro (LSB, 16.4 LSB per deg/s)
  gyroADCf*   gyro the PID actually sees, after the whole filter chain (deg/s)
  rpm*        per-motor mechanical frequency the RPM filter works from (Hz)

The point of the report is to separate "the filters are doing something" from
"there is something to filter": the filter chain can only pay off if the feed
carries motor vibration. A real 5" quad shows clearly visible peaks at the
motor fundamental and its harmonics; a clean simulator feed shows a flat floor.

Usage:
    python3 tools/analyze_gyro_noise.py [path/to/sitl-burst.log]

Default path: %LOCALAPPDATA%\\Betaflight-SITL\\sitl-burst.log (Windows) or
$LOCALAPPDATA/Betaflight-SITL/sitl-burst.log when run from a shell that has it.
"""

import os
import sys

import numpy as np

LSB_PER_DEG_S = 16.4      # LOCAL_GYRO_SCALE in src/sitl_local.c
CONTROL_HZ = (20.0, 80.0)  # a typical rate-loop bandwidth


def default_path():
    appdata = os.environ.get("LOCALAPPDATA")
    if appdata:
        return os.path.join(appdata, "Betaflight-SITL", "sitl-burst.log")
    return "/mnt/c/Users/ADMIN/AppData/Local/Betaflight-SITL/sitl-burst.log"


def load(path):
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        lines = [ln for ln in fh if ln.strip() and not ln.lstrip().startswith("#")]
    data = np.array([[float(v) for v in ln.split()] for ln in lines])
    return data


def band_rms(freqs, psd, lo, hi):
    sel = (freqs >= lo) & (freqs < hi)
    if not np.any(sel):
        return 0.0
    # one-sided PSD (no window) -> integrate
    df = freqs[1] - freqs[0]
    return float(np.sqrt(np.sum(psd[sel]) * df))


def spectrum(x, fs):
    n = len(x)
    win = np.hanning(n)
    xf = np.fft.rfft((x - x.mean()) * win)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    # amplitude of a sinusoid: |X| * 2 / sum(win)
    amp = np.abs(xf) * 2.0 / np.sum(win)
    return freqs, amp, (amp ** 2) / 2.0 / df_of(freqs)


def df_of(freqs):
    return freqs[1] - freqs[0]


def describe(tag, x, fs, motor_hz):
    freqs, amp, _ = spectrum(x, fs)
    df = df_of(freqs)
    psd = (amp ** 2) / 2.0 / df
    peak = int(np.argmax(amp))
    control = band_rms(freqs, psd, *CONTROL_HZ)
    mid = band_rms(freqs, psd, 80.0, 300.0)
    high = band_rms(freqs, psd, 300.0, min(1000.0, fs / 2.0 - 1))
    # amplitude at each motor fundamental (+-2 Hz) and its 2nd harmonic
    harms = []
    for f0 in motor_hz:
        for mult in (1, 2):
            f = f0 * mult
            sel = np.abs(freqs - f) <= 2.0
            harms.append(float(np.max(amp[sel])) if np.any(sel) else 0.0)
    print(f"  {tag:12s} std={x.std():8.3f}  diff-rms={np.sqrt(np.mean(np.diff(x) ** 2)):8.3f}  "
          f"peak={freqs[peak]:7.1f} Hz ({amp[peak]:.3f})  "
          f"band rms: {CONTROL_HZ[0]:.0f}-{CONTROL_HZ[1]:.0f}={control:.3f}  "
          f"80-300={mid:.3f}  300+={high:.3f}  "
          f"motor f1/f2 max={max(harms) if harms else 0.0:.3f}")
    return dict(std=float(x.std()), control=control, mid=mid, high=high,
                motor_peak=float(max(harms)) if harms else 0.0)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else default_path()
    if not os.path.exists(path):
        print(f"no burst log at {path}")
        return 1
    d = load(path)
    t = d[:, 0] * 1e-6
    dts = np.diff(t)
    fs = 1.0 / np.median(dts)
    print(f"{path}")
    print(f"  samples={len(d)}  duration={t[-1] - t[0]:.3f} s  "
          f"loop={np.median(dts) * 1e6:.0f} us ({fs:.0f} Hz)  "
          f"dt min/max={dts.min() * 1e6:.0f}/{dts.max() * 1e6:.0f} us")

    gyro_raw = d[:, 1:4] * (1.0 / LSB_PER_DEG_S)     # LSB -> deg/s
    gyro_filt = d[:, -3:]                            # deg/s
    pid_d = d[:, 10:13]          # dR dP dY
    motors = d[:, 19:23]         # m0..m3
    arm_flags = d[:, 27]
    rpm = d[:, 30:34]            # rpm0..rpm3 (pidDeltaUs, gyroDeltaUs come first)

    motor_hz = [float(np.mean(rpm[:, i])) for i in range(4)]
    print(f"  motors={np.mean(motors, axis=0).round(0)}  "
          f"motor f={np.round(motor_hz, 1)} Hz  armFlags=0x{int(np.max(arm_flags)):08X}")
    print("  fed gyro (what the host sends, deg/s):")
    stats = [describe(ax, gyro_raw[:, i], fs, motor_hz)
             for i, ax in enumerate(("roll", "pitch", "yaw"))]
    print("  filtered gyro (what the PID sees, deg/s):")
    for i, ax in enumerate(("roll", "pitch", "yaw")):
        describe(ax, gyro_filt[:, i], fs, motor_hz)
    print(f"  |D| mean = {np.mean(np.abs(pid_d), axis=0).round(3)} (deg/s * D gain)")

    ratio = max(s["motor_peak"] for s in stats)
    ctrl = max(s["control"] for s in stats)
    print()
    if ratio < 0.05:
        print("verdict: the fed gyro has no visible motor vibration "
              f"(peak at the motor harmonics = {ratio:.3f} deg/s). "
              "A filter chain can only add phase lag here - no vibration to remove.")
    else:
        print(f"verdict: motor vibration present in the feed "
              f"(peak {ratio:.3f} deg/s vs control band {ctrl:.3f} deg/s rms) - "
              "the notch/LPF chain has something to work on.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
