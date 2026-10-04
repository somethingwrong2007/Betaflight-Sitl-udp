"""Frequency response of the Betaflight RPM-filter notch bank.

Reproduces the exact recurrence of `rpmNotchApply()` (common/filter.c) and
cascades the notches the firmware would apply for a given motor frequency, so
the phase lag the RPM filter injects into the control band can be quantified.

    python3 tools/rpm_notch_response.py

`rpm_filter_harmonics` (default 3) places a notch at every multiple of the motor
mechanical frequency. Betaflight clamps a notch that would sit above
`0.48 * 1e6 / looptimeUs` to that limit - at the 1 kHz loop used here that is
480 Hz, i.e. 0.96 Nyquist - so the harmonics that do not fit end up as a stack of
notches at the same near-Nyquist frequency instead of notches at the vibration
frequencies. Those notches cannot filter anything the 1 kHz-sampled gyro can
represent, but they do cost phase margin (see the printout).
"""

import math
import numpy as np

FS = 1000.0                 # 1 kHz gyro/PID loop
DT = 1.0 / FS
Q = 500 / 100.0             # rpm_filter_q default (500) -> Q = 5
MIN_HZ = 100.0              # rpm_filter_min_hz default
WEIGHTS = [1.0, 1.0, 1.0]   # rpm_filter_weights default
MAX_HZ = 0.48 / DT          # rpmFilter.maxHz, the firmware's clamp


def notch_impulse(f0, weight, n=16384):
    # rpmNotchUpdate() + rpmNotchApply(), one axis
    f = math.tan(math.pi * f0 * DT)
    q = 1.0 / Q
    a1 = 1.0 / (1.0 + f * (f + q))
    a2 = f * a1
    wq = q * weight

    ic1 = ic2 = 0.0
    h = np.zeros(n)
    for k in range(n):
        x = 1.0 if k == 0 else 0.0
        v3 = x - ic2
        v1 = a1 * ic1 + a2 * v3
        v2 = ic2 + f * v1
        ic1 = 2.0 * v1 - ic1
        ic2 = 2.0 * v2 - ic2
        h[k] = x - wq * v1
    return h


def cascade(freqs, weights, n=16384):
    h_total = np.ones(n // 2 + 1, dtype=complex)
    for f0, w in zip(freqs, weights):
        h_total *= np.fft.rfft(notch_impulse(f0, w, n))
    return h_total


def report(name, freqs, weights, n=16384):
    response = cascade(freqs, weights, n)
    spectrum = np.fft.rfftfreq(n, DT)
    print(f"--- {name}: {len(freqs)} notches "
          f"{[round(f) for f in freqs]}")
    for probe in (20, 50, 80, 100, 120, 150, 200, 250, 300, 400):
        i = int(np.argmin(abs(spectrum - probe)))
        mag = 20 * math.log10(abs(response[i]) + 1e-12)
        phase = math.degrees(np.angle(response[i]))
        print(f"   {probe:4d} Hz: {mag:7.2f} dB   phase {phase:8.2f} deg")


def harmonics(motor_hz, count, drop_clamped):
    freqs, weights = [], []
    for mhz in motor_hz:
        for h in range(count):
            f0 = (h + 1) * mhz
            if drop_clamped and f0 > MAX_HZ:
                continue
            freqs.append(min(max(f0, MIN_HZ), MAX_HZ))
            weights.append(WEIGHTS[h])
    return freqs, weights


def main():
    # RPM the save-invariance harness feeds: 12000/11000/10000/9000
    harness_hz = [12000 / 60.0, 11000 / 60.0, 10000 / 60.0, 9000 / 60.0]

    report("harness feed, firmware default (clamped harmonics included)",
           *harmonics(harness_hz, 3, drop_clamped=False))
    report("harness feed, only harmonics that fit below 0.48/dt",
           *harmonics(harness_hz, 3, drop_clamped=True))

    # Unreal's motor model: RPM = KV * (V - I*R), KV = 2000, 6S.
    # ~45% duty (hover) -> ~15000 rpm, full throttle -> ~33000 rpm.
    for label, rpm in (("hover ~15000 rpm (250 Hz)", 15000.0),
                       ("full throttle ~33000 rpm (550 Hz)", 33000.0)):
        unreal_hz = [rpm / 60.0 * s for s in (1.0, 0.98, 0.96, 0.94)]
        print()
        report(f"Unreal {label}, firmware default",
               *harmonics(unreal_hz, 3, drop_clamped=False))
        report(f"Unreal {label}, only harmonics that fit",
               *harmonics(unreal_hz, 3, drop_clamped=True))

    print()
    report("1 harmonic per motor (harness feed)",
           *harmonics(harness_hz, 1, drop_clamped=False))
    report("RPM filter off", [], [])


if __name__ == "__main__":
    main()
