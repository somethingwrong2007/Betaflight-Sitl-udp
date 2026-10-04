/**
 * Save-invariance harness for the LOCAL-mode DLL.
 *
 * Runs the exact same deterministic control scenario twice on the same flight
 * controller - once before a configurator save and once after it - and diffs
 * the motor outputs sample by sample. The scenario is a pure function of the
 * step index modulo the scenario period, so both traces are phase-aligned and
 * comparable: a control loop that reacts differently to the same stick + gyro
 * input after a save is exactly the "aircraft shakes after Save" symptom.
 *
 * Scenario: armed, 1500 throttle, roll stick centre -> 1800 -> centre, plus a
 * deterministic body-rate waveform (2 Hz + 19 Hz) the PID and the gyro/D-term
 * filters have to work on.
 *
 *   sitl_local_save_compare save-only     plain "Save"  (MSP_EEPROM_WRITE)
 *   sitl_local_save_compare save-reboot   "Save and Reboot"
 *
 * Note: LOCAL mode reopens stdout to NUL (the DLL normally runs inside a GUI
 * host), so all output goes to stderr.
 */

#include "sitl_local.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MSP_PORT            5761
#define MSP_SET_REBOOT      68
#define MSP_EEPROM_WRITE    250
#define MSP_API_VERSION     1
#define MSP_PID             112
#define MSP_SET_PID         202
#define MSP_RC_TUNING       111
#define MSP_SET_RC_TUNING   204
#define MSP_FILTER_CONFIG   92
#define MSP_SET_FILTER_CONFIG 93
#define MSP_PID_ADVANCED    94
#define MSP_SET_PID_ADVANCED 95
#define MSP_MAX_PAYLOAD     200

#define SCENARIO_STEPS      2000     // 2 s at 1 kHz; both traces are one period
#define WARMUP_PERIODS      3
#define ARM_CHANNEL_FIRST   4        // rc_channels[]: 0..3 = AETR, 4.. = AUX1..

#define TRACE_THROTTLE      1500     // 1500 throttle in every scenario
#define ARM_THROTTLE        1000     // firmware only arms at low throttle

// --- arming -----------------------------------------------------------------

static int gArmChannel = ARM_CHANNEL_FIRST;
static uint16_t gArmHigh = 2000;
static uint16_t gArmLow = 1000;

static void makeInput(int phase, uint16_t throttle, bool armHigh, sitl_local_input_t *in);
static bool runSteps(int count, uint16_t throttle, bool armHigh);

// Scale factor for the fast gyro component. The `sensitivity` mode doubles it
// for the second trace: the control loop has to react visibly to that, which
// proves the harness can see a change in the loop response at all.
static double gNoiseScale = 1.0;

// Every step taken since sitl_local_init(). The RC frame cadence the firmware
// sees is anchored to absolute virtual time (one COMPLETE frame every 8 ms =
// every 8 steps), so a trace is only comparable to another one if both start at
// the same phase of that cadence - otherwise feedforward/smoothing see a
// shifted frame grid and the difference is a measurement artefact.
static uint64_t gStepCount = 0;
// Step counter value at the last sitl_local_init(): the firmware's RC frame grid
// is anchored to the init, so traces are padded relative to this.
static uint64_t gStepBase = 0;

static bool rawStep(const sitl_local_input_t *in, sitl_local_output_t *out)
{
    sitl_local_step(in, 1000, out);
    gStepCount++;
    return out->armed;
}

static void padToFrameGrid(void)
{
    while (((gStepCount - gStepBase) % 8) != 0) {
        sitl_local_input_t in;
        makeInput(0, TRACE_THROTTLE, true, &in);
        sitl_local_output_t out;
        rawStep(&in, &out);
    }
}

// --- scenarios ---------------------------------------------------------------
// Each scenario drives a stick doublet per axis (centre -> high -> low ->
// centre, so the simulated craft ends up level again). The gyro feed follows the
// commanded rate, i.e. the loop tracks its setpoint instead of being pinned
// against the motor limits - saturation would hide a change behind the clamp.
typedef struct {
    int start;              // phase the doublet starts (< 0 = axis unused)
    int reverse;            // switch to the low stick
    int back;               // back to centre
    uint16_t high;
    uint16_t low;
} stickStep_t;

typedef struct {
    const char *name;
    stickStep_t roll;
    stickStep_t pitch;
    stickStep_t yaw;
    stickStep_t throttle;
    double rateScale;       // scales the per-axis rate gains for this scenario
} scenario_t;

static const scenario_t gScenarios[] = {
    { "roll-step",  {800,  950, 1100, 1800, 1200}, {-1, 0, 0, 0, 0},
                    {-1, 0, 0, 0, 0},              {-1, 0, 0, 0, 0}, 1.0 },
    { "pitch-step", {-1, 0, 0, 0, 0},              {800,  950, 1100, 1800, 1200},
                    {-1, 0, 0, 0, 0},              {-1, 0, 0, 0, 0}, 1.0 },
    { "yaw-step",   {-1, 0, 0, 0, 0},              {-1, 0, 0, 0, 0},
                    {800,  950, 1100, 1700, 1300}, {-1, 0, 0, 0, 0}, 1.0 },
    { "snap",       {800,  850,  900, 1900, 1100}, {-1, 0, 0, 0, 0},
                    {-1, 0, 0, 0, 0},              {-1, 0, 0, 0, 0}, 1.0 },
    { "throttle",   {-1, 0, 0, 0, 0},              {-1, 0, 0, 0, 0},
                    {-1, 0, 0, 0, 0},              {800,  950, 1100, 1650, 1400}, 1.0 },
    { "combined",   {800,  950, 1100, 1750, 1250}, {850, 1000, 1150, 1750, 1250},
                    {900, 1050, 1200, 1650, 1350}, {-1, 0, 0, 0, 0}, 0.55 },
};

static const scenario_t *gScenario = &gScenarios[0];

// --- control-loop signal capture ---------------------------------------------
// The motor sum alone cannot say *which* part of the loop moved. Capture the
// signals that feed it (stick setpoint, filtered gyro, each PID term) for both
// traces, so a divergence can be attributed.
#define TRACE_SIGNALS 23
static const char *const gSignalNames[TRACE_SIGNALS] = {
    "gyroADCf[roll]", "gyroADC[roll]", "rcCommand[roll]",
    "pidP[roll]", "pidI[roll]", "pidD[roll]", "pidF[roll]", "pidSum[roll]",
    "gyroADCf[pitch]", "rcCommand[pitch]", "pidSum[pitch]", "pidI[pitch]",
    "pidDeltaUs", "gyroDeltaUs",
    "sampleSum[roll]", "sampleSum[pitch]", "lpf1State", "lpf2State",
    "lpf1K",
    "cfgP[roll]", "cfgI[roll]", "cfgD[roll]", "cfgF[roll]",
};
static float gSigA[SCENARIO_STEPS][TRACE_SIGNALS];
static float gSigB[SCENARIO_STEPS][TRACE_SIGNALS];
static float gSigC[SCENARIO_STEPS][TRACE_SIGNALS];
static float (*gSigTarget)[TRACE_SIGNALS] = gSigA;

static void captureSignals(int index)
{
    sitl_local_loop_state_t ls;
    if (sitl_local_get_loop_state(&ls) != 0) {
        return;
    }
    float *row = gSigTarget[index];
    row[0] = ls.gyroADCf[0];
    row[1] = ls.gyroADC[0];
    row[2] = ls.rcCommand[0];
    row[3] = ls.pidP[0];
    row[4] = ls.pidI[0];
    row[5] = ls.pidD[0];
    row[6] = ls.pidF[0];
    row[7] = ls.pidSum[0];
    row[8] = ls.gyroADCf[1];
    row[9] = ls.rcCommand[1];
    row[10] = ls.pidSum[1];
    row[11] = ls.pidI[1];
    row[12] = (float)ls.pidDeltaUs;
    row[13] = (float)ls.gyroDeltaUs;
    row[14] = ls.sampleSum[0];
    row[15] = ls.sampleSum[1];
    row[16] = ls.lpf1State;
    row[17] = ls.lpf2State;
    row[18] = ls.lpf1K;
    row[19] = (float)ls.cfgP[0];
    row[20] = (float)ls.cfgI[0];
    row[21] = (float)ls.cfgD[0];
    row[22] = (float)ls.cfgF[0];
}

// Which loop signal moved first? The motor mix is the last stage; the signal
// that diverges first is the one the save/restart actually changed.
static void compareSignals(const char *label, float a[][TRACE_SIGNALS],
                           float b[][TRACE_SIGNALS],
                           float ma[][4], float mb[][4])
{
    fprintf(stderr, "  loop signals (%s):\n", label);
    for (int s = 0; s < TRACE_SIGNALS; s++) {
        double maxDiff = 0.0;
        int first = -1;
        double beforeAt = 0.0, afterAt = 0.0;
        for (int i = 0; i < SCENARIO_STEPS; i++) {
            const double d = fabs((double)a[i][s] - (double)b[i][s]);
            if (d > maxDiff) {
                maxDiff = d;
                beforeAt = a[i][s];
                afterAt = b[i][s];
            }
            if (first < 0 && d > 1e-4) {
                first = i;
            }
        }
        fprintf(stderr, "   %-18s first divergence step %4d  max|d|=%10.5f "
                        "(%.5f vs %.5f)\n",
                gSignalNames[s], first, maxDiff, beforeAt, afterAt);
    }

    // Raw probe values, so the loop signals can be related to the motor mix
    // (they are captured in the same step as the motor packet).
    {
        static const int probe[] = {1, 2, 3, 4, 100, 700, 760, 800, 801, 802, 850, 1000, 1100, 1400, 1999};
        fprintf(stderr, "   probe: motor0 | sum (post-LPF2) -> lpf1In (implied, post-RPM) -> lpf1Out | k\n");
        for (size_t i = 0; i < sizeof(probe) / sizeof(probe[0]); i++) {
            const int s = probe[i];
            const double kA = a[s][18] > 0.0001f ? a[s][18] : 1e-9;
            const double kB = b[s][18] > 0.0001f ? b[s][18] : 1e-9;
            const double inA = a[s - 1][16] + (a[s][16] - a[s - 1][16]) / kA;
            const double inB = b[s - 1][16] + (b[s][16] - b[s - 1][16]) / kB;
            fprintf(stderr, "    %4d A: %8.2f | %9.3f -> %9.3f -> %9.3f | %.6f | cfgP/I/D/F %.0f/%.0f/%.0f/%.0f\n",
                    s, (double)ma[s][0],
                    (double)a[s][14], inA, (double)a[s][16], kA,
                    (double)a[s][19], (double)a[s][20], (double)a[s][21], (double)a[s][22]);
            fprintf(stderr, "         B: %8.2f | %9.3f -> %9.3f -> %9.3f | %.6f | cfgP/I/D/F %.0f/%.0f/%.0f/%.0f\n",
                    (double)mb[s][0],
                    (double)b[s][14], inB, (double)b[s][16], kB,
                    (double)b[s][19], (double)b[s][20], (double)b[s][21], (double)b[s][22]);
        }
    }
}

// Sticks only move during the warm-up/trace; the arming and settling steps hold
// them centred (and the throttle at the requested low value) so the firmware's
// arming rules and the runaway-takeoff deactivation are not disturbed.
static bool gScenarioActive = false;

// Set while a save is performed with every non-ARM AUX channel high, i.e. with
// whatever modes the configurator has bound there active (ANGLE, HORIZON,
// beeper, ...). The traces themselves run with the AUX channels in their normal
// position, so the comparison stays well defined; this only changes the state
// the save happens in.
static bool gAuxHighOverride = false;

static uint16_t stickAt(const stickStep_t *s, int phase)
{
    if (s->start < 0 || !gScenarioActive) {
        return 1500;
    }
    if (phase >= s->start && phase < s->reverse) {
        return s->high;
    }
    if (phase >= s->reverse && phase < s->back) {
        return s->low;
    }
    return 1500;
}

// 300 us of roll/pitch stick = 3.5 rad/s (~200 dps), the slope the original
// scenario used; yaw gets less because the mixer's yaw authority is smaller and
// a large yaw rate would just pin the motors against their limits. A throttle
// doublet gets no rate target (throttle is not a rate axis).
static double axisRateTarget(int axis, const stickStep_t *s, int phase)
{
    static const double gainPerAxis[3] = { 3.5 / 300.0, 3.5 / 300.0, 1.2 / 300.0 };

    if (s->start < 0 || !gScenarioActive || s == &gScenario->throttle) {
        return 0.0;
    }
    return (stickAt(s, phase) - 1500.0) * gainPerAxis[axis] * gScenario->rateScale;
}

static void printStoredArmSwitch(void)
{
    uint8_t aux = 0xFF;
    uint8_t startStep = 0;
    uint8_t endStep = 0;
    sitl_local_get_arm_switch(&aux, &startStep, &endStep);

    if (aux == 0xFF) {
        fprintf(stderr, "[arm] EEPROM has no ARM switch condition\n");
        return;
    }
    fprintf(stderr, "[arm] EEPROM ARM condition: aux index %u (rc channel %d), "
                    "steps %u..%u (%u..%u us)\n",
            (unsigned)aux, ARM_CHANNEL_FIRST + aux, (unsigned)startStep, (unsigned)endStep,
            (unsigned)(900 + startStep * 25), (unsigned)(900 + endStep * 25));
}

// Probe every AUX channel and both switch positions until the firmware really
// arms - the EEPROM range/step convention, an inverted switch and unconfigured
// leftover slots all make a static guess unreliable, and a comparison that
// never arms would compare four idle motors.
static bool tryArmOnChannel(int channel, uint16_t value, int steps)
{
    for (int i = 0; i < steps; i++) {
        sitl_local_input_t in;
        makeInput(i % SCENARIO_STEPS, ARM_THROTTLE, false, &in);
        for (int c = ARM_CHANNEL_FIRST; c < SITL_LOCAL_MAX_RC_CHANNELS; c++) {
            in.rc_channels[c] = 1000;
        }
        in.rc_channels[channel] = value;

        sitl_local_output_t out;
        if (rawStep(&in, &out)) {
            return true;
        }
    }
    return false;
}

// Arm with `on`, then confirm the switch really disarms with `off`: a channel
// that only *looks* like the ARM switch (because another channel happens to sit
// inside its range) fails here.
static bool verifyArmSwitch(int channel, uint16_t on, uint16_t off)
{
    gArmChannel = channel;
    gArmHigh = on;
    gArmLow = off;

    if (!tryArmOnChannel(channel, on, 600)) {
        return false;
    }
    runSteps(400, ARM_THROTTLE, false);
    if (sitl_local_get_armed()) {
        sitl_local_disarm();
        runSteps(200, ARM_THROTTLE, false);
    }
    if (sitl_local_get_armed()) {
        return false;
    }

    fprintf(stderr, "[arm] ARM switch: rc channel %d, %u us = armed, %u us = disarmed\n",
            channel, (unsigned)on, (unsigned)off);
    return true;
}

static bool findArmSwitch(void)
{
    printStoredArmSwitch();

    // The EEPROM condition is the most reliable source: arm inside its range,
    // disarm with the extreme that lies outside it. A plain "probe every channel"
    // cannot tell which channel did it, because the channels it holds low can sit
    // inside a low-side range (the ARM switch in this EEPROM is active at
    // 900..1175 us, i.e. low means armed).
    uint8_t aux = 0xFF;
    uint8_t startStep = 0;
    uint8_t endStep = 0;
    sitl_local_get_arm_switch(&aux, &startStep, &endStep);
    if (aux != 0xFF && ARM_CHANNEL_FIRST + aux < SITL_LOCAL_MAX_RC_CHANNELS) {
        const int lowUs = 900 + startStep * 25;
        const int highUs = 900 + endStep * 25;
        int mid = (lowUs + highUs) / 2;
        if (mid < 1000) mid = 1000;
        if (mid > 2000) mid = 2000;
        const uint16_t on = (uint16_t)mid;
        const uint16_t off = (lowUs > 1100) ? 1000 : 2000;
        if (verifyArmSwitch(ARM_CHANNEL_FIRST + aux, on, off)) {
            return true;
        }
        fprintf(stderr, "[arm] the EEPROM condition did not verify, scanning instead\n");
    }

    // Fallback: every AUX channel, both extremes. Each candidate is verified
    // (must arm in the on-position and disarm in the off-position) before use.
    for (int channel = ARM_CHANNEL_FIRST; channel < SITL_LOCAL_MAX_RC_CHANNELS; channel++) {
        if (verifyArmSwitch(channel, 2000, 1000) || verifyArmSwitch(channel, 1000, 2000)) {
            return true;
        }
    }
    return false;
}

static void printArmingFlags(uint32_t flags)
{
    static const struct { uint32_t bit; const char *name; } names[] = {
        {1u << 0,  "NO_GYRO"},           {1u << 1,  "FAILSAFE"},
        {1u << 2,  "RX_FAILSAFE"},       {1u << 3,  "NOT_DISARMED"},
        {1u << 4,  "BOXFAILSAFE"},       {1u << 5,  "RUNAWAY_TAKEOFF"},
        {1u << 6,  "CRASH_DETECTED"},    {1u << 7,  "THROTTLE"},
        {1u << 8,  "ANGLE"},             {1u << 9,  "BOOT_GRACE_TIME"},
        {1u << 10, "NOPREARM"},          {1u << 11, "LOAD"},
        {1u << 12, "CALIBRATING"},       {1u << 13, "CLI"},
        {1u << 14, "CMS_MENU"},          {1u << 15, "BST"},
        {1u << 16, "MSP"},               {1u << 17, "PARALYZE"},
        {1u << 18, "GPS"},               {1u << 19, "RESC"},
        {1u << 20, "DSHOT_TELEM"},       {1u << 21, "REBOOT_REQUIRED"},
        {1u << 22, "DSHOT_BITBANG"},     {1u << 23, "ACC_CALIBRATION"},
        {1u << 24, "MOTOR_PROTOCOL"},    {1u << 25, "CRASHFLIP"},
        {1u << 26, "ALTHOLD"},           {1u << 27, "POSHOLD"},
        {1u << 28, "AUTOPILOT"},         {1u << 29, "ARM_SWITCH"},
    };

    fprintf(stderr, "[arm] armFlags=0x%08X:", (unsigned)flags);
    if (flags == 0) {
        fprintf(stderr, " none\n");
        return;
    }
    for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
        if (flags & names[i].bit) {
            fprintf(stderr, " %s", names[i].name);
        }
    }
    fprintf(stderr, "\n");
}

// --- deterministic scenario --------------------------------------------------

static void makeInput(int phase, uint16_t throttle, bool armHigh,
                      sitl_local_input_t *in)
{
    memset(in, 0, sizeof(*in));

    in->orientation_quat[0] = 1.0;   // level: the FC derives accel/mag from this
    in->position_xyz[2] = 1.0;       // 1 m altitude
    in->battery_voltage = 16.8;

    for (int i = 0; i < SITL_LOCAL_MAX_RC_CHANNELS; i++) {
        in->rc_channels[i] = 1500;
    }
    // Every AUX low, then the ARM channel high - a mode bound to another AUX
    // (e.g. ALT HOLD) would block arming and mask the control loop entirely.
    for (int i = ARM_CHANNEL_FIRST; i < SITL_LOCAL_MAX_RC_CHANNELS; i++) {
        in->rc_channels[i] = gAuxHighOverride ? 2000 : 1000;
    }
    in->rc_channels[gArmChannel] = armHigh ? gArmHigh : gArmLow;

    // Throttle: the scenario's own doublet while the scenario runs, otherwise the
    // low "arming" value.
    in->rc_channels[2] = gScenarioActive ? stickAt(&gScenario->throttle, phase) : throttle;
    in->rc_channels[0] = stickAt(&gScenario->roll, phase);
    in->rc_channels[1] = stickAt(&gScenario->pitch, phase);
    in->rc_channels[3] = stickAt(&gScenario->yaw, phase);

    // Deterministic body rates: the commanded rate (the craft follows its own
    // setpoint, keeping the loop linear) plus a fast component per axis that the
    // gyro and D-term filters have to reject - a change in filter state, gains or
    // dt shows up directly in the motor outputs.
    const double t = phase * 0.001;
    in->angular_velocity_rpy[0] = axisRateTarget(0, &gScenario->roll, phase)
                                + gNoiseScale * 0.05 * sin(2.0 * M_PI * 19.0 * t);
    in->angular_velocity_rpy[1] = axisRateTarget(1, &gScenario->pitch, phase)
                                + gNoiseScale * 0.04 * sin(2.0 * M_PI * 23.0 * t);
    in->angular_velocity_rpy[2] = axisRateTarget(2, &gScenario->yaw, phase)
                                + gNoiseScale * 0.03 * sin(2.0 * M_PI * 13.0 * t);

    // Plausible telemetry so the RPM bridge has something to work on.
    in->motor_rpm[0] = 12000.0;
    in->motor_rpm[1] = 11000.0;
    in->motor_rpm[2] = 10000.0;
    in->motor_rpm[3] = 9000.0;
    in->motor_temperature[0] = in->motor_temperature[1] = 30.0;
    in->motor_temperature[2] = in->motor_temperature[3] = 30.0;
}

static bool stepWith(int phase, uint16_t throttle, bool armHigh,
                     float motors[4], uint8_t *motorCount)
{
    sitl_local_input_t in;
    makeInput(phase, throttle, armHigh, &in);

    sitl_local_output_t out;
    rawStep(&in, &out);

    if (motors) {
        for (int i = 0; i < 4; i++) {
            motors[i] = out.pwm_output_raw[i];
        }
    }
    if (motorCount) {
        *motorCount = out.motor_count;
    }
    return out.armed;
}

static bool runSteps(int count, uint16_t throttle, bool armHigh)
{
    bool armed = sitl_local_get_armed();
    bool prev = armed;
    for (int i = 0; i < count; i++) {
        armed = stepWith(i % SCENARIO_STEPS, throttle, armHigh, NULL, NULL);
        if (armed != prev) {
            fprintf(stderr, "[arm] %s at t=%llu us (throttle=%u, arm rc ch %d = %u, %s)\n",
                    armed ? "ARMED" : "DISARMED",
                    (unsigned long long)sitl_local_time_us(), (unsigned)throttle,
                    gArmChannel, (unsigned)(armHigh ? gArmHigh : gArmLow),
                    armHigh ? "switch on" : "switch off");
            if (!armed) {
                printArmingFlags(sitl_local_get_arming_flags());
            }
            prev = armed;
        }
    }
    return armed;
}

// The firmware disarms itself (ARMING_DISABLED_RUNAWAY_TAKEOFF) when the motors
// are pushing hard while the gyro says the craft is rotating on the bench - and
// that is exactly what a synthetic gyro feed looks like. A real pilot's first
// seconds of stable flight (throttle well above the deactivation threshold,
// sticks near centre, pidSum low) turn the protection off for the rest of the
// session, so the harness does the same before measuring.
static void deactivateRunawayTakeoffProtection(void)
{
    for (int i = 0; i < 2500; i++) {
        sitl_local_input_t in;
        makeInput(0, TRACE_THROTTLE, true, &in);
        in.rc_channels[0] = 1500;
        in.rc_channels[1] = 1500;
        in.rc_channels[3] = 1500;
        in.angular_velocity_rpy[0] = 0.0;
        in.angular_velocity_rpy[1] = 0.0;
        in.angular_velocity_rpy[2] = 0.0;

        sitl_local_output_t out;
        rawStep(&in, &out);
    }
    if (!sitl_local_get_armed()) {
        fprintf(stderr, "[arm] disarmed while settling at 1500 throttle\n");
        printArmingFlags(sitl_local_get_arming_flags());
    }
}

static void runTrace(float trace[SCENARIO_STEPS][4], uint8_t *motorCount,
                     bool *armedEver, float rangeOut[2])
{
    *armedEver = false;
    float lo = 1e9f;
    float hi = -1e9f;
    int saturated = 0;

    for (int phase = 0; phase < SCENARIO_STEPS; phase++) {
        bool armed = stepWith(phase, TRACE_THROTTLE, true, trace[phase], motorCount);
        captureSignals(phase);
        if (armed) {
            *armedEver = true;
        }
        bool axisSaturated = false;
        for (int m = 0; m < 4; m++) {
            if (trace[phase][m] < lo) lo = trace[phase][m];
            if (trace[phase][m] > hi) hi = trace[phase][m];
            if (trace[phase][m] >= 1990.0f || trace[phase][m] <= 1010.0f) {
                axisSaturated = true;
            }
        }
        if (axisSaturated) {
            saturated++;
        }
    }
    rangeOut[0] = lo;
    rangeOut[1] = hi;
    fprintf(stderr, "   (saturated on at least one motor: %.1f%% of the scenario)\n",
            100.0 * saturated / SCENARIO_STEPS);
}

// Warm-up and one recorded pass of the scenario, with the scenario's sticks
// active. Both traces always use this, so they line up phase for phase.
static void warmUpAndTrace(float trace[SCENARIO_STEPS][4], uint8_t *motorCount,
                           bool *armedEver, float rangeOut[2])
{
    gScenarioActive = true;
    padToFrameGrid();
    runSteps(SCENARIO_STEPS * WARMUP_PERIODS, TRACE_THROTTLE, true);
    runTrace(trace, motorCount, armedEver, rangeOut);
    gScenarioActive = false;
}

// --- MSP over the DLL's own TCP server (what the configurator talks to) ------

static int gSock = -1;
// Path of the harness's own EEPROM copy (empty = work on the real file).
static char gHarnessEepromCopy[MAX_PATH] = "";

// --- repairing a persisted test flip -----------------------------------------
static void mspSend(uint8_t cmd, const uint8_t *payload, uint8_t len);
static int mspRequest(uint8_t cmd, const uint8_t *payload, uint8_t len,
                      uint8_t *replyOut, size_t replyCap, int timeoutMs);

// The cfg-change-* modes write a flipped byte and save it, then only restore RAM.
// If a run is interrupted (or the parity is not what you expect) the file keeps
// the flipped value. `repair <pidProfHash>` walks the 8 combinations of the
// bytes those modes ever flip, saving after each and comparing the PID-profile
// hash the firmware logs, so the config can be put back to a known fingerprint.
typedef struct {
    uint8_t getCmd;
    uint8_t setCmd;
    const char *name;
    int byteIndex;
} flipSpec_t;

static const flipSpec_t gFlipSpecs[] = {
    { MSP_PID,           MSP_SET_PID,           "pid",    1 },
    { MSP_RC_TUNING,     MSP_SET_RC_TUNING,     "rc",     1 },
    { MSP_FILTER_CONFIG, MSP_SET_FILTER_CONFIG, "filter", 28 },
};
#define FLIP_COUNT ((int)(sizeof(gFlipSpecs) / sizeof(gFlipSpecs[0])))
static uint8_t gFlipPayload[FLIP_COUNT][MSP_MAX_PAYLOAD];
static int gFlipLen[FLIP_COUNT];

static void flipToggle(int idx)
{
    const flipSpec_t *spec = &gFlipSpecs[idx];
    gFlipLen[idx] = mspRequest(spec->getCmd, NULL, 0, gFlipPayload[idx],
                               sizeof(gFlipPayload[idx]), 2000);
    if (gFlipLen[idx] <= spec->byteIndex) {
        fprintf(stderr, "[repair] %s: read failed\n", spec->name);
        exit(3);
    }
    const uint8_t was = gFlipPayload[idx][spec->byteIndex];
    gFlipPayload[idx][spec->byteIndex] ^= 0x01;
    (void)mspRequest(spec->setCmd, gFlipPayload[idx], (uint8_t)gFlipLen[idx], NULL, 0, 2000);
    fprintf(stderr, "[repair] %s byte %d: 0x%02X -> 0x%02X\n",
            spec->name, spec->byteIndex, was, gFlipPayload[idx][spec->byteIndex]);
}

// Reads one hash field from the newest "state hash" line the firmware logged.
static bool readLastStateHash(const char *field, char *out, size_t outSize)
{
    const char *appData = getenv("LOCALAPPDATA");
    if (appData == NULL) {
        return false;
    }
    char path[MAX_PATH];
    _snprintf(path, sizeof(path), "%s\\Betaflight-SITL\\sitl-audit.log", appData);

    FILE *fp = fopen(path, "rb");
    if (fp == NULL) {
        return false;
    }
    fseek(fp, 0, SEEK_END);
    const long size = ftell(fp);
    long start = size - 65536;
    if (start < 0) {
        start = 0;
    }
    fseek(fp, start, SEEK_SET);
    char *buf = (char *)malloc((size_t)(size - start) + 1);
    if (buf == NULL) {
        fclose(fp);
        return false;
    }
    const size_t got = fread(buf, 1, (size_t)(size - start), fp);
    buf[got] = '\0';
    fclose(fp);

    bool found = false;
    char *line = buf;
    while (line != NULL && *line != '\0') {
        char *eol = strchr(line, '\n');
        if (eol != NULL) {
            *eol = '\0';
        }
        // Only the fingerprint line carries the hashes; the runtime line also
        // prints "pidProf=" but with the profile *index*.
        char needle[32];
        _snprintf(needle, sizeof(needle), " %s=", field);
        const char *p = (strstr(line, " hash ") != NULL) ? strstr(line, needle) : NULL;
        if (p != NULL) {
            p += strlen(needle);
            size_t n = 0;
            while (p[n] != '\0' && p[n] != ' ' && n + 1 < outSize) {
                out[n] = p[n];
                n++;
            }
            out[n] = '\0';
            found = true;
        }
        line = (eol != NULL) ? eol + 1 : NULL;
    }
    free(buf);
    return found;
}

static int mspConnect(void)
{
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        return -1;
    }
#endif
    gSock = (int)socket(AF_INET, SOCK_STREAM, 0);
    if (gSock < 0) {
        return -1;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(MSP_PORT);
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    for (int attempt = 0; attempt < 50; attempt++) {
        if (connect(gSock, (struct sockaddr *)&addr, sizeof(addr)) == 0) {
            return 0;
        }
#ifdef _WIN32
        Sleep(100);
#else
        usleep(100000);
#endif
    }
    return -1;
}

static void mspSend(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[8 + MSP_MAX_PAYLOAD];
    uint8_t checksum = (uint8_t)(len ^ cmd);
    frame[0] = 0x24; // '$'
    frame[1] = 0x4d; // 'M'
    frame[2] = 0x3c; // '<'
    frame[3] = len;
    frame[4] = cmd;
    for (uint8_t i = 0; i < len; i++) {
        frame[5 + i] = payload[i];
        checksum ^= payload[i];
    }
    frame[5 + len] = checksum;
    send(gSock, (const char *)frame, 6 + len, 0);
}

// Waits for the reply to `cmd` as a "the FC processed it" handshake, exactly
// like the configurator does before it sends MSP_REBOOT. `replyOut` (optional)
// receives the reply payload, `replyCap` its capacity; the return value is the
// payload length, or -1 on timeout.
static int mspRequest(uint8_t cmd, const uint8_t *payload, uint8_t len,
                      uint8_t *replyOut, size_t replyCap, int timeoutMs)
{
#ifdef _WIN32
    const DWORD start = GetTickCount();
#endif
    uint8_t buf[1024];
    size_t used = 0;

    mspSend(cmd, payload, len);

    for (;;) {
        fd_set rd;
        FD_ZERO(&rd);
        FD_SET(gSock, &rd);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 50 * 1000;
        if (select(gSock + 1, &rd, NULL, NULL, &tv) > 0) {
            const int n = recv(gSock, (char *)buf + used, (int)(sizeof(buf) - used), 0);
            if (n > 0) {
                used += (size_t)n;
                size_t i = 0;
                while (i + 6 <= used) {
                    if (buf[i] != 0x24 || buf[i + 1] != 0x4d || buf[i + 2] != 0x3e) {
                        i++;
                        continue;
                    }
                    const size_t total = i + 6 + buf[i + 3];
                    if (total > used) {
                        break;
                    }
                    const uint8_t repliedCmd = buf[i + 4];
                    const uint8_t repliedLen = buf[i + 3];
                    if (replyOut && repliedLen <= replyCap) {
                        memcpy(replyOut, buf + i + 5, repliedLen);
                    }
                    memmove(buf, buf + total, used - total);
                    used -= total;
                    i = 0;
                    if (repliedCmd == cmd) {
                        return (replyOut && repliedLen > replyCap) ? -1 : repliedLen;
                    }
                }
            }
        }
#ifdef _WIN32
        if ((int)(GetTickCount() - start) >= timeoutMs) {
            return -1;
        }
#endif
    }
}

static int mspWaitReply(uint8_t cmd, int timeoutMs)
{
    return mspRequest(cmd, NULL, 0, NULL, 0, timeoutMs) < 0 ? -1 : 0;
}

// The configurator's Save is not just MSP_EEPROM_WRITE: the page first writes
// its settings back over MSP. Replay that burst byte-for-byte - read each block,
// write the very same bytes back - so the harness goes through the same firmware
// handlers the configurator drives.
typedef struct {
    uint8_t getCmd;
    uint8_t setCmd;
    const char *name;
} cfgBlock_t;

static const cfgBlock_t cfgBlocks[] = {
    { MSP_RC_TUNING,     MSP_SET_RC_TUNING,     "RC_TUNING" },
    { MSP_FILTER_CONFIG, MSP_SET_FILTER_CONFIG, "FILTER_CONFIG" },
    { MSP_PID_ADVANCED,  MSP_SET_PID_ADVANCED,  "PID_ADVANCED" },
    { MSP_PID,           MSP_SET_PID,           "PID" },
};

// -1 = replay every block; otherwise only that index (bisect).
static int gCfgOnly = -1;

static void replayConfiguratorWrites(void)
{
    for (size_t i = 0; i < sizeof(cfgBlocks) / sizeof(cfgBlocks[0]); i++) {
        if (gCfgOnly >= 0 && (size_t)gCfgOnly != i) {
            continue;
        }
        uint8_t before[MSP_MAX_PAYLOAD];
        uint8_t after[MSP_MAX_PAYLOAD];

        const int lenBefore = mspRequest(cfgBlocks[i].getCmd, NULL, 0, before,
                                        sizeof(before), 2000);
        if (lenBefore < 0) {
            fprintf(stderr, "[cfg] %s: no reply to the read, skipped\n", cfgBlocks[i].name);
            continue;
        }

        (void)mspRequest(cfgBlocks[i].setCmd, before, (uint8_t)lenBefore, NULL, 0, 2000);

        const int lenAfter = mspRequest(cfgBlocks[i].getCmd, NULL, 0, after,
                                        sizeof(after), 2000);
        const bool same = lenAfter == lenBefore
                          && memcmp(before, after, (size_t)lenBefore) == 0;
        fprintf(stderr, "[cfg] %s: read %d bytes, wrote them back, config %s\n",
                cfgBlocks[i].name, lenBefore, same ? "unchanged" : "CHANGED (abort)");
        if (!same) {
            fprintf(stderr, "[cfg] aborting: the replayed write changed the config\n");
            exit(3);
        }
    }
}

// Functional check for the change-gated rebuilds: flip one byte of the filter
// block and write it back. The config now really differs, so the firmware has to
// rebuild the filter chain and the response must change - otherwise the gate
// would have broken filter tuning. The original bytes are written back
// afterwards and verified byte for byte. Nothing is persisted (no EEPROM write).
static uint8_t gCfgOriginal[MSP_MAX_PAYLOAD];
static int gCfgOriginalLen = -1;
static uint8_t gCfgGetCmd = 0;
static uint8_t gCfgSetCmd = 0;
static const char *gCfgName = "";

// What the *loop* sees, straight from currentPidProfile: an MSP write that the
// firmware stores but never applies shows up here.
static void printLoopGains(const char *tag)
{
    sitl_local_loop_state_t ls;
    if (sitl_local_get_loop_state(&ls) == 0) {
        fprintf(stderr, "[cfg-change] loop gains %s: cfgP/I/D/F roll = %u/%u/%u/%u, "
                        "pidProfIdx=%u, PG P[roll]=%u, curPtr=%08X pgPtr=%08X%s\n",
                tag, (unsigned)ls.cfgP[0], (unsigned)ls.cfgI[0],
                (unsigned)ls.cfgD[0], (unsigned)ls.cfgF[0],
                (unsigned)ls.cfgPidIndex, (unsigned)ls.pgRollP,
                (unsigned)ls.cfgPidPtr, (unsigned)ls.pgPidPtr,
                ls.cfgPidPtr == ls.pgPidPtr ? "" : "  <-- currentPidProfile is NOT the PG record");
    }
}

// Flip one payload byte and push it back. The mask has to be big enough that
// the edit is actually visible in the loop: a 1-bit change to an I gain or an
// expo value moves the motor output by ~0.003 us, i.e. below the comparison's
// 0.01 us threshold, and dterm_lpf2_type is not applied by this build at all -
// the case would "pass" only as a false negative.
static void applyConfigVariant(uint8_t getCmd, uint8_t setCmd, const char *name,
                               int byteIndex, uint8_t mask)
{
    gCfgGetCmd = getCmd;
    gCfgSetCmd = setCmd;
    gCfgName = name;

    gCfgOriginalLen = mspRequest(getCmd, NULL, 0, gCfgOriginal, sizeof(gCfgOriginal), 2000);
    if (gCfgOriginalLen < byteIndex + 1) {
        fprintf(stderr, "[cfg-change] %s: read failed (%d bytes)\n", name, gCfgOriginalLen);
        exit(3);
    }

    uint8_t changed[MSP_MAX_PAYLOAD];
    memcpy(changed, gCfgOriginal, (size_t)gCfgOriginalLen);
    changed[byteIndex] ^= mask;
    (void)mspRequest(setCmd, changed, (uint8_t)gCfgOriginalLen, NULL, 0, 2000);

    uint8_t now[MSP_MAX_PAYLOAD];
    const int len = mspRequest(getCmd, NULL, 0, now, sizeof(now), 2000);
    const bool applied = len == gCfgOriginalLen && now[byteIndex] == changed[byteIndex];
    fprintf(stderr, "[cfg-change] %s byte %d: 0x%02X -> 0x%02X (mask 0x%02X), read back %s - the "
                    "firmware must rebuild / re-apply now\n",
            name, byteIndex, gCfgOriginal[byteIndex], changed[byteIndex], mask,
            applied ? "ok" : "MISMATCH");

    printLoopGains("after the SET");
}

static void restoreConfigVariant(void)
{
    uint8_t now[MSP_MAX_PAYLOAD];
    (void)mspRequest(gCfgSetCmd, gCfgOriginal, (uint8_t)gCfgOriginalLen, NULL, 0, 2000);
    const int len = mspRequest(gCfgGetCmd, NULL, 0, now, sizeof(now), 2000);
    const bool same = len == gCfgOriginalLen
                      && memcmp(gCfgOriginal, now, (size_t)gCfgOriginalLen) == 0;
    fprintf(stderr, "[cfg-change] %s restored: %s\n", gCfgName, same ? "yes" : "NO");
}

// Raw bytes on the same socket (used for the CLI paths, which are plain text on
// the MSP link).
static void sendRaw(const char *text)
{
    send(gSock, text, (int)strlen(text), 0);
}

// The CLI's own `save` command: writeEEPROM() + reboot, i.e. a different entry
// point than MSP_EEPROM_WRITE and worth covering - it is how most people change
// settings by hand.
static void cliSave(void)
{
    sendRaw("\r\n#");
    Sleep(300);
    sendRaw("save\r\n");
    Sleep(1500);
    fprintf(stderr, "[cli] '#' + 'save' sent (writeEEPROM + reboot)\n");
}

static void saveConfig(const char *mode)
{
    // The reply can be late the first time the MSP thread is used from this
    // process; the write itself is idempotent (RAM == EEPROM), so retry.
    int wrote = -1;
    for (int attempt = 0; attempt < 3 && wrote != 0; attempt++) {
        mspSend(MSP_API_VERSION, NULL, 0);
        (void)mspWaitReply(MSP_API_VERSION, 1500);
        mspSend(MSP_EEPROM_WRITE, NULL, 0);
        wrote = mspWaitReply(MSP_EEPROM_WRITE, 3000);
    }
    fprintf(stderr, "[save] MSP_EEPROM_WRITE %s\n",
            wrote == 0 ? "acknowledged" : "TIMED OUT");

    if (strcmp(mode, "save-reboot") == 0) {
        mspSend(MSP_SET_REBOOT, NULL, 0);
        fprintf(stderr, "[save] MSP_SET_REBOOT sent\n");
    }
}

// --- comparison --------------------------------------------------------------

static void compare(const char *label, float a[SCENARIO_STEPS][4],
                    float b[SCENARIO_STEPS][4])
{
    double worst = 0.0;
    int firstDivergence = -1;

    for (int m = 0; m < 4; m++) {
        double maxDiff = 0.0;
        double sumDiff = 0.0;
        int first = -1;
        for (int i = 0; i < SCENARIO_STEPS; i++) {
            const double d = fabs((double)a[i][m] - (double)b[i][m]);
            sumDiff += d;
            if (d > maxDiff) {
                maxDiff = d;
            }
            if (first < 0 && d > 0.01) {
                first = i;
            }
        }
        fprintf(stderr, "  motor %d: max|d|=%9.4f us  mean|d|=%8.4f us  ", m, maxDiff,
                sumDiff / SCENARIO_STEPS);
        if (first < 0) {
            fprintf(stderr, "identical to 0.01 us\n");
        } else {
            fprintf(stderr, "diverges from scenario step %d (before=%.2f us, after=%.2f us)\n",
                    first, (double)a[first][m], (double)b[first][m]);
        }
        if (maxDiff > worst) {
            worst = maxDiff;
        }
        if (first >= 0 && (firstDivergence < 0 || first < firstDivergence)) {
            firstDivergence = first;
        }
    }

    if (firstDivergence < 0) {
        fprintf(stderr, "  VERDICT (%s): identical - the save did not change the "
                        "response (worst sample %.4f us)\n", label, worst);
    } else {
        fprintf(stderr, "  VERDICT (%s): DIVERGES from scenario step %d "
                        "(worst sample %.4f us)\n", label, firstDivergence, worst);
    }

    // Where the difference lives: a transient right after the stick input looks
    // completely different from a persistent offset for the rest of the run.
    fprintf(stderr, "  divergence profile (mean |d| per 250 steps, motors 0-3):\n");
    for (int seg = 0; seg < SCENARIO_STEPS / 250; seg++) {
        double sum[4] = {0, 0, 0, 0};
        double max[4] = {0, 0, 0, 0};
        for (int i = seg * 250; i < (seg + 1) * 250; i++) {
            for (int m = 0; m < 4; m++) {
                const double d = fabs((double)a[i][m] - (double)b[i][m]);
                sum[m] += d;
                if (d > max[m]) {
                    max[m] = d;
                }
            }
        }
        fprintf(stderr, "   steps %4d-%4d: mean %6.2f/%6.2f/%6.2f/%6.2f us  "
                        "max %7.2f/%7.2f/%7.2f/%7.2f us\n",
                seg * 250, (seg + 1) * 250 - 1,
                sum[0] / 250.0, sum[1] / 250.0, sum[2] / 250.0, sum[3] / 250.0,
                max[0], max[1], max[2], max[3]);
    }

    // Sample-level view of motor 0: a constant bias, a phase shift and a
    // transient all look identical in the segment means above.
    {
        static const int probe[] = {0, 1, 2, 100, 200, 300, 400, 500, 600, 700,
                                    760, 800, 850, 900, 950, 1000, 1050, 1100,
                                    1200, 1400, 1600, 1800, 1999};
        fprintf(stderr, "  motor 0 samples (step: before -> after, diff):\n");
        for (size_t i = 0; i < sizeof(probe) / sizeof(probe[0]); i++) {
            const int s = probe[i];
            fprintf(stderr, "    %4d: %9.3f -> %9.3f  (%+7.3f)\n", s,
                    (double)a[s][0], (double)b[s][0], (double)b[s][0] - (double)a[s][0]);
        }
    }
}

// Disarm, settle, re-arm and turn the runaway protection off again. Runs before
// every trace so all traces start from the same kind of state; the retry loop
// covers the local reboot leaving the arming state machine wanting a fresh
// switch transition.
static bool settleAndRearm(void)
{
    runSteps(400, ARM_THROTTLE, false);
    if (sitl_local_get_armed()) {
        sitl_local_disarm();
        runSteps(100, ARM_THROTTLE, false);
    }
    runSteps(1000, ARM_THROTTLE, false);

    for (int attempt = 0; attempt < 3; attempt++) {
        if (runSteps(1500, ARM_THROTTLE, true)) {
            deactivateRunawayTakeoffProtection();
            return sitl_local_get_armed();
        }
        uint8_t aux = 0xFF;
        uint8_t startStep = 0;
        uint8_t endStep = 0;
        fprintf(stderr, "[arm] re-arm attempt %d failed (flightModes=0x%08X)\n",
                attempt, (unsigned)sitl_local_get_flight_modes());
        printArmingFlags(sitl_local_get_arming_flags());
        sitl_local_get_arm_switch(&aux, &startStep, &endStep);
        fprintf(stderr, "[arm] EEPROM ARM condition is now: aux %u, steps %u..%u\n",
                (unsigned)aux, (unsigned)startStep, (unsigned)endStep);
        runSteps(600, ARM_THROTTLE, false);
    }
    return false;
}

int main(int argc, char **argv)
{
    const char *mode = (argc > 1) ? argv[1] : "save-only";

    // LOCAL mode freopen()s stdout to NUL (the DLL runs inside a GUI host), so
    // everything here goes to stderr - unbuffered, so nothing is lost.
    setvbuf(stderr, NULL, _IONBF, 0);
    fprintf(stderr, "[harness] mode=%s\n", mode);
    if (argc > 2) {
        for (size_t i = 0; i < sizeof(gScenarios) / sizeof(gScenarios[0]); i++) {
            if (strcmp(argv[2], gScenarios[i].name) == 0) {
                gScenario = &gScenarios[i];
                break;
            }
        }
        fprintf(stderr, "[harness] scenario=%s\n", gScenario->name);
    }
    if (argc > 3) {
        gCfgOnly = atoi(argv[3]);
        fprintf(stderr, "[harness] configurator block filter=%d\n", gCfgOnly);
    }

    // Never touch the real EEPROM: the harness saves configurations (and the
    // cfg-change-* modes save a *flipped* byte before restoring it), so every run
    // works on a copy of the user's config in %TEMP%. The `repair` mode is the
    // one exception - it is meant to fix the real file.
    if (strcmp(mode, "repair") != 0) {
        const char *appData = getenv("LOCALAPPDATA");
        const char *tempDir = getenv("TEMP");
        if (appData != NULL && tempDir != NULL) {
            char src[MAX_PATH];
            char dst[MAX_PATH];
            _snprintf(src, sizeof(src), "%s\\Betaflight-SITL\\eeprom.bin", appData);
            _snprintf(dst, sizeof(dst), "%s\\sitl-harness-eeprom.bin", tempDir);

            FILE *in = fopen(src, "rb");
            FILE *out = (in != NULL) ? fopen(dst, "wb") : NULL;
            if (in != NULL && out != NULL) {
                char copyBuf[4096];
                size_t n;
                while ((n = fread(copyBuf, 1, sizeof(copyBuf), in)) > 0) {
                    fwrite(copyBuf, 1, n, out);
                }
            }
            if (in != NULL) fclose(in);
            if (out != NULL) fclose(out);
            if (out != NULL) {
                _putenv_s("BF_SITL_EEPROM", dst);
                strncpy(gHarnessEepromCopy, dst, sizeof(gHarnessEepromCopy) - 1);
                fprintf(stderr, "[harness] EEPROM copy: %s\n", dst);
            } else {
                fprintf(stderr, "[harness] WARNING: could not copy the EEPROM, "
                                "working on the real file\n");
            }
        }
    }

    // Tell the FC to use the copy *before* boot: sitl_local_init() applies a
    // pre-boot EEPROM path, so the real configuration file is never even opened.
    if (gHarnessEepromCopy[0] != '\0' && strcmp(mode, "repair") != 0) {
        if (sitl_local_set_eeprom_path(gHarnessEepromCopy) != 0) {
            fprintf(stderr, "[harness] WARNING: could not select the EEPROM copy\n");
        }
    }

    if (sitl_local_init() != 0) {
        fprintf(stderr, "sitl_local_init() failed\n");
        return 1;
    }
    gStepBase = gStepCount;
    if (mspConnect() != 0) {
        fprintf(stderr, "could not connect to the MSP server on 127.0.0.1:%d\n", MSP_PORT);
        sitl_local_shutdown();
        return 1;
    }

    // Maintenance: put a configuration back to a known PID-profile fingerprint
    // after one of the byte-flip test runs persisted a flip (see gFlipSpecs).
    if (strcmp(mode, "repair") == 0) {
        // usage: repair <field> <targetHash>   (field: pidProf | rates | gyro | ...)
        const char *field = (argc > 2) ? argv[2] : "pidProf";
        const char *target = (argc > 3) ? argv[3] : "";
        char now[32] = "";
        (void)readLastStateHash(field, now, sizeof(now));
        fprintf(stderr, "[repair] field %s: target=%s current=%s\n", field, target, now);
        if (target[0] != '\0' && strcmp(now, target) == 0) {
            fprintf(stderr, "[repair] already matches, nothing to do\n");
            sitl_local_shutdown();
            return 0;
        }
        for (int combo = 0; combo < (1 << FLIP_COUNT); combo++) {
            for (int i = 0; i < FLIP_COUNT; i++) {
                if (combo & (1 << i)) {
                    flipToggle(i);
                }
            }
            saveConfig("save-only");
            char after[32] = "";
            if (!readLastStateHash(field, after, sizeof(after))) {
                fprintf(stderr, "[repair] could not read the audit log\n");
                sitl_local_shutdown();
                return 3;
            }
            fprintf(stderr, "[repair] combo %d -> %s=%s\n", combo, field, after);
            if (target[0] != '\0' && strcmp(after, target) == 0) {
                fprintf(stderr, "[repair] SUCCESS with combo %d (bit i = flip gFlipSpecs[i])\n",
                        combo);
                sitl_local_shutdown();
                return 0;
            }
            // Undo this attempt before trying the next combination.
            for (int i = 0; i < FLIP_COUNT; i++) {
                if (combo & (1 << i)) {
                    flipToggle(i);
                }
            }
            saveConfig("save-only");
        }
        fprintf(stderr, "[repair] no combination matched the target\n");
        sitl_local_shutdown();
        return 3;
    }

    if (!findArmSwitch()) {
        fprintf(stderr, "[arm] FAILED: the firmware did not arm on any AUX channel.\n");
        printArmingFlags(sitl_local_get_arming_flags());
        sitl_local_shutdown();
        return 2;
    }

    static float traceA[SCENARIO_STEPS][4];
    static float traceB[SCENARIO_STEPS][4];
    uint8_t motorCountA = 0;
    uint8_t motorCountB = 0;
    bool armedA = false;
    bool armedB = false;
    float rangeA[2] = {0, 0};
    float rangeB[2] = {0, 0};

    // --- before the save -----------------------------------------------------
    // Arm at low throttle first (the firmware refuses to spin up otherwise),
    // then run the warm-up and the recorded scenario at 1500 throttle.
    if (!runSteps(1500, ARM_THROTTLE, true)) {
        fprintf(stderr, "[arm] FAILED to arm - the comparison would be meaningless.\n");
        printArmingFlags(sitl_local_get_arming_flags());
        sitl_local_shutdown();
        return 2;
    }
    fprintf(stderr, "[arm] armed\n");
    deactivateRunawayTakeoffProtection();
    if (!sitl_local_get_armed()) {
        fprintf(stderr, "[arm] FAILED: lost arming before the scenario.\n");
        printArmingFlags(sitl_local_get_arming_flags());
        sitl_local_shutdown();
        return 2;
    }
    warmUpAndTrace(traceA, &motorCountA, &armedA, rangeA);
    fprintf(stderr, "[trace] before save: armed=%d motors=%u motor range %.0f..%.0f us\n",
            armedA ? 1 : 0, (unsigned)motorCountA, (double)rangeA[0], (double)rangeA[1]);

    const bool sensitivityMode = strcmp(mode, "sensitivity") == 0;
    const bool controlMode = strcmp(mode, "control") == 0;
    const bool apiOnlyMode = strcmp(mode, "api-only") == 0;
    const bool cfgSetsMode = strcmp(mode, "cfg-sets") == 0;
    const bool cfgSaveMode = strcmp(mode, "cfg-save") == 0;
    const bool cfgSaveRebootMode = strcmp(mode, "cfg-save-reboot") == 0;
    const bool cfgChangeMode = strcmp(mode, "cfg-change") == 0;
    const bool cfgChangeFilterMode = strcmp(mode, "cfg-change-filter") == 0;
    const bool cfgChangeRateMode = strcmp(mode, "cfg-change-rate") == 0;
    const bool cfgChangePidMode = strcmp(mode, "cfg-change-pid") == 0;
    const bool twiceMode = strcmp(mode, "twice") == 0;
    const bool reinitMode = strcmp(mode, "reinit") == 0;
    const bool repeatMode = strcmp(mode, "repeat") == 0;
    const bool armedSaveMode = strcmp(mode, "armed-save") == 0;
    const bool cliSaveMode = strcmp(mode, "cli-save") == 0;
    const bool auxSaveMode = strcmp(mode, "aux-save") == 0;

    // The armed-save case first attempts a save exactly the way the configurator
    // would while flying: the firmware rejects MSP_EEPROM_WRITE while armed, so
    // nothing may change. Then it falls through to the normal sequence, which
    // makes the before/after traces comparable.
    if (armedSaveMode) {
        fprintf(stderr, "[armed-save] MSP_EEPROM_WRITE while armed (must be rejected)\n");
        saveConfig("save-only");
        runSteps(200, TRACE_THROTTLE, true);
        fprintf(stderr, "[armed-save] still armed=%d\n", sitl_local_get_armed() ? 1 : 0);
    }

    // A save while armed is rejected by the firmware, exactly like the
    // configurator; drop the ARM switch first.
    runSteps(400, ARM_THROTTLE, false);
    if (sitl_local_get_armed()) {
        sitl_local_disarm();
        runSteps(100, ARM_THROTTLE, false);
    }
    fprintf(stderr, "[trace] disarmed for the save\n");
    printArmingFlags(sitl_local_get_arming_flags());

    if (controlMode) {
        fprintf(stderr, "[control] no save, no MSP traffic - repeats the same "
                        "sequence to show what the harness measures without a save\n");
    } else if (apiOnlyMode) {
        // Bisect: same MSP round trip as a save, but no EEPROM write.
        mspSend(MSP_API_VERSION, NULL, 0);
        (void)mspWaitReply(MSP_API_VERSION, 1500);
        fprintf(stderr, "[api-only] MSP_API_VERSION round trip, no EEPROM write\n");
    } else if (cfgSetsMode) {
        fprintf(stderr, "[cfg] configurator SET burst only (no EEPROM write)\n");
        replayConfiguratorWrites();
    } else if (cfgSaveMode || cfgSaveRebootMode) {
        fprintf(stderr, "[cfg] configurator SET burst + %s\n",
                cfgSaveRebootMode ? "save and reboot" : "save");
        replayConfiguratorWrites();
        saveConfig(cfgSaveRebootMode ? "save-reboot" : "save-only");
    } else if (cfgChangeMode) {
        fprintf(stderr, "[cfg-change] real filter change (must still take effect)\n");
        // Byte 23 is the high byte of gyro_lpf2_static_hz (500 -> 244 Hz): a
        // real change to the downsampling lowpass, and reverting the byte
        // restores the exact filter.
        applyConfigVariant(MSP_FILTER_CONFIG, MSP_SET_FILTER_CONFIG, "filter", 23, 0x01);
    } else if (cfgChangeFilterMode) {
        fprintf(stderr, "[cfg-change] real filter change + save (must take effect)\n");
        applyConfigVariant(MSP_FILTER_CONFIG, MSP_SET_FILTER_CONFIG, "filter", 23, 0x01);
        saveConfig("save-only");
        printLoopGains("after the save");
    } else if (cfgChangeRateMode) {
        fprintf(stderr, "[cfg-change] real rate/expo change + save (must take effect)\n");
        // Byte 0 is rcRates[ROLL] in hundredths (100 -> 108): an 8% rate change
        // the stick doublet has to react to.
        applyConfigVariant(MSP_RC_TUNING, MSP_SET_RC_TUNING, "rc_tuning", 0, 0x08);
        saveConfig("save-only");
        printLoopGains("after the save");
    } else if (cfgChangePidMode) {
        fprintf(stderr, "[cfg-change] real PID change + save (must take effect)\n");
        // Byte 0 is P[ROLL] in tenths (45 -> 61): a 35% gain change.
        applyConfigVariant(MSP_PID, MSP_SET_PID, "pid", 0, 0x10);
        saveConfig("save-only");
        printLoopGains("after the save");
    } else if (twiceMode) {
        fprintf(stderr, "[twice] save, measure, save again, measure\n");
        saveConfig("save-only");
    } else if (reinitMode) {
        fprintf(stderr, "[reinit] save, measure, then shutdown + init (fresh boot)\n");
        saveConfig("save-only");
    } else if (armedSaveMode) {
        // Already done above, before the disarm.
        fprintf(stderr, "[armed-save] proceeding with the normal save after the rejected one\n");
        saveConfig("save-only");
    } else if (cliSaveMode) {
        fprintf(stderr, "[cli-save] CLI '#': save (writeEEPROM + reboot)\n");
        cliSave();
    } else if (repeatMode) {
        fprintf(stderr, "[repeat] five saves in a row\n");
        saveConfig("save-only");
    } else if (auxSaveMode) {
        fprintf(stderr, "[aux-save] save while every non-ARM AUX is high "
                        "(other mode boxes active)\n");
        gAuxHighOverride = true;
        runSteps(400, TRACE_THROTTLE, false);
        saveConfig("save-only");
        runSteps(400, TRACE_THROTTLE, false);
        gAuxHighOverride = false;
        fprintf(stderr, "[aux-save] AUX back to normal, arming flags now:\n");
        printArmingFlags(sitl_local_get_arming_flags());
    } else if (strcmp(mode, "reload") == 0) {
        // Runtime reload of the *same* file: the values cannot change, so the FC
        // must keep responding identically. Any difference here is the reload
        // injecting control-loop state (the "shakes after switching aircraft"
        // mechanism), which is what localRunPendingReloadInternal must avoid.
        fprintf(stderr, "[reload] sitl_local_set_eeprom_path + sitl_local_reload_config "
                        "(same file)\n");
        if (gHarnessEepromCopy[0] != '\0') {
            (void)sitl_local_set_eeprom_path(gHarnessEepromCopy);
        }
        (void)sitl_local_reload_config();
        runSteps(600, TRACE_THROTTLE, false);
    } else if (sensitivityMode) {
        fprintf(stderr, "[sensitivity] no save: the second trace doubles the gyro "
                        "noise instead, to prove a loop change is visible\n");
    } else {
        saveConfig(mode);
    }

    // Let the background thread persist and the queued reboot/reload work run on
    // the stepping thread (that is where a firmware reboot re-applies the
    // config), then re-arm. Identical settle sequence before every trace, so the
    // traces are comparable.
    if (cliSaveMode) {
        runSteps(1000, TRACE_THROTTLE, false);
    }
    if (!settleAndRearm()) {
        fprintf(stderr, "[arm] FAILED to re-arm after the save.\n");
        sitl_local_shutdown();
        return 2;
    }

    // --- after the save ------------------------------------------------------
    padToFrameGrid();
    gSigTarget = gSigB;
    if (sensitivityMode) {
        gNoiseScale = 2.0;
    }
    warmUpAndTrace(traceB, &motorCountB, &armedB, rangeB);
    fprintf(stderr, "[trace] after save: armed=%d motors=%u motor range %.0f..%.0f us\n",
            armedB ? 1 : 0, (unsigned)motorCountB, (double)rangeB[0], (double)rangeB[1]);

    fprintf(stderr, "\n=== %s / %s: same input before vs after ===\n", mode, gScenario->name);
    compare(mode, traceA, traceB);

    compareSignals("before vs after the save", gSigA, gSigB, traceA, traceB);

    // Stability check: repeat the after-trace with no further save. If the two
    // after-traces match while the before/after pair does not, the save left the
    // loop on a *different but stable* operating point instead of a transient
    // that just needs more settling time.
    static float traceC[SCENARIO_STEPS][4];
    uint8_t motorCountC = 0;
    bool armedC = false;
    float rangeC[2] = {0, 0};
    const bool cfgChangedConfig = cfgChangeMode || cfgChangeFilterMode
                                  || cfgChangeRateMode || cfgChangePidMode;
    if (cfgChangedConfig) {
        restoreConfigVariant();
    }
    if (twiceMode) {
        fprintf(stderr, "[twice] second save, nothing changed since the first\n");
        saveConfig("save-only");
    }
    if (repeatMode) {
        // Four more saves (the first one already happened): a drift check.
        for (int i = 0; i < 4; i++) {
            saveConfig("save-only");
            runSteps(300, TRACE_THROTTLE, false);
        }
        fprintf(stderr, "[repeat] five saves done, measuring again\n");
    }
    if (reinitMode) {
        fprintf(stderr, "[reinit] sitl_local_shutdown() + sitl_local_init()\n");
        sitl_local_shutdown();
        gSock = -1;
        if (sitl_local_init() != 0) {
            fprintf(stderr, "[reinit] sitl_local_init() failed\n");
            return 1;
        }
        gStepBase = gStepCount;
        if (mspConnect() != 0) {
            fprintf(stderr, "[reinit] MSP reconnect failed\n");
            return 1;
        }
        if (!findArmSwitch()) {
            fprintf(stderr, "[reinit] could not arm after the re-init\n");
            return 2;
        }
    }
    if (!settleAndRearm()) {
        fprintf(stderr, "[arm] FAILED to re-arm for the stability trace.\n");
        sitl_local_shutdown();
        return 2;
    }
    gSigTarget = gSigC;
    warmUpAndTrace(traceC, &motorCountC, &armedC, rangeC);
    if (cfgChangedConfig) {
        fprintf(stderr, "\n=== %s: after writing the original bytes back ===\n", mode);
        compare("change-reverted", traceA, traceC);
    } else if (twiceMode) {
        fprintf(stderr, "\n=== twice: two saves in a row, second one changed nothing ===\n");
        compare("after1-vs-after2", traceB, traceC);
    } else if (repeatMode) {
        fprintf(stderr, "\n=== repeat: five saves, no drift allowed ===\n");
        compare("after1-vs-after5", traceB, traceC);
    } else if (reinitMode) {
        fprintf(stderr, "\n=== reinit: post-save state vs a fresh in-process boot ===\n");
        compare("after-save-vs-fresh-boot", traceB, traceC);
        compareSignals("post-save vs fresh in-process boot", gSigB, gSigC, traceB, traceC);
    } else {
        fprintf(stderr, "\n=== %s: after-save trace repeated (stability check) ===\n", mode);
        compare("after-vs-after", traceB, traceC);
    }

    sitl_local_shutdown();
    return 0;
}
