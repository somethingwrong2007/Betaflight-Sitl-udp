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
 *   sitl_local_save_compare reinit        in-process FC restart (shutdown+init)
 *   sitl_local_save_compare osd-reboot    every reboot trigger must redraw the
 *                                         OSD boot splash ("MENU:THR MID")
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

// Test tools must not fight a live host for the configurator ports: Windows lets
// a second socket bind 5761 with SO_REUSEADDR and silently take the new
// connections. The harness therefore moves its own instance (and its MSP
// client) to BF_SITL_TCP_BASE, which main() sets before sitl_local_init().
#define DEFAULT_MSP_PORT    5761
#define MSP_SET_REBOOT      68
#define MSP_EEPROM_WRITE    250
#define MSP_API_VERSION     1
#define MSP_OSD_CHAR_WRITE  87
#define MSP_BATTERY_STATE   130
#define MSP_BATTERY_CONFIG  32
#define MSP_SET_BATTERY_CONFIG 33
#define MSP_PID             112
#define MSP_SET_PID         202
#define MSP_RC_TUNING       111
#define MSP_SET_RC_TUNING   204
#define MSP_FILTER_CONFIG   92
#define MSP_SET_FILTER_CONFIG 93
#define MSP_BLACKBOX_CONFIG 80
#define MSP_SET_BLACKBOX_CONFIG 81
#define MSP_FEATURE_CONFIG 36
#define MSP_SET_FEATURE_CONFIG 37
// config/feature.h: FEATURE_OSD = 1 << 18
#define FEATURE_BIT_OSD 0x00040000u
// blackbox.h's blackboxDevice_e, not includable from a test tool.
#define BLACKBOX_DEV_NONE    0
#define BLACKBOX_DEV_VIRTUAL 4
#define MSP_PID_ADVANCED    94
#define MSP_SET_PID_ADVANCED 95
#define MSP_MAX_PAYLOAD     200

#define SCENARIO_STEPS      2000     // 2 s at 1 kHz; both traces are one period
#define WARMUP_PERIODS      3
#define ARM_CHANNEL_FIRST   4        // rc_channels[]: 0..3 = AETR, 4.. = AUX1..

// Sample-to-sample difference that still counts as "the same response". The two
// traces are prepared by separate arm/settle/warm-up sequences, so the PID
// integrator's random walk leaves a residue that scales with the loop rate: at a
// 1 kHz loop it is ~2e-3 us, at 4 kHz ~5e-2 us (with the gyro, the filters and
// the RC command bit-identical). The tolerance therefore scales with the loop
// period the firmware reports; everything the harness is looking for - a live
// re-init of a stateful chain, a save changing the stick response - shows up as
// tens to hundreds of microseconds, i.e. two to three orders of magnitude more.
static double gTraceToleranceUs = 0.05;

#define TRACE_THROTTLE      1500     // 1500 throttle in every scenario
#define ARM_THROTTLE        1000     // firmware only arms at low throttle

// --- arming -----------------------------------------------------------------

static int gArmChannel = ARM_CHANNEL_FIRST;
// Arming probes feed a quiet, level input (no injected body rates, sticks
// centred): the firmware integrates the rates into its attitude estimate, and a
// tilted estimate blocks arming with ARMING_DISABLED_ANGLE.
static bool gQuietInput = false;
static uint16_t gArmHigh = 2000;
static uint16_t gArmLow = 1000;
// Battery current the virtual meter reports (mA are integrated by the DLL).
// Zero by default so the save/consistency scenarios stay identical to before.
static double gHarnessBatteryCurrent = 0.0;
// Pack voltage the virtual meter reports (V).
static double gHarnessBatteryVoltage = 16.8;
// Optional extra AUX channel / stick overrides for the mode tests.
static int gHarnessModeChannel = -1;
static bool gHarnessModeHigh = false;
static int gHarnessRollOverride = -1;

static void makeInput(int phase, uint16_t throttle, bool armHigh, sitl_local_input_t *in);
static bool runSteps(int count, uint16_t throttle, bool armHigh);

// The MSP port of *this process's* FC instance. Matches the DLL's own
// BF_SITL_TCP_BASE + 1 (see serial_tcp_win.c), so a harness run cannot talk to
// somebody else's flight controller by accident.
static uint16_t harnessMspPort(void)
{
    const char *env = getenv("BF_SITL_TCP_BASE");
    const long v = (env != NULL && env[0] != '\0') ? strtol(env, NULL, 10) : 0;
    return (uint16_t)((v >= 1024 && v <= 65000) ? (v + 1) : DEFAULT_MSP_PORT);
}

// Move this instance off the configurator's ports unless the caller asked for a
// specific block: Windows lets a second socket bind 5761 with SO_REUSEADDR and
// silently steal the new connections, which would disturb a live host.
static void harnessClaimPrivatePorts(void)
{
    if (getenv("BF_SITL_TCP_BASE") == NULL) {
        _putenv_s("BF_SITL_TCP_BASE", "15760");
    }
    if (getenv("BF_SITL_WS_PORT") == NULL) {
        _putenv_s("BF_SITL_WS_PORT", "16761");
    }
    // Keep the test tool's logs out of the live host's files (this process does
    // not touch %LOCALAPPDATA%\Betaflight-SITL while a flight is running).
    const char *tempDir = getenv("TEMP");
    if (tempDir != NULL && getenv("BF_SITL_AUDIT_LOG") == NULL) {
        static char auditPath[MAX_PATH];
        _snprintf(auditPath, sizeof(auditPath), "%s\\sitl-harness-audit.log", tempDir);
        _putenv_s("BF_SITL_AUDIT_LOG", auditPath);
    }
    if (tempDir != NULL && getenv("BF_SITL_BURST_LOG") == NULL) {
        static char burstPath[MAX_PATH];
        _snprintf(burstPath, sizeof(burstPath), "%s\\sitl-harness-burst.log", tempDir);
        _putenv_s("BF_SITL_BURST_LOG", burstPath);
    }
    // The blackbox rotates the newest N logs by renaming them; doing that in the
    // live host's folder while the host holds a log open fails ("could not
    // renumber ... file in use?") and disturbs the user's own logs.
    if (tempDir != NULL && getenv("BF_SITL_BLACKBOX_DIR") == NULL) {
        static char bbPath[MAX_PATH];
        _snprintf(bbPath, sizeof(bbPath), "%s\\sitl-harness-blackbox", tempDir);
        _putenv_s("BF_SITL_BLACKBOX_DIR", bbPath);
    }
    fprintf(stderr, "[harness] FC instance on 127.0.0.1:%u (ws %s)\n",
            (unsigned)harnessMspPort(), getenv("BF_SITL_WS_PORT"));
}

#ifdef _WIN32
static double harnessWallSeconds(void)
{
    LARGE_INTEGER freq, now;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&now);
    return (double)now.QuadPart / (double)freq.QuadPart;
}

static double harnessCpuSeconds(void)
{
    FILETIME create, exitT, kernel, user;
    if (!GetProcessTimes(GetCurrentProcess(), &create, &exitT, &kernel, &user)) {
        return 0.0;
    }
    ULARGE_INTEGER k, u;
    k.LowPart = kernel.dwLowDateTime; k.HighPart = kernel.dwHighDateTime;
    u.LowPart = user.dwLowDateTime;   u.HighPart = user.dwHighDateTime;
    return (double)(k.QuadPart + u.QuadPart) * 1e-7;
}
#else
static double harnessWallSeconds(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static double harnessCpuSeconds(void)
{
    return (double)clock() / (double)CLOCKS_PER_SEC;
}
#endif

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

// --- closed-loop plant --------------------------------------------------------
// `closed-loop` closes the harness around a deliberately simple aircraft model:
// the body rate is a first-order lag driven by the motor differential (the same
// differential the firmware's mixer produces), so the FC sees its own motor
// output instead of a prescribed waveform. That is what turns a phase-margin
// problem into the visible symptom: a sustained limit cycle.
#define PLANT_TAU_S      0.025   // 25 ms rate response
static double gPlantGain = 0.017;   // rad/s per us of motor differential
#define PLANT_GAIN       gPlantGain
// 0 = a pure integrator with no angular damping (BF_HARNESS_PLANT_TAU_MS=0),
// i.e. what an ideal rigid-body simulator looks like.
static double gPlantTauS = PLANT_TAU_S;
static bool gClosedLoop = false;
static double gPlantRate[3] = { 0.0, 0.0, 0.0 };
static float gLastMotorsForPlant[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
static double gPlantPkPk = 0.0;   // motor 0 peak-to-peak over the last 500 steps
static double gRatePkPk = 0.0;    // plant roll rate peak-to-peak, same window
static double gClosedMin[2] = { 0.0, 0.0 };   // { motor0, plant rate } min
static double gClosedMax[2] = { 0.0, 0.0 };   // { motor0, plant rate } max

// Step-period jitter: BF_SITL_DT_JITTER=1 feeds the FC a *varying* dt the way a
// host that forwards its own (jittery) frame time does, instead of the fixed
// 1 ms grid the LOCAL API is specified around. Every filter in the chain is
// designed for one dt; the RPM filter's notches in particular are narrow, so a
// varying sample interval moves them off the motor harmonic.
static bool gStepDtJitter = false;
static uint32_t gStepUs = 1000;      // host step (BF_HARNESS_STEP_US)
static uint32_t gLastStepDtUs = 1000;

// RPM input source: a fixed bench value, or - like Unreal's ESC model - the
// motor speeds *derived from the motor outputs the FC itself just produced*
// (RPM = KV * duty), optionally through the motor/prop's physical time constant.
// This is the "input data" the RPM filter works from; if it carries the PID's
// own noise, the notches chase the loop.
static bool gRpmFromMotors = false;
static double gMotorTauMs = 0.0;   // 0 = instant (an "ideal" ESC model)
static double gRpmLagged[4] = { 0.0, 0.0, 0.0, 0.0 };

// Motor-induced body vibration in the *gyro input data*: a real gyro sees the
// motor harmonics, and that is exactly what the RPM filter exists to remove. A
// rigid-body simulator has none, which is why the notches can only cost phase
// there. Amplitudes are in rad/s per motor harmonic (BF_HARNESS_VIB_RADPS).
static double gVibAmp = 0.0;

static double sitlHarnessMotorRpm(int index)
{
    const double target = 24000.0 * (gLastMotorsForPlant[index] - 1000.0f) / 1000.0;
    if (gMotorTauMs <= 0.0) {
        return target;
    }
    const double dt = gLastStepDtUs * 1e-6;
    const double k = dt / (gMotorTauMs * 1e-3 + dt);
    gRpmLagged[index] += k * (target - gRpmLagged[index]);
    return gRpmLagged[index];
}

static uint32_t nextStepDtUs(void)
{
    if (!gStepDtJitter) {
        return gStepUs;
    }
    // Deterministic +/-20% pattern around the configured step; the physics tick
    // of a real engine jitters around this much and is usually quantized to
    // 100 us by the host (at the default 1000 us step that is exactly
    // 1000/1200/800/1100/900).
    static const int pattern[] = { 0, +20, -20, +10, -10, 0, +20, -20 };
    const int pct = pattern[gStepCount % (sizeof(pattern) / sizeof(pattern[0]))];
    return (uint32_t)((int)gStepUs + ((int)gStepUs * pct) / 100);
}

static bool rawStep(const sitl_local_input_t *in, sitl_local_output_t *out)
{
    const uint32_t dtUs = nextStepDtUs();
    sitl_local_step(in, dtUs, out);
    gLastStepDtUs = dtUs;
    gStepCount++;
    for (int m = 0; m < 4; m++) {
        gLastMotorsForPlant[m] = out->pwm_output_raw[m];
    }
    return out->armed;
}

static void padToFrameGrid(void)
{
    // The firmware's RC frame cadence is 8 ms of virtual time; align the trace
    // to it in *steps*, which depends on the host step size (8 steps at 1 ms,
    // 32 at 250 us).
    const uint64_t stepsPerFrame = (8000 + gStepUs / 2) / gStepUs;
    while (stepsPerFrame > 0 && ((gStepCount - gStepBase) % stepsPerFrame) != 0) {
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
#define TRACE_SIGNALS 28
static const char *const gSignalNames[TRACE_SIGNALS] = {
    "gyroADCf[roll]", "gyroADC[roll]", "rcCommand[roll]",
    "pidP[roll]", "pidI[roll]", "pidD[roll]", "pidF[roll]", "pidSum[roll]",
    "gyroADCf[pitch]", "rcCommand[pitch]", "pidSum[pitch]", "pidI[pitch]",
    "pidDeltaUs", "gyroDeltaUs",
    "sampleSum[roll]", "sampleSum[pitch]", "lpf1State", "lpf2State",
    "lpf1K",
    "cfgP[roll]", "cfgI[roll]", "cfgD[roll]", "cfgF[roll]",
    "rpmNotch1Hz", "rpmNotch2Hz", "rpmNotch3Hz", "rpmMotorHz", "cycleMult",
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
    // Scale the comparison tolerance with the loop rate the firmware reports
    // (gyroDeltaUs = the measured gyro period): the shorter the period, the more
    // integrator steps per trace, the larger the residual random walk.
    if (ls.gyroDeltaUs > 0) {
        gTraceToleranceUs = 0.05 * (1000.0 / (double)ls.gyroDeltaUs);
    }
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
    row[23] = ls.rpmNotchHz[0];
    row[24] = ls.rpmNotchHz[1];
    row[25] = ls.rpmNotchHz[2];
    row[26] = ls.rpmMotorHz[0];
    row[27] = ls.cycleTimeMultiplier;
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

// --- pure-tone probe for the RPM filter --------------------------------------
// When set, the roll gyro carries a pure sine at this frequency (in Hz) instead
// of the scenario's rate waveform, and the sticks stay centred. Comparing what
// reaches the PID (gyroADCf) against what feeds the chain (sampleSum, the
// downsample output) with the RPM filter enabled and disabled is a direct
// functional test: the notches must sit on the motor harmonics.
static double gToneHz = 0.0;
static double gToneAmp = 3.0;     // rad/s on the roll axis (~170 dps)
static bool gToneActive = false;  // only during the recorded trace: the arming
                                  // sequence and the warm-up must not see a
                                  // 200 Hz roll rate, or the angle-mode arming
                                  // check never passes
static double gTraceAmpIn = 0.0;  // max |sampleSum[roll]|, second half of a trace
static double gTraceAmpOut = 0.0; // max |gyroADCf[roll]|, second half of a trace


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

// Arm with the stick command (throttle down + yaw right held for the firmware's
// ARM_DELAY_MS). This is the path a config with no usable ARM box falls back to -
// isUsingSticksToArm() is true then, processRcStickPositions() ignores every AUX
// channel, and a channel scan can only fail. A host that arms the same aircraft
// uses the same stick move.
static bool tryArmWithSticks(void)
{
    for (int i = 0; i < 8000; i++) {          // 2 s at 250 us, ARM_DELAY_MS is 1 s
        sitl_local_input_t in;
        sitl_local_output_t out;
        makeInput(0, ARM_THROTTLE, false, &in);
        in.rc_channels[3] = 2000;             // yaw right, throttle stays low
        if (rawStep(&in, &out)) {
            gArmChannel = 3;
            gArmHigh = 2000;
            gArmLow = 1500;
            fprintf(stderr, "[arm] armed with the stick command (throttle low + yaw right)\n");
            return true;
        }
    }
    return false;
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

static bool findArmSwitchQuiet(void)
{
    printStoredArmSwitch();

    // Let the link and the attitude estimate settle first. The firmware latches
    // RX_FAILSAFE for a while after the RC link appears, and the attitude
    // estimate carries whatever body rates the harness injected before - either
    // one blocks arming (ARMING_DISABLED_RX_FAILSAFE / _ANGLE) and makes a probe
    // look like "the firmware refuses to arm".
    sitl_local_disarm();
    runSteps(4000, ARM_THROTTLE, false);

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
    return tryArmWithSticks();
}

// The arming probes run on a quiet, level feed (see gQuietInput); the scenarios
// that follow want their own injected body rates back.
static bool findArmSwitch(void)
{
    gQuietInput = true;
    const bool found = findArmSwitchQuiet();
    gQuietInput = false;
    return found;
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
    in->battery_voltage = gHarnessBatteryVoltage;
    in->battery_current = gHarnessBatteryCurrent;

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

    // Pure-tone probe (RPM filter functional test): replace the roll rate with a
    // single sine, sticks centred, so only the filter chain shapes it.
    if (gToneHz > 0.0 && gToneActive) {
        // Phase it from the *flight controller's* clock, so the tone's frequency
        // is exact in the FC's sample domain whatever dt the host feeds. A
        // jittering dt then shows up purely as the filters running at the wrong
        // rate (they are all designed for one dt).
        const double fcTime = (double)sitl_local_time_us() * 1e-6;
        in->angular_velocity_rpy[0] = gToneAmp * sin(2.0 * M_PI * gToneHz * fcTime);
        in->angular_velocity_rpy[1] = 0.0;
        in->angular_velocity_rpy[2] = 0.0;
    }

    // Arming probes must not inject body rates: the flight controller integrates
    // them into its attitude estimate, and a craft that looks tilted cannot arm
    // (ARMING_DISABLED_ANGLE). A quiet, level feed is what a host uses to arm.
    if (gQuietInput) {
        in->angular_velocity_rpy[0] = 0.0;
        in->angular_velocity_rpy[1] = 0.0;
        in->angular_velocity_rpy[2] = 0.0;
        in->rc_channels[0] = in->rc_channels[1] = in->rc_channels[3] = 1500;
    }

    // Mode-test overrides (crash flip switch / a full roll stick). Applied last
    // so they also survive gQuietInput's centering.
    if (gHarnessModeChannel >= ARM_CHANNEL_FIRST &&
        gHarnessModeChannel < SITL_LOCAL_MAX_RC_CHANNELS) {
        // 1750, not 2000: BF's range check is start-inclusive/end-exclusive, so
        // a 1500..2000 condition does *not* match a 2000 us channel value.
        in->rc_channels[gHarnessModeChannel] = gHarnessModeHigh ? 1750 : 1000;
    }
    if (gHarnessRollOverride >= 0) {
        in->rc_channels[0] = (uint16_t)gHarnessRollOverride;
    }

    // Closed-loop plant: the gyro is the model's own rate, driven by the motor
    // differential the FC produced on the previous step.
    if (gClosedLoop) {
        // QUADX motor layout (Betaflight mixerQuadX): 0 REAR_R, 1 FRONT_R,
        // 2 REAR_L, 3 FRONT_L. A positive roll PID demand raises the left
        // motors, so the resulting roll acceleration is (left - right).
        const float *m = gLastMotorsForPlant;
        const double rollTorque  = (m[2] + m[3]) - (m[0] + m[1]);
        const double pitchTorque = (m[1] + m[2]) - (m[0] + m[3]);
        const double yawTorque   = ((m[0] + m[1] + m[2]) / 3.0 - m[3]) * 0.5;
        const double torque[3] = { rollTorque, pitchTorque, yawTorque };
        for (int axis = 0; axis < 3; axis++) {
            // BF_HARNESS_PLANT_TAU_MS = 0 makes this a pure integrator with no
            // angular damping - which is what an ideal rigid-body simulator (and
            // the Unreal pawn) looks like. The default 25 ms lag is a forgiving
            // plant that hides loop-margin problems.
            const double damp = (gPlantTauS > 0.0) ? gPlantRate[axis] / gPlantTauS : 0.0;
            gPlantRate[axis] += (gLastStepDtUs * 1e-6) * (PLANT_GAIN * torque[axis] - damp);
        }
        in->angular_velocity_rpy[0] = gPlantRate[0];
        in->angular_velocity_rpy[1] = gPlantRate[1];
        in->angular_velocity_rpy[2] = gPlantRate[2];
    }

    // Motor-induced vibration in the gyro data (what a real gyro measures at the
    // motor harmonics). Present whenever the motors are spinning, so the RPM
    // filter has something to remove.
    if (gVibAmp > 0.0) {
        const double fcT = (double)sitl_local_time_us() * 1e-6;
        double vib = 0.0;
        for (int m = 0; m < 4; m++) {
            const double rpm = sitlHarnessMotorRpm(m);
            if (rpm <= 0.0) {
                continue;
            }
            const double f = rpm / 60.0;
            vib += sin(2.0 * M_PI * f * fcT) + 0.5 * sin(2.0 * M_PI * 2.0 * f * fcT);
        }
        in->angular_velocity_rpy[0] += gVibAmp * vib / 4.0;
        in->angular_velocity_rpy[1] += gVibAmp * vib / 4.0 * 0.6;
        in->angular_velocity_rpy[2] += gVibAmp * vib / 4.0 * 0.3;
    }

    // Plausible telemetry so the RPM bridge has something to work on.
    in->motor_rpm[0] = 12000.0;
    in->motor_rpm[1] = 11000.0;
    in->motor_rpm[2] = 10000.0;
    in->motor_rpm[3] = 9000.0;
    // In closed-loop mode the RPM is derived from the motor output the FC just
    // produced, exactly like Unreal's ESC model (RPM = KV * duty) - so the RPM
    // filter's notch frequencies track the PID output and jitter with it, which
    // is what makes the missing RPM lowpass visible.
    if (gRpmFromMotors || gClosedLoop) {
        for (int i = 0; i < 4; i++) {
            in->motor_rpm[i] = sitlHarnessMotorRpm(i);
        }
    }
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
    gClosedMin[0] = gClosedMin[1] = 1e9;
    gClosedMax[0] = gClosedMax[1] = -1e9;
    gToneActive = (gToneHz > 0.0);

    for (int phase = 0; phase < SCENARIO_STEPS; phase++) {
        bool armed = stepWith(phase, TRACE_THROTTLE, true, trace[phase], motorCount);
        captureSignals(phase);
        if (gToneHz > 0.0 && phase >= SCENARIO_STEPS / 2) {
            const double inAmp = fabs((double)gSigTarget[phase][14]);
            const double outAmp = fabs((double)gSigTarget[phase][0]);
            if (inAmp > gTraceAmpIn) { gTraceAmpIn = inAmp; }
            if (outAmp > gTraceAmpOut) { gTraceAmpOut = outAmp; }
        }
        if (gClosedLoop && phase >= SCENARIO_STEPS - 500) {
            if (trace[phase][0] < gClosedMin[0]) { gClosedMin[0] = trace[phase][0]; }
            if (trace[phase][0] > gClosedMax[0]) { gClosedMax[0] = trace[phase][0]; }
            if (gPlantRate[0] < gClosedMin[1]) { gClosedMin[1] = gPlantRate[0]; }
            if (gPlantRate[0] > gClosedMax[1]) { gClosedMax[1] = gPlantRate[0]; }
        }
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
    gToneActive = false;
    gPlantPkPk = gClosedMax[0] - gClosedMin[0];
    gRatePkPk = gClosedMax[1] - gClosedMin[1];
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
    addr.sin_port = htons(harnessMspPort());
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

static void printClosedLoopResult(const char *tag)
{
    fprintf(stderr, "[closed-loop] %-34s residual motor0 pk-pk %8.2f us, "
                    "roll rate pk-pk %7.3f rad/s\n",
            tag, gPlantPkPk, gRatePkPk);
}

// --- RPM filter functional test -------------------------------------------------
// `rpm-tone`: feed a pure sine at the motor fundamental into the roll gyro and
// compare what reaches the PID (gyroADCf) with what feeds the filter chain
// (sampleSum, the LPF2/downsample output) with the RPM filter enabled and
// disabled. If the notch bank sits on the motor harmonics the tone must be
// crushed with the filter on and pass with it off - a direct check of "does the
// RPM filter do anything", independent of the loop's tuning.
static void printToneAmps(const char *tag)
{
    const double attIn = gTraceAmpIn > 0.01 ? 20.0 * log10(gTraceAmpOut / gTraceAmpIn) : 0.0;
    fprintf(stderr, "[rpm-tone] %-34s input(chain) %8.3f counts -> pid(gyroADCf) "
                    "%8.3f counts  (%.1f dB)\n",
            tag, gTraceAmpIn, gTraceAmpOut, attIn);
}

// Flip rpm_filter_harmonics through the configurator's own MSP path (the filter
// config block, byte 43 - see the MSP_FILTER_CONFIG layout in msp.c) so the
// firmware rebuilds the notch bank exactly like the configurator would.
static void setRpmHarmonicsViaMsp(int harmonics, const char *tag)
{
    uint8_t cfg[MSP_MAX_PAYLOAD];
    const int len = mspRequest(MSP_FILTER_CONFIG, NULL, 0, cfg, sizeof(cfg), 2000);
    if (len < 44) {
        fprintf(stderr, "[rpm-tone] could not read the filter config (%d bytes)\n", len);
        return;
    }
    cfg[43] = (uint8_t)harmonics;
    (void)mspRequest(MSP_SET_FILTER_CONFIG, cfg, (uint8_t)len, NULL, 0, 2000);
    sitl_local_loop_state_t ls;
    if (sitl_local_get_loop_state(&ls) == 0) {
        fprintf(stderr, "[rpm-tone] %s: rpm_filter_harmonics=%u (config now reports %u)\n",
                tag, (unsigned)harmonics, (unsigned)ls.rpmHarmonics);
    }
}

// Write the whole gyro/D-term filter block through the configurator's own MSP
// path, so the firmware rebuilds the chains exactly like a configurator save.
// Byte offsets follow MSP_SET_FILTER_CONFIG (msp.c): 0/20-21 gyro_lpf1,
// 22-23 gyro_lpf2, 29-32 gyro dyn LPF1, 1-2 dterm_lpf1, 43 rpm harmonics.
static void setFilterConfigViaMsp(int lpf1, int lpf2, int dynMin, int dynMax,
                                  int dtermLpf, int rpmHarmonics)
{
    uint8_t cfg[MSP_MAX_PAYLOAD];
    const int len = mspRequest(MSP_FILTER_CONFIG, NULL, 0, cfg, sizeof(cfg), 2000);
    if (len < 45) {
        fprintf(stderr, "[vib] could not read the filter config (%d bytes)\n", len);
        return;
    }
    cfg[0] = (uint8_t)lpf1;
    cfg[1] = (uint8_t)(dtermLpf & 0xFF);
    cfg[2] = (uint8_t)((dtermLpf >> 8) & 0xFF);
    cfg[20] = (uint8_t)(lpf1 & 0xFF);
    cfg[21] = (uint8_t)((lpf1 >> 8) & 0xFF);
    cfg[22] = (uint8_t)(lpf2 & 0xFF);
    cfg[23] = (uint8_t)((lpf2 >> 8) & 0xFF);
    cfg[29] = (uint8_t)(dynMin & 0xFF);
    cfg[30] = (uint8_t)((dynMin >> 8) & 0xFF);
    cfg[31] = (uint8_t)(dynMax & 0xFF);
    cfg[32] = (uint8_t)((dynMax >> 8) & 0xFF);
    cfg[43] = (uint8_t)rpmHarmonics;
    (void)mspRequest(MSP_SET_FILTER_CONFIG, cfg, (uint8_t)len, NULL, 0, 2000);
}

// --- step-time histogram (cycle) ---------------------------------------------
// Arm/disarm cycling is what makes the firmware's own file I/O visible from the
// step path: every disarm runs fc/stats.c's writeEEPROM() through the dispatcher
// (which runs inside the step) and stops the blackbox log, and every arm starts
// one - open, directory scan, prune. This records the per-step wall time so the
// *peaks* can be compared, not just the average.
#define CYCLE_MAX_SAMPLES 40000
static double gCycleSamples[CYCLE_MAX_SAMPLES];
static int gCycleCount = 0;

static void cycleStep(uint16_t throttle, bool armHigh)
{
    sitl_local_input_t in;
    sitl_local_output_t out;
    makeInput(0, throttle, armHigh, &in);
    const double t0 = harnessWallSeconds();
    rawStep(&in, &out);
    if (gCycleCount < CYCLE_MAX_SAMPLES) {
        gCycleSamples[gCycleCount++] = (harnessWallSeconds() - t0) * 1e6;
    }
}

static int cmpDouble(const void *a, const void *b)
{
    const double x = *(const double *)a;
    const double y = *(const double *)b;
    return (x < y) ? -1 : (x > y) ? 1 : 0;
}

static void cycleReport(const char *tag)
{
    if (gCycleCount < 10) {
        fprintf(stderr, "[cycle] %s: no samples\n", tag);
        return;
    }
    qsort(gCycleSamples, gCycleCount, sizeof(double), cmpDouble);
    fprintf(stderr, "[cycle] %-26s n=%5d  p50=%6.1f  p99=%7.1f  p99.9=%8.1f  max=%9.1f us\n",
            tag, gCycleCount,
            gCycleSamples[gCycleCount / 2],
            gCycleSamples[(int)(gCycleCount * 0.99)],
            gCycleSamples[(int)(gCycleCount * 0.999)],
            gCycleSamples[gCycleCount - 1]);
    gCycleCount = 0;
}

static void cyclePhases(int cycles, int stepsPerPhase)
{
    for (int c = 0; c < cycles; c++) {
        for (int i = 0; i < stepsPerPhase; i++) {
            cycleStep(ARM_THROTTLE, true);
        }
        for (int i = 0; i < stepsPerPhase; i++) {
            cycleStep(ARM_THROTTLE, false);
        }
    }
}

// Point the blackbox at NONE (0) or back at the virtual file device (4) through
// the configurator's own MSP path, so arm/disarm cycling can be measured with
// and without the flight log.
static bool setBlackboxDeviceViaMsp(int device)
{
    uint8_t cfg[MSP_MAX_PAYLOAD];
    const int len = mspRequest(MSP_BLACKBOX_CONFIG, NULL, 0, cfg, sizeof(cfg), 2000);
    if (len < 2) {
        fprintf(stderr, "[cycle] could not read the blackbox config (%d bytes)\n", len);
        return false;
    }
    cfg[1] = (uint8_t)device;
    (void)mspRequest(MSP_SET_BLACKBOX_CONFIG, cfg, (uint8_t)len, NULL, 0, 2000);
    return true;
}

// --- vibration attenuation probe (vib) ---------------------------------------
// The harness can put motor-harmonic vibration into the gyro data it feeds
// (BF_HARNESS_VIB_RADPS, the same signal a real gyro measures). This probe
// measures what the firmware's filter chain does with it: it holds a steady
// hover, reads the gyro the sensor produced (gyroADC) and the gyro the PID
// sees (gyroADCf) on every step, and fits the amplitude of the motor
// fundamental with a DFT against the flight controller's own virtual clock. The
// harness feeds a fixed 12000 rpm on motor 0, i.e. 200 Hz.
static void runVibrationProbe(const char *tag)
{
    const int steps = 4000;
    const double freqHz = 12000.0 / 60.0;
    double siIn = 0.0, coIn = 0.0, siOut = 0.0, coOut = 0.0;
    double motorMin = 1e9, motorMax = -1e9;
    int samples = 0;

    for (int i = 0; i < steps; i++) {
        sitl_local_input_t in;
        sitl_local_output_t out;
        makeInput(0, TRACE_THROTTLE, true, &in);
        rawStep(&in, &out);

        sitl_local_loop_state_t ls;
        if (sitl_local_get_loop_state(&ls) != 0) {
            continue;
        }
        // Settle first: a filter change rebuilds the chains and the dynamic LPF
        // follows the throttle at the start of the segment.
        if (i < steps / 4) {
            continue;
        }
        const double t = (double)sitl_local_time_us() * 1e-6;
        const double ph = 2.0 * M_PI * freqHz * t;
        const double s = sin(ph);
        const double c = cos(ph);
        siIn += ls.gyroADC[0] * s;
        coIn += ls.gyroADC[0] * c;
        siOut += ls.gyroADCf[0] * s;
        coOut += ls.gyroADCf[0] * c;
        const double m = (double)out.pwm_output_raw[0];
        if (m < motorMin) { motorMin = m; }
        if (m > motorMax) { motorMax = m; }
        samples++;
    }

    if (samples < 100) {
        fprintf(stderr, "[vib] %-34s no samples\n", tag);
        return;
    }
    const double ampIn = 2.0 * sqrt(siIn * siIn + coIn * coIn) / (double)samples;
    const double ampOut = 2.0 * sqrt(siOut * siOut + coOut * coOut) / (double)samples;
    const double attDb = ampIn > 1e-6 ? 20.0 * log10(ampOut / ampIn) : 0.0;
    fprintf(stderr, "[vib] %-34s fed@%.0fHz %7.3f deg/s -> PID sees %7.3f deg/s "
                    "(%6.1f dB)  motor0 pk-pk %6.0f us\n",
            tag, freqHz, ampIn, ampOut, attDb, motorMax - motorMin);
}


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
// Read whatever the FC has already queued back (no waiting), NUL-terminated.
// Returns the number of bytes.
static int readSocketQuiet(char *buf, size_t cap)
{
    int total = 0;
    while ((size_t)total + 1 < cap) {
        fd_set rd;
        FD_ZERO(&rd);
        FD_SET(gSock, &rd);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        if (select(gSock + 1, &rd, NULL, NULL, &tv) <= 0) {
            break;
        }
        const int n = recv(gSock, buf + total, (int)(cap - 1 - (size_t)total), 0);
        if (n <= 0) {
            break;
        }
        total += n;
    }
    buf[total] = '\0';
    return total;
}

// Type one CLI command the way the configurator's CLI tab does and report what
// the FC answered ("Rebooting", or the CLI's own error text).
//
// The '#' only *queues* the CLI request: msp_serial.c acts on it from the MSP
// port state machine ~100 ms after the last byte on the port, and a line typed
// before the CLI is actually up is consumed as port noise and never runs. That
// is why the harness's cli-save mode was "experimental" (the CLI entered, the
// `save` line had already gone past). So: send '#', wait until the firmware
// itself reports the CLI active (ARMING_DISABLED_CLI), and only then type.
static bool cliRunCommand(const char *command, const char *label)
{
    static char log[4096];
    size_t used = 0;
    log[0] = '\0';

    sendRaw("\r\n#");
    bool cliUp = false;
    for (int i = 0; i < 200 && !cliUp; i++) {        // up to 10 s
        // ARMING_DISABLED_CLI is set by the firmware's own arming check, which
        // only runs while the FC is stepped - so step while waiting, exactly
        // like a host that keeps the physics loop running while the user types.
        stepWith(0, 1000, false, NULL, NULL);
        Sleep(50);
        used += (size_t)readSocketQuiet(log + used, sizeof(log) - used);
        cliUp = (sitl_local_get_arming_flags() & 0x00002000u) != 0     // ARMING_DISABLED_CLI
                || strstr(log, "# ") != NULL;                          // the CLI prompt
    }
    if (!cliUp) {
        for (size_t i = 0; i < used; i++) {
            if (log[i] == '\r') {
                log[i] = ' ';
            }
        }
        fprintf(stderr, "[osd] %s: the CLI never became active ('#' ignored); FC said (%.300s)\n",
                label, log);
        return false;
    }

    sendRaw(command);
    // Keep stepping while the CLI runs the command and (for save/exit) the
    // reboot is applied on the next step.
    for (int i = 0; i < 40; i++) {
        stepWith(0, 1000, false, NULL, NULL);
        Sleep(50);
        used += (size_t)readSocketQuiet(log + used, sizeof(log) - used);
    }
    log[used] = '\0';
    for (size_t i = 0; i < used; i++) {
        if (log[i] == '\r') {
            log[i] = ' ';
        }
    }
    fprintf(stderr, "[osd] %s: CLI said (%.600s)\n", label, log);
    return true;
}

static void cliSave(void)
{
    const bool typed = cliRunCommand("save\r\n", "CLI save");
    fprintf(stderr, "[cli] '#' + 'save' sent (writeEEPROM + reboot)%s\n",
            typed ? "" : " - FAILED to enter the CLI");
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

// ---------------------------------------------------------------------------
// mode: osd-reboot - "does the BF logo come back after every reboot?"
//
// A real FC redraws its boot splash after any reboot, so every trigger the
// configurator or the CLI can pull has to leave the OSD state machine back in
// OSD_STATE_INIT. The LOCAL build reproduces reboots in-process
// (localRunPendingReboot()), and a trigger that only persisted the config
// without resetting the OSD would silently keep the previous session's HUD
// frame - invisible in the motor traces the rest of this harness compares.
//
// The check uses the canvas the DLL publishes (sitl_local_get_osd_frame):
// step until the splash is gone, fire one trigger, then require the splash's
// static text to be back on screen.
// ---------------------------------------------------------------------------

// cms.h's CMS_STARTUP_HELP_TEXT1, the splash's centre line. The logo glyphs and
// the version string are the rest of the splash, but no other screen draws this
// text, so it is an unambiguous "the FC has just booted" marker.
#define OSD_SPLASH_MARKER "MENU:THR MID"

static bool osdFrameHasSplash(const sitl_local_osd_frame_t *frame)
{
    const size_t len = strlen(OSD_SPLASH_MARKER);
    if (frame->cols == 0 || frame->rows == 0) {
        return false;
    }
    for (unsigned y = 0; y < frame->rows; y++) {
        for (unsigned x = 0; x + len <= (unsigned)frame->cols; x++) {
            bool match = true;
            for (size_t i = 0; i < len; i++) {
                const unsigned idx = y * SITL_LOCAL_OSD_MAX_COLS + (x + i);
                if (frame->kind[idx] != SITL_LOCAL_OSD_CELL_CHAR ||
                    frame->character[idx] != (uint8_t)OSD_SPLASH_MARKER[i]) {
                    match = false;
                    break;
                }
            }
            if (match) {
                return true;
            }
        }
    }
    return false;
}

static int osdDrawnCells(const sitl_local_osd_frame_t *frame)
{
    int drawn = 0;
    for (unsigned i = 0; i < (unsigned)frame->cols * frame->rows; i++) {
        if (frame->kind[i] != SITL_LOCAL_OSD_CELL_BLANK) {
            drawn++;
        }
    }
    return drawn;
}

// Step (disarmed, throttle low) while sampling the canvas every 20 ms.
// `want` = true: return the ms at which the splash is on screen (-1 = never).
// `want` = false: return the ms at which it is gone (-1 = still there).
static int stepForSplashState(bool want, int maxSteps, int *cellsAtEnd)
{
    for (int i = 0; i < maxSteps; i++) {
        stepWith(0, 1000, false, NULL, NULL);
        if ((i % 20) != 0) {
            continue;
        }
        sitl_local_osd_frame_t frame;
        if (sitl_local_get_osd_frame(&frame) != 0) {
            continue;
        }
        if (cellsAtEnd != NULL) {
            *cellsAtEnd = osdDrawnCells(&frame);
        }
        if (osdFrameHasSplash(&frame) == want) {
            return i;
        }
    }
    return -1;
}

// --- the reboot triggers the configurator / CLI can pull ---------------------

static void triggerSaveAndReboot(void)
{
    saveConfig("save-reboot");                  // MSP_EEPROM_WRITE + MSP_SET_REBOOT
}

static void triggerPlainReboot(void)
{
    mspSend(MSP_API_VERSION, NULL, 0);
    (void)mspWaitReply(MSP_API_VERSION, 1500);
    mspSend(MSP_SET_REBOOT, NULL, 0);
    fprintf(stderr, "[osd] MSP_SET_REBOOT (no save) sent\n");
}

static void triggerEnterBootloader(void)
{
    // msp.c's rebootMode: 1 = MSP_REBOOT_BOOTLOADER_ROM ("Enter bootloader /
    // DFU"). The LOCAL build has no bootloader to enter and treats it as a
    // firmware reboot (systemResetToBootloader() in wincompat.c).
    const uint8_t mode = 1;
    mspSend(MSP_API_VERSION, NULL, 0);
    (void)mspWaitReply(MSP_API_VERSION, 1500);
    mspSend(MSP_SET_REBOOT, &mode, 1);
    fprintf(stderr, "[osd] MSP_SET_REBOOT bootloader(ROM) sent\n");
}

static void triggerCliSave(void)
{
    cliSave();                                  // writeEEPROM + reboot
    // Diagnostic: ARMING_DISABLED_CLI is set exactly while the CLI is active.
    fprintf(stderr, "[osd] after CLI save: armFlags=0x%08X (CLI active: %s)\n",
            sitl_local_get_arming_flags(),
            (sitl_local_get_arming_flags() & 0x00002000u) ? "yes" : "no");
}

static void triggerCliExit(void)
{
    (void)cliRunCommand("exit\r\n", "CLI exit");   // reboot without saving
    fprintf(stderr, "[osd] after CLI exit: armFlags=0x%08X (CLI active: %s)\n",
            sitl_local_get_arming_flags(),
            (sitl_local_get_arming_flags() & 0x00002000u) ? "yes" : "no");
}

static int checkOsdSplashAcrossReboots(void)
{
    static const struct {
        const char *name;
        void (*fire)(void);
    } triggers[] = {
        { "Save and Reboot (config)", triggerSaveAndReboot },
        { "Reboot (config)",          triggerPlainReboot },
        { "Enter bootloader / DFU",   triggerEnterBootloader },
        { "CLI save",                 triggerCliSave },
        { "CLI exit",                 triggerCliExit },
    };

    // BF_HARNESS_OSD_TRIGGER=<substring> runs one trigger only, for bisecting a
    // failure without waiting for the ones before it.
    const char *only = getenv("BF_HARNESS_OSD_TRIGGER");
    if (only != NULL && only[0] == '\0') {
        only = NULL;
    }

    fprintf(stderr, "[osd-reboot] every reboot must end with the boot splash on the OSD\n");

    // This process's own boot drew the splash; wait it out so the state before
    // the first trigger is provably "no splash" (otherwise "it is still there"
    // could be mistaken for "the reboot redrew it").
    if (stepForSplashState(false, 8000, NULL) < 0) {
        fprintf(stderr, "[osd-reboot] the splash of this boot never cleared - cannot test\n");
        return 4;
    }

    int failures = 0;
    for (size_t i = 0; i < sizeof(triggers) / sizeof(triggers[0]); i++) {
        if (only != NULL && strstr(triggers[i].name, only) == NULL) {
            continue;
        }
        // The previous iteration's splash (or this boot's) needs to be gone
        // again: its window is 4 s, so give it 6 s of virtual time.
        if (stepForSplashState(false, 6000, NULL) < 0) {
            fprintf(stderr, "[osd-reboot] FAIL %-26s: splash still up before the trigger\n",
                    triggers[i].name);
            failures++;
        }

        const uint64_t beforeUs = sitl_local_time_us();
        triggers[i].fire();

        int cells = 0;
        const int splashAt = stepForSplashState(true, 4000, &cells);
        if (splashAt < 0) {
            fprintf(stderr, "[osd-reboot] FAIL %-26s: no splash within 4 s "
                            "(canvas %d cells, t=+%llu us)\n",
                    triggers[i].name, cells,
                    (unsigned long long)(sitl_local_time_us() - beforeUs));
            failures++;
        } else {
            fprintf(stderr, "[osd-reboot] PASS %-26s: splash back %d ms after the trigger\n",
                    triggers[i].name, splashAt);
        }

        // A reboot can drop the MSP link (a real configurator reconnects the
        // same way); reconnect so the next trigger has a socket to send on.
        if (gSock >= 0) {
            closesocket(gSock);
            gSock = -1;
        }
        if (mspConnect() != 0) {
            fprintf(stderr, "[osd-reboot] MSP reconnect failed before the next trigger\n");
            return 5;
        }
    }

    fprintf(stderr, "[osd-reboot] RESULT: %s (%d of %d triggers failed)\n",
            failures == 0 ? "PASS" : "FAIL", failures,
            (int)(sizeof(triggers) / sizeof(triggers[0])));
    return failures == 0 ? 0 : 1;
}

// --- comparison --------------------------------------------------------------

// ---------------------------------------------------------------------------
// mode: battery - does the virtual battery exist from boot, armed or not?
//
// Two things have to hold for the SITL's "battery is always connected" model:
//
//   1. the virtual ADC meters are selected *before* batteryInit()/tasksInit()
//      (initPhase3), otherwise TASK_BATTERY_VOLTAGE/CURRENT/ALERTS are never
//      enabled and voltage/current/mAh stay 0 for the whole session;
//   2. batteryCellCount gets established even when the host arms immediately,
//      because stock firmware only runs batteryUpdatePresence() while disarmed
//      (TASK_BATTERY_ALERTS) - the SITL keeps probing from the 50 Hz voltage
//      task until the count exists.
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t cells;
    uint16_t capacity;
    uint8_t legacyVolts;
    uint16_t mah;
    int16_t amps;
    uint8_t state;
    uint16_t volts;
} batteryState_t;

static bool readBatteryState(batteryState_t *out)
{
    uint8_t reply[32];
    const int len = mspRequest(MSP_BATTERY_STATE, NULL, 0, reply, sizeof(reply), 3000);
    if (len < 11) {
        fprintf(stderr, "[battery] MSP_BATTERY_STATE failed (len %d)\n", len);
        return false;
    }
    out->cells = reply[0];
    out->capacity = (uint16_t)(reply[1] | (reply[2] << 8));
    out->legacyVolts = reply[3];
    out->mah = (uint16_t)(reply[4] | (reply[5] << 8));
    out->amps = (int16_t)(reply[6] | (reply[7] << 8));
    out->state = reply[8];
    out->volts = (uint16_t)(reply[9] | (reply[10] << 8));
    return true;
}

// Reproduce an aircraft whose *saved* configuration had both meters switched
// off (the configurator's Battery tab allows VOLTAGE_METER_NONE). tasksInit()
// derives TASK_BATTERY_* from these sources, so they have to be ADC *before*
// initPhase3 runs - pinning them after the boot (localApplyLinkOverrides) only
// changes what MSP reports, not the tasks that were already disabled.
static bool forceStoredMeterSourcesNone(void)
{
    uint8_t reply[32];
    const int len = mspRequest(MSP_BATTERY_CONFIG, NULL, 0, reply, sizeof(reply), 3000);
    if (len < 7) {
        fprintf(stderr, "[battery] MSP_BATTERY_CONFIG failed (len %d)\n", len);
        return false;
    }
    uint8_t payload[7] = {
        reply[0], reply[1], reply[2],            // vbat min/max/warn (0.1 V)
        reply[3], reply[4],                      // capacity (u16, little endian)
        0,                                       // VOLTAGE_METER_NONE
        0,                                       // CURRENT_METER_NONE
    };
    if (mspRequest(MSP_SET_BATTERY_CONFIG, payload, sizeof(payload), NULL, 0, 3000) < 0) {
        fprintf(stderr, "[battery] MSP_SET_BATTERY_CONFIG failed\n");
        return false;
    }
    fprintf(stderr, "[battery] stored battery meters switched to NONE (voltage/current)\n");
    return true;
}

// ---------------------------------------------------------------------------
// mode: modes - the generic mode-range and 3D exports the game uses to bind
// switches (ANGLE/HORIZON/... by permanent ID) without MSP traffic.
// ---------------------------------------------------------------------------
// mode: crashflip. Verifies the FC-side half of "does crash flip push one side":
// the latch (switch on before arming), the differential motor output on a full
// roll stick, and the two ways to clear it.
static int checkCrashFlip(void)
{
    if (sitl_local_set_arm_switch(4, 24, 44) != 0 ||
        sitl_local_set_mode_range(35, 5, 24, 44) != 0) {
        fprintf(stderr, "[crashflip] FAIL: could not bind ARM / CRASH FLIP\n");
        return 1;
    }
    gArmChannel = ARM_CHANNEL_FIRST + 4;
    gArmHigh = 1750;
    gArmLow = 1000;
    gHarnessModeChannel = ARM_CHANNEL_FIRST + 5;
    gHarnessModeHigh = true;
    gQuietInput = true;

    {
        uint8_t a = 0, s = 0, e = 0;
        sitl_local_get_mode_range(35, &a, &s, &e);
        fprintf(stderr, "[crashflip] bound range: aux=%u steps %u..%u (rc index %d)\n",
                (unsigned)a, (unsigned)s, (unsigned)e, gHarnessModeChannel);
    }

    runSteps(2500, 1000, false);
    {
        uint8_t rcReply[64];
        const int len = mspRequest(105 /* MSP_RC */, NULL, 0, rcReply, sizeof(rcReply), 3000);
        fprintf(stderr, "[crashflip] MSP_RC len=%d ch8..ch10:", len);
        for (int ch = 8; ch <= 10; ch++) {
            if (2 * ch + 1 < len) {
                fprintf(stderr, " %d", rcReply[2 * ch] | (rcReply[2 * ch + 1] << 8));
            }
        }
        fprintf(stderr, "\n");
    }
    fprintf(stderr, "[crashflip] box active (disarmed)=%d latch=%d\n",
            sitl_local_get_mode_active(35), sitl_local_is_crash_flip_active());
    if (sitl_local_get_mode_active(35) != 1) {
        fprintf(stderr, "[crashflip] FAIL: the box is not active with AUX2 high\n");
        return 1;
    }
    if (sitl_local_is_crash_flip_active() != 0) {
        fprintf(stderr, "[crashflip] FAIL: latched before arming\n");
        return 1;
    }

    const bool armed = runSteps(2500, 1000, true);
    fprintf(stderr, "[crashflip] armed=%d latch=%d\n",
            armed ? 1 : 0, sitl_local_is_crash_flip_active());
    if (!armed) {
        fprintf(stderr, "[crashflip] INCONCLUSIVE: did not arm\n");
        printArmingFlags(sitl_local_get_arming_flags());
        return 2;
    }
    if (sitl_local_is_crash_flip_active() != 1) {
        fprintf(stderr, "[crashflip] FAIL: not latched at arming (box was on)\n");
        return 1;
    }

    gHarnessRollOverride = 2000;
    float motors[4] = {0, 0, 0, 0};
    for (int i = 0; i < 200; i++) {
        stepWith(i % SCENARIO_STEPS, 1000, true, motors, NULL);
    }
    fprintf(stderr, "[crashflip] roll full -> FC PWM %.0f %.0f %.0f %.0f\n",
            (double)motors[0], (double)motors[1], (double)motors[2], (double)motors[3]);
    float maxPwm = motors[0], minPwm = motors[0];
    for (int i = 1; i < 4; i++) {
        if (motors[i] > maxPwm) maxPwm = motors[i];
        if (motors[i] < minPwm) minPwm = motors[i];
    }
    if (maxPwm - minPwm < 100.0f) {
        fprintf(stderr, "[crashflip] FAIL: no differential output (spread %.0f us)\n",
                (double)(maxPwm - minPwm));
        return 1;
    }

    gHarnessRollOverride = -1;
    gHarnessModeHigh = false;
    runSteps(300, 1000, true);
    fprintf(stderr, "[crashflip] switch off -> latch=%d armed=%d\n",
            sitl_local_is_crash_flip_active(), sitl_local_get_armed());
    if (sitl_local_is_crash_flip_active() != 0) {
        fprintf(stderr, "[crashflip] FAIL: switch off did not clear the latch\n");
        return 1;
    }

    runSteps(400, 1000, false);
    if (!runSteps(2500, 1000, true)) {
        fprintf(stderr, "[crashflip] INCONCLUSIVE: could not re-arm\n");
        return 2;
    }
    gHarnessModeHigh = true;
    runSteps(300, 1000, true);
    fprintf(stderr, "[crashflip] armed then switch on -> latch=%d\n",
            sitl_local_is_crash_flip_active());
    if (sitl_local_is_crash_flip_active() != 0) {
        fprintf(stderr, "[crashflip] FAIL: latched without the arm-edge order\n");
        return 1;
    }

    gHarnessModeHigh = false;
    gQuietInput = false;
    runSteps(300, 1000, false);
    fprintf(stderr, "[crashflip] RESULT: PASS\n");
    return 0;
}

static int checkModeRanges(void)
{
    // ANGLE = permanent ID 1. Bind AUX1 (auxChannel 4) to 1500..2000 us.
    if (sitl_local_set_mode_range(1, 4, 24, 44) != 0) {
        fprintf(stderr, "[modes] FAIL: set ANGLE range failed\n");
        return 1;
    }
    uint8_t aux = 0, start = 0, end = 0;
    sitl_local_get_mode_range(1, &aux, &start, &end);
    fprintf(stderr, "[modes] ANGLE range: aux=%u steps %u..%u\n",
            (unsigned)aux, (unsigned)start, (unsigned)end);
    if (aux != 4 || start != 24 || end != 44) {
        fprintf(stderr, "[modes] FAIL: range read back as %u/%u/%u\n",
                (unsigned)aux, (unsigned)start, (unsigned)end);
        return 1;
    }

    gArmChannel = ARM_CHANNEL_FIRST + 4;
    gArmHigh = 1750;
    gArmLow = 1000;
    runSteps(600, 1000, false);
    const int lowActive = sitl_local_get_mode_active(1);
    runSteps(600, 1000, true);
    const int highActive = sitl_local_get_mode_active(1);
    fprintf(stderr, "[modes] ANGLE active: low=%d high=%d\n", lowActive, highActive);
    if (lowActive != 0 || highActive != 1) {
        fprintf(stderr, "[modes] FAIL: mode activity does not follow the switch\n");
        return 1;
    }

    // FLIP OVER AFTER CRASH (35) is only in the active box list because
    // initActiveBoxIds() reports DShot while it runs; the firmware's real motor
    // protocol must be back to PWM afterwards (the matrix's motor traces cover
    // that half - a leaked DShot protocol leaves the virtual PWM backend dead).
    uint8_t ids[128];
    const int idsLen = mspRequest(119 /* MSP_BOXIDS */, NULL, 0, ids, sizeof(ids), 3000);
    bool crashFlipListed = false;
    for (int i = 0; i < idsLen; i++) {
        if (ids[i] == 35) {
            crashFlipListed = true;
        }
    }
    fprintf(stderr, "[modes] MSP_BOXIDS: %d boxes, FLIP OVER AFTER CRASH listed=%d\n",
            idsLen, crashFlipListed ? 1 : 0);
    if (idsLen <= 0 || !crashFlipListed) {
        fprintf(stderr, "[modes] FAIL: BOXCRASHFLIP is not in the active box list\n");
        return 1;
    }
    if (sitl_local_set_mode_range(35, 4, 24, 44) != 0) {
        fprintf(stderr, "[modes] FAIL: could not bind FLIP OVER AFTER CRASH\n");
        return 1;
    }
    uint8_t cfAux = 0, cfStart = 0, cfEnd = 0;
    sitl_local_get_mode_range(35, &cfAux, &cfStart, &cfEnd);
    if (cfAux != 4 || cfStart != 24 || cfEnd != 44) {
        fprintf(stderr, "[modes] FAIL: FLIP OVER AFTER CRASH range read back as %u/%u/%u\n",
                (unsigned)cfAux, (unsigned)cfStart, (unsigned)cfEnd);
        return 1;
    }
    runSteps(400, 1000, false);
    const int cfLow = sitl_local_get_mode_active(35);
    runSteps(400, 1000, true);
    const int cfHigh = sitl_local_get_mode_active(35);
    fprintf(stderr, "[modes] FLIP OVER AFTER CRASH active: low=%d high=%d\n", cfLow, cfHigh);
    if (cfLow != 0 || cfHigh != 1) {
        fprintf(stderr, "[modes] FAIL: FLIP OVER AFTER CRASH does not follow its switch\n");
        return 1;
    }

    // FEATURE_3D toggle (also re-derives the active boxes + mixer profile).
    sitl_local_disarm();
    const int before = sitl_local_get_3d_enabled();
    if (sitl_local_set_3d_enabled(1) != 0 || sitl_local_get_3d_enabled() != 1) {
        fprintf(stderr, "[modes] FAIL: could not enable 3D (was %d)\n", before);
        return 1;
    }
    if (sitl_local_set_3d_enabled(0) != 0 || sitl_local_get_3d_enabled() != 0) {
        fprintf(stderr, "[modes] FAIL: could not disable 3D\n");
        return 1;
    }
    fprintf(stderr, "[modes] 3D feature toggled on and back off\n");

    // Invalid inputs are rejected, and an unknown permanent ID clears to 0xFF.
    if (sitl_local_set_mode_range(255, 4, 24, 44) != -1 ||
        sitl_local_set_mode_range(1, 4, 44, 24) != -1) {
        fprintf(stderr, "[modes] FAIL: invalid ranges were accepted\n");
        return 1;
    }
    uint8_t unknownAux = 0, unknownStart = 1, unknownEnd = 1;
    sitl_local_get_mode_range(255, &unknownAux, &unknownStart, &unknownEnd);
    if (unknownAux != 0xFF || unknownStart != 0 || unknownEnd != 0) {
        fprintf(stderr, "[modes] FAIL: unknown box did not read as 0xFF/0/0\n");
        return 1;
    }

    fprintf(stderr, "[modes] RESULT: PASS\n");
    return 0;
}

static int checkBatteryMeters(void)
{
    batteryState_t st;

    // 16.8 V / 12 A from the simulator: 4S, 4.2 V/cell.
    gHarnessBatteryCurrent = 12.0;

    // Bind BOXARM ourselves instead of scanning (findArmSwitch() would run four
    // seconds of disarmed steps and establish the cell count before the test
    // starts). AUX1, 1500..2000 us: mid = 1750 on, 1000 off.
    if (sitl_local_set_arm_switch(4, 24, 44) != 0) {
        fprintf(stderr, "[battery] FAIL: could not bind the ARM switch\n");
        return 1;
    }
    gArmChannel = ARM_CHANNEL_FIRST + 4;
    gArmHigh = 1750;
    gArmLow = 1000;

    // An aircraft whose saved config has the meters off is the case the ordering
    // fix exists for. Persist that, reboot, and let initPhase3 derive the tasks.
    if (!forceStoredMeterSourcesNone()) {
        return 1;
    }
    saveConfig("save-only");
    sitl_local_shutdown();
    if (gSock >= 0) {
        closesocket(gSock);
        gSock = -1;
    }
    if (sitl_local_init() != 0 || mspConnect() != 0) {
        fprintf(stderr, "[battery] FAIL: could not restart the FC\n");
        return 1;
    }

    // --- booted disarmed: the boot-time source pinning must have enabled
    //     TASK_BATTERY_VOLTAGE / CURRENT / ALERTS -------------------------------
    runSteps(2500, 1000, false);
    if (!readBatteryState(&st)) {
        return 1;
    }
    fprintf(stderr, "[battery] disarmed boot: cells=%u volts=%.2fV amps=%.2fA mAh=%u state=%u\n",
            st.cells, st.volts * 0.01, st.amps * 0.01, st.mah, st.state);
    if (st.cells != 4) {
        fprintf(stderr, "[battery] FAIL: expected 4 cells for 16.8 V, got %u "
                        "(battery presence never established)\n", st.cells);
        return 1;
    }
    if (st.volts < 1650 || st.volts > 1700) {
        fprintf(stderr, "[battery] FAIL: voltage %.2f V is not the injected 16.8 V "
                        "(TASK_BATTERY_VOLTAGE not running?)\n", st.volts * 0.01);
        return 1;
    }
    if (st.amps < 1100 || st.amps > 1300) {
        fprintf(stderr, "[battery] FAIL: current %.2f A is not the injected 12 A "
                        "(TASK_BATTERY_CURRENT not running?)\n", st.amps * 0.01);
        return 1;
    }
    if (st.mah == 0) {
        fprintf(stderr, "[battery] FAIL: mAh stayed 0\n");
        return 1;
    }

    // --- armed before the pack is "ready" -----------------------------------
    // Stock firmware only calls batteryUpdatePresence() while disarmed, so a
    // host that arms before the voltage filter reports a stable pack used to end
    // up with batteryCellCount 0 (and 0 in every cell-voltage consumer) until it
    // disarmed and armed again. The SITL battery is always connected, so the
    // 50 Hz voltage task keeps probing until the count exists.
    //
    // Reproduce it without fighting the ~4 s RX_FAILSAFE latch: fresh boot, feed
    // a pack below the "present" threshold (8 V for a 4S auto-detect), wait out
    // the latch disarmed, then arm and raise it to 16.8 V. Only the armed probe
    // can establish the count from here.
    sitl_local_shutdown();
    if (gSock >= 0) {
        closesocket(gSock);
        gSock = -1;
    }
    if (sitl_local_init() != 0 || mspConnect() != 0) {
        fprintf(stderr, "[battery] FAIL: could not restart the FC\n");
        return 1;
    }
    // Keep the pack *unstable* through the RX latch (~1 s) by jittering it, so
    // the disarmed presence probe cannot lock a count in. Then arm and hold
    // 16.8 V: only the armed probe can establish the count from there.
    for (int t = 0; t < 20; t++) {
        gHarnessBatteryVoltage = (t & 1) ? 17.6 : 16.0;
        runSteps(50, 1000, false);
    }
    gHarnessBatteryVoltage = 16.8;
    if (!readBatteryState(&st)) {
        return 1;
    }
    if (st.cells != 0) {
        fprintf(stderr, "[battery] INCONCLUSIVE: the cell count was already established "
                        "1 s after boot (cells=%u)\n", st.cells);
        return 2;
    }
    gQuietInput = true;                     // arming needs a level, centred feed
    const bool armed = runSteps(2500, 1000, true);
    gQuietInput = false;
    if (!armed) {
        fprintf(stderr, "[battery] INCONCLUSIVE: the FC did not arm with AUX1 high\n");
        printArmingFlags(sitl_local_get_arming_flags());
        return 2;
    }
    // Let the voltage filter settle so the count is the final 4S, not a transient.
    runSteps(1500, 1000, true);
    if (!readBatteryState(&st)) {
        return 1;
    }
    fprintf(stderr, "[battery] armed before a stable pack: cells=%u volts=%.2fV\n",
            st.cells, st.volts * 0.01);
    // The count only has to be *established* while armed. (The count itself is
    // autoDetectCellCount()'s call on the filtered voltage; this test steps the
    // pack from 8 V to 16.8 V, so a mid-settle count of 2-3 is BF's own
    // behaviour - with a pack that is at 16.8 V from boot it settles on 4.)
    if (st.cells == 0) {
        fprintf(stderr, "[battery] FAIL: the cell count was never established while armed "
                        "(disarm/rearm used to be the workaround)\n");
        return 1;
    }

    fprintf(stderr, "[battery] RESULT: PASS - boot enables the meters and the cell "
                    "count is established armed or not\n");
    return 0;
}

// ---------------------------------------------------------------------------
// mode: osd-font - what the configurator flashes must be what the host reads
//
// Flashing a font in the Configurator's OSD tab walks all 256 characters with
// MSP_OSD_CHAR_WRITE (payload = 1-byte character address + the 54 visible
// bytes - see bf-configurator's src/js/utils/osdFont.js: FONT.msp.encode() and
// FONT.upload()). This replays exactly that sequence and reads the font back
// through sitl_local_get_osd_font(), so the host API cannot silently drift from
// the path the configurator uses.
// ---------------------------------------------------------------------------

// Wait for the reply to `cmd` while keeping the FC stepping. Accepts both
// `$M>` (ok) and `$M!` (the firmware's MSP_RESULT_ERROR reply - see
// msp_serial.c's mspSerialEncode, which flips the direction byte on an error);
// `*errorOut` reports which one arrived, so a failed request cannot be mistaken
// for a lost one.
//
// Needed for MSP_OSD_CHAR_WRITE: the firmware's displayport driver does
// delay(80) after each character and SITL's delay() spins on the *virtual*
// clock, which only advances inside sitl_local_step(). A host that waits
// without stepping would deadlock the MSP thread (the real host steps at its
// physics rate, so it never sees this).
static bool mspWaitReplySteppingEx(uint8_t cmd, int timeoutMs, bool *errorOut)
{
    static uint8_t buf[1024];
    const DWORD start = GetTickCount();
    size_t used = 0;
    while ((int)(GetTickCount() - start) < timeoutMs) {
        stepWith(0, 1000, false, NULL, NULL);

        fd_set rd;
        FD_ZERO(&rd);
        FD_SET(gSock, &rd);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        if (select(gSock + 1, &rd, NULL, NULL, &tv) > 0 && used + 8 < sizeof(buf)) {
            const int n = recv(gSock, (char *)buf + used, (int)(sizeof(buf) - used), 0);
            if (n > 0) {
                used += (size_t)n;
            }
        }

        size_t i = 0;
        while (i + 6 <= used) {
            if (buf[i] != 0x24 || buf[i + 1] != 0x4d ||
                (buf[i + 2] != 0x3e && buf[i + 2] != 0x21)) {
                i++;
                continue;
            }
            const bool isError = (buf[i + 2] == 0x21);
            const size_t total = i + 6 + buf[i + 3];
            if (total > used) {
                break;
            }
            const uint8_t repliedCmd = buf[i + 4];
            memmove(buf, buf + total, used - total);
            used -= total;
            i = 0;
            if (repliedCmd == cmd) {
                if (errorOut != NULL) {
                    *errorOut = isError;
                }
                return true;
            }
        }
    }
    return false;
}

static bool mspWaitReplyStepping(uint8_t cmd, int timeoutMs)
{
    bool error = false;
    return mspWaitReplySteppingEx(cmd, timeoutMs, &error) && !error;
}

static bool sendFontCharacter(unsigned address, const char *bytes)
{
    uint8_t payload[1 + SITL_LOCAL_OSD_FONT_CHAR_BYTES];
    payload[0] = (uint8_t)address;
    memcpy(payload + 1, bytes, SITL_LOCAL_OSD_FONT_CHAR_BYTES);
    mspSend(MSP_OSD_CHAR_WRITE, payload, sizeof(payload));
    // The firmware answers each write, but it also delay(80)s per character in
    // displayport_msp.c - and delay() spins on the *virtual* clock, which only
    // advances inside sitl_local_step(). So the reply has to be waited for
    // while stepping (a real host steps at its physics rate and never notices);
    // ~80 ms of simulated time per character, i.e. ~20 s for a full font.
    bool error = false;
    if (!mspWaitReplySteppingEx(MSP_OSD_CHAR_WRITE, 10000, &error)) {
        fprintf(stderr, "[osd-font] character %u: no reply to MSP_OSD_CHAR_WRITE\n", address);
        return false;
    }
    if (error) {
        // This is what the configurator turns into "font upload failed": the
        // firmware refused the write (MSP_RESULT_ERROR).
        fprintf(stderr, "[osd-font] character %u: the FC answered MSP_RESULT_ERROR\n", address);
        return false;
    }
    return true;
}

// Deterministic per-character pattern, different for every seed, so neither a
// zeroed buffer nor a "one character for all" mix-up can pass a comparison.
static void fillFontPattern(char *out, unsigned seed)
{
    for (unsigned i = 0; i < SITL_LOCAL_OSD_FONT_CHAR_COUNT; i++) {
        for (unsigned j = 0; j < SITL_LOCAL_OSD_FONT_CHAR_BYTES; j++) {
            out[i * SITL_LOCAL_OSD_FONT_CHAR_BYTES + j] =
                (char)(1 + ((i * 7u + j * 13u + seed * 31u) % 254u));
        }
    }
}

// ---------------------------------------------------------------------------
// Font on disk: <EEPROM directory>\osd_font.bin
// ---------------------------------------------------------------------------

// Where this EEPROM's sidecar font lives (same rule as the DLL: the EEPROM's
// directory + osd_font.bin).
static void harnessFontSidecar(const char *eepromPath, char *out, size_t size)
{
    const char *slash = NULL;
    out[0] = '\0';
    if (eepromPath == NULL || eepromPath[0] == '\0' || size < 32) {
        return;
    }
    for (const char *p = eepromPath; *p != '\0'; p++) {
        if (*p == '\\' || *p == '/') {
            slash = p;
        }
    }
    if (slash == NULL) {
        return;
    }
    const size_t dirLen = (size_t)(slash - eepromPath) + 1;
    if (dirLen + 16 > size) {
        return;
    }
    memcpy(out, eepromPath, dirLen);
    strcpy(out + dirLen, "osd_font.bin");
}

static bool harnessFileExists(const char *path)
{
    const DWORD attrs = GetFileAttributesA(path);
    return attrs != INVALID_FILE_ATTRIBUTES && !(attrs & FILE_ATTRIBUTE_DIRECTORY);
}

static bool harnessFileMatches(const char *path, const void *data, size_t len)
{
    FILE *fp = fopen(path, "rb");
    static char buf[SITL_LOCAL_OSD_FONT_BYTES];
    if (fp == NULL || len > sizeof(buf)) {
        if (fp != NULL) fclose(fp);
        return false;
    }
    const size_t got = fread(buf, 1, sizeof(buf), fp);
    fclose(fp);
    return got == len && memcmp(buf, data, len) == 0;
}

static bool harnessCopyFile(const char *src, const char *dst)
{
    FILE *in = fopen(src, "rb");
    FILE *out = (in != NULL) ? fopen(dst, "wb") : NULL;
    if (in == NULL || out == NULL) {
        if (in != NULL) fclose(in);
        if (out != NULL) fclose(out);
        return false;
    }
    char buf[4096];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), in)) > 0) {
        fwrite(buf, 1, n, out);
    }
    fclose(in);
    fclose(out);
    return true;
}

// Flash a whole font (256 characters, exactly like the configurator) and give
// the host-side getter a chance to run (it is what writes the sidecar file).
static bool uploadWholeFontAndSettle(const char *pattern)
{
    for (unsigned i = 0; i < SITL_LOCAL_OSD_FONT_CHAR_COUNT; i++) {
        if (!sendFontCharacter(i, &pattern[i * SITL_LOCAL_OSD_FONT_CHAR_BYTES])) {
            return false;
        }
    }
    runSteps(50, 1000, false);
    (void)sitl_local_get_osd_font_revision();   // triggers the pending save
    runSteps(20, 1000, false);
    return true;
}

// Point the FC at another aircraft (its own eeprom.bin) and let the queued
// path switch apply.
static void switchAircraftEeprom(const char *eepromPath)
{
    (void)sitl_local_set_eeprom_path(eepromPath);
    runSteps(50, 1000, false);
}

static int checkOsdFontPersistence(void)
{
    static char fontA[SITL_LOCAL_OSD_FONT_BYTES];
    static char fontB[SITL_LOCAL_OSD_FONT_BYTES];
    static char readback[SITL_LOCAL_OSD_FONT_BYTES];
    char baseDir[MAX_PATH];
    char dirA[MAX_PATH];
    char dirB[MAX_PATH];
    char eepromA[MAX_PATH];
    char eepromB[MAX_PATH];
    char fontFileA[MAX_PATH];
    char fontFileB[MAX_PATH];
    const char *tempDir = getenv("TEMP");
    if (tempDir == NULL || tempDir[0] == '\0') {
        tempDir = ".";
    }

    fillFontPattern(fontA, 1);
    fillFontPattern(fontB, 2);

    // Two "aircraft" directories, each with its own eeprom.bin, so the font has
    // to follow the EEPROM path - which is what sitl_local_set_eeprom_path() is
    // for in the game (Content/Drone/<craft>/eeprom.bin).
    snprintf(baseDir, sizeof(baseDir), "%s\\sitl-font-test", tempDir);
    snprintf(dirA, sizeof(dirA), "%s\\a", baseDir);
    snprintf(dirB, sizeof(dirB), "%s\\b", baseDir);
    snprintf(eepromA, sizeof(eepromA), "%s\\eeprom.bin", dirA);
    snprintf(eepromB, sizeof(eepromB), "%s\\eeprom.bin", dirB);
    snprintf(fontFileA, sizeof(fontFileA), "%s\\osd_font.bin", dirA);
    snprintf(fontFileB, sizeof(fontFileB), "%s\\osd_font.bin", dirB);
    CreateDirectoryA(baseDir, NULL);
    CreateDirectoryA(dirA, NULL);
    CreateDirectoryA(dirB, NULL);
    if (gHarnessEepromCopy[0] == '\0' ||
        !harnessCopyFile(gHarnessEepromCopy, eepromA) ||
        !harnessCopyFile(gHarnessEepromCopy, eepromB)) {
        fprintf(stderr, "[osd-font] WARN: no EEPROM copy available, "
                        "skipping the persistence check\n");
        return 0;
    }
    DeleteFileA(fontFileA);
    DeleteFileA(fontFileB);

    fprintf(stderr, "[osd-font] persistence: %s\n", fontFileA);

    // --- aircraft A: nothing stored yet -------------------------------------
    switchAircraftEeprom(eepromA);
    if (sitl_local_get_osd_font_revision() != 0) {
        fprintf(stderr, "[osd-font] FAIL: aircraft A already has a font\n");
        return 1;
    }

    // --- flash A: the host-side getter must persist it ----------------------
    if (!uploadWholeFontAndSettle(fontA)) {
        fprintf(stderr, "[osd-font] FAIL: could not flash font A\n");
        return 1;
    }
    if (!harnessFileExists(fontFileA)) {
        fprintf(stderr, "[osd-font] FAIL: %s was not written after the upload\n", fontFileA);
        return 1;
    }
    if (!harnessFileMatches(fontFileA, fontA, sizeof(fontA))) {
        fprintf(stderr, "[osd-font] FAIL: %s does not match the flashed font\n", fontFileA);
        return 1;
    }
    fprintf(stderr, "[osd-font] aircraft A: font saved (%u bytes), matches the upload\n",
            (unsigned)sizeof(fontA));

    // --- switch to aircraft B: A's font must not leak over ------------------
    switchAircraftEeprom(eepromB);
    if (harnessFileExists(fontFileB)) {
        fprintf(stderr, "[osd-font] FAIL: aircraft B already has %s\n", fontFileB);
        return 1;
    }
    if (sitl_local_get_osd_font_revision() != 0 ||
        sitl_local_get_osd_font(readback, sizeof(readback)) != 0) {
        fprintf(stderr, "[osd-font] FAIL: aircraft A's font is still live after switching "
                        "to an aircraft without one\n");
        return 1;
    }
    fprintf(stderr, "[osd-font] aircraft B: no font of its own (A's font dropped)\n");

    // --- flash B, then go back to A: A's font has to come off the disk ------
    if (!uploadWholeFontAndSettle(fontB)) {
        fprintf(stderr, "[osd-font] FAIL: could not flash font B\n");
        return 1;
    }
    if (!harnessFileMatches(fontFileB, fontB, sizeof(fontB))) {
        fprintf(stderr, "[osd-font] FAIL: %s does not match the flashed font\n", fontFileB);
        return 1;
    }
    if (!harnessFileMatches(fontFileA, fontA, sizeof(fontA))) {
        fprintf(stderr, "[osd-font] FAIL: aircraft A's stored font was overwritten\n");
        return 1;
    }

    switchAircraftEeprom(eepromA);
    const uint32_t revisionA = sitl_local_get_osd_font_revision();
    if (revisionA == 0) {
        fprintf(stderr, "[osd-font] FAIL: aircraft A's stored font was not loaded back\n");
        return 1;
    }
    if (sitl_local_get_osd_font(readback, sizeof(readback)) != (int)sizeof(readback) ||
        memcmp(readback, fontA, sizeof(fontA)) != 0) {
        fprintf(stderr, "[osd-font] FAIL: the font loaded for aircraft A is not the "
                        "stored one\n");
        return 1;
    }
    fprintf(stderr, "[osd-font] aircraft A: stored font loaded back (revision %u)\n", revisionA);

    // --- give the harness its own EEPROM back -------------------------------
    switchAircraftEeprom(gHarnessEepromCopy);
    return 0;
}

static int checkOsdFontCapture(void)
{
    static char expected[SITL_LOCAL_OSD_FONT_BYTES];
    static char readback[SITL_LOCAL_OSD_FONT_BYTES];
    fillFontPattern(expected, 0);

    // The OSD (and with it the MSP displayport) has to be running before a
    // font write can reach the driver.
    runSteps(500, 1000, false);
    sitl_local_osd_diag_t diag;
    memset(&diag, 0, sizeof(diag));
    sitl_local_get_osd_diag(&diag);
    fprintf(stderr, "[osd-font] OSD: feature=%u dpReady=%u wrapped=%u device=%u (3 = MSP)\n",
            diag.osdFeatureEnabled, diag.displayPortReady, diag.wrapped, diag.displayPortDevice);
    if (diag.displayPortDevice != 3) {
        fprintf(stderr, "[osd-font] SKIP: the OSD is not on the MSP displayport, "
                        "an SD OSD cannot capture a font\n");
        return 0;
    }

    // A font persisted by an earlier run of this harness would make the FC boot
    // with one already (that is the point of the sidecar file), so start from a
    // clean slate - but only ever in the harness's own EEPROM directory.
    if (gHarnessEepromCopy[0] != '\0') {
        char sidecar[MAX_PATH];
        harnessFontSidecar(gHarnessEepromCopy, sidecar, sizeof(sidecar));
        if (sidecar[0] != '\0') {
            DeleteFileA(sidecar);
        }
    }

    if (sitl_local_get_osd_font_revision() != 0 ||
        sitl_local_get_osd_font(readback, sizeof(readback)) != 0) {
        fprintf(stderr, "[osd-font] FAIL: a font was already available before the upload\n");
        return 1;
    }
    fprintf(stderr, "[osd-font] before the upload: no font (revision 0, 0 bytes)\n");

    // Sanity probe: the MSP link itself must be answering before an empty reply
    // can be blamed on the font command.
    mspSend(MSP_API_VERSION, NULL, 0);
    fprintf(stderr, "[osd-font] probe: MSP_API_VERSION reply %s\n",
            mspWaitReplyStepping(MSP_API_VERSION, 3000) ? "ok" : "MISSING");

    // Half the font: an incomplete upload must not be handed out.
    for (unsigned i = 0; i < 128; i++) {
        if (!sendFontCharacter(i, &expected[i * SITL_LOCAL_OSD_FONT_CHAR_BYTES])) {
            fprintf(stderr, "[osd-font] FAIL: no reply for character %u\n", i);
            return 1;
        }
    }
    const uint32_t revHalf = sitl_local_get_osd_font_revision();
    const int lenHalf = sitl_local_get_osd_font(readback, sizeof(readback));
    if (revHalf != 0 || lenHalf != 0) {
        fprintf(stderr, "[osd-font] FAIL: half an upload is already visible "
                        "(revision=%u bytes=%d)\n", revHalf, lenHalf);
        return 1;
    }
    fprintf(stderr, "[osd-font] 128/256 characters: still no font (revision 0)\n");

    for (unsigned i = 128; i < SITL_LOCAL_OSD_FONT_CHAR_COUNT; i++) {
        if (!sendFontCharacter(i, &expected[i * SITL_LOCAL_OSD_FONT_CHAR_BYTES])) {
            fprintf(stderr, "[osd-font] FAIL: no reply for character %u\n", i);
            return 1;
        }
    }

    const uint32_t revAfter = sitl_local_get_osd_font_revision();
    const int lenAfter = sitl_local_get_osd_font(readback, sizeof(readback));
    if (revAfter == 0 || lenAfter != (int)sizeof(readback)) {
        fprintf(stderr, "[osd-font] FAIL: the complete font is not available "
                        "(revision=%u bytes=%d)\n", revAfter, lenAfter);
        return 1;
    }
    if (memcmp(readback, expected, sizeof(expected)) != 0) {
        for (size_t i = 0; i < sizeof(expected); i++) {
            if (readback[i] != expected[i]) {
                fprintf(stderr, "[osd-font] FAIL: byte %zu differs "
                                "(character %zu, byte %zu): sent 0x%02X, read 0x%02X\n",
                        i, i / SITL_LOCAL_OSD_FONT_CHAR_BYTES,
                        i % SITL_LOCAL_OSD_FONT_CHAR_BYTES,
                        (unsigned)(uint8_t)expected[i], (unsigned)(uint8_t)readback[i]);
                break;
            }
        }
        return 1;
    }

    // Reading twice must be stable, and a too-small buffer must be refused
    // instead of half-filling.
    const uint32_t revAgain = sitl_local_get_osd_font_revision();
    const int shortLen = sitl_local_get_osd_font(readback, (int)sizeof(readback) - 1);
    if (revAgain != revAfter || shortLen != 0) {
        fprintf(stderr, "[osd-font] FAIL: unstable read (revision %u -> %u, short buffer %d)\n",
                revAfter, revAgain, shortLen);
        return 1;
    }

    fprintf(stderr, "[osd-font] RESULT: PASS - all %d bytes of the flashed font are readable "
                    "(revision %u, %u bytes/char)\n",
            (int)sizeof(readback), revAfter, (unsigned)SITL_LOCAL_OSD_FONT_CHAR_BYTES);
    return 0;
}

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
            if (first < 0 && d > gTraceToleranceUs) {
                first = i;
            }
        }
        fprintf(stderr, "  motor %d: max|d|=%9.4f us  mean|d|=%8.4f us  ", m, maxDiff,
                sumDiff / SCENARIO_STEPS);
        if (first < 0) {
            fprintf(stderr, "identical to %.2f us\n", gTraceToleranceUs);
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
    // rpm-tone: optional tone frequency / amplitude (defaults: the motor
    // fundamental of the RPM the harness feeds, 12000 rpm -> 200 Hz).
    if (strcmp(mode, "rpm-tone") == 0) {
        gToneHz = (argc > 4) ? atof(argv[4]) : 200.0;
        if (argc > 5) {
            gToneAmp = atof(argv[5]);
        }
        fprintf(stderr, "[harness] rpm-tone: %.1f Hz, %.2f rad/s\n", gToneHz, gToneAmp);
    }
    // BF_SITL_DT_JITTER=1: feed the FC a varying dt (a host that forwards its
    // own frame time) instead of the fixed 1 ms grid every filter is designed
    // for. Applies to every mode.
    {
        const char *jitter = getenv("BF_SITL_DT_JITTER");
        gStepDtJitter = (jitter != NULL && jitter[0] != '\0' && jitter[0] != '0');
        if (gStepDtJitter) {
            fprintf(stderr, "[harness] step dt JITTER enabled (1000/1200/800/1100/900 us)\n");
        }
    }
    // BF_HARNESS_STEP_US: the host step size (default 1000 us = 1 kHz). Use 250
    // to feed the FC at 4 kHz like the Unreal integration does; the scenario is
    // step-indexed, so it simply runs faster in wall-clock terms.
    {
        const char *stepEnv = getenv("BF_HARNESS_STEP_US");
        if (stepEnv != NULL && stepEnv[0] != '\0') {
            const long v = strtol(stepEnv, NULL, 10);
            if (v >= 100 && v <= 10000) {
                gStepUs = (uint32_t)v;
            }
        }
        fprintf(stderr, "[harness] host step = %u us (%.0f Hz)\n",
                (unsigned)gStepUs, 1e6 / (double)gStepUs);
    }
    // BF_HARNESS_RPM_FROM_MOTORS=1 feeds RPM = 24000 * (motor-1000)/1000, i.e.
    // the speed the FC's own output commands (Unreal's ESC model), optionally
    // through a motor time constant (BF_HARNESS_MOTOR_TAU_MS, 0 = instant).
    {
        const char *fromMotors = getenv("BF_HARNESS_RPM_FROM_MOTORS");
        gRpmFromMotors = (fromMotors != NULL && fromMotors[0] != '\0' && fromMotors[0] != '0');
        const char *tau = getenv("BF_HARNESS_MOTOR_TAU_MS");
        if (tau != NULL && tau[0] != '\0') {
            gMotorTauMs = atof(tau);
        }
        if (gRpmFromMotors) {
            fprintf(stderr, "[harness] RPM input follows the motor outputs "
                            "(motor tau %.1f ms)\n", gMotorTauMs);
        }
    }
    {
        const char *vib = getenv("BF_HARNESS_VIB_RADPS");
        if (vib != NULL && vib[0] != '\0') {
            gVibAmp = atof(vib);
        }
        if (gVibAmp > 0.0) {
            fprintf(stderr, "[harness] motor vibration in the gyro: %.3f rad/s per "
                            "motor harmonic\n", gVibAmp);
        }
    }
    // closed-loop: the gyro is the plant's own rate, not the scenario waveform.
    if (strcmp(mode, "closed-loop") == 0) {
        gClosedLoop = true;
        const char *gain = getenv("BF_HARNESS_PLANT_GAIN");
        if (gain != NULL && gain[0] != '\0') {
            gPlantGain = atof(gain);
        }
        const char *tau = getenv("BF_HARNESS_PLANT_TAU_MS");
        if (tau != NULL && tau[0] != '\0') {
            gPlantTauS = atof(tau) * 1e-3;
        }
        fprintf(stderr, "[harness] closed-loop mode: gyro comes from the plant "
                        "(gain %.4f rad/s per us, tau %.1f ms%s), step dt %s\n",
                gPlantGain, gPlantTauS * 1000.0,
                (gPlantTauS <= 0.0) ? " = pure integrator, no damping" : "",
                gStepDtJitter ? "JITTERS (1000/1200/800/1100/900 us)" : "fixed 1000 us");
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

    // Must happen before the DLL boots: the listen ports are read at init.
    harnessClaimPrivatePorts();

    if (sitl_local_init() != 0) {
        fprintf(stderr, "sitl_local_init() failed\n");
        return 1;
    }
    gStepBase = gStepCount;
    if (mspConnect() != 0) {
        fprintf(stderr, "could not connect to the MSP server on 127.0.0.1:%u\n",
                (unsigned)harnessMspPort());
        sitl_local_shutdown();
        return 1;
    }

    // BF_HARNESS_FILTERS_OFF=1 zeroes the whole gyro/D filter block through the
    // configurator's own MSP path before any trace runs, so a run can compare
    // "filters as configured" against "no filters at all" with everything else
    // identical. Used to separate a filter problem from a loop-rate problem.
    {
        const char *off = getenv("BF_HARNESS_FILTERS_OFF");
        if (off != NULL && off[0] != '\0' && off[0] != '0') {
            setFilterConfigViaMsp(0, 0, 0, 0, 0, 0);
            fprintf(stderr, "[harness] gyro/D filters zeroed (BF_HARNESS_FILTERS_OFF=1)\n");
        }
    }

    // mode: osd-reboot - does the boot splash come back after every reboot
    // trigger? Runs here (before any arming or trace) and returns on its own.
    if (strcmp(mode, "osd-reboot") == 0) {
        const int rc = checkOsdSplashAcrossReboots();
        sitl_local_shutdown();
        return rc;
    }

    // mode: osd-font - replay the configurator's font flash and read it back
    // through the host API.
    if (strcmp(mode, "osd-font") == 0) {
        int rc = checkOsdFontCapture();
        if (rc == 0) {
            rc = checkOsdFontPersistence();
        }
        sitl_local_shutdown();
        return rc;
    }

    // mode: battery - virtual battery present from boot, armed or not. Runs
    // before any arming probe: those step for seconds while disarmed, which
    // would establish the cell count before the armed case is tested.
    if (strcmp(mode, "battery") == 0) {
        const int rc = checkBatteryMeters();
        sitl_local_shutdown();
        return rc;
    }

    // mode: modes - mode-range / 3D exports (no MSP needed).
    if (strcmp(mode, "modes") == 0) {
        const int rc = checkModeRanges();
        sitl_local_shutdown();
        return rc;
    }

    // mode: crashflip - FC-side crash-flip latch + differential motor output.
    if (strcmp(mode, "crashflip") == 0) {
        const int rc = checkCrashFlip();
        sitl_local_shutdown();
        return rc;
    }

    // The aircraft config may have no ARM switch at all: the Unreal side binds
    // one at runtime through sitl_local_set_arm_switch(), and the firmware's
    // stick-arming fallback cannot work here (its arm delay accumulates
    // getTaskDeltaTimeUs(TASK_SELF), i.e. the event-driven RX task's 1 us
    // period). Bind BOXARM to AUX1 with a 1500..2000 us range on the EEPROM copy
    // the harness works on, exactly like the host does.
    {
        uint8_t aux = 0xFF;
        uint8_t startStep = 0;
        uint8_t endStep = 0;
        sitl_local_get_arm_switch(&aux, &startStep, &endStep);
        if (aux == 0xFF) {
            if (sitl_local_set_arm_switch(4, 24, 44) == 0) {   // AUX1, 1500..2000 us
                fprintf(stderr, "[arm] config has no ARM switch: bound BOXARM to AUX1 "
                                "(1500..2000 us)\n");
            } else {
                fprintf(stderr, "[arm] could not bind an ARM switch\n");
            }
        }
    }

    // BF_HARNESS_NO_BLACKBOX=1 switches the flight log off through MSP before any
    // trace runs, so a crash can be attributed to the blackbox path (or cleared
    // of it) without a rebuild.
    {
        const char *noBb = getenv("BF_HARNESS_NO_BLACKBOX");
        if (noBb != NULL && noBb[0] != '\0' && noBb[0] != '0') {
            if (setBlackboxDeviceViaMsp(BLACKBOX_DEV_NONE)) {
                fprintf(stderr, "[harness] blackbox device NONE (BF_HARNESS_NO_BLACKBOX=1)\n");
            }
        }
    }

    // BF_HARNESS_NO_OSD=1 clears FEATURE_OSD through MSP before any trace runs -
    // the A/B for "does the OSD feature itself change whether the firmware arms".
    {
        const char *noOsd = getenv("BF_HARNESS_NO_OSD");
        if (noOsd != NULL && noOsd[0] != '\0' && noOsd[0] != '0') {
            uint8_t cfg[8];
            const int len = mspRequest(MSP_FEATURE_CONFIG, NULL, 0, cfg, sizeof(cfg), 2000);
            if (len >= 4) {
                const uint32_t before = (uint32_t)cfg[0] | ((uint32_t)cfg[1] << 8)
                                      | ((uint32_t)cfg[2] << 16) | ((uint32_t)cfg[3] << 24);
                const uint32_t after = before & ~FEATURE_BIT_OSD;
                cfg[0] = (uint8_t)(after & 0xFF);
                cfg[1] = (uint8_t)((after >> 8) & 0xFF);
                cfg[2] = (uint8_t)((after >> 16) & 0xFF);
                cfg[3] = (uint8_t)((after >> 24) & 0xFF);
                (void)mspRequest(MSP_SET_FEATURE_CONFIG, cfg, 4, NULL, 0, 2000);
                fprintf(stderr, "[harness] FEATURE_OSD cleared: 0x%08X -> 0x%08X\n",
                        (unsigned)before, (unsigned)after);
            } else {
                fprintf(stderr, "[harness] could not read the feature mask (%d bytes)\n", len);
            }
        }
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
    } else if (strcmp(mode, "closed-loop") == 0) {
        // traceA above already ran closed-loop with the RPM filter as configured.
        fprintf(stderr, "[closed-loop] plant: rate = %.0f ms first-order lag driven by "
                        "the motor differential\n", PLANT_TAU_S * 1000.0);
        printClosedLoopResult("rpm filter as configured");

        setRpmHarmonicsViaMsp(0, "rpm filter off");
        gPlantRate[0] = gPlantRate[1] = gPlantRate[2] = 0.0;
        if (!settleAndRearm()) {
            fprintf(stderr, "[closed-loop] could not re-arm for the rpm-off run\n");
            sitl_local_shutdown();
            return 2;
        }
        warmUpAndTrace(traceA, &motorCountA, &armedA, rangeA);
        printClosedLoopResult("rpm filter off (harmonics 0)");

        setRpmHarmonicsViaMsp(3, "rpm filter restored");
        gPlantRate[0] = gPlantRate[1] = gPlantRate[2] = 0.0;
        if (!settleAndRearm()) {
            fprintf(stderr, "[closed-loop] could not re-arm for the rpm-on run\n");
            sitl_local_shutdown();
            return 2;
        }
        warmUpAndTrace(traceA, &motorCountA, &armedA, rangeA);
        printClosedLoopResult("rpm filter restored (harmonics 3)");

        sitl_local_shutdown();
        return 0;
    } else if (strcmp(mode, "cycle") == 0) {
        // Arm/disarm cycling with a step-time histogram: the A/B for "is the
        // step-path file I/O real". Run as configured, then with the blackbox
        // switched off through MSP, then back, in the same process.
        const int cycles = 12;
        const int stepsPerPhase = 400;
        fprintf(stderr, "[cycle] arm/disarm cycling, %d cycles x %d steps per phase\n",
                cycles, stepsPerPhase);
        cyclePhases(3, stepsPerPhase);
        cycleReport("warm-up");
        cyclePhases(cycles, stepsPerPhase);
        cycleReport("as configured");

        if (setBlackboxDeviceViaMsp(BLACKBOX_DEV_NONE)) {
            cyclePhases(cycles, stepsPerPhase);
            cycleReport("blackbox device NONE");
            setBlackboxDeviceViaMsp(BLACKBOX_DEV_VIRTUAL);
        }
        sitl_local_shutdown();
        return 0;
    } else if (strcmp(mode, "bench") == 0) {
        // What does this build cost per step? Wall and process CPU time for a
        // fixed number of steps at the configured host step (BF_HARNESS_STEP_US,
        // default 1000 us; use 250 for the 4 kHz LOCAL link). Compare builds,
        // BF_SITL_GYRO_HZ values, and `bench aux` (all AUX channels high, so a
        // bound blackbox switch logs) to see what each part costs.
        const int warmup = 4000;
        // 24 s of virtual time, so the burst recorder (one 2 s window every 12 s)
        // always contributes its share to the measurement.
        const int steps = 96000;
        gAuxHighOverride = (argc > 2 && strcmp(argv[2], "aux") == 0);

        // Disarmed pass first: same loop, but the mixer does not drive the
        // motors and the blackbox does not log. The difference to the armed pass
        // is what the armed-only work costs (mixer/motor output + the flight log).
        for (int i = 0; i < warmup; i++) {
            sitl_local_input_t in;
            sitl_local_output_t out;
            makeInput(0, ARM_THROTTLE, false, &in);
            rawStep(&in, &out);
        }
        double dWall0 = harnessWallSeconds();
        double dCpu0 = harnessCpuSeconds();
        double dVus0 = (double)sitl_local_time_us();
        for (int i = 0; i < steps; i++) {
            sitl_local_input_t in;
            sitl_local_output_t out;
            makeInput(0, ARM_THROTTLE, false, &in);
            rawStep(&in, &out);
        }
        const double dWall = harnessWallSeconds() - dWall0;
        const double dCpu = harnessCpuSeconds() - dCpu0;
        const double dVirtualS = ((double)sitl_local_time_us() - dVus0) * 1e-6;
        fprintf(stderr, "[bench] disarmed: per step wall %.3f us cpu %.3f us => %.2f%% of one core\n",
                dWall * 1e6 / steps, dCpu * 1e6 / steps,
                100.0 * dCpu / dVirtualS);

        // Arm the way the scenarios do - low throttle first, the firmware will
        // not arm otherwise - so the measured path is the armed one (mixer,
        // motor output and, when the config binds it, the blackbox log).
        if (!runSteps(1500, ARM_THROTTLE, true)) {
            fprintf(stderr, "[bench] warning: could not arm, measuring the disarmed path\n");
        }
        deactivateRunawayTakeoffProtection();
        fprintf(stderr, "[bench] armed=%d armFlags=0x%08X auxHigh=%d\n",
                sitl_local_get_armed(), (unsigned)sitl_local_get_arming_flags(),
                gAuxHighOverride ? 1 : 0);
        for (int i = 0; i < warmup; i++) {
            sitl_local_input_t in;
            sitl_local_output_t out;
            makeInput(0, TRACE_THROTTLE, true, &in);
            rawStep(&in, &out);
        }
        const double wall0 = harnessWallSeconds();
        const double cpu0 = harnessCpuSeconds();
        const double vus0 = (double)sitl_local_time_us();
        bool armed = false;
        int armedSteps = 0;
        for (int i = 0; i < steps; i++) {
            sitl_local_input_t in;
            sitl_local_output_t out;
            makeInput(0, TRACE_THROTTLE, true, &in);
            armed = rawStep(&in, &out);
            if (armed) {
                armedSteps++;
            }
        }
        const double wall = harnessWallSeconds() - wall0;
        const double cpu = harnessCpuSeconds() - cpu0;
        const double virtualS = ((double)sitl_local_time_us() - vus0) * 1e-6;
        const double coreFraction = (virtualS > 0.0) ? cpu / virtualS : 0.0;
        fprintf(stderr, "[bench] steps=%d armedSteps=%d virtual=%.3f s wall=%.3f s cpu=%.3f s\n",
                steps, armedSteps, virtualS, wall, cpu);
        fprintf(stderr, "[bench] per step: wall %.3f us cpu %.3f us  "
                        "=> %.2f%% of one core at %.0f Hz (%.0f steps/s wall)\n",
                wall * 1e6 / steps, cpu * 1e6 / steps,
                100.0 * coreFraction, (virtualS > 0.0) ? steps / virtualS : 0.0,
                steps / wall);

        // Where the per-step time actually goes: the harness's own input
        // assembly, the DLL step, and the two clocks the DLL can use. A clock
        // that costs microseconds per call is easy to miss in a profile but
        // obvious here (clock_gettime on MinGW is not free).
        {
            sitl_local_input_t in;
            sitl_local_output_t out;
            makeInput(0, TRACE_THROTTLE, true, &in);
            const int n = 200000;
            double t0 = harnessWallSeconds();
            for (int i = 0; i < n; i++) {
                makeInput(0, TRACE_THROTTLE, true, &in);
            }
            const double tMake = (harnessWallSeconds() - t0) * 1e6 / n;
            t0 = harnessWallSeconds();
            for (int i = 0; i < n; i++) {
                sitl_local_step(&in, gStepUs, &out);
            }
            const double tStep = (harnessWallSeconds() - t0) * 1e6 / n;
            fprintf(stderr, "[bench] phases: makeInput %.3f us  sitl_local_step %.3f us  "
                            "sum %.3f us (input struct %u B)\n",
                    tMake, tStep, tMake + tStep, (unsigned)sizeof(sitl_local_input_t));
        }
        sitl_local_shutdown();
        return 0;
    } else if (strcmp(mode, "vib") == 0) {
        // Does the filter chain actually remove motor vibration? The harness
        // puts the harmonics a real gyro measures into the gyro data it feeds
        // (motor 0 of its telemetry runs at 12000 rpm = 200 Hz), and this
        // measures how much of that survives to the PID with the filters off
        // vs on. Nothing on the DLL side is involved - the point is to test the
        // firmware's chain, not the host.
        if (gVibAmp <= 0.0) {
            gVibAmp = 0.10;   // rad/s per motor harmonic: ~8.6 deg/s fed peak
        }
        fprintf(stderr, "[vib] gyro feed = physics rate + %.2f rad/s motor "
                        "harmonics (BF_HARNESS_VIB_RADPS)\n", gVibAmp);
        runVibrationProbe("as configured");
        setFilterConfigViaMsp(0, 0, 0, 0, 0, 0);
        runVibrationProbe("all gyro/D filters off");
        setFilterConfigViaMsp(275, 550, 275, 550, 75, 1);
        runVibrationProbe("LPF1 275-550, LPF2 550, dterm 75, RPM h1");
        setFilterConfigViaMsp(275, 550, 275, 550, 75, 0);
        runVibrationProbe("same LPFs, RPM filter off");
        sitl_local_shutdown();
        return 0;
    } else if (strcmp(mode, "rpm-tone") == 0) {
        // traceA above already ran with the RPM filter as configured.
        fprintf(stderr, "[rpm-tone] %s: roll gyro = %.1f rad/s sine at %.1f Hz\n",
                gScenario->name, gToneAmp, gToneHz);
        printToneAmps("rpm filter as configured");

        setRpmHarmonicsViaMsp(0, "rpm filter off");
        gTraceAmpIn = gTraceAmpOut = 0.0;
        gSigTarget = gSigA;
        warmUpAndTrace(traceA, &motorCountA, &armedA, rangeA);
        printToneAmps("rpm filter off (harmonics 0)");

        setRpmHarmonicsViaMsp(3, "rpm filter restored");
        gTraceAmpIn = gTraceAmpOut = 0.0;
        gSigTarget = gSigA;
        warmUpAndTrace(traceA, &motorCountA, &armedA, rangeA);
        printToneAmps("rpm filter restored (harmonics 3)");

        sitl_local_shutdown();
        return 0;
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

    // What the RPM filter notch bank is actually tuned to (same for both
    // traces unless the config changed; the signal table above catches that).
    {
        sitl_local_loop_state_t ls;
        if (sitl_local_get_loop_state(&ls) == 0) {
            fprintf(stderr, "  rpm filter: harmonics=%u minHz=%u q=%u fade=%u lpf=%u "
                            "weights=%u/%u/%u cycleMult=%.4f\n",
                    (unsigned)ls.rpmHarmonics, (unsigned)ls.rpmMinHz, (unsigned)ls.rpmQ,
                    (unsigned)ls.rpmFadeRangeHz, (unsigned)ls.rpmLpfHz,
                    (unsigned)ls.rpmWeight[0], (unsigned)ls.rpmWeight[1],
                    (unsigned)ls.rpmWeight[2],
                    (double)ls.cycleTimeMultiplier);
            fprintf(stderr, "  rpm input : raw rpm/60 = %.1f/%.1f/%.1f/%.1f Hz (host data), "
                            "filtered = %.1f/%.1f/%.1f/%.1f Hz -> notches %.0f/%.0f/%.0f Hz\n",
                    (double)ls.rpmRawMotorHz[0], (double)ls.rpmRawMotorHz[1],
                    (double)ls.rpmRawMotorHz[2], (double)ls.rpmRawMotorHz[3],
                    (double)ls.rpmMotorHz[0], (double)ls.rpmMotorHz[1],
                    (double)ls.rpmMotorHz[2], (double)ls.rpmMotorHz[3],
                    (double)ls.rpmNotchHz[0], (double)ls.rpmNotchHz[1],
                    (double)ls.rpmNotchHz[2]);
        }
    }

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
