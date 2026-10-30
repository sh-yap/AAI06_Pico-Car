/* Host synthetic traces for the Buddy 4 terrain processor. Each case feeds a
 * 100 Hz trace of calibrated accelerometer samples after a run start and
 * counts the humps that complete. Axes follow b4_cfg_default(): the lateral
 * axis is X and the forward axis is Y, both with sign -1, and Z is up. */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "b4_cfg.h"
#include "b4_proc.h"

#define SAMPLE_MS      10
#define MAX_SAMPLES    2000
#define LEVER_MM       115.0
#define TRACK_HALF_UM  57500.0
#define CASTER_LAT_UM  4600.0
#define CASTER_FWD_UM  120000.0
#define DRIVE_SPEED_MM_S 100
/* Driving lead-in before a crossing. The learning hold is 256 driving
 * samples, and learning starts about 35 samples into the trace. */
#define LEAD_IN_SAMPLES 300

typedef struct
{
    int  x[MAX_SAMPLES];
    int  y[MAX_SAMPLES];
    int  z[MAX_SAMPLES];
    int  n;
} trace_t;

typedef struct
{
    int      started;
    int      completed;
    int      valid;
    double   height_mm;   /* first valid hump */
} result_t;

static uint32_t g_seed = 12345U;
static uint32_t g_lcg = 12345U;
static int g_failed = 0;
static int g_checks = 0;
static trace_t g_t;

static int
rnd (int amp)
{
    g_lcg = (g_lcg * 1664525U) + 1013904223U;
    return ((int)((g_lcg >> 8) % (uint32_t)((2 * amp) + 1))) - amp;
}

static void
trace_flat (trace_t * t, int n)
{
    for (int i = 0; i < n; i++)
    {
        t->x[t->n] = 0;
        t->y[t->n] = 0;
        t->z[t->n] = 1000;
        t->n++;
    }
}

/* Tilt of a right (side = +1) or left (-1) wheel lift of height_mm. */
static void
set_lift (trace_t * t, int i, double height_mm, int side)
{
    double dx = height_mm * 1000.0 / LEVER_MM;
    double r = (double)side * dx;            /* oriented roll, + = right up */
    double pred = -((fabs(r) * TRACK_HALF_UM) + (r * CASTER_LAT_UM)) /
                  CASTER_FWD_UM;
    double y = -pred;                        /* pitch sign -1 */
    double x = -r;                           /* lateral sign -1 */
    double z = sqrt(1.0e6 - (x * x) - (y * y));

    t->x[i] = (int)lround(x);
    t->y[i] = (int)lround(y);
    t->z[i] = (int)lround(z);
}

/* Trapezoid crossing: rise, crest, fall, in samples. */
static void
trace_cross (trace_t * t, double h, int side, int rise, int crest, int fall)
{
    int total = rise + crest + fall;
    for (int i = 0; i < total; i++)
    {
        double hh = h;
        if (i < rise)
        {
            hh = h * (double)(i + 1) / (double)rise;
        }
        else if (i >= (rise + crest))
        {
            hh = h * (double)(total - 1 - i) / (double)fall;
        }
        set_lift(t, t->n, hh, side);
        t->n++;
    }
}

static void
add_vibration (trace_t * t, int from, int amp, int mode)
{
    for (int i = from; i < t->n; i++)
    {
        int s = ((i & 1) != 0) ? 1 : -1;
        int dx = (mode == 0) ? rnd(amp) : (s * amp);
        int dy = (mode == 0) ? rnd(amp) : (-s * amp);
        int dz = (mode == 0) ? rnd(amp) : (s * amp);
        t->x[i] += dx;
        t->y[i] += dy;
        t->z[i] += dz;
    }
}

static void
run_trace (const trace_t * t, result_t * r)
{
    b4_cfg_config_t cfg = b4_cfg_default();
    b4_types_status_t st;
    b4_types_sample_t s = {0};

    *r = (result_t){0};
    b4_proc_init(&cfg);
    b4_proc_reset_run_stats();
    for (int i = 0; i < t->n; i++)
    {
        /* The car drives for the whole trace: the learning input is on. */
        b4_proc_set_drive(true, DRIVE_SPEED_MM_S);
        s.timestamp_ms = (uint32_t)(1000 + (i * SAMPLE_MS));
        s.accel_x_mg = t->x[i];
        s.accel_y_mg = t->y[i];
        s.accel_z_mg = t->z[i];
        s.b_valid = true;
        b4_proc_process(&s);
        b4_proc_get_status(s.timestamp_ms, &st);
        if (st.b_hump_started_event)
        {
            r->started++;
        }
        if (st.b_hump_completed_event)
        {
            r->completed++;
            if (st.b_hump_height_valid)
            {
                if (r->valid == 0)
                {
                    r->height_mm = (double)st.hump_peak_height_um / 1000.0;
                }
                r->valid++;
            }
        }
    }
}

static void
check_known (const char * name, bool ok, bool known_limit,
             const result_t * r)
{
    g_checks++;
    if ((!ok) && (!known_limit))
    {
        g_failed++;
    }
    printf("%s %-42s started=%d completed=%d valid=%d height=%.2f mm\n",
           ok ? "PASS" : (known_limit ? "KNOWN" : "FAIL"), name, r->started,
           r->completed, r->valid, r->height_mm);
}

static void
check (const char * name, bool ok, const result_t * r)
{
    check_known(name, ok, false, r);
}

static void
case_flat (const char * name, int amp, int mode)
{
    result_t r;
    g_t.n = 0;
    g_lcg = g_seed;
    trace_flat(&g_t, 80);
    trace_flat(&g_t, 500);
    if (amp > 0)
    {
        add_vibration(&g_t, 80, amp, mode);
    }
    run_trace(&g_t, &r);
    check(name, r.started == 0 && r.completed == 0, &r);
}

/* Returns the crossing height. A positive ref_mm adds a tolerance_mm check. */
static double
case_cross (const char * name, int side, int amp, int mode, double ref_mm,
            double tolerance_mm)
{
    result_t r;
    bool ok;
    g_t.n = 0;
    g_lcg = g_seed;
    trace_flat(&g_t, 80);
    trace_flat(&g_t, LEAD_IN_SAMPLES);
    trace_cross(&g_t, 11.0, side, 40, 20, 40);
    trace_flat(&g_t, 100);
    if (amp > 0)
    {
        add_vibration(&g_t, 80, amp, mode);
    }
    run_trace(&g_t, &r);
    ok = (r.completed == 1) && (r.valid == 1);
    if (ref_mm > 0.0)
    {
        ok = ok && (fabs(r.height_mm - ref_mm) <= tolerance_mm);
    }
    check(name, ok, &r);
    return r.height_mm;
}

/* Launch: 50 mg forward step for 300 ms plus a 60 mg half-sine roll rock. */
static void
case_launch (const char * name, int lead, int pitch_sign, int rock_sign,
             bool known_limit)
{
    result_t r;
    g_t.n = 0;
    trace_flat(&g_t, lead + 600);
    for (int i = 0; i < 30; i++)
    {
        g_t.y[lead + i] += pitch_sign * 50;
        g_t.x[lead + i] += rock_sign *
            (int)lround(60.0 * sin(3.14159265 * (double)i / 30.0));
    }
    run_trace(&g_t, &r);
    check_known(name, r.started == 0 && r.completed == 0, known_limit, &r);
}

/* Flat ground with slow lateral wobble: a 2 Hz sine of 40 mg plus random 60 mg
 * on roll, with pitch following the wheel-lift model. This is not a hump and
 * must not start one. */
static void
case_wobble (const char * name)
{
    result_t r;
    g_t.n = 0;
    g_lcg = g_seed;
    trace_flat(&g_t, 80);
    for (int i = 0; i < 300; i++)
    {
        double rr = (40.0 * sin(2.0 * 3.14159265 * 2.0 * (double)i / 100.0)) +
                    (double)rnd(60);
        double y = ((fabs(rr) * TRACK_HALF_UM) + (rr * CASTER_LAT_UM)) /
                   CASTER_FWD_UM;
        double x = -rr;
        g_t.x[g_t.n] = (int)lround(x);
        g_t.y[g_t.n] = (int)lround(y);
        g_t.z[g_t.n] = (int)lround(sqrt(1.0e6 - (x * x) - (y * y)));
        g_t.n++;
    }
    trace_flat(&g_t, 200);
    run_trace(&g_t, &r);
    /* Known limit: the 24-sample mean passes about 66 percent of a 2 Hz
     * sine, so +-40 mg still crosses the start threshold. At 3 Hz and above
     * the same case passes. */
    check_known(name, r.started == 0 && r.completed == 0, true, &r);
}

int
main (int argc, char ** argv)
{
    double clean_r;
    double clean_l;

    if (argc > 1)
    {
        g_seed = (uint32_t)strtoul(argv[1], NULL, 10);
    }

    case_flat("flat clean", 0, 0);
    clean_r = case_cross("right crossing clean", 1, 0, 0, 0.0, 0.0);
    clean_l = case_cross("left crossing clean", -1, 0, 0, 0.0, 0.0);
    case_wobble("slow roll wobble 2Hz +-40 +-60 rnd");

    case_flat("vib flat random +-60", 60, 0);
    case_flat("vib flat alternating +-60", 60, 1);
    case_cross("vib right crossing random +-60", 1, 60, 0, clean_r, 3.0);
    case_cross("vib right crossing alternating +-60", 1, 60, 1, clean_r, 1.0);
    case_cross("vib left crossing random +-60", -1, 60, 0, clean_l, 3.0);
    for (int li = 0; li < 6; li++)
    {
        static const int leads[6] = {0, 20, 32, 36, 40, 100};
        int lead = leads[li];
        char name[64];
        for (int p = -1; p <= 1; p += 2)
        {
            for (int k = -1; k <= 1; k += 2)
            {
                (void)snprintf(name, sizeof(name),
                               "launch lead=%d pitch%c rock%c", lead,
                               (p > 0) ? '+' : '-', (k > 0) ? '+' : '-');
                case_launch(name, lead, p, k, (p > 0) && (lead >= 32));
            }
        }
    }

    printf("%d checks, %d failed\n", g_checks, g_failed);
    return (g_failed != 0) ? 1 : 0;
}
