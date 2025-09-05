/********************************************************************
 *  imu_demo.c – Full demo for turning BerryIMU data into a quaternion
 *                and comparing it against a target orientation.
 *
 *  Dependencies:
 *      • madgwick.h / madgwick.c  (included below)
 *      • math library (-lm)
 *
 *  Build (Linux/macOS):
 *      gcc imu_demo.c madgwick.c -lm -o imu_demo
 *
 *  Author:  Lumo (Proton) – July 2025
 ********************************************************************/

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "madgwick.h"

/* ------------------------------------------------------------------
 *  USER‑DEFINED SENSOR INTERFACE
 *
 *  Replace these stubs with the actual code you use to talk to the
 *  BerryIMU (e.g., via I²C).  They must return the most recent sensor
 *  reading in the units expected by the filter:
 *
 *      • Gyroscope : rad · s⁻¹
 *      • Accelerometer : g (9.81 m s⁻²) – any consistent scale works
 *      • Magnetometer : µT (or any consistent scale)
 *
 *  The functions should block until a fresh sample is available or
 *  return the last cached value.
 * ------------------------------------------------------------------ */
static bool read_gyro(float *gx, float *gy, float *gz)
{
    /* TODO: fill in real reading code */
    /* Example dummy data – stationary device */
    *gx = 0.0f; *gy = 0.0f; *gz = 0.0f;
    return true;            /* false => read error */
}

static bool read_accel(float *ax, float *ay, float *az)
{
    /* TODO: fill in real reading code */
    *ax = 0.0f; *ay = 0.0f; *az = 1.0f;   /* gravity pointing +Z */
    return true;
}

static bool read_mag(float *mx, float *my, float *mz)
{
    /* TODO: fill in real reading code */
    *mx = 0.2f; *my = 0.0f; *mz = 0.5f;   /* arbitrary Earth field */
    return true;
}

/* ------------------------------------------------------------------
 *  Helper: compute the angular distance (in degrees) between two
 *          unit quaternions qA and qB.
 *
 *  The formula is:
 *        θ = 2 * acos(|qA·qB|)
 *  where "·" is the quaternion dot product.
 *
 *  The result is always the smallest rotation (0 – 180 deg).
 * ------------------------------------------------------------------ */
static float quat_angle_deg(const float qA[4], const float qB[4])
{
    /* Dot product (both quaternions should be normalized) */
    float dot = qA[0]*qB[0] + qA[1]*qB[1] + qA[2]*qB[2] + qA[3]*qB[3];

    /* Clamp to [-1,1] to avoid NaNs from rounding errors */
    if (dot >  1.0f) dot =  1.0f;
    if (dot < -1.0f) dot = -1.0f;

    /* Absolute value makes the result invariant to sign flips */
    dot = fabsf(dot);

    /* Smallest rotation angle (radians) then convert to degrees */
    float theta_rad = 2.0f * acosf(dot);
    return theta_rad * (180.0f / (float)M_PI);
}

/* ------------------------------------------------------------------
 *  look_at_compare()
 *
 *  Returns true if the angular distance between `current_q` and
 *  `target_q` is less than or equal to `tolerance_deg`.
 *
 *  It also writes the computed angular distance to `out_angle_deg`
 *  (if the pointer is non‑NULL) so you can log or display it.
 * ------------------------------------------------------------------ */
static bool look_at_compare(const float current_q[4],
                            const float target_q[4],
                            float tolerance_deg,
                            float *out_angle_deg)
{
    float ang = quat_angle_deg(current_q, target_q);
    if (out_angle_deg) *out_angle_deg = ang;
    return (ang <= tolerance_deg);
}

/* ------------------------------------------------------------------
 *  MAIN DEMO LOOP
 * ------------------------------------------------------------------ */
int main(void)
{
    /* ----------------------------------------------------------------
     * 1️⃣ Initialise the Madgwick filter.
     *
     *    • β (beta) controls the trade‑off between responsiveness and
     *      smoothness. 0.1–0.3 works well for most hobby IMUs.
     *    • Start with the identity quaternion (no rotation).
     * ---------------------------------------------------------------- */
    const float beta = 0.2f;               /* tweak as needed */
    madgwick_init(beta, 1.0f, 0.0f, 0.0f, 0.0f);

    /* ----------------------------------------------------------------
     * 2️⃣ Define a target orientation you want to compare against.
     *
     *    For illustration we use a 45° rotation around the Z‑axis.
     *    Quaternion for a rotation θ about axis (x,y,z) is:
     *        q = [cos(θ/2), sin(θ/2)*x, sin(θ/2)*y, sin(θ/2)*z]
     * ---------------------------------------------------------------- */
    const float target_angle_rad = M_PI/4.0f;   /* 45° */
    const float half = target_angle_rad/2.0f;
    const float target_q[4] = {
        cosf(half),          /* w */
        0.0f,                /* x */
        0.0f,                /* y */
        sinf(half)           /* z */
    };

    /* ----------------------------------------------------------------
     * 3️⃣ Main acquisition / processing loop.
     *
     *    In a real embedded program you would likely run this at a
     *    fixed rate (e.g., 200 Hz).  Here we simply use a constant
     *    `dt` and sleep a bit to keep the demo readable.
     * ---------------------------------------------------------------- */
    const float dt = 0.01f;               /* 100 Hz sample period */
    const float tolerance_deg = 10.0f;    /* how close we consider “aligned” */

    printf("Starting IMU demo – press Ctrl‑C to stop.\n");
    printf("Target orientation: 45° yaw (Z‑axis).\n\n");

    while (true) {
        float gx, gy, gz;   /* rad/s */
        float ax, ay, az;   /* g */
        float mx, my, mz;   /* µT (any consistent unit) */

        /* ---- Read sensors ------------------------------------------------ */
        if (!read_gyro(&gx, &gy, &gz) ||
            !read_accel(&ax, &ay, &az) ||
            !read_mag(&mx, &my, &mz)) {
            fprintf(stderr, "Sensor read error – skipping this cycle.\n");
            continue;
        }

        /* ---- Feed data to the filter ------------------------------------ */
        madgwick_update(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

        /* ---- Pull out the current quaternion ---------------------------- */
        float cur_q[4];
        madgwick_get_quaternion(&cur_q[0], &cur_q[1], &cur_q[2], &cur_q[3]);

        /* ---- Compare with the target orientation ------------------------ */
        float angle_to_target;
        bool aligned = look_at_compare(cur_q, target_q,
                                       tolerance_deg, &angle_to_target);

        /* ---- Print a concise status line -------------------------------- */
        printf("\rAngle to target: %6.2f°  |  %s",
               angle_to_target,
               aligned ? "ALIGNED   " : "not aligned");

        fflush(stdout);

        /* ---- Simple timing – replace with a proper RTOS delay if needed */
        #ifdef _WIN32
            Sleep((DWORD)(dt*1000));   /* Windows Sleep expects ms */
        #else
            struct timespec ts = {0, (long)(dt*1e9)};
            nanosleep(&ts, NULL);
        #endif
    }

    /* Unreachable, but good practice */
    return 0;
}

/********************************************************************
 *  END OF DEMO PROGRAM
 ********************************************************************/
