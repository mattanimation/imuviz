/*  main.c
 *
 *  Compile:
 *      gcc -Wall -O2 main.c -lX11 -lGL -lGLU -lm -o imuviz
 *
 *  Run:
 *      ./imuviz -useSim true
 *
 *  This program opens an X11 window, creates an old‑style OpenGL context,
 *  draws a rotating wire‑frame cube whose orientation is driven by a
 *  simulated 9‑DOF IMU (gyro + accelerometer + magnetometer).
 *
 *  Dependencies: libX11, libGL, libm (standard on Linux).
 */

//#define _GNU_SOURCE  // for M_PI
#define _POSIX_C_SOURCE 199309L   /* for nanosleep */

#include <X11/Xlib.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glx.h>
#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>

#include <wiringPi.h>
#include <stdbool.h>
#include <string.h>
#include <sys/stat.h>  // Required for realpath
#include <sys/types.h>
#include <linux/limits.h> // for PATH_MAX
#include <pwd.h> // for pwu and 
#include "IMU.h"
#include "MadgwickAHRS.h"

#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720
#define WINDOW_RATIO WINDOW_WIDTH / WINDOW_HEIGHT

#define DT 0.02 // [s/loop] loop period. 20ms
#define AA 0.97 // complementary filter constant

#define A_GAIN 0.0573 // [deg/LSB]
#define G_GAIN 0.070  // [deg/s/LSB]

/* -------------------------------------------------------------
 *  Simple quaternion utilities (no external libs)
 * ------------------------------------------------------------- */
typedef struct { double w, x, y, z; } quat;

/* Normalize quaternion */
static void quat_normalize(quat *q) {
    double norm = sqrt(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    if (norm > 0.0) {
        q->w /= norm; q->x /= norm; q->y /= norm; q->z /= norm;
    }
}

/* Multiply two quaternions:  result = a * b */
static quat quat_mul(const quat a, const quat b) {
    quat r;
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return r;
}

/* Convert quaternion to a 4×4 column‑major matrix (OpenGL expects column major) */
static void quat_to_mat4(const quat q, double m[16]) {
    double xx = q.x*q.x, yy = q.y*q.y, zz = q.z*q.z;
    double xy = q.x*q.y, xz = q.x*q.z, yz = q.y*q.z;
    double wx = q.w*q.x, wy = q.w*q.y, wz = q.w*q.z;

    m[0]  = 1.0 - 2.0*(yy + zz);
    m[1]  = 2.0*(xy + wz);
    m[2]  = 2.0*(xz - wy);
    m[3]  = 0.0;

    m[4]  = 2.0*(xy - wz);
    m[5]  = 1.0 - 2.0*(xx + zz);
    m[6]  = 2.0*(yz + wx);
    m[7]  = 0.0;

    m[8]  = 2.0*(xz + wy);
    m[9]  = 2.0*(yz - wx);
    m[10] = 1.0 - 2.0*(xx + yy);
    m[11] = 0.0;

    m[12] = m[13] = m[14] = 0.0;
    m[15] = 1.0;
}

/* -------------------------------------------------------------
 *  Simulated 9‑DOF IMU
 *
 *  We generate three angular velocities (rad/s) that vary sinusoidally.
 *  In a real system you would read them from the gyroscope.
 * ------------------------------------------------------------- */
static void simulate_imu(double t, double gyro[3], double accel[3], double mag[3]) {
    /* Gyroscope: slow roll/pitch/yaw oscillations */
    gyro[0] = 0.5 * sin(t * 0.7);   // roll rate
    gyro[1] = 0.5 * sin(t * 0.9);   // pitch rate
    gyro[2] = 0.5 * sin(t * 1.1);   // yaw rate

    /* Accelerometer: just gravity plus a tiny sinusoid */
    accel[0] = 0.0;
    accel[1] = 0.0;
    accel[2] = -9.81 + 0.2 * sin(t * 0.5);

    /* Magnetometer: constant field with a small wobble */
    mag[0] = 0.3 + 0.05 * sin(t * 0.3);
    mag[1] = 0.0;
    mag[2] = 0.5 + 0.05 * cos(t * 0.3);
}

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
//static bool read_gyro(float *gx, float *gy, float *gz)
//{
//    /* TODO: fill in real reading code */
//    /* Example dummy data – stationary device */
//    *gx = 0.0f; *gy = 0.0f; *gz = 0.0f;
//    return true;            /* false => read error */
//}

//static bool read_accel(float *ax, float *ay, float *az)
//{
//    /* TODO: fill in real reading code */
//    *ax = 0.0f; *ay = 0.0f; *az = 1.0f;   /* gravity pointing +Z */
//    return true;
//}

//static bool read_mag(float *mx, float *my, float *mz)
//{
//    /* TODO: fill in real reading code */
//    *mx = 0.2f; *my = 0.0f; *mz = 0.5f;   /* arbitrary Earth field */
//    return true;
//}

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

/* -------------------------------------------------------------
 *  Rendering helpers
 * ------------------------------------------------------------- */

/* Manual version (core OpenGL only) */
void set_perspective(double fovDeg, double aspect, double zNear, double zFar) {
    double f = 1.0 / tan(fovDeg * M_PI / 360.0);   // cotangent of half‑fov
    double proj[16] = {
        f/aspect, 0, 0,                           0,
        0,       f, 0,                           0,
        0,       0, (zFar+zNear)/(zNear-zFar),  -1,
        0,       0, (2*zFar*zNear)/(zNear-zFar), 0
    };
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd(proj);
}

static void draw_wire_cube(void) {
    static const GLfloat verts[8][3] = {
        {-0.5f,-0.5f,-0.5f}, { 0.5f,-0.5f,-0.5f},
        { 0.5f, 0.5f,-0.5f}, {-0.5f, 0.5f,-0.5f},
        {-0.5f,-0.5f, 0.5f}, { 0.5f,-0.5f, 0.5f},
        { 0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f}
    };
    static const GLuint edges[12][2] = {
        {0,1},{1,2},{2,3},{3,0},
        {4,5},{5,6},{6,7},{7,4},
        {0,4},{1,5},{2,6},{3,7}
    };

    glColor3f(0.2f, 0.9f, 0.3f);
    glBegin(GL_LINES);
    for (int i=0;i<12;i++) {
        glVertex3fv(verts[edges[i][0]]);
        glVertex3fv(verts[edges[i][1]]);
    }
    glEnd();
}

/* -------------------------------------------------------------
 * Global flag that the handler will set.
 * Must be volatile sig_atomic_t to be safe across signal boundaries.
 * ------------------------------------------------------------- */
static volatile sig_atomic_t got_sigint = 0;

/* -------------------------------------------------------------
 * Minimal signal handler – only sets the flag.
 * No unsafe library calls (printf, malloc, etc.) here.
 * ------------------------------------------------------------- */
static void sigint_handler(int signo)
{
    (void)signo;          /* silence unused‑parameter warning */
    got_sigint = 1;       /* atomic write – safe */
}

/* -------------------------------------------------------------
 * Optional helper to perform cleanup before exit.
 * You can register it with atexit() if you want guaranteed execution.
 * ------------------------------------------------------------- */
static void cleanup(void)
{
    /* Example: close files, free resources, flush logs, etc. */
    fprintf(stderr, "\n[cleanup] Performing graceful shutdown…\n");
    /* Insert real cleanup code here */
}


void loadIMUCalibration(float *offsets, const char* path){

    FILE *file = fopen(path, "r");
    if (file == NULL) {
        printf("Error opening file %s\n", path);
        return;
    }

    char line[1024];
    if (fgets(line, 1024, file) == NULL) {
        printf("Error reading header row\n");
        fclose(file);
        return;
    }

    // Read the subsequent rows
    while (fgets(line, 1024, file) != NULL) {
        // Parse the values in the row
        char *token = strtok(line, ",");
        for (int i = 0; i < 15; i++) {
            if (token == NULL) {
                printf("Error parsing row: not enough values\n");
                fclose(file);
                return;
            }
            float offset;
            if (sscanf(token, "%f", &offset) != 1) {
                // handle error
                printf("failed to parse value %d\n", i);
            } else {
                offsets[i] = offset;
            }
            token = strtok(NULL, ",");
        }
    }
    printf("loaded imu calibration: %.4f %.4f %.4f \n", offsets[0], offsets[1], offsets[2]);

    fclose(file);

}

bool useSim = false;

/* -------------------------------------------------------------
 *  Main program
 * ------------------------------------------------------------- */
int main(int argc, char **argv) {

    if(argc > 1){
        for(int i=0; i < argc; i++){
            printf("arg: %d = %s\n", i, argv[i]);
            if(strcmp(argv[i], "-useSim") == 0){
                if(strcmp(argv[i+1], "true") == 0){
                    useSim = true;
                } else {
                    useSim = false;
                }
            }
        }
        
    }

    struct sigaction sa;

    /* ----- Install the handler for SIGINT (Ctrl‑C) ----- */
    sa.sa_handler = sigint_handler;      /* our tiny handler */
    sigemptyset(&sa.sa_mask);            /* don’t block other signals while handling */
    sa.sa_flags = 0;                     /* no special flags – SA_RESTART is optional */

    if (sigaction(SIGINT, &sa, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }

    /* Register cleanup to run on normal exit (optional but handy) */
    if (atexit(cleanup) != 0) {
        perror("atexit");
        /* Not fatal – continue anyway */
    }
    
    /* ----- IMU setup ----- */
    float imu_offsets[15] = {0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0};
    loadIMUCalibration(imu_offsets, "/home/rktman/imu_calibration.csv");
    
    detectIMU();
	enableIMU();

    /* ---------- X11 / GLX setup ---------- */
    Display *dpy = XOpenDisplay(NULL);
    if (!dpy) {
        fprintf(stderr, "Cannot open X display\n");
        return EXIT_FAILURE;
    }

    int screen = DefaultScreen(dpy);
    Window root   = RootWindow(dpy, screen);

    /* Choose a visual supporting OpenGL */
    int attribs[] = {
        GLX_RGBA,
        GLX_DOUBLEBUFFER,
        GLX_DEPTH_SIZE, 24,
        None
    };
    XVisualInfo *vi = glXChooseVisual(dpy, screen, attribs);
    if (!vi) {
        fprintf(stderr, "No appropriate GLX visual found\n");
        return EXIT_FAILURE;
    }

    Colormap cmap = XCreateColormap(dpy, root, vi->visual, AllocNone);
    XSetWindowAttributes swa;
    swa.colormap = cmap;
    swa.event_mask = ExposureMask | KeyPressMask | StructureNotifyMask;

    Window win = XCreateWindow(dpy, root,
                               0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, 0,
                               vi->depth, InputOutput,
                               vi->visual,
                               CWColormap | CWEventMask, &swa);
    XStoreName(dpy, win, "X11 | OpenGL Cube | simulated 9-DOF IMU");
    XMapWindow(dpy, win);

    /* Create GLX context */
    GLXContext ctx = glXCreateContext(dpy, vi, NULL, GL_TRUE);
    glXMakeCurrent(dpy, win, ctx);

    /* Enable depth testing */
    glEnable(GL_DEPTH_TEST);

    /* Projection matrix (simple perspective) */
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, WINDOW_RATIO, 0.1, 100.0);
    // OR just use this and don't include GLU.h
    //set_perspective(45.0, WINDOW_RATIO, 0.1, 100.0);

    /* -------------------------------------------------
     *  Main loop variables
     * ------------------------------------------------- */
    struct timespec last_ts;
    clock_gettime(CLOCK_MONOTONIC, &last_ts);

    quat orientation = {1.0, 0.0, 0.0, 0.0};   // identity quaternion
    
    int m_accRaw[3] = {0, 0, 0};
    int m_gyrRaw[3] = {0, 0, 0};
    int m_magRaw[3] = {0, 0, 0};
    // imu calibration offsets
    float m_gyroX_bias = imu_offsets[0];
    float m_gyroY_bias = imu_offsets[1];
    float m_gyroZ_bias = imu_offsets[2];
    float m_rate_gyr_x = 0.0;
    float m_rate_gyr_y = 0.0;
    float m_rate_gyr_z = 0.0;
    float m_gyroXangle = 0.0;
    float m_gyroYangle = 0.0;
    float m_gyroZangle = 0.0;
    
    /* ----------------------------------------------------------------
     * 1 Initialise the Madgwick filter.
     *
     *    • β (beta) controls the trade‑off between responsiveness and
     *      smoothness. 0.1–0.3 works well for most hobby IMUs.
     *    • Start with the identity quaternion (no rotation).
     * ---------------------------------------------------------------- */
    //const float beta = 0.2f;               // tweak as needed
    //madgwick_init(beta, 1.0f, 0.0f, 0.0f, 0.0f);
    Madgwick filter;
    filter.begin(25);

    /* ----------------------------------------------------------------
     * 2️ Define a target orientation you want to compare against.
     *
     *    For illustration we use a 45deg rotation around the Z‑axis.
     *    Quaternion for a rotation θ about axis (x,y,z) is:
     *        q = [cos(θ/2), sin(θ/2)*x, sin(θ/2)*y, sin(θ/2)*z]
     * ---------------------------------------------------------------- */
    const float target_angle_rad = M_PI/4.0f;   // 45 deg
    const float half = target_angle_rad/2.0f;
    const float target_q[4] = {
        cosf(half),          // w
        0.0f,                // x
        0.0f,                // y
        sinf(half)           // z
    };
    
    /* ----------------------------------------------------------------
     * 3 Main acquisition / processing loop.
     *
     *    In a real embedded program you would likely run this at a
     *    fixed rate (e.g., 200 Hz).  Here we simply use a constant
     *    `dt` and sleep a bit to keep the demo readable.
     * ---------------------------------------------------------------- */
    //const float dt = 0.01f;               // 100 Hz sample period 
    const float tolerance_deg = 10.0f;    // how close we consider “aligned”

    printf("Starting IMU demo – press Ctrl‑C to stop.\n");
    printf("Target orientation: 45° yaw (Z‑axis).\n\n");
    

    while (!got_sigint) {
        /* ----- Handle X events ----- */
        while (XPending(dpy)) {
            XEvent ev;
            XNextEvent(dpy, &ev);
            if (ev.type == Expose) {
                /* nothing special – redraw on next iteration */
            } else if (ev.type == ConfigureNotify) {
                // resize
                XConfigureEvent ce = ev.xconfigure;
                glViewport(0, 0, ce.width, ce.height);
                glMatrixMode(GL_PROJECTION);
                glLoadIdentity();
                gluPerspective(45.0, (double)ce.width/ce.height, 0.1, 100.0);
            } else if (ev.type == KeyPress) {
                // Escape key quits
                char buf[32];
                KeySym ks;
                XLookupString(&ev.xkey, buf, sizeof(buf), &ks, NULL);
                if (ks == XK_Escape) got_sigint = 1;
            }
        }

        /* ----- Time step ----- */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double dt = (now.tv_sec - last_ts.tv_sec) +
                    (now.tv_nsec - last_ts.tv_nsec) * 1e-9;
        last_ts = now;

        double half_dt = 0.5 * dt;
        /* ----- Simulate IMU ----- */
        if(useSim) {
            double gyro[3], accel[3], mag[3];
            double t = now.tv_sec + now.tv_nsec * 1e-9;
            simulate_imu(t, gyro, accel, mag);

            /* Integrate gyro rates into quaternion.
             * For small dt we can treat angular velocity as a rotation vector.
             */
            
            quat delta = {
                1.0,
                gyro[0]*half_dt,
                gyro[1]*half_dt,
                gyro[2]*half_dt
            };
            orientation = quat_mul(orientation, delta);
            quat_normalize(&orientation);
        } else {
        
            /* ----- read IMU ----- */
            // Read raw gyro data
            readACC(m_accRaw);
            readGYR(m_gyrRaw);
            readMAG(m_magRaw);
            
            m_rate_gyr_x = (m_gyrRaw[0] - m_gyroX_bias) * G_GAIN;
			m_rate_gyr_y = (m_gyrRaw[1] - m_gyroY_bias) * G_GAIN;
			m_rate_gyr_z = (m_gyrRaw[2] - m_gyroZ_bias) * G_GAIN;
            
            //printf("x: %.3f, y: %.3f, z: %.3f, \n", m_rate_gyr_x, m_rate_gyr_y, m_rate_gyr_z);
            //printf("x: %d, y: %d, z: %d, \n", m_gyrRaw[0], m_gyrRaw[1], m_gyrRaw[2]);
            printf("x: %.3f, y: %.3f, z: %.3f, \n", (m_gyrRaw[0] - m_gyroX_bias), (m_gyrRaw[1] - m_gyroY_bias), (m_gyrRaw[2] - m_gyroZ_bias));
            
            m_gyroXangle += m_rate_gyr_x * dt;
			m_gyroYangle += m_rate_gyr_y * dt;
			m_gyroZangle += m_rate_gyr_z * dt;

			//data.roll_rate = m_gyroYangle;
			//data.pitch_rate = m_gyroXangle;
			//data.yaw_rate = m_gyroZangle;
            
            // Feed data to the filter
            filter.update(
                m_rate_gyr_x, m_rate_gyr_y, m_rate_gyr_z,
                m_accRaw[0], m_accRaw[1], m_accRaw[2],
                m_magRaw[0], m_magRaw[1], m_magRaw[2]
            );
            //madgwick_update(
            //    m_rate_gyr_x, m_rate_gyr_y, m_rate_gyr_z,
            //    m_accRaw[0], m_accRaw[1], m_accRaw[2],
            //    m_magRaw[0], m_magRaw[1], m_magRaw[2],
            //    dt
            //);

            // Pull out the current quaternion
            float cur_q[4] = {filter.getQ0(), filter.getQ1(), filter.getQ2(), filter.getQ3()};
            printf("p: %f, r: %f, y: %f\n", filter.getPitch(), filter.getRoll(), filter.getYaw());
            //madgwick_get_quaternion(&cur_q[0], &cur_q[1], &cur_q[2], &cur_q[3]);

            /* ---- Compare with the target orientation ------------------------ */
            float angle_to_target;
            bool aligned = look_at_compare(cur_q, target_q,
                                          tolerance_deg, &angle_to_target);

            /* ---- Print a concise status line -------------------------------- */
            if(0){
            printf("\rAngle to target: %6.2f°  |  %s \n",
                   angle_to_target,
                   aligned ? "ALIGNED   " : "not aligned");
               }
            
            //quat delta = {
            //    1.0,
            //    m_gyroXangle,
            //    m_gyroYangle,
            //    m_gyroZangle
            //};
            //orientation = quat_mul(orientation, delta);
            orientation.w = cur_q[0];
            orientation.x = cur_q[1];
            orientation.y = cur_q[2];
            orientation.z = cur_q[3];
            quat_normalize(&orientation);
        }

        /* ----- Render ----- */
        glClearColor(0.07f, 0.07f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0.0f, 0.0f, -3.0f);   // pull back camera

        /* Apply orientation from quaternion */
        double mat[16];
        quat_to_mat4(orientation, mat);
        glMultMatrixd(mat);

        draw_wire_cube();

        glXSwapBuffers(dpy, win);

        /* ----- Sleep a bit to keep CPU usage reasonable ----- */
        struct timespec ts = {.tv_sec = 0, .tv_nsec = 16000000}; // ~60 fps
        nanosleep(&ts, NULL);
    }

    /* Cleanup */
    glXMakeCurrent(dpy, None, NULL);
    glXDestroyContext(dpy, ctx);
    XDestroyWindow(dpy, win);
    XCloseDisplay(dpy);
    return EXIT_SUCCESS;
}
