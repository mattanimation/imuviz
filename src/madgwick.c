#include "madgwick.h"
#include <math.h>

/* -----------------------------------------------------------------------
 * Internal state – a single quaternion and the gain β.
 * ----------------------------------------------------------------------- */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float beta = 0.1f;

/* Helper: normalise a 3‑vector */
static void normalize3(float *x, float *y, float *z)
{
    float norm = sqrtf((*x)*(*x) + (*y)*(*y) + (*z)*(*z));
    if (norm == 0.0f) return;          // avoid division by zero
    *x /= norm; *y /= norm; *z /= norm;
}

/* ----------------------------------------------------------------------- */
void madgwick_init(float b,
                   float init_q0, float init_q1,
                   float init_q2, float init_q3)
{
    beta = b;
    q0 = init_q0; q1 = init_q1; q2 = init_q2; q3 = init_q3;
}

/* ----------------------------------------------------------------------- */
/* Update when you have all three sensors (full AHRS) */
void madgwick_update(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz,
                     float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float hx, hy, _2bx, _2bz;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx;
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q0q2 = 2.0f * q0 * q2;
    float _2q2q3 = 2.0f * q2 * q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    /* ----------- Normalise accelerometer measurement -------------------- */
    normalize3(&ax, &ay, &az);
    if (ax == 0 && ay == 0 && az == 0) return;   // invalid data

    /* ----------- Normalise magnetometer measurement ------------------- */
    normalize3(&mx, &my, &mz);
    if (mx == 0 && my == 0 && mz == 0) {
        /* Magnetometer failed – fall back to IMU‑only update */
        madgwick_update_imu(gx, gy, gz, ax, ay, az, dt);
        return;
    }

    /* ----------- Reference direction of Earth's magnetic field --------- */
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1
         + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2
         - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = 2.0f * ( _2q0mx * q2 + _2q0my * q1 - mz * q0q0
                 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3
                 - mz * q2q2 + mz * q3q3 );

    /* ----------- Gradient descent algorithm corrective step ------------ */
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax)
         + _2q1 * (2.0f * q0q1 + _2q2q3 - ay)
         - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3)
                       + _2bz * (q1q3 - q0q2) - mx)
         + (-_2bx * q3 + _2bz * q1) *
           (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
         + _2bx * q2 *
           (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 =  _2q3 * (2.0f * q1q3 - _2q0q2 - ax)
         + _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
         - 4.0f * q1 *
           (1 - 2.0f * (q1q1 + q2q2) - az)
         + _2bz * q3 *
           (_2bx * (0.5f - q2q2 - q3q3)
            + _2bz * (q1q3 - q0q2) - mx)
         + (_2bx * q2 + _2bz * q0) *
           (_2bx * (q1q2 - q0q3)
            + _2bz * (q0q1 + q2q3) - my)
         + (_2bx * q3 - _4bz * q1) *
           (_2bx * (q0q2 + q1q3)
            + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax)
         + _2q3 * (2.0f * q0q1 + _2q2q3 - ay)
         - 4.0f * q2 *
           (1 - 2.0f * (q1q1 + q2q2) - az)
         + (-_4bx * q2 - _2bz * q0) *
           (_2bx * (0.5f - q2q2 - q3q3)
            + _2bz * (q1q3 - q0q2) - mx)
         + (_2bx * q1 + _2bz * q3) *
           (_2bx * (q1q2 - q0q3)
            + _2bz * (q0q1 + q2q3) - my)
         + (_2bx * q0 - _4bz * q2) *
           (_2bx * (q0q2 + q1q3)
            + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 =  _2q1 * (2.0f * q1q3 - _2q0q2 - ax)
         + _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
         + (-_4bx * q3 + _2bz * q1) *
           (_2bx * (0.5f - q2q2 - q3q3)
            + _2bz * (q1q3 - q0q2) - mx)
         + (-_2bx * q0 + _2bz * q2) *
           (_2bx * (q1q2 - q0q3)
            + _2bz * (q0q1 + q2q3) - my)
         + _2bx * q1 *
           (_2bx * (q0q2 + q1q3)
            + _2bz * (0.5f - q1q1 - q2q2) - mz);

    /* Normalise step magnitude */
    recipNorm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    /* ----------- Apply feedback step ---------------------------------- */
    /* Rate of change of quaternion from gyroscope */
    float qDot0 = 0.5f*(-q1*gx - q2*gy - q3*gz) - beta*s0;
    float qDot1 = 0.5f*( q0*gx + q2*gz - q3*gy) - beta*s1;
    float qDot2 = 0.5f*( q0*gy - q1*gz + q3*gx) - beta*s2;
    float qDot3 = 0.5f*( q0*gz + q1*gy - q2*gx) - beta*s3;

    /* Integrate to yield new quaternion */
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    /* Normalise quaternion */
    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

/* ----------------------------------------------------------------------- */
/* IMU‑only version (no magnetometer) – useful if you only have accel+gyro */
void madgwick_update_imu(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    /* Normalise accelerometer */
    normalize3(&ax, &ay, &az);
    if (ax == 0 && ay == 0 && az == 0) return;   // invalid

    /* Gradient decent algorithm corrective step */
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay
         - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
         - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    recipNorm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    /* Quaternion rate from gyroscope */
    float qDot0 = 0.5f*(-q1*gx - q2*gy - q3*gz) - beta*s0;
    float qDot1 = 0.5f*( q0*gx + q2*gz - q3*gy) - beta*s1;
    float qDot2 = 0.5f*( q0*gy - q1*gz + q3*gx) - beta*s2;
    float qDot3 = 0.5f*( q0*gz + q1*gy - q2*gx) - beta*s3;

    /* Integrate */
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    /* Normalise quaternion */
    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

/* ----------------------------------------------------------------------- */
void madgwick_get_quaternion(float *out_q0, float *out_q1,
                             float *out_q2, float *out_q3)
{
    if (out_q0) *out_q0 = q0;
    if (out_q1) *out_q1 = q1;
    if (out_q2) *out_q2 = q2;
    if (out_q3) *out_q3 = q3;
}
