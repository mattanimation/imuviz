#ifndef MADGWICK_H
#define MADGWICK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Public API -------------------------------------------------------------- */

/* Initialise the filter.
 *   beta      – algorithm gain (typical 0.1 … 0.5, lower = smoother)
 *   q0,q1,q2,q3 – initial quaternion (usually identity: 1,0,0,0)
 */
void madgwick_init(float beta,
                   float q0, float q1, float q2, float q3);

/* Update the filter with a new set of sensor samples.
 *   gx,gy,gz – gyroscope in rad/s (body frame)
 *   ax,ay,az – accelerometer in g (or any consistent unit)
 *   mx,my,mz – magnetometer in µT (or any consistent unit)
 *   dt       – sample period in seconds
 *
 * After the call the internal quaternion holds the latest orientation.
 */
void madgwick_update_imu(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float dt);               // use if you have no mag

void madgwick_update(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz,
                     float dt);

/* Retrieve the current quaternion (w,x,y,z). */
void madgwick_get_quaternion(float *q0, float *q1,
                             float *q2, float *q3);

#ifdef __cplusplus
}
#endif
#endif /* MADGWICK_H */
