#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

void init_mpu_migration(long _axo, long _ayo, long _azo, long _gxo, long _gyo, long _gzo);
void kalman_filter_mpu_data(float ax, float ay, float az, float gx, float gy, float gz);
#endif // KALMAN_FILTER_H_
