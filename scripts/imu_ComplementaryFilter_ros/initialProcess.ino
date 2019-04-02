float sum_gx = 0;
float sum_gy = 0;
float sum_gz = 0;


void init_gyro_process() {
  for (int i = 0; i < NUM_OF_SAMPLES_FOR_INIT; i++) {
    // wait for new data available
    while (imu_LSM9DS1.gyroAvailable() != 1) {
    }
    imu_LSM9DS1.readGyro();
    sum_gx += imu_LSM9DS1.gx;
    sum_gy += imu_LSM9DS1.gy;
    sum_gz += imu_LSM9DS1.gz;
  }

  offset_gx = sum_gx / NUM_OF_SAMPLES_FOR_INIT;
  offset_gy = sum_gy / NUM_OF_SAMPLES_FOR_INIT;
  offset_gz = sum_gz / NUM_OF_SAMPLES_FOR_INIT;
}
