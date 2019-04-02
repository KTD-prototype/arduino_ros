

void get_IMU_data() {
  imu_LSM9DS1.readGyro();
  imu_LSM9DS1.readAccel();
  gyroX = imu_LSM9DS1.calcGyro(imu_LSM9DS1.gx - offset_gx);
  gyroY = imu_LSM9DS1.calcGyro(imu_LSM9DS1.gy - offset_gy);
  gyroZ = imu_LSM9DS1.calcGyro(imu_LSM9DS1.gz - offset_gz);
  accelX = imu_LSM9DS1.calcAccel(imu_LSM9DS1.ax);
  accelY = imu_LSM9DS1.calcAccel(imu_LSM9DS1.ay);
  accelZ = imu_LSM9DS1.calcAccel(imu_LSM9DS1.az);
  magX = imu_LSM9DS1.calcMag(imu_LSM9DS1.mx);
  magY = imu_LSM9DS1.calcMag(imu_LSM9DS1.my);
  magZ = imu_LSM9DS1.calcMag(imu_LSM9DS1.mz);
}

void normarize_gyroZ() {
  if (gyroZ == 0.0700 || gyroZ == 0.1400 || gyroZ == 0.2800 || gyroZ == -0.0700 || gyroZ == -0.1400 || gyroZ == -0.2800) {
    gyroZ = 0;
  }
}

void get_posture_complementary_filter() {
  g = sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2));
  //  k = 0.05 - (g - 1) / 100;
  //  if (k < 0) {
  //    k = 0;
  //  }
  k = 0.1 * pow(65536, -1 * (pow((1 - g), 2) / C));
  ACCroll = 0.9 * ACCroll + 0.1 * (atan2(accelY,  accelZ) * 180 / M_PI);
  ACCpitch = 0.9 * ACCpitch + 0.1 * (atan2(accelX, sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / M_PI);

  //calculate pitch/roll/yaw by gyrosensor (About pitch & roll, using Complementary Filter)
  //(とりあえずピッチのみ）角速度を相補フィルタリング後の値から再計算する。
  roll = (1 - k) * (roll + gyroX  / SAMPLING_RATE) + k * ACCroll;
  pitch = (1 - k) * (pitch + (-1 * gyroY) / SAMPLING_RATE) + k * ACCpitch; //ピッチの角速度データが正負逆で入っているようだ。

  if (abs(gyroZ) < 0.01) {
    heading = heading;
  }
  else heading = heading + gyroZ / SAMPLING_RATE;

  if (heading > 180) {
    heading = -360 + heading;
  }
  else if (heading < -180) {
    heading = 360 + heading;
  }
}
