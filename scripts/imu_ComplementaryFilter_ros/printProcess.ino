void print_time() {
  present_time = micros();
  passed_time = present_time - previous_time;
  Serial.println(passed_time);
  previous_time = present_time;
}

void print_posture() {
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.println(heading);
  //  Serial.println(k);
}


void print_accel() {
  Serial.print(accelX);
  Serial.print(",");
  Serial.print(accelY);
  Serial.print(",");
  Serial.println(accelZ);
  //  Serial.println(k);
}

void print_gyro() {
  Serial.print(gyroX);
  Serial.print(",");
  Serial.print(gyroY);
  Serial.print(",");
  Serial.println(gyroZ, 4);
}
