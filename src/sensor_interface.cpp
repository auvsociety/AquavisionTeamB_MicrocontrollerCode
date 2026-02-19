#include "sensor_interface.hpp"

#include "config.hpp"

long reportIntervalUs = 50;                           //Update Interval

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

void initializeIMU() {
  // mag.begin();
  // gyro.begin();
  // gyro.setAccelerometerRange(ACCELERO_METER_RANGE_2);
  // gyro.setGyroscopeRange(GYROSCOPE_RANGE_250);
  // gyro.setSampleRateDivider(0);
  // gyro.disableSleepMode();

  bno08x.enableReport(SH2_ROTATION_VECTOR, reportIntervalUs); //Good Calibrated YPR
  // bno08x.enableReport(SH2_LINEAR_ACCELERATION, reportIntervalUs);//Linear Accel
  // bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, reportIntervalUs);//Gy
  // bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, reportIntervalUs);//Mag

}



void initializeDepthSensor() {
  bool depth_sensor_status = depth_sensor.init();
  // depth_sensor.setModel(MS5837::MS5837_30BA);
  depth_sensor.setFluidDensity(997);  // kg/m^3 (freshwater, 1029 for seawater)
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      if (ypr->yaw <0) ypr->yaw +=360.0; //mapping negative values
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void updateIMUReadings(float& ax, float& ay, float& az, float& gx, float& gy,
                       float& gz, float& mx, float& my, float& mz) {
  // sensors_event_t accel_event, mag_event;
  // mag.getEvent( &mag_event);
  // float Ax, Ay, Gx, Gy;
  // gyro.getSensorsReadings(Ax, Ay, az, Gx, Gy, gz);
  // ax = -Ay / G;
  // ay = Ax / G;
  // az = az / G;
  // gx = Gy;
  // gy = -Gx;

  // mx = mag_event.magnetic.x;
  // my = mag_event.magnetic.y;
  // mz = mag_event.magnetic.z;

  sh2_SensorValue_t sensorValue;

  // Serial.print("Linear Acceration - x: ");
  // Serial.print(sensorValue.un.linearAcceleration.x);
  // Serial.print(" y: ");
  // Serial.print(sensorValue.un.linearAcceleration.y);
  // Serial.print(" z: ");
  // Serial.println(sensorValue.un.linearAcceleration.z);

  ax = sensorValue.un.linearAcceleration.x;
  ay = sensorValue.un.linearAcceleration.y;
  az = sensorValue.un.linearAcceleration.z;

  // Serial.print("Gyro - x: ");
  // Serial.print(sensorValue.un.gyroscope.x);
  // Serial.print(" y: ");
  // Serial.print(sensorValue.un.gyroscope.y);
  // Serial.print(" z: ");
  // Serial.println(sensorValue.un.gyroscope.z);

  gx = sensorValue.un.gyroscope.x;
  gy = sensorValue.un.gyroscope.y;
  gz = sensorValue.un.gyroscope.z;

  // Serial.print("Magnetic Field - x: ");
  // Serial.print(sensorValue.un.magneticField.x);
  // Serial.print(" y: ");
  // Serial.print(sensorValue.un.magneticField.y);
  // Serial.print(" z: ");
  // Serial.println(sensorValue.un.magneticField.z);

  mx = sensorValue.un.magneticField.x;
  my = sensorValue.un.magneticField.y;
  mz = sensorValue.un.magneticField.z;

}
// void setCalibrationMode(bool &calib_status){
//   bool calibration_mode = calib_status;
// }

// void callUpdateOffset(bool& calibration_mode) {
//   if (calibration_mode) {
//     updateOffset(gyro);
//   }
// }

void updateOrientation(float &roll,float &pitch, float &yaw){
  sh2_SensorValue_t sensorValue;

  if (bno08x.getSensorEvent(&sensorValue)) {
    // sh2_RotationVectorWAcc_t* rotational_vector = &sensorValue.un.rotationVector;
    // quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, &ypr, true);
    quaternionToEulerRV(&sensorValue.un.rotationVector, &ypr, true);

    Serial.print(sensorValue.status);     Serial.print("\t Yaw:");  // This is accuracy in the range of 0 to 3
    Serial.print(ypr.yaw);                Serial.print("\t Pitch:");
    Serial.print(ypr.pitch);              Serial.print("\t Roll");
    Serial.println(ypr.roll);
  }

  roll = ypr.roll;
  pitch = ypr.pitch;
  yaw = ypr.yaw;

}

void updateDepthSensorReadings(float& depth) {
  depth_sensor.read();
  depth = depth_sensor.depth();
}

void getAccuracy(int &accuracy){
  sh2_SensorValue_t sensorValue;
  accuracy = sensorValue.status;
}
