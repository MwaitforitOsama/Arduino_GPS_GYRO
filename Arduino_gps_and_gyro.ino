#include <Wire.h>
#include <Kalman.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <MadgwickAHRS.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
Kalman_h kalmanX;
Kalman_h kalmanY;
Madgwick filter;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  filter.begin(100);

  // Set up the Kalman filters with some initial values
  kalmanX.setState(0, 0, 0, 0);
  kalmanY.setState(0, 0, 0, 0);
}

void loop() {
  // Read raw sensor data from the MPU6050
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Use the Madgwick filter to calculate orientation quaternions
  filter.updateIMU(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);

  // Use the TinyGPS++ library to read GPS data
  while (Serial.available()) {
    gps.encode(Serial.read());
  }

  // Apply Kalman filter on GPS data to reduce noise
  double latitude = kalmanX.updateEstimate(gps.location.lat());
  double longitude = kalmanY.updateEstimate(gps.location.lng());

  // Print filtered GPS data and orientation quaternions
  Serial.print("Latitude: ");
  Serial.print(latitude);
  Serial.print(" Longitude: ");
  Serial.println(longitude);
  Serial.print("Orientation Quaternions: x=");
  Serial.print(filter.q0);
  Serial.print(" y=");
  Serial.print(filter.q1);
  Serial.print(" z=");
  Serial.print(filter.q2);
  Serial.print(" w=");
  Serial.println(filter.q3);

  delay(100);
}
