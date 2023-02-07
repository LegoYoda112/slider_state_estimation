// Basic demo for accelerometer & gyro readings from Adafruit
// LSM6DSOX sensor

#include <Adafruit_LSM6DSOX.h>

// For SPI mode, we need a CS pin
#define LSM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

double roll = 0;
double pitch = 0;
double yaw = 0;

double roll_offset = 0;
double pitch_offset = 0;
double yaw_offset = 0;

Adafruit_LSM6DSOX sox;

struct Quaternion{
  double w;
  double x;
  double y;
  double z;
};

struct Vector{
  double x;
  double y;
  double z;
};

struct Quaternion scale_quaternion(struct Quaternion quat, double factor){
  struct Quaternion scaled;
  scaled.w = quat.w * factor;
  scaled.x = quat.x * factor;
  scaled.y = quat.y * factor;
  scaled.z = quat.z * factor;

  return scaled;
}

struct Quaternion lerp_quaternion(struct Quaternion quat1, struct Quaternion quat2, double factor){
  struct Quaternion result;
  result.w = quat1.w * factor + quat2.w * (1 - factor);
  result.x = quat1.x * factor + quat2.x * (1 - factor);
  result.y = quat1.y * factor + quat2.y * (1 - factor);
  result.z = quat1.z * factor + quat2.z * (1 - factor);

  return result;
}

struct Quaternion invert_quaternion(struct Quaternion quat){
  struct Quaternion inverse;
  inverse.w = quat.w;
  inverse.x = - quat.x;
  inverse.y = - quat.y;
  inverse.z =  - quat.z;

  return inverse;
}

struct Quaternion multiply_quaternion(struct Quaternion quat1, struct Quaternion quat2){
  struct Quaternion multiplied;
  multiplied.w = quat1.w*quat2.w - quat1.x*quat2.x - quat1.y*quat2.y - quat1.z*quat2.z;
  multiplied.x = quat1.w*quat2.x + quat1.x*quat2.w + quat1.y*quat2.z - quat1.z*quat2.y;
  multiplied.y = quat1.w*quat2.y + quat1.y*quat2.w + quat1.z*quat2.x - quat1.x*quat2.z;
  multiplied.z = quat1.w*quat2.z + quat1.z*quat2.w + quat1.x*quat2.y - quat1.y*quat2.x;

  return multiplied;
}

struct Vector cross_vector(struct Vector v1, struct Vector v2){
  struct Vector crossed;
  crossed.x = v1.y * v2.z - v1.z * v2.y;
  crossed.y = v1.z * v2.x - v1.x * v2.z;
  crossed.z = v1.x * v2.y - v1.y * v2.x;
  return crossed;
}

double dot_vector(struct Vector v1, struct Vector v2){
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

double norm_vector(struct Vector v){
  return sqrt(pow(v.x, 2) +
              pow(v.y, 2) +
              pow(v.z, 2));
}

struct Quaternion sum_quaternion(struct Quaternion quat1, struct Quaternion quat2){
  struct Quaternion summed;


  summed.w = quat1.w + quat2.w;
  summed.x = quat1.x + quat2.x;
  summed.y = quat1.y + quat2.y;
  summed.z = quat1.z + quat2.z;

  return summed;
}

struct Quaternion normalize_quaternion(struct Quaternion quat){
  double mag = sqrt(pow(quat.x, 2) +
                    pow(quat.y, 2) +
                    pow(quat.z, 2) +
                    pow(quat.w, 2));
                    
  struct Quaternion normalized;

  normalized.w = quat.w / mag;
  normalized.x = quat.x / mag;
  normalized.y = quat.y / mag;
  normalized.z = quat.z / mag;

  return normalized;
} 
struct Quaternion heading_estimate;
struct Quaternion angular_velocity_estimate;
struct Quaternion gravity_direction;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LSM6DSOX test!");

  if (!sox.begin_I2C()) {
    // if (!sox.begin_SPI(LSM_CS)) {
    // if (!sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    // Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DSOX Found!");

  // sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (sox.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  Serial.print("Gyro range set to: ");
  switch (sox.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DSOX
  }

  // sox.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (sox.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  sox.setGyroDataRate(LSM6DS_RATE_208_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (sox.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  int sample_count = 1;
  for(int i = 0; i < sample_count; i++){
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    sox.getEvent(&accel, &gyro, &temp);

    roll_offset += gyro.gyro.y;
    pitch_offset += gyro.gyro.x;
    yaw_offset += gyro.gyro.z;
    delay(10);
  }

  roll_offset = roll_offset / (float(sample_count) * 5.0);
  pitch_offset = pitch_offset / (float(sample_count) * 5.0);
  yaw_offset = yaw_offset / (float(sample_count) * 5.0);

  Serial.println("Finished calibration");
  Serial.print(roll_offset);
  Serial.print(" ");
  Serial.print(pitch_offset);
  Serial.print(" ");
  Serial.print(yaw_offset);
  Serial.println(" ");

  roll_offset = 0.0;
  pitch_offset = 0.0;
  yaw_offset = 0.0;

  // Init estimators
  heading_estimate.w = 1.0;
  heading_estimate.x = 0.0;
  heading_estimate.y = 0.0;
  heading_estimate.z = 0.0;
}

long start_millis;
void loop() {
  // long currentMillis = millis();
  double dt_seconds = 0.016;

  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);

  struct Quaternion angular_velocity_local;
  angular_velocity_local.w = 0;
  angular_velocity_local.x = -gyro.gyro.x;
  angular_velocity_local.y = -gyro.gyro.y;
  angular_velocity_local.z = gyro.gyro.z;

  struct Quaternion angular_velocity_global;
  angular_velocity_global = multiply_quaternion( multiply_quaternion(heading_estimate, angular_velocity_local), invert_quaternion(heading_estimate));

  struct Quaternion heading_dot;
  heading_dot = scale_quaternion(multiply_quaternion(angular_velocity_global, heading_estimate), 0.5);

  // Calculate gravity quaternion
  struct Quaternion gravity_orientation;

  struct Vector up;
  up.x = 0;
  up.y = 0;
  up.z = 1;

  struct Vector gravity_vector;
  gravity_vector.x = accel.acceleration.x;
  gravity_vector.y = accel.acceleration.y;
  gravity_vector.z = accel.acceleration.z;

  struct Vector v_cross = cross_vector(up, gravity_vector);

  gravity_orientation.x = v_cross.x;
  gravity_orientation.y = v_cross.y;
  gravity_orientation.z = v_cross.z;
  gravity_orientation.w = (norm_vector(up) * norm_vector(gravity_vector)) + dot_vector(up, gravity_vector);

  gravity_orientation = normalize_quaternion(gravity_orientation);

  // GYRO PREDICTION
  heading_estimate = sum_quaternion(heading_estimate, scale_quaternion(heading_dot, dt_seconds));
  heading_estimate = normalize_quaternion(heading_estimate);

  // ACCELEROMETER CORRECTION
  heading_estimate = lerp_quaternion(heading_estimate, gravity_orientation, 0.99);


  // Send heading referance
  Serial.print(String(heading_estimate.w, 3));
  Serial.print(" ");
  Serial.print(String(heading_estimate.x, 3));
  Serial.print(" ");
  Serial.print(String(heading_estimate.y, 3));
  Serial.print(" ");
  Serial.print(String(heading_estimate.z, 3));

  // Send gyro
  Serial.print(" ");
  Serial.print(String(gyro.gyro.x, 3));
  Serial.print(" ");
  Serial.print(String(gyro.gyro.y, 3));
  Serial.print(" ");
  Serial.print(String(gyro.gyro.z, 3));
  
  // Send acceleration
  Serial.print(" ");
  Serial.print(String(accel.acceleration.x, 3));
  Serial.print(" ");
  Serial.print(String(accel.acceleration.y, 3));
  Serial.print(" ");
  Serial.print(String(accel.acceleration.z, 3));

  Serial.println();

  delay(10);

  // Serial.println(millis() - currentMillis);
}