#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

elapsedMillis timeMillis;
elapsedMillis secondaryTimeMillis;
elapsedMillis mpuTimeMillis;

Servo ESC;     // create servo object to control the ESC
const int hallEffectSensorPin = 14;
const int stopValue = 90;
const int goValue = 150;
volatile int hallEffectCounter = 0;
int revolutions = 0;
int numGos = 0;

void setup() {
  Serial.begin(9600);
  delay(3000); // wait for serial without requiring it

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    int numMpuTries = 0;

    // retry up to thirty times in 3 seconds, loop forever if still failing
    while (1) {
      delay(100);
      numMpuTries++;
      if (numMpuTries <= 30) {
        if (mpu.begin()) {
          Serial.println("Retried and found MPU6050 chip");
          break;
        }
      }
    }
  }

  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  pinMode(hallEffectSensorPin, INPUT);
  ESC.attach(24,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  delay(1000);

  timeMillis = 0;
  secondaryTimeMillis = 0;
  mpuTimeMillis = 0;
}

// we use two thresholds to ensure we have the right direction of change, and
// they are far enough apart that noise in readings shouldn't mess up our logic
const int HALL_THRESHOLD1 = 450;
const int HALL_THRESHOLD2 = 550;
bool reachedThreshold1 = false;
bool reachedThreshold2 = false;
bool goingUp = false;

// 0 == init state
// 1 == go state
// 2 == stop state
int motorState = 0;

void loop() {
  //pot.update();
  //int value = map(pot.getValue(), 0, 1024, 90, 130);
  //if(value != lastValue) {
		//Serial.println(value);
    //lastValue = value;
  //}

  if (motorState == 0) {
    // init
    ESC.write(stopValue);
    //Serial.println("init");
    if (secondaryTimeMillis > 3000) {
      //Serial.println("init done");
      motorState = 1;
      secondaryTimeMillis = 0;
    }
  } else if (motorState == 1) {
    // go
    ESC.write(goValue);
    if (secondaryTimeMillis > 250) {
      //Serial.println("go done");
      motorState = 2;
      secondaryTimeMillis = 0;
    }
  } else {
    // stop
    ESC.write(stopValue);
    if (secondaryTimeMillis > 250 && numGos < 7200) {
      numGos++;
      motorState = 1;
      secondaryTimeMillis = 0;
    }
  }

  const int hallSensorValue = analogRead(hallEffectSensorPin);
  if (goingUp) {
    if (hallSensorValue > HALL_THRESHOLD2) {
      reachedThreshold2 = true;
    } else if (hallSensorValue > HALL_THRESHOLD1) {
      reachedThreshold1 = true;
      reachedThreshold2 = false;
    }

    if (reachedThreshold1 && reachedThreshold2) {
      revolutions++;
      reachedThreshold1 = false;
      reachedThreshold2 = false;
      goingUp = false;
    }
  } else {
    if (hallSensorValue < HALL_THRESHOLD1) {
      goingUp = true;
    }
  }

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.println(temp.temperature);
  // loop forever when too hot, ie require user to reboot so that they are in
  // control of when it turns back on.
  while (temp.temperature > 45.0) {
    ESC.write(stopValue);
    delay(500);
  }

  const float squareSum = a.acceleration.x * a.acceleration.x +
    a.acceleration.y * a.acceleration.y +
    a.acceleration.z * a.acceleration.z;
  const float acc = sqrt(squareSum) - 9.81;

  /* Print out the values */
  if (mpuTimeMillis > 40) {
    mpuTimeMillis = 0;
    //Serial.println(acc);
    // Serial.print("Acceleration X: ");
    // Serial.print(a.acceleration.x);
    // Serial.print(", Y: ");
    // Serial.print(a.acceleration.y);
    // Serial.print(", Z: ");
    // Serial.println(a.acceleration.z);
    //Serial.println(" m/s^2");
  }
    
  if (timeMillis > 50) {
    float frequency = (float) revolutions / ((float) timeMillis / 1000.0);
    float rpm = frequency * 60.0; 
    //Serial.println(rpm);
    //Serial.println((float) revolutions / ((float) timeMillis / 1000.0));
    revolutions = 0;
    timeMillis = 0;
  }
  // Serial.println(hallEffectCounter);
  // delay(1);
}

void incrementHallEffectCount() {
  hallEffectCounter++;
}


//    // Basic demo for accelerometer readings from Adafruit MPU6050
//    
//    #include <Adafruit_MPU6050.h>
//    #include <Adafruit_Sensor.h>
//    #include <Wire.h>
//    
//    Adafruit_MPU6050 mpu;
//    
//    void setup(void) {
//      Serial.begin(115200);
//      while (!Serial)
//        delay(10); // will pause Zero, Leonardo, etc until serial console opens
//    
//      Serial.println("Adafruit MPU6050 test!");
//    
//      // Try to initialize!
//      if (!mpu.begin()) {
//        Serial.println("Failed to find MPU6050 chip");
//        while (1) {
//          delay(10);
//        }
//      }
//      Serial.println("MPU6050 Found!");
//    
//      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//      Serial.print("Accelerometer range set to: ");
//      switch (mpu.getAccelerometerRange()) {
//      case MPU6050_RANGE_2_G:
//        Serial.println("+-2G");
//        break;
//      case MPU6050_RANGE_4_G:
//        Serial.println("+-4G");
//        break;
//      case MPU6050_RANGE_8_G:
//        Serial.println("+-8G");
//        break;
//      case MPU6050_RANGE_16_G:
//        Serial.println("+-16G");
//        break;
//      }
//      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//      Serial.print("Gyro range set to: ");
//      switch (mpu.getGyroRange()) {
//      case MPU6050_RANGE_250_DEG:
//        Serial.println("+- 250 deg/s");
//        break;
//      case MPU6050_RANGE_500_DEG:
//        Serial.println("+- 500 deg/s");
//        break;
//      case MPU6050_RANGE_1000_DEG:
//        Serial.println("+- 1000 deg/s");
//        break;
//      case MPU6050_RANGE_2000_DEG:
//        Serial.println("+- 2000 deg/s");
//        break;
//      }
//    
//      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
//      Serial.print("Filter bandwidth set to: ");
//      switch (mpu.getFilterBandwidth()) {
//      case MPU6050_BAND_260_HZ:
//        Serial.println("260 Hz");
//        break;
//      case MPU6050_BAND_184_HZ:
//        Serial.println("184 Hz");
//        break;
//      case MPU6050_BAND_94_HZ:
//        Serial.println("94 Hz");
//        break;
//      case MPU6050_BAND_44_HZ:
//        Serial.println("44 Hz");
//        break;
//      case MPU6050_BAND_21_HZ:
//        Serial.println("21 Hz");
//        break;
//      case MPU6050_BAND_10_HZ:
//        Serial.println("10 Hz");
//        break;
//      case MPU6050_BAND_5_HZ:
//        Serial.println("5 Hz");
//        break;
//      }
//    
//      Serial.println("");
//      delay(100);
//    }
//    
//    void loop() {
//    
//      /* Get new sensor events with the readings */
//      sensors_event_t a, g, temp;
//      mpu.getEvent(&a, &g, &temp);
//    
//      /* Print out the values */
//      Serial.print("Acceleration X: ");
//      Serial.print(a.acceleration.x);
//      Serial.print(", Y: ");
//      Serial.print(a.acceleration.y);
//      Serial.print(", Z: ");
//      Serial.println(a.acceleration.z);
//      //Serial.println(" m/s^2");
//    
//      // Serial.print("Rotation X: ");
//      // Serial.print(g.gyro.x);
//      // Serial.print(", Y: ");
//      // Serial.print(g.gyro.y);
//      // Serial.print(", Z: ");
//      // Serial.print(g.gyro.z);
//      // Serial.println(" rad/s");
//    
//      // Serial.print("Temperature: ");
//      // Serial.print(temp.temperature);
//      // Serial.println(" degC");
//    
//      // Serial.println("");
//      delay(50);
//    }
