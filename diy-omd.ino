#include <Servo.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 5

// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature temperatureSensors(&oneWire);

elapsedMillis timeMillis;
elapsedMillis secondaryTimeMillis;
elapsedMillis tempTimeMillis;

const int escPin = 24;
const int hallEffectSensorPin = 14;

Servo ESC;     // create servo object to control the ESC

const int OMD = 0;
const int BALANCE = 1;
const int AXE_550_CALIBRATE = 2;
const int MODE = OMD;

const int RC = 0; // rc cars
const int QUAD = 1; // quadcopters
const int escType = RC;

const int stopValue = escType == RC ? 90 : 0; // quadcopters don't go in reverse!
const int reverseValue = 0; // only valid for RC!
const int OMD_accValue = escType == RC ? 115 : 36;
const int OMD_goValue = escType == RC ? 123 : 50;
const int BALANCE_goValue = escType == RC ? 108 : 36; // go slower while we balance!
const int AXE_550_CALIBRATE_goValue = 180;
const int goValue = MODE == OMD ? OMD_goValue : (
  MODE == BALANCE ? BALANCE_goValue : AXE_550_CALIBRATE_goValue
);
volatile int hallEffectCounter = 0;
int revolutions = 0;
int numGos = 0;
const int SWITCH_DELAY = 250;
const int ACC_DELAY = 1000;

void setup() {
  Serial.begin(9600);
  temperatureSensors.setWaitForConversion(false);

  pinMode(hallEffectSensorPin, INPUT);
  ESC.attach(escPin, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds) 
  delay(1000);

  timeMillis = 0;
  secondaryTimeMillis = 0;
  tempTimeMillis = 0;
}

// we use two thresholds to ensure we have the right direction of change, and
// they are far enough apart that noise in readings shouldn't mess up our logic
const int HALL_THRESHOLD1 = 600;
const int HALL_THRESHOLD2 = 800;
bool reachedThreshold1 = false;
bool reachedThreshold2 = false;
bool goingUp = false;

// 0 == init state
// 1 == go state
// 2 == stop state
// 3 == reverse state
const int INIT = 0;
const int ACC = 1;
const int GO = 2;
const int STOP = 3;
const int REVERSE = 4;
int motorState = INIT;

void loop() {
  if (tempTimeMillis > 60000) {
    tempTimeMillis = 0;
    temperatureSensors.requestTemperatures();
    Serial.println("Temp C @ Index 0: " + String(temperatureSensors.getTempCByIndex(0))); // Get the first temperature.
  }

  if (motorState == INIT) {
    ESC.write(stopValue);
    //Serial.println("init");
    if (secondaryTimeMillis > 4000 && MODE == OMD) {
      //Serial.println("init done");
      motorState = ACC;
      secondaryTimeMillis = 0;
    } else if (secondaryTimeMillis > 4000 && MODE == BALANCE) {
      //Serial.println("init done");
      motorState = GO;
      secondaryTimeMillis = 0;
    } else if (MODE == AXE_550_CALIBRATE) {
      // Serial.println("AXE_550_calibrate init");
      // Serial.println(secondaryTimeMillis);
      digitalWrite(13, secondaryTimeMillis % 2000 > 1000 ? HIGH : LOW);

      if (secondaryTimeMillis > 10000) {
        motorState = GO;
        secondaryTimeMillis = 0;
      }
    }
  } else if (motorState == ACC) {
    // we accelerate for a longer time at first, to get over the initial friction
    digitalWrite(13, HIGH);
    ESC.write(OMD_accValue);
    if (secondaryTimeMillis > ACC_DELAY && MODE == OMD) {
      motorState = STOP;
      secondaryTimeMillis = 0;
    }
  } else if (motorState == GO) {
    digitalWrite(13, HIGH);
    ESC.write(goValue);
    if (secondaryTimeMillis > SWITCH_DELAY && MODE == OMD) {
      //Serial.println("go done");
      motorState = STOP;
      secondaryTimeMillis = 0;
    } else if (MODE == BALANCE) {
      // do nothing: we just keep going in balance mode!
      // XXX: temporarily limit it to 250ms
      // if (secondaryTimeMillis > 10000) {
      //   while (1) {
      //     digitalWrite(13, LOW);
      //     ESC.write(stopValue);
      //   }
      // }
    } else if (MODE == AXE_550_CALIBRATE) {
      // Serial.println("AXE_550_calibrate go");
      // Serial.println(secondaryTimeMillis % 1000);
      digitalWrite(13, secondaryTimeMillis % 1000 > 500 ? HIGH : LOW);

      if (secondaryTimeMillis > 10000) {
        motorState = REVERSE;
        secondaryTimeMillis = 0;
      }
    }
  } else if (motorState == STOP) {
    digitalWrite(13, LOW);
    // stop
    ESC.write(stopValue);
    if (secondaryTimeMillis > SWITCH_DELAY && numGos < 7200) {
      numGos++;
      motorState = GO;
      secondaryTimeMillis = 0;
    }
  } else if (motorState == REVERSE) {
      Serial.println("AXE_550_calibrate reverse");
      Serial.println(secondaryTimeMillis % 500);
      digitalWrite(13, secondaryTimeMillis % 500 > 250 ? HIGH : LOW);
      if (secondaryTimeMillis > 10000) {
        ESC.write(stopValue);
      } else {
        ESC.write(reverseValue);
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

  if (timeMillis > 50) {
    float frequency = (float) revolutions / ((float) timeMillis / 1000.0);
    float rpm = frequency * 60.0; 
    //Serial.println(hallSensorValue);
    Serial.println(rpm);
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
