#include <ArduinoJson.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

StaticJsonDocument<200> doc;

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

Servo ESC;     // create servo object to control the ESC
const int BALANCE = 100;
const int OMD = 101;
const int AXE_550_CALIBRATE = 102;
const int OFF = 103;
int MODE = BALANCE;
const int hallEffectSensorPin = 14;
const int stopValue = 90;
const int reverseValue = 0;
int OMD_goValue = 132;
int BALANCE_goValue = 108; // is closer to a resonant frequency on the case
int AXE_550_CALIBRATE_goValue = 180;
int goValue = MODE == OMD ? OMD_goValue : (
  MODE == BALANCE ? BALANCE_goValue : AXE_550_CALIBRATE_goValue
);
volatile int hallEffectCounter = 0;
int revolutions = 0;
int numGos = 0;
int switchDelay = 250;

SoftwareSerial btSerial(7,8); // RX, TX (from pinout, not BL)
String inData;
bool start = false;
const char PARSE_END = '>';
const char PARSE_START = '<';
bool start_parse = false;
const int MIN_SPEED = 100;
const int MAX_SPEED = 130;

/*StaticJsonDocument<200> kModeToCode;

kModeToCode["BALANCE"] = "100";
kModeToCode["OMD"] = "101";
kModeToCode["AXE_550_CALIBRATE"] = "102";
kModeToCode["OFF"] = "103";*/

char json[] = "{\"BALANCE\":\"100\",\"OMD\"101\", \"AXE_550_CALIBRATE\":\"102\",\"OFF\": \"103\"}";

DynamicJsonDocument kModeToCode(1024);
DeserializationError error = deserializeJson(kModeToCode, json);

void setup() {
  Serial.begin(9600);
  btSerial.begin(9600);
  temperatureSensors.setWaitForConversion(false);

  pinMode(hallEffectSensorPin, INPUT);
  ESC.attach(24,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  delay(1000);

  timeMillis = 0;
  secondaryTimeMillis = 0;
  tempTimeMillis = 0;
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
// 3 == reverse state
const int INIT = 0;
const int GO = 1;
const int STOP = 2;
const int REVERSE = 3;
int motorState = INIT;

void loop() {
  if (tempTimeMillis > 60000) {
    tempTimeMillis = 0;
    temperatureSensors.requestTemperatures();
    Serial.println("Temp C @ Index 0: " + String(temperatureSensors.getTempCByIndex(0))); // Get the first temperature.
  }

  processIncomingBTData();
  operateMotor();
  processHallSensor();
  logRPM();
}

void logRPM() {
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

void processHallSensor() {
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
}

void init() {
  ESC.write(stopValue);
  //Serial.println("init");
  if (secondaryTimeMillis > 3000 && (MODE == OMD || MODE == BALANCE)) {
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
}

void go() {
  digitalWrite(13, HIGH);
  ESC.write(goValue);
  if (secondaryTimeMillis > switchDelay && MODE == OMD) {
		//Serial.println("go done");
		motorState = STOP;
		secondaryTimeMillis = 0;
  } else if (MODE == BALANCE) {
		// do nothing: we just keep going in balance mode!
  } else if (MODE == AXE_550_CALIBRATE) {
		// Serial.println("AXE_550_calibrate go");
		// Serial.println(secondaryTimeMillis % 1000);
		digitalWrite(13, secondaryTimeMillis % 1000 > 500 ? HIGH : LOW);

		if (secondaryTimeMillis > 10000) {
			motorState = REVERSE;
			secondaryTimeMillis = 0;
		}
  }
}

void processMode(const char* modeString) {
  int mode = atoi(kModeToCode[modeString]);
  switch (mode) {
    case OMD: {
			MODE = OMD;
      break;
    }

    case BALANCE: {
			MODE = BALANCE;
      break;
    }

    case AXE_550_CALIBRATE: {
			MODE = AXE_550_CALIBRATE;
      break;
    }

    case OFF: {
			MODE = OFF;
      break;
    }
  }
}

void processIncomingBTData() {
  /*if (Serial.available() > 0) {
    btSerial.write(Serial.read());
  }*/

  char appData;

  if (btSerial.available() > 0) {
    appData = btSerial.read();

    if (appData == PARSE_START || start == true) {
      inData += appData;  // save the data in string format
      start = true;
    }
    //Serial.println(appData);
  }

  if (appData == PARSE_END) {
    start = false;
    processJSON();
  }
}

void processJSON() {
  Serial.println("Processing JSON...");
  inData.replace(">", "");
  inData.replace("<", "");
  Serial.println("Parsing the follow: ");
  Serial.println(inData);

  // Deserialize the JSON document
  char buffer[50];
  inData.toCharArray(buffer, 50);
  DeserializationError error = deserializeJson(doc, buffer);
  inData = "";

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  processCmd();
}

void processCmd() {
  const char* modeString = doc["mode"];
  const char* on = doc["on"];
  const char* off = doc["off"];
  const char* speed = doc["hz"];

  if (modeString) {
    processMode(modeString);
    return;
  }

  if (on) {
    // TODO: Change value for on sequence
    int onSequenceMilli = atoi(on);
    Serial.println("On sequence: ");
    Serial.println(onSequenceMilli);
    switchDelay = onSequenceMilli;
		secondaryTimeMillis = 0;
    return;
  }

  if (off) {
    // TODO: Change value for off sequence
    int offSequenceMilli = atoi(off);
    Serial.println("Off sequence: ");
    Serial.println(offSequenceMilli);
    return;
  }

  if (speed) {
    // TODO: Change value for off sequence
    int speedValue = atoi(speed);
    Serial.println("Speed value: ");
    Serial.println(speedValue);
		switch (MODE) {
			case OMD: {
				OMD_goValue = speedValue;
				break;
			}

			case BALANCE: {
				BALANCE_goValue = speedValue;
				break;
			}

			case AXE_550_CALIBRATE: {
				AXE_550_CALIBRATE_goValue = speedValue;
				break;
			}
		}
    return;
  }
}


void operateMotor() {
  if (motorState == INIT) {
  	init();
  } else if (motorState == GO) {
		go();
  } else if (motorState == STOP) {
    digitalWrite(13, LOW);
    // stop
    ESC.write(stopValue);
    if (secondaryTimeMillis > switchDelay && numGos < 7200) {
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
  } else if (motorState == OFF) {
		digitalWrite(13, LOW);
		ESC.write(stopValue);
	}
}

void incrementHallEffectCount() {
  hallEffectCounter++;
}
