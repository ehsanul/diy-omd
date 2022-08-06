#include <ArduinoJson.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>


// Model types
// RC Car ESC
#define RC 0
// Quadcopter ESC
#define QUAD 1
// Sidewinder 4 ESC
#define SIDEWINDER 2


// How many hours to operate for in go mode before stopping
#define HOURS_OPERATION 4


// Set model here. No equals, just the value after ESC_TYPE
#define ESC_TYPE SIDEWINDER


#if ESC_TYPE == RC

#define ESC_PIN1 2
#define ESC_PIN2 11
#define ESC_PIN3 24
#define HALL_PIN 14

#define STOP_VALUE 90
#define REVERSE_VALUE 0
#define ACC_VALUE 115
#define CALIBRATE_VALUE 180
#define OMD_GO_VALUE_1 135
#define OMD_GO_VALUE_2 135
#define OMD_GO_VALUE_3 135
#define BALANCE_GO_VALUE 108
#define MAX_GO_VALUE 130
#define BLUETOOTH_RX 7
#define BLUETOOTH_TX 8

#elif ESC_TYPE == QUAD

#define ESC_PIN1 2
#define ESC_PIN2 11
#define ESC_PIN3 24
#define HALL_PIN 14

#define STOP_VALUE 0
#define REVERSE_VALUE 0
#define ACC_VALUE 15
#define CALIBRATE_VALUE 180
#define OMD_GO_VALUE_1 27
#define OMD_GO_VALUE_2 23
#define OMD_GO_VALUE_3 23
#define BALANCE_GO_VALUE 20
#define MAX_GO_VALUE 60
#define BLUETOOTH_RX 7
#define BLUETOOTH_TX 8

#elif ESC_TYPE == SIDEWINDER

#define ESC_PIN1 37
#define ESC_PIN2 15
#define ESC_PIN3 14
#define HALL_PIN 23

#define STOP_VALUE 90
#define REVERSE_VALUE 0
#define ACC_VALUE 99
#define CALIBRATE_VALUE 90
#define OMD_GO_VALUE_1 101
#define OMD_GO_VALUE_2 101
#define OMD_GO_VALUE_3 101
#define BALANCE_GO_VALUE 100
#define MAX_GO_VALUE 110

#define BLUETOOTH_RX 21
#define BLUETOOTH_TX 20

#endif


StaticJsonDocument<200> doc;

// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 4

// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature temperatureSensors(&oneWire);

elapsedMillis timeMillis;
elapsedMillis secondaryTimeMillis;
elapsedMillis tempTimeMillis;
elapsedMillis bluetoothMillis;
elapsedMillis spinCycleMillis;

const int WINDOW_FOR_PEAK_FREQUENCY = 750;

const int escPin1 = ESC_PIN1;
const int escPin2 = ESC_PIN2;
const int escPin3 = ESC_PIN3;
const int hallEffectSensorPin = HALL_PIN;  

Servo ESC1;     // create servo object to control the ESC
Servo ESC2;     // create servo object to control the ESC
Servo ESC3;     // create servo object to control the ESC
const int BALANCE = 100;
const int OMD = 101;
const int AXE_550_CALIBRATE = 102;
const int OFF = 103;
int MODE = OMD;
float minFrequencyInWindow = 9999999.0;
float maxFrequencyInWindow = -999999.0;

bool parsingStarted = false;
bool transmittingData = false;

// Operations
const int OP_INIT = 100;


const int stopValue = STOP_VALUE; // quadcopters don't go in reverse!
const int reverseValue = REVERSE_VALUE; // only valid for RC!
const int OMD_accValue = ACC_VALUE;

// for quad esc, starting the esc with throttle at max is how to calibrate the
// max throttle value. otherwise, this seems to be random, causing strange
// speed fluctuations on later runs
const int calibrationInitValue = CALIBRATE_VALUE;

int OMD_GOVALUE1_ADDR = 1;
int OMD_GOVALUE2_ADDR = 2;
int OMD_GOVALUE3_ADDR = 3;
int MOTOR_STATE_ADDR  = 4;
int BALANCE_GOVALUE_ADDR = 5;
int MODE_ADDR = 6;
int ON_SEQUENCE_ADDR = 8;
int OFF_SEQUENCE_ADDR = 12;

int OMD_goValue1 = OMD_GO_VALUE_1;
int OMD_goValue2 = OMD_GO_VALUE_2;
int OMD_goValue3 = OMD_GO_VALUE_3;
bool stopMessageSent = false;


int BALANCE_goValue = BALANCE_GO_VALUE;

int AXE_550_CALIBRATE_goValue = 180;
int goValue1 = MODE == OMD ? OMD_goValue1 : (
  MODE == BALANCE ? BALANCE_goValue : AXE_550_CALIBRATE_goValue
);
int goValue2 = MODE == OMD ? OMD_goValue2 : (
  MODE == BALANCE ? BALANCE_goValue : AXE_550_CALIBRATE_goValue
);
int goValue3 = MODE == OMD ? OMD_goValue3 : (
  MODE == BALANCE ? BALANCE_goValue : AXE_550_CALIBRATE_goValue
);
volatile int hallEffectCounter = 0;
int revolutions = 0;
int numGos = 0;
int onSequence = 250;
int offSequence = 250;

// Given the number of hours, and how long the sequence is for, calculate number of iterations
// before stopping
double period_operation_seconds = double(onSequence + offSequence) / 1000;
int num_iterations = int(HOURS_OPERATION * 3600 / period_operation_seconds);

SoftwareSerial btSerial(BLUETOOTH_RX, BLUETOOTH_TX); // Was 7, 8. RX, TX (from pinout, not BL)
String inData;
const char PARSE_END = '>';
const char PARSE_START = '<';

char MODES[] = "{\"BALANCE\":\"100\",\"OMD\":\"101\", \"CALIBRATE\":\"102\",\"OFF\": \"103\"}";
char OPERATIONS[] = "{\"INIT\":\"100\"}";

DynamicJsonDocument kModeToCode(1024);
DynamicJsonDocument kOperationToCode(1024);
DeserializationError error1 = deserializeJson(kModeToCode, MODES);
DeserializationError error2 = deserializeJson(kOperationToCode, OPERATIONS);
const int ACC_DELAY = 1000;

void setup() {
  Serial.begin(9600);
  btSerial.begin(9600);
  temperatureSensors.setWaitForConversion(false);
  loadParameters();

  pinMode(hallEffectSensorPin, INPUT);

  delay(3000);
  ESC1.attach(escPin1, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds) 
  ESC2.attach(escPin2, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds) 
  ESC3.attach(escPin3, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds)

  timeMillis = 0;
  secondaryTimeMillis = 0;
  tempTimeMillis = 0;
  updateNumIterations();
  Serial.println("Setup complete");

   Serial.print("GO mode operation time: ");
  Serial.print(HOURS_OPERATION * 3600);
  Serial.println(" seconds.");

  Serial.print("Number iterations: ");
  Serial.println(num_iterations);
}

// we use two thresholds to ensure we have the right direction of change, and
// they are far enough apart that noise in readings shouldn't mess up our logic
const int HALL_THRESHOLD1 = 600;
const int HALL_THRESHOLD2 = 800;
bool reachedThreshold1 = false;
bool reachedThreshold2 = false;
bool goingUp = false;

const int CALIBRATION_INIT = 0;
const int INIT = 1;
const int ACC = 2;
const int GO = 3;
const int STOP = 4;
const int REVERSE = 5;
const int PAUSE = 6; // Stops motors, but doesn't reset
int motorState = CALIBRATION_INIT;

bool hard_stop = false;

void loop() { 
  if (hard_stop){
    Serial.println("Hard stop is on, must restart Teensy");
    ESC1.write(STOP_VALUE);
    ESC2.write(STOP_VALUE);
    ESC3.write(STOP_VALUE);
    delay(10000);
    return;
  }

   
  if (tempTimeMillis > 60000) {
    tempTimeMillis = 0;
    temperatureSensors.requestTemperatures();
    // Serial.println("Temp C @ Index 0: " + String(temperatureSensors.getTempCByIndex(0))); // Get the first temperature.
  }

  processIncomingBTData();
  operateMotor();
  processHallSensor();
  logRPM();
  //transmitRealtimeData();
}

void transmitBTData(String json) {
  const char* j = json.c_str();
  btSerial.write(j);
}

void logRPM() {
  if (transmittingData) {
    return;
  }

  if (timeMillis > 50) {
    float frequency = (float) revolutions / ((float) timeMillis / 1000.0);
    float rpm = frequency * 60.0;

    if (rpm < minFrequencyInWindow) {
      minFrequencyInWindow = rpm;
    }

    if (rpm > maxFrequencyInWindow) {
      maxFrequencyInWindow = rpm;
    }

    if (spinCycleMillis > WINDOW_FOR_PEAK_FREQUENCY) {
      spinCycleMillis = 0;

      String speed = String("{\"min\":") + "\"" + String(minFrequencyInWindow) + "\"}";
      transmitBTData(speed);
      delay(50);
      speed = String("{\"max\":") + "\"" + String(maxFrequencyInWindow) + "\"}";
      transmitBTData(speed);

      maxFrequencyInWindow = -9999;
      minFrequencyInWindow = 9999;
    }

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

void updateNumIterations(){
  // Given the number of hours, and how long the sequence is for, calculate number of iterations
  // before stopping.
  period_operation_seconds = double(onSequence + offSequence) / 1000;
  num_iterations = int(HOURS_OPERATION * 3600 / period_operation_seconds);
  Serial.print("Number of iterations is: ");
  Serial.println(num_iterations);
  Serial.print("Period [s]: ");
  Serial.println(period_operation_seconds);
}

void loadParameters() {
  int omdGoValue1StoredValue = EEPROM.read(OMD_GOVALUE1_ADDR);
  int omdGoValue2StoredValue = EEPROM.read(OMD_GOVALUE2_ADDR);
  int omdGoValue3StoredValue = EEPROM.read(OMD_GOVALUE3_ADDR);
  int balanceGoValueStoredValue = EEPROM.read(BALANCE_GOVALUE_ADDR);
  int modeStoredValue = EEPROM.read(MODE_ADDR);
  int onSequenceStoredValue = EEPROM.read(ON_SEQUENCE_ADDR);
  int offSequenceStoredValue = EEPROM.read(OFF_SEQUENCE_ADDR);

  onSequence = onSequenceStoredValue == 255 ? onSequence : onSequenceStoredValue * 10;
  offSequence = offSequenceStoredValue == 255 ? offSequence : offSequenceStoredValue * 10;
  modeStoredValue = modeStoredValue == 255 ? MODE : modeStoredValue;
  Serial.println("onSequence");
  Serial.println(onSequence);
  Serial.println("offSequence");
  Serial.println(offSequence);

  updateNumIterations();

  MODE = modeStoredValue;
  if (modeStoredValue == BALANCE) {
    BALANCE_goValue = balanceGoValueStoredValue == 255 ? BALANCE_goValue : balanceGoValueStoredValue;
    goValue1 = BALANCE_goValue;
    goValue2 = BALANCE_goValue;
    goValue3 = BALANCE_goValue;
  } else {
    goValue1 = omdGoValue1StoredValue == 255 ? OMD_goValue1 :  omdGoValue1StoredValue;
    goValue2 = omdGoValue2StoredValue == 255 ? OMD_goValue2 :  omdGoValue2StoredValue;
    goValue3 = omdGoValue3StoredValue == 255 ? OMD_goValue3 :  omdGoValue3StoredValue;
  }

  Serial.println("goValue_1:");
  Serial.println(goValue1);
  Serial.println("goValue_2:");
  Serial.println(goValue2);
  Serial.println("goValue_3:");
  Serial.println(goValue3);

  Serial.println("Balance value:");
  Serial.println(BALANCE_goValue);
}

void calibrationInit() {
  if (ESC_TYPE == QUAD || ESC_TYPE == SIDEWINDER) {
    ESC1.write(calibrationInitValue);
    ESC2.write(calibrationInitValue);
    ESC3.write(calibrationInitValue);
    if (secondaryTimeMillis > 2000) {
      secondaryTimeMillis = 0;
      motorState = INIT;
    }
  } else {
    motorState = INIT;
  }
}

void init() {
  ESC1.write(stopValue);
  ESC2.write(stopValue);
  ESC3.write(stopValue);
  //Serial.println("init");
  if (secondaryTimeMillis > 4000 && MODE == OMD) {
    Serial.println("init done (OMD mode)");
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
}

void go() {
  digitalWrite(13, HIGH);
  ESC1.write(goValue1);
  ESC2.write(goValue2);
  ESC3.write(goValue3);
  if (secondaryTimeMillis > onSequence && MODE == OMD) {
    Serial.println("go done");
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

  // We haven't sent a stop message to the serial since we last started
  stopMessageSent = false;
}

void processMode(const char* modeString) {
  int mode = atoi(kModeToCode[modeString]);

  Serial.println("Switching mode to");
  switch (mode) {
    case OMD: {
      Serial.println("OMD");
      MODE = OMD;
      motorState = STOP;
      EEPROM.write(MODE_ADDR, OMD);
      secondaryTimeMillis = 0;
      break;
    }

    case BALANCE: {
      Serial.println("BALANCE");
      MODE = BALANCE;
      motorState = STOP;
      EEPROM.write(MODE_ADDR, BALANCE);
      break;
    }

    case AXE_550_CALIBRATE: {
      Serial.println("CALIBRATE");
      MODE = AXE_550_CALIBRATE;
      motorState = STOP;
      EEPROM.write(MODE_ADDR, AXE_550_CALIBRATE);
      break;
    }

    case PAUSE: {
      Serial.println("PAUSE");
      motorState = PAUSE;
      EEPROM.write(MODE_ADDR, PAUSE);
      break;
    }

    case OFF: {
      Serial.println("OFF");
      motorState = OFF;
      EEPROM.write(MODE_ADDR, OFF);
      break;
    }

    default: {
      Serial.println("Unknown cmd");
      MODE = OFF;
      EEPROM.write(MODE_ADDR, OFF);
      break;
    }
  }

  loadParameters();
}

void processIncomingBTData() {
  char appData;

  if (btSerial.available() > 0) {
    appData = btSerial.read();
    Serial.println(appData);

    if (appData == '<') {
      Serial.println("Parsing started..");
    }

    if (appData == '<' || parsingStarted == true) {
      inData += appData;  // save the data in string format
      parsingStarted = true;
    }
    //Serial.println(appData);
    if (appData == '>') {
      Serial.println("Parsing end");
      parsingStarted = false;
      processJSON();
    }
  }

}

void processJSON() {
  Serial.println("Processing JSON...");
  inData.replace(PARSE_START, "");
  inData.replace(PARSE_END, "");
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

void processOperation(const char* operation) {
  int op = atoi(kOperationToCode[operation]);
  Serial.println("Received operation:");
  Serial.println(op);
  Serial.println(operation);

  switch (op) {
    case OP_INIT: {
      Serial.println("Sending saved parameters to bluetooth device.");
      int omdGoValue1StoredValue = EEPROM.read(OMD_GOVALUE1_ADDR);
      int omdGoValue2StoredValue = EEPROM.read(OMD_GOVALUE2_ADDR);
      int omdGoValue3StoredValue = EEPROM.read(OMD_GOVALUE3_ADDR);
      int balanceGoValueStoredValue = EEPROM.read(BALANCE_GOVALUE_ADDR);
      int modeStoredValue = EEPROM.read(MODE_ADDR);
      int onSequenceStoredValue = EEPROM.read(ON_SEQUENCE_ADDR);
      int offSequenceStoredValue = EEPROM.read(OFF_SEQUENCE_ADDR);

      omdGoValue1StoredValue = omdGoValue1StoredValue == 255 ? OMD_goValue1 :  omdGoValue1StoredValue;
      omdGoValue2StoredValue = omdGoValue2StoredValue == 255 ? OMD_goValue2 :  omdGoValue2StoredValue;
      omdGoValue3StoredValue = omdGoValue3StoredValue == 255 ? OMD_goValue3 :  omdGoValue3StoredValue;
      balanceGoValueStoredValue = balanceGoValueStoredValue == 255 ? BALANCE_goValue : balanceGoValueStoredValue;
      modeStoredValue = modeStoredValue == 255 ? MODE : modeStoredValue;
      onSequenceStoredValue = onSequenceStoredValue == 255 ? onSequence : onSequenceStoredValue;
      offSequenceStoredValue = offSequenceStoredValue == 255 ? offSequence : offSequenceStoredValue;

      transmittingData = true;

      String tmp = String("{\"esc1\":") + "\"" + String(omdGoValue1StoredValue) + "\"}";
      transmitBTData(tmp);
      delay(50);
      tmp = String("{\"esc2\":") + "\"" + String(omdGoValue2StoredValue) + "\"}";
      transmitBTData(tmp);
      delay(50);
      tmp = String("{\"esc3\":") + "\"" + String(omdGoValue3StoredValue) + "\"}";
      transmitBTData(tmp);
      delay(50);
      tmp = String("{\"balance\":") + "\"" + String(balanceGoValueStoredValue) + "\"}";
      transmitBTData(tmp);
      delay(50);
      tmp = String("{\"mode\":") + "\"" + String(modeStoredValue) + "\"}";
      transmitBTData(tmp);
      delay(50);
      tmp = String("{\"on\":") + "\"" + String(onSequenceStoredValue) + "\"}";
      transmitBTData(tmp);
      delay(50);
      tmp = String("{\"off\":") + "\"" + String(offSequenceStoredValue) + "\"}";
      transmitBTData(tmp);
      delay(100);

      transmittingData = false;
      break;
    }

    default: {
      Serial.println("Unknown operation");
      MODE = OFF;
      break;
    }
  }
}

void processCmd() {
  const char* modeString = doc["mode"];
  const char* operation = doc["op"];
  const char* on = doc["on"];
  const char* off = doc["off"];
  const char* esc1Speed = doc["esc1Speed"];
  const char* esc2Speed = doc["esc2Speed"];
  const char* esc3Speed = doc["esc3Speed"];
  const char* balanceSpeed = doc["balanceSpeed"];

  if (operation) {
    Serial.println("This is an operation");
    processOperation(operation);
  }

  if (modeString) {
    processMode(modeString);
  }

  if (on) {
    // TODO: Change value for on sequence
    int tempOnSequence = atoi(on) * 10;
    if (tempOnSequence <= 500) {
      onSequence = tempOnSequence;
      EEPROM.write(ON_SEQUENCE_ADDR, atoi(on));
      secondaryTimeMillis = 0;
      Serial.println("On sequence: ");
      Serial.println(onSequence);
      updateNumIterations();
    }
  }

  if (off) {
    // TODO: Change value for off sequence
    int tempOffSequence = atoi(off) * 10;
    if (tempOffSequence >= 150) {
      offSequence = tempOffSequence;
      EEPROM.write(OFF_SEQUENCE_ADDR, atoi(off));
      secondaryTimeMillis = 0;
      Serial.println("Off sequence: ");
      Serial.println(offSequence);
      updateNumIterations();
    }
  }

  if (esc1Speed) {
    changeSpeed(esc1Speed, goValue1, OMD_GOVALUE1_ADDR);
  }

  if (esc2Speed) {
    changeSpeed(esc2Speed, goValue2, OMD_GOVALUE2_ADDR);
  }

  if (esc3Speed) {
    changeSpeed(esc3Speed, goValue3, OMD_GOVALUE3_ADDR);
  }

  if (balanceSpeed) {
    changeSpeed(balanceSpeed, BALANCE_goValue, BALANCE_GOVALUE_ADDR);
  }
}

void changeSpeed(int escSpeed, int &goValue, int goValueAddr) {
  int speedValue = atoi(escSpeed);
  Serial.println("Changing speed to: ");
  Serial.println(speedValue);

  if (speedValue > MAX_GO_VALUE) {
    Serial.println("Exceeding max speed value!!!!!!");
    return;
  }

  switch (MODE) {
    case OMD: {
      goValue = speedValue;
      EEPROM.write(goValueAddr, goValue);
      break;
    }

    case BALANCE: {
      goValue1 = speedValue;
      goValue2 = speedValue;
      goValue3 = speedValue;
      EEPROM.write(BALANCE_GOVALUE_ADDR, speedValue);
      break;
    }

    case AXE_550_CALIBRATE: {
      AXE_550_CALIBRATE_goValue = speedValue;
      break;
    }
  }
}

void accelerate() {
  // we accelerate for a longer time at first, to get over the initial friction
  digitalWrite(13, HIGH);
  ESC1.write(OMD_accValue);
  ESC2.write(OMD_accValue);
  ESC3.write(OMD_accValue);
  if (secondaryTimeMillis > ACC_DELAY && MODE == OMD) {
    Serial.println("Acc done");
    motorState = STOP;
    secondaryTimeMillis = 0;
  }
}

void stop() {
  digitalWrite(13, LOW);
  // stop
  ESC1.write(stopValue);
  ESC2.write(stopValue);
  ESC3.write(stopValue);
  if (secondaryTimeMillis > offSequence && numGos < num_iterations) {
    numGos++;
    
    Serial.print("Stop done. Iteration ");
    Serial.print(numGos);
    Serial.print("/");
    Serial.println(num_iterations);
    
    motorState = GO;
    secondaryTimeMillis = 0;
  }else if (secondaryTimeMillis > offSequence && numGos >= num_iterations){
    // End of sequence
    if (!stopMessageSent){
      Serial.println("OMD Sequence finished due to reaching iteration limit");
    }

    stopMessageSent = true;
  }
}

void axe_550_reverse() {
  Serial.println("AXE_550_calibrate reverse");
  Serial.println(secondaryTimeMillis % 500);
  digitalWrite(13, secondaryTimeMillis % 500 > 250 ? HIGH : LOW);
  if (secondaryTimeMillis > 10000) {
    ESC1.write(stopValue);
    ESC2.write(stopValue);
    ESC3.write(stopValue);
  } else {
    ESC1.write(reverseValue);
    ESC2.write(reverseValue);
    ESC3.write(reverseValue);
  }
}

void operateMotor() {
  if (motorState == CALIBRATION_INIT) {
    calibrationInit();
  } else if (motorState == INIT) {
    init();
  } else if (motorState == ACC) {
    accelerate();
  } else if (motorState == GO) {
    go();
  } else if (motorState == STOP) {
    stop();
  } else if (motorState == REVERSE) {
    axe_550_reverse();
  } else if (motorState == PAUSE) {
    digitalWrite(13, LOW);
    ESC1.write(stopValue);
    ESC2.write(stopValue);
    ESC3.write(stopValue);
    numGos = 0;
  } else if (motorState == OFF) {
    digitalWrite(13, LOW);
    ESC1.write(stopValue);
    ESC2.write(stopValue);
    ESC3.write(stopValue);
    numGos = 0;
  }
}

void incrementHallEffectCount() {
  hallEffectCounter++;
}
