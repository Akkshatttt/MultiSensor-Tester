#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Servo.h>
#include <DHT.h>
#include <NewPing.h>
#include <IRremote.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
const int TRIGGER_PIN = 8; // D8
const int ECHO_PIN = 9; // D9
const int MAX_DISTANCE = 350;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

const int rotaryPinA = 10;
const int rotaryPinB = 11;
const int buttonPin = 3; // D3
const int sensorPin = 2; // D2
const int analogSensorPin = A3;
const int servoPin = A2;
const int buzzerPin = 4; // D4
const int redPin = 7; // D5
const int greenPin = 6; // D6
const int bluePin = 5; // D7
const int RECV_PIN = sensorPin; // Pin connected to the IR receiver
IRrecv irrecv(RECV_PIN);
decode_results results;

Servo myServo;
#define DHTPIN sensorPin     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11       // DHT 11
DHT dht(DHTPIN, DHTTYPE);

const char* const menuItems[] = {
  "1) IR Module", "2) IR Receiver",
  "3) Temperature(D)", "4) Temperature(A)", "5) DHT11 Sensor",
  "6) Touch Sensor", "7) Tap Module",
  "8) Rain Detector", "9) Soil Moisture",
  "10) RGB Module", "11) Joystick", "12) Rotary Encoder", 
  "13) Servo Motor", "14) Relay",
  "15) LDR Module", "16) Photoresistor", "17) Buzzer",
  "18) Ultrasound", "19) Hall Sensor(D)", "20) Hall Sensor(A)",
  "21) Sound Sensor(D)", "22) Sound Sensor(A)",
  "23) Reed Switch", "24) Tilt Switch", "25) Button Module",
  "26) Vibration Sensor"
};

const int numMenuItems = sizeof(menuItems) / sizeof(menuItems[0]);
int menuPos = 0;
int selectedSensor = -1;

const unsigned long sensorInterval = 1000; // Update interval for sensor data (in milliseconds)
unsigned long previousMillis = 0; // Store the last time the sensor data was updated

void setup() {
  lcd.init();
  lcd.backlight();

  pinMode(rotaryPinA, INPUT_PULLUP);
  pinMode(rotaryPinB, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(sensorPin, INPUT);
  pinMode(analogSensorPin, INPUT);
  pinMode(servoPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  myServo.attach(servoPin);

  displayMenu();
  Serial.begin(9600);
}

bool (*sensorFunctions[])(void) = {
  testLowHighSensor, testIRReceiver,
  testTemperatureSensor, testAnalogTemperatureSensor, testDHT11Sensor,
  testLowHighSensor, testLowHighSensor,
  testLowHighSensor, testLowHighSensor,
  testRGBModule, testJoystick, testRotaryEncoder, 
  testServoMotor, testLowHighSensor,
  testLowHighSensor, testAnalogSensor, testBuzzer,
  testUltrasound, testLowHighSensor, testAnalogSensor,
  testLowHighSensor, testAnalogSensor,
  testReedSwitch, testLowHighSensor, testButtonModule, 
  testAnalogSensor
};

void loop() {
  static int lastState = 0;
  int currentState = (digitalRead(rotaryPinA) << 1) | digitalRead(rotaryPinB);
  if (currentState != lastState) {
    if ((lastState == 0b00 && currentState == 0b01) ||
        (lastState == 0b01 && currentState == 0b11) ||
        (lastState == 0b11 && currentState == 0b10) ||
        (lastState == 0b10 && currentState == 0b00)) {
      menuPos--;
    } else {
      menuPos++;
    }
    menuPos = (menuPos + numMenuItems) % numMenuItems;
    displayMenu();
  }
  lastState = currentState;

  if (digitalRead(buttonPin) == LOW) {
    delay(100);
    if (selectedSensor == -1) {
      selectedSensor = menuPos;
      lcd.clear();
      lcd.print(F("Selected: "));
      lcd.setCursor(0, 1);
      lcd.print(menuItems[selectedSensor]);
      delay(2000);
      displayMenu();
    } else {
      selectedSensor = -1;
      displayMenu();
    }
  }

  if (selectedSensor != -1) {
    bool exitFunction = runSensorFunction(selectedSensor);
    if (exitFunction) {
      selectedSensor = -1;
      displayMenu();
    }
  }

  delay(100);
}


void displayMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(menuItems[menuPos]);
}

bool runSensorFunction(int sensor) {
  if (sensor >= 0 && sensor < sizeof(sensorFunctions) / sizeof(sensorFunctions[0])) {
    return sensorFunctions[sensor]();
  } else {
    Serial.println(F("Invalid sensor selected"));
    return true;
  }
}

bool testLowHighSensor() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sensorInterval) {
    previousMillis = currentMillis;
    int sensorValue = digitalRead(sensorPin);
    lcd.clear();
    lcd.print(menuItems[selectedSensor]);
    lcd.setCursor(0, 1);
    if (sensorValue == HIGH) {
      lcd.print(F("state: HIGH"));
    } else {
      lcd.print(F("state: LOW"));
    }
  }
  if (digitalRead(buttonPin) == LOW) {
    return true;
  }
  return false;
}

bool testAnalogSensor() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sensorInterval) {
    previousMillis = currentMillis;
    int sensorValue = analogRead(analogSensorPin);
    lcd.clear();
    lcd.print(menuItems[selectedSensor]);
    lcd.setCursor(0, 1);
    lcd.print(F("Value: "));
    lcd.print(sensorValue);
  }
  if (digitalRead(buttonPin) == LOW) {
    return true;
  }
  return false;
}

bool testBuzzer() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sensorInterval) {
    previousMillis = currentMillis;
    lcd.clear();
    lcd.print(F("Buzzer"));
    lcd.setCursor(0, 1);
    lcd.print(F("Beeping Buzzer"));
    digitalWrite(buzzerPin, HIGH);
    delay(100);
    digitalWrite(buzzerPin, LOW); // Play a tone on the buzzer
  } else {
    digitalWrite(buzzerPin, LOW); // Stop the tone
  }
  if (digitalRead(buttonPin) == LOW) {
    return true;
  }
  return false;
}

bool testUltrasound() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sensorInterval) {
    previousMillis = currentMillis;
    lcd.clear();
    lcd.print(F("Ultrasound"));
    delay(10);
    unsigned int uS = sonar.ping();
    lcd.setCursor(0, 1);
    lcd.print(F("Distance: "));
    lcd.print(uS / US_ROUNDTRIP_CM);
    lcd.print(F("cm"));
  }
  if (digitalRead(buttonPin) == LOW) {
    return true;
  }
  return false;
}

bool testTemperatureSensor() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sensorInterval) {
    previousMillis = currentMillis;
    const float baselineTemp = 25.0;
    const float degreesPerBit = 0.25;
    int tempSensorValue = digitalRead(sensorPin);
    float temperature = baselineTemp + (tempSensorValue * degreesPerBit);
    lcd.clear();
    lcd.print(menuItems[selectedSensor]);
    lcd.setCursor(0, 1);
    lcd.print(F("Temp: "));
    lcd.print(temperature);
    lcd.print(F("C"));
  }
  if (digitalRead(buttonPin) == LOW) {
    return true;
  }
  return false;
}

bool testAnalogTemperatureSensor() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sensorInterval) {
    previousMillis = currentMillis;
    int sensorValue = analogRead(analogSensorPin);
    float voltage = sensorValue * (5.0 / 1023.0); // Convert analog value to voltage
    const float temperatureCoefficient = 0.01; // Change this value to match your temperature sensor
    const float temperatureOffset = -0.5; // Change this value to match your temperature sensor
    float temperature = (voltage - temperatureOffset) / temperatureCoefficient;
    lcd.clear();
    lcd.print(F("Analog Temp Sensor"));
    lcd.setCursor(0, 1);
    lcd.print(F("Temp: "));
    lcd.print(temperature);
    lcd.print(F("C"));
  }
  if (digitalRead(buttonPin) == LOW) {
    return true;
  }
  return false;
}

bool testRGBModule() {
  static int currentColor = 0;
  if (currentColor == 0) {
    // Red
    analogWrite(redPin, 255);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, 0);
    lcd.clear();
    lcd.print(F("Color: Red"));
    currentColor = 1;
    delay(sensorInterval);
  } else if (currentColor == 1) {
    // Green
    analogWrite(redPin, 0);
    analogWrite(greenPin, 255);
    analogWrite(bluePin, 0);
    lcd.clear();
    lcd.print(F("Color: Green"));
    currentColor = 2;
    delay(sensorInterval);
  } else if (currentColor == 2) {
    // Blue
    analogWrite(redPin, 0);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, 255);
    lcd.clear();
    lcd.print(F("Color: Blue"));
    currentColor = 0;
    delay(sensorInterval);
  }

  if (digitalRead(buttonPin) == LOW) {
    return true;
  }
  return false;
}

bool testReedSwitch() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sensorInterval) {
    previousMillis = currentMillis;
    int sensorValue = digitalRead(sensorPin);
    lcd.clear();
    lcd.print(F("Reed Switch"));
    lcd.setCursor(0, 1);
    if (sensorValue == HIGH) {
      lcd.print(F("state: Closed"));
    } else {
      lcd.print(F("state: Open"));
    }
  }
  if (digitalRead(buttonPin) == LOW) {
    return true;
  }
  return false;
}

bool testJoystick() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sensorInterval) {
    previousMillis = currentMillis;
    int xValue = analogRead(TRIGGER_PIN); // Use TRIGGER_PIN as joystick X-axis
    int yValue = analogRead(ECHO_PIN); // Use ECHO_PIN as joystick Y-axis
    lcd.clear();
    lcd.print(F("Joystick"));
    lcd.setCursor(0, 1);
    lcd.print(F("X: "));
    lcd.print(xValue);
    lcd.print(F(" Y: "));
    lcd.print(yValue);
  }
  if (digitalRead(buttonPin) == LOW) {
    return true;
  }
  return false;
}

bool testButtonModule() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sensorInterval) {
    previousMillis = currentMillis;
    int sensorValue = digitalRead(sensorPin);
    lcd.clear();
    lcd.print(F("Button Module"));
    lcd.setCursor(0, 1);
    if (sensorValue == HIGH) {
      lcd.print(F("state: Pressed"));
    } else {
      lcd.print(F("state: Released"));
    }
  }
  if (digitalRead(buttonPin) == LOW) {
    return true;
  }
  return false;
}

bool testServoMotor() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sensorInterval) {
    previousMillis = currentMillis;
    lcd.clear();
    lcd.print(F("Servo Motor"));
    myServo.write(180); // Rotate the servo to 180 degrees
    delay(sensorInterval);
    myServo.write(0); // Rotate the servo back to 0 degrees
  }
  if (digitalRead(buttonPin) == LOW) {
    return true;
  }
  return false;
}

bool testRotaryEncoder() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sensorInterval) {
    previousMillis = currentMillis;
    static int rotaryValue = 0;
    static int rotaryState = 0;
    int pinA = digitalRead(TRIGGER_PIN); // CLK
    int pinB = digitalRead(ECHO_PIN); // DT
    int state = (pinA << 1) | pinB;
    if (state != rotaryState) {
      if ((rotaryState == 0b00 && state == 0b01) || (rotaryState == 0b01 && state == 0b11) || (rotaryState == 0b11 && state == 0b10) || (rotaryState == 0b10 && state == 0b00)) {
        rotaryValue++;
      } else {
        rotaryValue--;
      }
      rotaryState = state;
    }
    lcd.clear();
    lcd.print(F("Rotary Encoder"));
    lcd.setCursor(0, 1);
    lcd.print(F("Value: "));
    lcd.print(rotaryValue);
  }
  if (digitalRead(buttonPin) == LOW) {
    return true;
  }
  return false;
}

bool testDHT11Sensor() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sensorInterval) {
    previousMillis = currentMillis;
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    if (isnan(humidity) || isnan(temperature)) {
      lcd.clear();
      lcd.print(F("Failed to read"));
      lcd.setCursor(0, 1);
      lcd.print(F("from DHT sensor!"));
    } else {
      lcd.clear();
      lcd.print(F("DHT11 Sensor"));
      lcd.setCursor(0, 1);
      lcd.print(F("Temp: "));
      lcd.print(temperature);
      lcd.print(F("C Hum: "));
      lcd.print(humidity);
      lcd.print(F("%"));
    }
  }
  if (digitalRead(buttonPin) == LOW) {
    return true;
  }
  return false;
}

bool testIRReceiver() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= sensorInterval) {
    previousMillis = currentMillis;
    if (irrecv.decode(&results)) {
      lcd.clear();
      lcd.print(F("IR Receiver"));
      lcd.setCursor(0, 1);
      lcd.print(F("Code: 0x"));
      lcd.print(results.value, HEX);
      irrecv.resume(); // Receive the next value
    } else {
      lcd.clear();
      lcd.print(F("IR Receiver"));
      lcd.setCursor(0, 1);
      lcd.print(F("No data received"));
    }
  }
  if (digitalRead(buttonPin) == LOW) {
    return true;
  }
  return false;
}