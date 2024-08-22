#include <Wire.h> // for I2C communicaton used by BMP280
#include <SPI.h> // SPI communication used by NRF
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h> // for controlling servo motors
#include <Adafruit_BMP280.h> // for interfacing with the BMP280 

// NRF24L01+ Setup
RF24 radio(9, 10); // RF24 object -> CE = 9, CSN = 10 - NRF control
const byte address[6] = "00001"; // address for communication

// BMP280 Setup
Adafruit_BMP280 bmp; // creates an object to manage the BMP280 sensor

//Joystick struct
struct JoystickData {
  int xLeft;
  int yLeft;
  int xRight;
  int yRight;
};

// Declare a  variable of JoystickData type to store received data
JoystickData joystickData;
//Creates an objects of Servo class
Servo esc;     // ESC 
Servo servo1;  //  D2
Servo servo2;  //  D4
Servo servo3;  // D5

// LED var.
const int ledPinGreen = 7; // G LED -> D7
const int ledPinRed = 6;   // R LED -> D6

// Var. to store 
float lastAltitude = 0.0;

void setup() {
  Serial.begin(9600);

  // Initialize BMP280
  if (!bmp.begin(0x76)) {  // initializes the BMP280 sensor using I2C address 0x76 -> returns false if the sensor cannot be initalized
    Serial.println(F("BMP280 NE DELA!"));
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);
  /*  MODE_NORMAL: Continuous measurement mode.
  SAMPLING_X2: Oversampling x2 for temperature.
  SAMPLING_X16: Oversampling x16 for pressure.
  FILTER_X16: A filter with coefficient x16 to reduce noise.
  STANDBY_MS_500: Sets the standby time between measurements to 500 milliseconds.  */ 
  // Initialize radio for NRF24L01+
  radio.begin();
  if (!radio.isChipConnected()) {
    Serial.println("NRF24L01 NE DELA.");
    while (1); 
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(0x76);
  radio.openReadingPipe(0, address); // Opens a reading pipe on address 00001 
  radio.enableDynamicPayloads();
  radio.setAutoAck(false);
  radio.startListening();  

  // Attach servos
  esc.attach(2); // ESC - D2
  servo1.attach(3); // Servo1 - D3
  servo2.attach(4); // Servo2 - D4
  servo3.attach(5); // Servo3 - D5
  
  // Initialize servos to central position
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);

  // Set up LED pins
  pinMode(ledPinGreen, OUTPUT);
  pinMode(ledPinRed, OUTPUT);

}

void loop() {
  static int lastEscSignal = 1500; // Default stop signal - static to keep the value between function calls

  if (radio.available()) {
    radio.read(&joystickData, sizeof(joystickData)); // takes the address of joystickdata and the size of the structure

    if (joystickData.yLeft >= 504) {
      lastEscSignal = map(joystickData.yLeft, 504, 1023, 1500, 2000);
    } else {
      lastEscSignal = 1500; // Stop
    }

    int servo1Pos = map(joystickData.xRight, 0, 1023, 0, 120);
    int servo3Pos = map(joystickData.xRight, 0, 1023, 0, 120);
    int servo2Pos = map(joystickData.yRight, 0, 1023, 180, 0);

    servo1.write(servo1Pos);
    servo3.write(servo3Pos);
    servo2.write(servo2Pos);
  }
  
  // Continuously send the last known signal to the ESC - constant motor speed control
  esc.writeMicroseconds(lastEscSignal);

  // Read BMP280 sensor data
  float currentAltitude = bmp.readAltitude(1013.25); // Trbovlje atmospheric pressure
  if (currentAltitude > lastAltitude) {
    digitalWrite(ledPinGreen, HIGH);
    digitalWrite(ledPinRed, LOW);
  } else if (currentAltitude < lastAltitude) {
    digitalWrite(ledPinGreen, LOW);
    digitalWrite(ledPinRed, HIGH);
  }
  lastAltitude = currentAltitude; // Update the last altitude

  /*Serial.print(F("Temp = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Altitude = "));
  Serial.print(currentAltitude);
  Serial.println(" m");

  Serial.println();*/

  delay(20); 
}
