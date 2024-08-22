#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10); // RF24 object -> CE = 9, CSN = 10 - NRF control
const byte address[6] = "00001"; // address for communication

//Joystick struct
struct JoystickData {
  int xLeft;
  int yLeft;
  int xRight;
  int yRight;
};

// Declaration of global variable joystickData of type JoystickData
JoystickData joystickData;

void setup() {
  Serial.begin(9600);
  radio.begin();
  /*if (!radio.isChipConnected()) {
    Serial.println("NRF24L01 NE DELUJE!!!!");
    while (1) {} 
  }*/

  radio.setPALevel(RF24_PA_HIGH);    // Power Amplifier to High
  radio.setChannel(0x76);           // Set the channel to avoid interference
  radio.openWritingPipe(address);   // Open a writing pipe to an address
  radio.enableDynamicPayloads();    // Enable dynamic payloads
  radio.setAutoAck(false);          // Disable auto acknowledgment
  radio.stopListening();            // Set to transmitter mode

  // Initialize joystick pins
  pinMode(A0, INPUT); // xLeft
  pinMode(A1, INPUT); // yLeft
  pinMode(A2, INPUT); // xRight
  pinMode(A3, INPUT); // yRight

  
}

void loop() {
  // Read joystick values
  joystickData.xLeft = analogRead(A0); 
  joystickData.yLeft = analogRead(A1); 
  joystickData.xRight = analogRead(A2); 
  joystickData.yRight = analogRead(A3); 

  bool report = radio.write(&joystickData, sizeof(joystickData)); // Send the data
  /*if (report) {
    Serial.print("XLeft: ");
    Serial.print(joystickData.xLeft);
    Serial.print(" YLeft: ");
    Serial.print(joystickData.yLeft);
    Serial.print(" | XRight: ");
    Serial.print(joystickData.xRight);
    Serial.print(" YRight: ");
    Serial.println(joystickData.yRight);
  } else {
    Serial.println("Sending failed");
  }*/
  delay(20); 
}