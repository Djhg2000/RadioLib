/*
   RadioLib CC1101 Transmit with Interrupts Example

   This example transmits FSK packets with one second delays
   between them. Each packet contains up to 64 bytes
   of data, in the form of:
    - Arduino String
    - null-terminated char array (C-string)
    - arbitrary binary data (byte array)
*/

// include the library
#include <RadioLib.h>

// CC1101 module is in slot A on the shield
CC1101 cc = RadioShield.ModuleA;

void setup() {
  Serial.begin(9600);

  // initialize CC1101 with default settings
  Serial.print(F("[CC1101] Initializing ... "));
  // carrier frequency:                   868.0 MHz
  // bit rate:                            4.8 kbps
  // Rx bandwidth:                        325.0 kHz
  // frequency deviation:                 48.0 kHz
  // sync word:                           0xD391
  int state = cc.begin();
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // set the function that will be called 
  // when packet transmission is finished
  cc.setGdo0Action(setFlag);

  // start transmitting the first packet
  Serial.print(F("[CC1101] Sending first packet ... "));

  // you can transmit C-string or Arduino string up to
  // 64 characters long
  state = cc.startTransmit("Hello World!");

  // you can also transmit byte array up to 64 bytes long
  /*
    byte byteArr[] = {0x01, 0x23, 0x45, 0x56,
                      0x78, 0xAB, 0xCD, 0xEF};
    state = cc.transmit(byteArr, 8);
  */
  
  if (state != ERR_NONE) {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}

// flag to indicate that a packet was received
volatile bool transmittedFlag = false;

void setFlag(void) {
  // packet transmission is finished, set the flag
  transmittedFlag = true;
}

void loop() {
  // check if the previous transmission finished
  if(transmittedFlag) {
    Serial.println(F("[CC1101] Packet transmission finished!"));

    // wait one second before next transmission
    delay(1000);

    // send another packet
    Serial.print(F("[CC1101] Sending another packet ... "));

    // you can transmit C-string or Arduino string up to
    // 64 characters long
    int state = cc.startTransmit("Hello World!");
  
    // you can also transmit byte array up to 256 bytes long
    /*
      byte byteArr[] = {0x01, 0x23, 0x45, 0x56,
                        0x78, 0xAB, 0xCD, 0xEF};
      int state = cc.transmit(byteArr, 8);
    */
    
    if (state != ERR_NONE) {
      Serial.print(F("failed, code "));
      Serial.println(state);
    }
  }

}
