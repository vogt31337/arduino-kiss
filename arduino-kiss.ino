#include <Arduino.h>
#include "cc1101.h" // CC1101 device
#include "ccpacket.h"

// as debugging via serial is not possible (it is used for the kiss
// protocol), I use a couple of LEDs
#define pinLedError 3
#define pinLedRecv 4
#define pinLedSend 5
#define pinLedHB 17

#define CC1101Interrupt 0
#define CC1101_GDO0 2

#define FEND  0xC0

// this is an example implementation using a "PanStamp"-driver for
// CC1101 radio.
// PanStamp: https://github.com/panStamp/arduino_avr.git
// Port to arduino: https://github.com/veonik/arduino-cc1101.git
CC1101 radio;
bool packetAvailable = false;
byte syncWord[2] = {199, 10};

#define FILTER_LENGTH 4
word freq_reg = 0;
byte freqOffset = 0;

byte updateFreqOffset(byte input) {
  if (abs(input - freqOffset) < 50) {
    freq_reg = freq_reg - (freq_reg >> FILTER_LENGTH) + input;
  }
  return freq_reg >> FILTER_LENGTH;
}

/* Handle interrupt from CC1101 (INT0) gdo0 on pin2 */
void cc1101signalsInterrupt(void){
// set the flag that a package is available
  packetAvailable = true;
  freqOffset = updateFreqOffset(radio.readReg(CC1101_FREQEST, CC1101_STATUS_REGISTER));
}

void getRadio() {
  detachInterrupt(cc1101signalsInterrupt);
  CCPACKET packet;
  if (radio.receiveData(&packet) > 0) {
    if(packet.crc_ok) {
      Serial.write(FEND);
      Serial.write(0x00);
      for(uint8_t i=0; i<packet.length; i++) {
        Serial.write(packet.data[i]);
      }
      Serial.write(FEND);
    }
  }
  packetAvailable = false;
  attachInterrupt(CC1101_GDO0, cc1101signalsInterrupt, FALLING);
}

bool initRadio() {
	radio.init();
	delay(100);

	digitalWrite(pinLedRecv, LOW);
	digitalWrite(pinLedSend, LOW);
	digitalWrite(pinLedError, LOW);
	digitalWrite(pinLedHB, LOW);

  radio.setSyncWord(syncWord);
  radio.setCarrierFreq(CFREQ_433);
  radio.disableAddressCheck();
//  radio.setChannel(0);
  radio.setTxPowerAmp(PA_LongDistance);
  radio.writeReg(CC1101_FSCTRL0, freqOffset);

  putSerial("Radio init!", 11);
	return true;
}

bool resetRadio() {
	radio.reset();
	delay(5); // 5ms is required

	return initRadio();
}

void setup() {
	// the arduino talks with 9600bps to the linux system
	Serial.begin(115200);
  attachInterrupt(CC1101_GDO0, cc1101signalsInterrupt, FALLING);
  
	pinMode(pinLedRecv, OUTPUT);
	digitalWrite(pinLedRecv, HIGH);
	pinMode(pinLedSend, OUTPUT);
	digitalWrite(pinLedSend, HIGH);
	pinMode(pinLedError, OUTPUT);
	digitalWrite(pinLedError, HIGH);
	pinMode(pinLedHB, OUTPUT);
	digitalWrite(pinLedHB, HIGH);

  initRadio()
}

void loop() {
	const unsigned long int now = millis();
	static unsigned long int pHB = 0;

// heartbeating
	if (now - pHB >= 500) {
		static bool state = true;
		digitalWrite(pinLedHB, state ? HIGH : LOW);
		state = !state;
		pHB = now;
	}

  if(Serial.available() >= 64) {
    CCPACKET packet;
    packet.length = 0;
    while(paket.length < 62) {
      Char ch = Serial.read();
      // Should ignore FEND chars and the modem number.
      if (ch != FEND || (ch != 0x00 && packet.length != 0)) {
        packet.data[packet.length] = ch;
        packet.length++;
      }
    }

    radio.sendData(packet);
  }

  if (packetAvailable) {
    getRadio();
  }
}
