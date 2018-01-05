#include "kiss.h"
#include <Arduino.h>
#include <ccpacket.h>
#include <cc1101.h> // CC1101 device

// as debugging via serial is not possible (it is used for the kiss
// protocol), I use a couple of LEDs
#define pinLedError 3
#define pinLedRecv 4
#define pinLedSend 5
#define pinLedHB 6
#define CC1101_GDO0 2

// this is an example implementation using a "PanStamp"-driver for
// CC1101 radio.
// PanStamp: https://github.com/panStamp/arduino_avr.git
// Port to arduino: https://github.com/veonik/arduino-cc1101.git
CC1101 cc1101;
bool packetAvailable = false;
byte syncWord[2] = {199, 10};

/* Handle interrupt from CC1101 (INT0) gdo0 on pin2 */
void cc1101signalsInterrupt(void){
// set the flag that a package is available
  packetAvailable = true;
}

// the 'kiss'-class requires a couple of callback functions. these
// functions do the low-level work so that the kiss-class is generic

// a call-back function which you can adjust into something that
// peeks in the radio-buffer if anything is waiting
bool peekRadio() {
	return packetAvailable;
}

// if there's data in your radio, then this callback should retrieve it
void getRadio(uint8_t *const whereTo, uint16_t *const n) {
//	uint8_t dummy = *n;
//	rf95.recv(whereTo, &dummy);
//	*n = dummy;
  detachInterrupt(cc1101signalsInterrupt);
  CCPACKET packet;
  if (cc1101.receiveData(&packet) > 0) {
    if(packet.crc_ok) {
      *n = packet.length;
      memcpy(whereTo, packet.data, packet.length);
    }
//    else {
//      k.debug("crc not ok!");
//  }
  }
  packetAvailable = false;
  attachInterrupt(CC1101_GDO0, cc1101signalsInterrupt, FALLING);
}

void putRadio(const uint8_t *const what, const uint16_t size) {
  detachInterrupt(cc1101signalsInterrupt);
  CCPACKET packet;
  packet.length = size + 3; // some room for special chars: crc, ...
  memcpy(packet.data, what, size);

  if (cc1101.sendData(packet)) {
//    k.debug("sent ok.");
//  } else {
//    k.debug("sent failed.");
  }
//	rf95.send(what, size);
//	rf95.waitPacketSent();
  attachInterrupt(CC1101_GDO0, cc1101signalsInterrupt, FALLING);
}

// some arduino-platforms (teensy, mega) have multiple serial ports
// there you need to replace Serial by e.g. Serial2 or so
uint16_t peekSerial() {
	return Serial.available();
}

bool getSerial(uint8_t *const whereTo, const uint16_t n, const unsigned long int to) {
	for(uint16_t i=0; i<n; i++) {
		while(!Serial.available()) {
			if (millis() >= to)
				return false;
		}

		whereTo[i] = Serial.read();
	}

	return true;
}

void putSerial(const uint8_t *const what, const uint16_t size) {
	Serial.write(what, size);
}

bool initRadio() {
	cc1101.init();
		delay(100);

		digitalWrite(pinLedRecv, LOW);
		digitalWrite(pinLedSend, LOW);
		digitalWrite(pinLedError, LOW);
		digitalWrite(pinLedHB, LOW);

    cc1101.setSyncWord(syncWord);
    cc1101.setCarrierFreq(CFREQ_433);
    cc1101.disableAddressCheck();
    cc1101.setChannel(0);
    cc1101.setTxPowerAmp(PA_LongDistance);

		return true;
}

bool resetRadio() {
	cc1101.reset();
	delay(5); // 5ms is required

	return initRadio();
}

// CC1101 device can have a packetsize of CCPACKET_BUFFER_LEN (64) bytes
kiss k(CCPACKET_BUFFER_LEN, peekRadio, getRadio, putRadio, peekSerial, getSerial, putSerial, resetRadio, pinLedRecv, pinLedSend, pinLedError);

void setup() {
	// the arduino talks with 9600bps to the linux system
	Serial.begin(9600);
  attachInterrupt(CC1101_GDO0, cc1101signalsInterrupt, FALLING);
  
	pinMode(pinLedRecv, OUTPUT);
	digitalWrite(pinLedRecv, HIGH);
	pinMode(pinLedSend, OUTPUT);
	digitalWrite(pinLedSend, HIGH);
	pinMode(pinLedError, OUTPUT);
	digitalWrite(pinLedError, HIGH);
	pinMode(pinLedHB, OUTPUT);
	digitalWrite(pinLedHB, HIGH);

	k.begin();

	if (!resetRadio())
		k.debug("Radio init failed");

	k.debug("Go!");
}

void loop() {
	k.loop();

	const unsigned long int now = millis();
	static unsigned long int pHB = 0;

	if (now - pHB >= 500) {
		static bool state = true;
		digitalWrite(pinLedHB, state ? HIGH : LOW);
		state = !state;
		pHB = now;
	}

	static unsigned long int lastReset = 0;
	const unsigned long int resetInterval = 301000; // every 5 min
	if (now - lastReset >= resetInterval) {
		k.debug("Reset radio");

		if (!resetRadio()) {
			for(byte i=0; i<3; i++) {
				digitalWrite(pinLedError, HIGH);
				delay(250);
				digitalWrite(pinLedError, LOW);
				delay(250);
			}
		}

		lastReset = now;
	}
}
