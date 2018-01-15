#include <Arduino.h>
#include "cc1101.h" // CC1101 device
#include "ccpacket.h"
#include "kiss.h"

// as debugging via serial is not possible (it is used for the kiss
// protocol), I use a couple of LEDs
#define pinLedError 3
#define pinLedRecv 4
#define pinLedSend 5
#define pinLedHB 17

#define CC1101Interrupt 0
#define CC1101_GDO0 2

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
  if (radio.receiveData(&packet) > 0) {
    if(packet.crc_ok) {
      *n = packet.length;
//      memcpy(whereTo, packet.data, packet.length);
      for(uint8_t i=0; i<packet.length; i++) {
        whereTo[i] = packet.data[i];
      }
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
//  memcpy(packet.data, what, size);
  for(uint8_t i=0; i<size; i++) {
    packet.data[i] = what[i];
  }

  if (radio.sendData(packet)) {
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

// CC1101 device can have a packetsize of CCPACKET_BUFFER_LEN (64) bytes
kiss k(CCPACKET_BUFFER_LEN, peekRadio, getRadio, putRadio, peekSerial, getSerial, putSerial, resetRadio, pinLedRecv, pinLedSend, pinLedError);

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

	k.begin();

	if (!initRadio())
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
