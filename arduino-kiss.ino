#include <Arduino.h>
#include "cc1101.h" // CC1101 device
#include "ccpacket.h"

// as debugging via serial is not possible (it is used for the kiss
// protocol), I use a couple of LEDs
#define pinLedError 3
#define pinLedRecv  4
#define pinLedSend  5
#define pinLedHB    13

#define CC1101Interrupt 0
#define CC1101_GDO0 2

#define FEND    0xC0
#define FESC    0xDB
#define TFEND   0xDC
#define TFESC   0xDD

// address of the modem, maybe in future it will be configurable.
#define ADDRESS 0x00

// this is an example implementation using a "PanStamp"-driver for
// CC1101 radio.
// PanStamp: https://github.com/panStamp/arduino_avr.git
// Port to arduino: https://github.com/veonik/arduino-cc1101.git

CC1101 radio;
CCPACKET packet;

boolean use_crc = false;
uint8_t state = 0;

uint16_t crc_table[] = {
    0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
    0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
    0xcc01, 0xcc0,  0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
    0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
    0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
    0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
    0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
    0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
    0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
    0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
    0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
    0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
    0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
    0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
    0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
    0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
    0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
    0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
    0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
    0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
    0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
    0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
    0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
    0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
    0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
    0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
    0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
    0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
    0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
    0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
    0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
    0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040
};

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
  detachInterrupt(CC1101_GDO0);
  CCPACKET packet;
  if (radio.receiveData(&packet) > 0) {
    if(packet.crc_ok) {
      Serial.write(byte(FEND));
      Serial.write(byte(ADDRESS));
      for(uint8_t i=0; i<packet.length; i++) {
        Serial.write(packet.data[i]);
      }
      if (use_crc) {
        Serial.write(calc_crc(packet.data, packet.length));
      }
      Serial.write(byte(FEND));
      Serial.flush();
    }
  }
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

  initRadio();

  packet.length = 0;
}

void fill(CCPACKET packet, char ch) {
  for(byte i = packet.length; i < 62; i++) {
    packet.data[i] = ch;
  }
  packet.length = 62;
}

uint16_t calc_crc(byte *buf, const uint8_t sz)
{
  int n = sz;
  uint16_t crc = 0;
  while (--n > 0)
    crc = ((crc >> 8) & 0xff) ^ crc_table[(crc ^ *buf++) & 0xff];
  return crc;
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

  if(Serial.available() > 0) {
    byte ch = (byte)Serial.read();

    switch (state) {
      // state 0 check for start character
      case 0: {
          if (ch == byte(FEND)) {
            state = 1;
          }
        }
        break;
      // state 1 start char received now check address, command, and SMACK protocol
      case 1: {
          uint8_t cmnd =  ch & 0b00001111;
          uint8_t addr = (ch & 0b01110000) >> 4;

          if ((ch & 0b1000000) == 0b1000000) { // activate SMACK
            use_crc = true;
          }
            
          if (addr == byte(ADDRESS)) { // our address transit to state 2
            state = 2;
          } else if (ch == byte(0xFF)) { // reset commmand
            use_crc = false;
            state = 0;
            packet.length = 0;
          }
        }
        break;
      // state 2 parsing incomming data
      case 2: {
            if (ch == byte(FEND)) { // end symbol received
              state = 4;
              break;
            } else if (ch == byte(FESC)) { // escaping found
              state = 3;
              break;
            }
            packet.data[packet.length] = ch;
            packet.length++;
            if (packet.length > CCPACKET_DATA_LEN) { // end of buffer
              state = 4;
            }
        }
        break;
      // state 3 parse escaping
      case 3: {
          if (ch == byte(TFEND)) {
            packet.data[packet.length] = byte(FEND);
            packet.length++;
          } else if (ch == byte(TFESC)) {
            packet.data[packet.length] = byte(FESC);
            packet.length++;
          } else { // error!
            state = 0;
            packet.length = 0;
          }
        }
        break;
      case 4: {
          detachInterrupt(CC1101_GDO0);
          if (use_crc) { // if smack, check crc...
            if (calc_crc(packet.data, packet.length) == 0) {
              packet.length = packet.length - 2; // remove crc, the cc1101 does it's own crc.
              radio.sendData(packet);            
              freqOffset = updateFreqOffset(radio.readReg(CC1101_FREQEST, CC1101_STATUS_REGISTER));
            }
          } else {
            radio.sendData(packet);
            freqOffset = updateFreqOffset(radio.readReg(CC1101_FREQEST, CC1101_STATUS_REGISTER));
          }
          packet.length = 0;
          state = 0;
          attachInterrupt(CC1101_GDO0, cc1101signalsInterrupt, FALLING);
        }
        break;
    };
  } 
  
  if (packetAvailable) {
    Serial.println("rcx");
    getRadio();
    packetAvailable = false;
  }
}

// Test:
// C0 00 82 A0 88 A4 62 68 E0 88 8E 6A 9A AC 40 6E AE 92 88 8A 62 40 62 AE 92 88 8A 64 40 63 03 F0 3D 35 31 31 39 2E 35 20 4E 2F 30 30 39 32 36 2E 30 20 45 24 30 30 31 2E 30 30 30 4D 48 7A 20 C0 
// C0 00 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 C0
