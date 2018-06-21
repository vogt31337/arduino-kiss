#include <avr/power.h>
#include <avr/sleep.h>
#include <Arduino.h>
#include "cc1101.h" // CC1101 device
#include "ccpacket.h"

// as debugging via serial is not possible (it is used for the kiss
// protocol), I use a couple of LEDs
#define pinLedError 3
#define pinLedRecv  4
#define pinLedSend  5

#define CC1101Interrupt 0
#define CC1101_GDO0 2

#define FEND  0xC0

// this is an example implementation using a "PanStamp"-driver for
// CC1101 radio.
// PanStamp: https://github.com/panStamp/arduino_avr.git
// Port to arduino: https://github.com/veonik/arduino-cc1101.git

CC1101 radio;
CCPACKET packet;

boolean packetAvailable = false;
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

void getRadio() {
  CCPACKET packet;
  if (radio.receiveData(&packet) > 0) {
    if(packet.crc_ok) {
      Serial.write(byte(FEND));
      Serial.write(byte(0x00));
      for(uint8_t i=0; i<packet.length; i++) {
        Serial.write(packet.data[i]);
      }
      Serial.write(byte(FEND));
      Serial.flush();
    }
  }
  packetAvailable = false;
}

/* Handle interrupt from CC1101 (INT0) gdo0 on pin2 */
void cc1101signalsInterrupt(void){
//ISR (PCINT0_vect) {
  packetAvailable = true;
  freqOffset = updateFreqOffset(radio.readReg(CC1101_FREQEST, CC1101_STATUS_REGISTER));
}

bool initRadio() {
	radio.init();
	delay(100);

	digitalWrite(pinLedRecv, LOW);
	digitalWrite(pinLedSend, LOW);
	digitalWrite(pinLedError, LOW);

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

void sendData() {
//  detachInterrupt(CC1101_GDO0);
  radio.sendData(packet);
  freqOffset = updateFreqOffset(radio.readReg(CC1101_FREQEST, CC1101_STATUS_REGISTER));
  packet.length = 0;
//  attachInterrupt(CC1101_GDO0, cc1101signalsInterrupt, FALLING);
}

void serialEvent() {
  if(Serial.available() > 0) {
    byte ch = (byte)Serial.read();
    if (ch == byte(0x00) && packet.length == 0) {
      //just do nothing it's the TNC address.
    } else if (ch != byte(FEND)) { // ignore FEND characters.
      packet.length++;
      packet.data[packet.length] = ch;
    } else if (ch == byte(FEND) && packet.length > 0) {
      sendData();
    }
  }

  if (packet.length == CCPACKET_DATA_LEN) {
    sendData();
  }
}

void setup() {
	Serial.begin(115200);
  attachInterrupt(CC1101_GDO0, cc1101signalsInterrupt, FALLING);
  
	pinMode(pinLedRecv, OUTPUT);
	digitalWrite(pinLedRecv, HIGH);
	pinMode(pinLedSend, OUTPUT);
	digitalWrite(pinLedSend, HIGH);
	pinMode(pinLedError, OUTPUT);
	digitalWrite(pinLedError, HIGH);

  // <save_power> taken from: 
  // http://www.fiz-ix.com/2012/11/save-power-by-disabling-arduino-peripherals/
 
  // Disable the ADC by setting the ADEN bit (bit 7)  of the
  // ADCSRA register to zero.
  ADCSRA = ADCSRA & B01111111;

  // Disable the analog comparator by setting the ACD bit
  // (bit 7) of the ACSR register to one.
  ACSR = B10000000;

  // Disable digital input buffers on all analog input pins
  // by setting bits 0-5 of the DIDR0 register to one.
  // Of course, only do this if you are not using the analog 
  // inputs for your project.
  DIDR0 = DIDR0 | B00111111;

  // turn off everything
  power_all_disable();

  // and repower everything we need.
  power_spi_enable();
  power_usart0_enable();

  // </save_power>
  
  initRadio();

  packet.length = 0;
}

void sleepNow() {
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  attachInterrupt(CC1101_GDO0, cc1101signalsInterrupt, FALLING);
  sleep_mode();
  sleep_disable();  
}

void fill(CCPACKET packet, char ch) {
  for(byte i = packet.length; i < 62; i++) {
    packet.data[i] = ch;
  }
  packet.length = 62;
}

void loop() {
  if (packetAvailable) {
    getRadio();
  } else {
    delay(100);
    sleepNow();
  }
}

// Test:
// C0 00 82 A0 88 A4 62 68 E0 88 8E 6A 9A AC 40 6E AE 92 88 8A 62 40 62 AE 92 88 8A 64 40 63 03 F0 3D 35 31 31 39 2E 35 20 4E 2F 30 30 39 32 36 2E 30 20 45 24 30 30 31 2E 30 30 30 4D 48 7A 20 C0 
// C0 00 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 C0
