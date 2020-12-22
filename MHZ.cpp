/* MHZ library

    By Tobias Schürg
*/

#include "MHZ.h"

const int MHZ14A = 14;
const int MHZ19B = 19;

const unsigned long MHZ14A_PREHEATING_TIME = 3L * 60L * 1000L;
const unsigned long MHZ19B_PREHEATING_TIME = 3L * 60L * 1000L;

const unsigned long MHZ14A_RESPONSE_TIME = 60 * 1000;
const unsigned long MHZ19B_RESPONSE_TIME = 120 * 1000;

const int STATUS_NO_RESPONSE = -2;
const int STATUS_CHECKSUM_MISMATCH = -3;
const int STATUS_INCOMPLETE = -4;
const int STATUS_NOT_READY = -5;
const int STATUS_PWM_NOT_CONFIGURED = -6;
const int STATUS_SERIAL_NOT_CONFIGURED = -7;
const int STATUS_BAD_SPAN_VALUE = -8;

unsigned long lastRequest = 0;

bool SerialConfigured = true;
bool PwmConfigured = true;

MHZ::MHZ(uint8_t rxpin, uint8_t txpin, uint8_t pwmpin, uint8_t type) {
  SoftwareSerial * ss = new SoftwareSerial(rxpin, txpin);
  _pwmpin = pwmpin;
  _type = type;

  ss->begin(9600);
  _serial = ss;
}

MHZ::MHZ(uint8_t rxpin, uint8_t txpin, uint8_t type) {
  SoftwareSerial * ss = new SoftwareSerial(rxpin, txpin);
  _type = type;

  ss->begin(9600);
  _serial = ss;

  PwmConfigured = false;
}

MHZ::MHZ(uint8_t pwmpin, uint8_t type) {
  _pwmpin = pwmpin;
  _type = type;

  SerialConfigured = false;
}

MHZ::MHZ(Stream * serial, uint8_t pwmpin, uint8_t type) {
  _serial = serial;
  _pwmpin = pwmpin;
  _type = type;
}

MHZ::MHZ(Stream * serial, uint8_t type) {
  _serial = serial;
  _type = type;

  PwmConfigured = false;
}

/**
 * Enables or disables the debug mode (more logging).
 */
void MHZ::setDebug(boolean enable) {
  debug = enable;
  if (debug) {
    Serial.println(F("MHZ: debug mode ENABLED"));
  } else {
    Serial.println(F("MHZ: debug mode DISABLED"));
  }
}

boolean MHZ::isPreHeating() {
  if (_type == MHZ14A) {
    return millis() < (MHZ14A_PREHEATING_TIME);
  } else if (_type == MHZ19B) {
    return millis() < (MHZ19B_PREHEATING_TIME);
  } else {
    Serial.println(F("MHZ::isPreHeating() => UNKNOWN SENSOR"));
    return false;
  }
}

boolean MHZ::isReady() {
  if (isPreHeating()) return false;
  if (_type == MHZ14A)
    return lastRequest < millis() - MHZ14A_RESPONSE_TIME;
  else if (_type == MHZ19B)
    return lastRequest < millis() - MHZ19B_RESPONSE_TIME;
  else {
    Serial.print(F("MHZ::isReady() => UNKNOWN SENSOR \""));
    Serial.print(_type);
    Serial.println(F("\""));
    return true;
  }
}

int MHZ::setCO2ABCmode(boolean enable) {
  if (!SerialConfigured) {
    if (debug) Serial.println(F("-- serial is not configured"));
    return STATUS_SERIAL_NOT_CONFIGURED;
  }
  if (!isReady()) return STATUS_NOT_READY;
  byte cmd[9] = {0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86};
  // basic command structure, change byte 3 to set mode
  // Byte 3 0xA0 = ABC on; Byte3 is 0x00 = ABC off
  // Also need to change checksum
  if (enable) {
    if (debug) Serial.println(F("-- Enable CO2 auto-baseline = ABC on ---"));
    cmd[3] = 0xA0;
    cmd[8] = 0xE6;
  } else {
    if (debug) Serial.println(F("-- Disable CO2 auto-baseline = ABC off ---"));
    // note command above was for off, so no change
  }
  if (debug) Serial.print(F("  >> Sending CO2 ABC mode request"));
  _serial->write(cmd, 9);  // send command to set ABC mode
  lastRequest = millis();
  delay(100);  // wait a short moment to avoid false reading
}

int MHZ::setCO2background() {
  // From the manual: ZERO POINT is 400PPM, PLS MAKE SURE
  // THE SENSOR HAD BEEN WORKED UNDER 400PPM FOR OVER 20MINUTES
  if (!SerialConfigured) {
    if (debug) Serial.println(F("-- serial is not configured"));
    return STATUS_SERIAL_NOT_CONFIGURED;
  }
  if (!isReady()) return STATUS_NOT_READY;
  byte cmd[9] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};
  if (debug) Serial.println(F("-- manually setting CO2 background ---"));
  if (debug) Serial.print(F("  >> Sending CO2 background request"));
  _serial->write(cmd, 9);  // set current level to background
  lastRequest = millis();
  delay(100);  // wait a short moment to avoid false reading
}

int MHZ::setCO2span(int span_ppm) {
  // From the manual:
  // example: SPAN is 2000ppm，HIGH = 2000 / 256；LOW = 2000 % 256
  // Note: Pls do ZERO calibration before span calibration
  // Please make sure the sensor worked under a certain level co2
  // for over 20 minutes. Suggest using 2000ppm as span, at least 1000ppm

  if (span_ppm < 1000 or span_ppm > 5000) {
    if (debug) Serial.println(F("span value out of range use 1000 <= span < 5000 ppm "));
    return STATUS_BAD_SPAN_VALUE;
  }
  if (!SerialConfigured) {
    if (debug) Serial.println(F("-- serial is not configured"));
    return STATUS_SERIAL_NOT_CONFIGURED;
  }
  if (!isReady()) return STATUS_NOT_READY;
  byte cmd[9] = {0xFF, 0x01, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  // fill in span value in bytes 3 and 4
  cmd[3] = span_ppm / 256;
  cmd[4] = span_ppm % 256;
  // calculate checksum into byte 8
  byte check = getCheckSum(cmd);
  cmd[8] = check;
  if (debug) {
    Serial.print(F("MHZ span checksum is: "));
    Serial.println(cmd[8], HEX);
    Serial.println(F("-- manually setting CO2 span ---"));
    Serial.print(F("  >> Sending CO2 span request"));
  }
  _serial->write(cmd, 9);  // set current level to span_ppm
  lastRequest = millis();
  delay(100);  // wait a short moment to avoid false reading
}

int MHZ::readCO2UART() {
  if (!SerialConfigured) {
    if (debug) Serial.println(F("-- serial is not configured"));
    return STATUS_SERIAL_NOT_CONFIGURED;
  }
  if (!isReady()) return STATUS_NOT_READY;
  if (debug) Serial.println(F("-- read CO2 uart ---"));
  byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  byte response[9];  // for answer

  if (debug) Serial.print(F("  >> Sending CO2 request"));
  _serial->write(cmd, 9);  // request PPM CO2
  lastRequest = millis();

  // clear the buffer
  memset(response, 0, 9);

  int waited = 0;
  while (_serial->available() == 0) {
    if (debug) Serial.print(".");
    delay(100);  // wait a short moment to avoid false reading
    if (waited++ > 10) {
      if (debug) Serial.println(F("No response after 10 seconds"));
      _serial->flush();
      return STATUS_NO_RESPONSE;
    }
  }
  if (debug) Serial.println();

  // The serial stream can get out of sync. The response starts with 0xff, try
  // to resync.
  // TODO: I think this might be wrong any only happens during initialization?
  boolean skip = false;
  while (_serial->available() > 0 && (unsigned char)_serial->peek() != 0xFF) {
    if (!skip) {
      Serial.print(F("MHZ: - skipping unexpected readings:"));
      skip = true;
    }
    Serial.print(" ");
    Serial.print(_serial->peek(), HEX);
    _serial->read();
  }
  if (skip) Serial.println();

  if (_serial->available() > 0) {
    int count = _serial->readBytes(response, 9);
    if (count < 9) {
      _serial->flush();
      return STATUS_INCOMPLETE;
    }
  } else {
    _serial->flush();
    return STATUS_INCOMPLETE;
  }

  if (debug) {
    // print out the response in hexa
    Serial.print(F("  << "));
    for (int i = 0; i < 9; i++) {
      Serial.print(response[i], HEX);
      Serial.print(F("  "));
    }
    Serial.println(F(""));
  }

  // checksum
  byte check = getCheckSum(response);
  if (response[8] != check) {
    Serial.println(F("MHZ: Checksum not OK!"));
    Serial.print(F("MHZ: Received: "));
    Serial.println(response[8], HEX);
    Serial.print(F("MHZ: Should be: "));
    Serial.println(check, HEX);
    temperature = STATUS_CHECKSUM_MISMATCH;
    _serial->flush();
    return STATUS_CHECKSUM_MISMATCH;
  }

  int ppm_uart = 256 * (int)response[2] + response[3];

  temperature = response[4] - 40;  // was - 44, but I think that was incorrect;

  byte status = response[5];
  if (debug) {
    Serial.print(F(" # PPM UART: "));
    Serial.println(ppm_uart);
    Serial.print(F(" # Temperature? "));
    Serial.println(temperature);
  }

  // Is always 0 for version 14a  and 19b
  // Version 19a?: status != 0x40
  if (debug && status != 0) {
    Serial.print(F(" ! Status maybe not OK ! "));
    Serial.println(status, HEX);
  } else if (debug) {
    Serial.print(F(" Status  OK: "));
    Serial.println(status, HEX);
  }

  _serial->flush();
  return ppm_uart;
}

int MHZ::getLastTemperature() {
  if (!SerialConfigured) {
    if (debug) Serial.println(F("-- serial is not configured"));
    return STATUS_SERIAL_NOT_CONFIGURED;
  }
  if (isPreHeating()) return STATUS_NOT_READY;
  return temperature;
}

byte MHZ::getCheckSum(byte* packet) {
  if (!SerialConfigured) {
    if (debug) Serial.println(F("-- serial is not configured"));
    return STATUS_SERIAL_NOT_CONFIGURED;
  }
  if (debug) Serial.println(F("  getCheckSum()"));
  byte i;
  unsigned char checksum = 0;
  for (i = 1; i < 8; i++) {
    checksum += packet[i];
  }
  checksum = 0xff - checksum;
  checksum += 1;
  return checksum;
}

int MHZ::readCO2PWM() {
  if (!PwmConfigured) {
    if (debug) Serial.println(F("-- pwm is not configured "));
    return STATUS_PWM_NOT_CONFIGURED;
  }
  //if (!isReady()) return STATUS_NOT_READY; not needed?
  if (debug) Serial.print(F("-- reading CO2 from pwm "));
  unsigned long th, tl, ppm_pwm = 0;
  do {
    if (debug) Serial.print(".");
    th = pulseIn(_pwmpin, HIGH, 1004000) / 1000;
    tl = 1004 - th;
    ppm_pwm = 2000 * (th - 2) / (th + tl - 4);
  } while (th == 0);
  if (debug) {
    Serial.print(F("\n # PPM PWM: "));
    Serial.println(ppm_pwm);
  }
  return ppm_pwm;
}
