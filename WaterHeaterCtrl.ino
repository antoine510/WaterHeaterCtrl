#include <OneWire.h>	// For DS18B20

enum Pins {
  PIN_LED = 13,
  PIN_TEMPERATURE = 12,
  PIN_WATER_POWER = 11,
};

enum CommandID : uint8_t {
	NONE,
	READ_DATA,
	ENABLE_WATER_HEATER,
  DISABLE_WATER_HEATER
};

struct SerialData {
	int16_t water_temp_dc = 0; // deciCelcius
};
SerialData sdata;

int16_t GetTemperature_dC() {
  static OneWire tsens(PIN_TEMPERATURE);

	tsens.reset();
	tsens.skip();
	tsens.write(0xbe);
	byte l = tsens.read(), h = tsens.read();
	int16_t res = ((int16_t)(l | (h << 8)) * 10 + 5) >> 4;

	tsens.reset();
	tsens.skip();
	tsens.write(0x44);	// Start next measurement
	return res;
}

void setup() {
  pinMode(PIN_WATER_POWER, OUTPUT);
  digitalWrite(PIN_WATER_POWER, LOW);

  pinMode(PIN_LED, OUTPUT);

  Serial.begin(9600);
}

constexpr unsigned long stateUpdatePeriod = 1000ul;

void runCommand(CommandID command) {
	switch(command) {
	case READ_DATA:
		Serial.write((uint8_t*)(&sdata), sizeof(SerialData));
		break;
	case ENABLE_WATER_HEATER:
		digitalWrite(PIN_WATER_POWER, HIGH);
		break;
  case DISABLE_WATER_HEATER:
    digitalWrite(PIN_WATER_POWER, LOW);
    break;
	}
}

void updateState() {
	sdata.water_temp_dc = GetTemperature_dC();
}


enum SerialSeq : uint8_t {
	MAGIC1, MAGIC2, COMMAND
} serial_state = MAGIC1;

uint8_t updateSerialState(uint8_t byte) {
  static constexpr const uint8_t cmd_magic[] = {0x4f, 0xc7};
  switch(serial_state) {
		case MAGIC1: return byte == cmd_magic[0] ? MAGIC2 : MAGIC1;
		case MAGIC2: return byte == cmd_magic[1] ? COMMAND : MAGIC1;
		case COMMAND: runCommand((CommandID)byte); return MAGIC1;
	}
}

void serialEvent() {
	do {
    serial_state = (SerialSeq)updateSerialState(Serial.read());
	} while(Serial.available());
}

void loop() {
  static unsigned long lastStateUpdate = 0;
  static bool led_on = false;
  if(millis() - lastStateUpdate > stateUpdatePeriod) {
    updateState();

    digitalWrite(PIN_LED, led_on);
    led_on = !led_on;

    lastStateUpdate = millis();
  }
}
