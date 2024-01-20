#include <OneWire.h>	// For DS18B20

enum Pins {
  PIN_BUTTON = 11,
  PIN_TEMPERATURE = 12,
  PIN_WATER_POWER = 13
};

enum CommandID : uint8_t {
	NONE,
	READ_DATA,
	ENABLE_WATER_HEATER,
  DISABLE_WATER_HEATER
};

struct SerialData {
	int16_t water_temp_dc = 0; // deciCelcius
  uint8_t heater_on = 0;
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

  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Serial.begin(9600);
}

void startHeating() {
  digitalWrite(PIN_WATER_POWER, HIGH);
  sdata.heater_on = true;
}
void stopHeating() {
  digitalWrite(PIN_WATER_POWER, LOW);
  sdata.heater_on = false;
}

constexpr unsigned long stateUpdatePeriod = 5000ul;

void runCommand(CommandID command) {
	switch(command) {
	case READ_DATA:
		Serial.write((uint8_t*)(&sdata), sizeof(SerialData));
		break;
	case ENABLE_WATER_HEATER:
		startHeating();
		break;
  case DISABLE_WATER_HEATER:
    stopHeating();
    break;
	}
}


int16_t maxTempCycle = 0;
void heatCycle() {
  startHeating();
  maxTempCycle = 0;
}

void updateState() {
	sdata.water_temp_dc = GetTemperature_dC();
  if(sdata.water_temp_dc > maxTempCycle) maxTempCycle = sdata.water_temp_dc;

  if(sdata.heater_on && sdata.water_temp_dc + 4 < maxTempCycle) { // Heating up but temperature droping means we hit water heater target temp
    stopHeating(); // Heat cycle is done, stop water heater
  }
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
  if(millis() - lastStateUpdate > stateUpdatePeriod) {
    updateState();

    lastStateUpdate = millis();
  }
  if(digitalRead(PIN_BUTTON) == LOW && !sdata.heater_on) {
    unsigned long buttonStart = millis();
    while(millis() - buttonStart < 500) {
      if(digitalRead(PIN_BUTTON) != LOW) return;  // Wait for a clear low signal
    }
    heatCycle();
  }
}
