#include <Arduino.h>
#include <ModbusMaster.h>

float battChargeCurrent, battOverallCurrent, battChargePower;
float bvoltage, btemp, bremaining, lpower, lcurrent, pvvoltage, pvcurrent, pvpower;

ModbusMaster node;

void AddressRegistry_3100() {
	uint8_t result = node.readInputRegisters(0x3100, 8);

	if (result != node.ku8MBSuccess) {
		Serial.printf("Read register 0x3100 failed with error code: %d\n", result);
		return;
	}
		
	pvvoltage = node.getResponseBuffer(0x00) / 100.0f;
	Serial.printf("PV Voltage: %.2f\n", pvvoltage);

	pvcurrent = node.getResponseBuffer(0x01) / 100.0f;
	Serial.printf("PV Current: %.2f\n", pvcurrent);

	pvpower = (node.getResponseBuffer(0x03) << 16 | node.getResponseBuffer(0x02)) / 100.0f;
	Serial.printf("PV Power: %.2f\n", pvpower);
	
	bvoltage = node.getResponseBuffer(0x04) / 100.0f;
	Serial.printf("Battery Voltage: %.2f\n", bvoltage);
	
	battChargeCurrent = node.getResponseBuffer(0x05) / 100.0f;
	Serial.printf("Battery Charge Current: %.2f\n", battChargeCurrent);

	battChargePower = (node.getResponseBuffer(0x07) << 16 | node.getResponseBuffer(0x06)) / 100.0f;
	Serial.printf("Battery Charge Power: %.2f\n", battChargePower);
}


void AddressRegistry_331B() {
	uint8_t result = node.readInputRegisters(0x331B, 2);
	
	if (result != node.ku8MBSuccess) {
		Serial.printf("Read register 0x331B failed with error code: %d\n", result);
		return;
	}

	battOverallCurrent = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16) / 100.0f;
	Serial.printf("Battery Overall Current: %.2f\n", battOverallCurrent);
}

//Battery status
/*
D3-D0: 01H Overvolt , 00H Normal ,  02H Under
Volt, 03H Low Volt Disconnect, 04H Fault
D7-D4: 00H Normal, 01H Over Temp.(Higher
than the warning settings), 02H Low Temp.(
Lower than the warning settings),
D8: Battery inerternal resistance abnormal 1,
normal 0
D15: 1-Wrong identification for rated voltage
*/

void handleBatteryStatus(uint16_t batteryStatus) {
	switch (batteryStatus & 0b1111) { //D3-D0
		case 0b0000:
			Serial.printf("\n[MPPT-BATTERY] Normal\n");
			break;
		case 0b0001:
			Serial.printf("\n[MPPT-BATTERY] Overvolt\n");
			break;
		case 0b0010:
			Serial.printf("\n[MPPT-BATTERY] Undervolt\n");
			break;
		case 0b0011:
			Serial.printf("\n[MPPT-BATTERY] Low Volt Disconnect\n");
			break;
		case 0b0100:
			Serial.printf("\n[MPPT-BATTERY] Fault\n");
			break;
	};

	//D8 bit
	if (batteryStatus & (1 << 8)) {
		Serial.printf("[MPPT-BATTERY] Internal Resistance Abnormal\n");
	} else {
		Serial.printf("[MPPT-BATTERY] Internal Resistance Normal\n");
	}

	//D15 bit
	if (batteryStatus & (1 << 15)) {
		Serial.printf("[MPPT-BATTERY] Wrong Identification for Rated Voltage\n");
	}
}

/*
D15-D14: Input volt status. 00 normal, 01 no
power connected, 02H Higher volt input, 03H
Input volt error.
D13:  Charging MOSFET is short.
D12:  Charging or Anti-reverse MOSFET is short.
D11:  Anti-reverse MOSFET is short.
D10:  Input is over current.
D9:  The load is Over current.
D8:  The load is short.
D7:  Load MOSFET is short.
D4: PV Input is short.
D3-2: Charging status. 00 No charging,01 Float,02
Boost,03 Equlization.
D1: 0 Normal, 1 Fault.
D0: 1 Running, 0 Standby
*/
void handleChargingEquipmentStatus(uint16_t chargingEquipmentStatus) {

	if (chargingEquipmentStatus & 1) { //D0
		Serial.printf("\n[MPPT-PV] Equipment-Running\n");
	} else {
		Serial.printf("\n[MPPT-PV] Equipment-Standby\n");
	}

	if (chargingEquipmentStatus & (1 << 1)) { //D1
		Serial.printf("[MPPT-PV] Equipment-Status-Fault\n"); //If load current is not connected, but battery is being discharged, it will be considered as fault.
	} else {
		Serial.printf("[MPPT-PV] Equipment-Status-Normal\n"); //
	}

	switch ((chargingEquipmentStatus >> 2) & 0b11) { //D3-D2
		case 0b00:
			Serial.printf("[MPPT-PV] Charging-None\n");
			break;
		case 0b01:
			Serial.printf("[MPPT-PV] Charging-Float\n");
			break;
		case 0b10:
			Serial.printf("[MPPT-PV] Charging-Boost\n");
			break;
		case 0b11:
			Serial.printf("[MPPT-PV] Charging-Equalization\n");
			break;
	}

	if (chargingEquipmentStatus & (1 << 4)) { //D4
		Serial.printf("[MPPT-PV] Input-Short\n");
	}

	//Skip loads

	if (chargingEquipmentStatus & (1 << 10)) { //D10
		Serial.printf("[MPPT-PV] Input-Over-Current\n");
	}

	if (chargingEquipmentStatus & (1 << 11)) { //D11
		Serial.printf("[MPPT-PV] Anti-Reverse-MOSFET-Short\n");
	}

	if (chargingEquipmentStatus & (1 << 12)) { //D12
		Serial.printf("[MPPT-PV] Charging-or-Anti-Reverse-MOSFET-Short\n");
	}

	if (chargingEquipmentStatus & (1 << 13)) { //D13
		Serial.printf("[MPPT-PV] Charging-MOSFET-Short\n");
	}

	//D15 - D14

	switch ((chargingEquipmentStatus >> 14) & 0b11) { //D15-D14
		case 0b00:
			Serial.printf("[MPPT-PV] Input-Volt-Status-Normal\n");
			break;
		case 0b01:
			Serial.printf("[MPPT-PV] Input-Volt-Status-No-Power-Connected\n");
			break;
		case 0b10:
			Serial.printf("[MPPT-PV] Input-Volt-Status-Higher-Volt-Input\n");
			break;
		case 0b11:
			Serial.printf("[MPPT-PV] Input-Volt-Status-Input-Volt-Error\n");
			break;
	}

	//Print all bits as binary
	for (int i = 15; i >= 0; i--) {
		Serial.printf("%d", (chargingEquipmentStatus >> i) & 1);
	}
	Serial.printf("\n");
}

void AddressRegistry_3200() {

	static unsigned long lastRead = 0;
	if (millis() - lastRead < 8000) {
		return;
	}
	lastRead = millis();

	uint8_t result = node.readInputRegisters(0x3200, 2);
	if (result != node.ku8MBSuccess) {
		Serial.printf("\nRead register 0x3200 failed!\n");
		return;
	}

	uint16_t batteryStatus = node.getResponseBuffer(0x00);
	handleBatteryStatus(batteryStatus);

	uint16_t chargingEquipmentStatus = node.getResponseBuffer(0x01);
	handleChargingEquipmentStatus(chargingEquipmentStatus);
}

// A list of the regisities to query in order
typedef void (*RegistryList[])();


RegistryList Registries = {
  	AddressRegistry_3100,
	AddressRegistry_331B,
	AddressRegistry_3200
};

// keep log of where we are
uint8_t currentRegistryNumber = 0;

// function to switch to next registry
void nextRegistryNumber() {
    // better not use modulo, because after overlow it will start reading in incorrect order
    currentRegistryNumber++;
	#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
    if (currentRegistryNumber >= ARRAY_SIZE(Registries)) {
      currentRegistryNumber = 0;
    }
}

// exec a function of registry read (cycles between different addresses)
void executeCurrentRegistryFunction() {
	Registries[currentRegistryNumber]();
}

void mppt_task(void *parameters) {
	constexpr gpio_num_t mppt_rx_pin = GPIO_NUM_22;
	constexpr gpio_num_t mppt_tx_pin = GPIO_NUM_23;

    Serial.begin(115200);
	Serial2.begin(115200, SERIAL_8N1, mppt_rx_pin, mppt_tx_pin);
	Serial.printf("[MPPT]Modbus started!\n");

	constexpr int slave_address = 1;
    node.begin(slave_address, Serial2);
    Serial.printf("[MPPT]Setup OK!");

	while (true) {
		executeCurrentRegistryFunction();
		nextRegistryNumber();

		vTaskDelay(pdMS_TO_TICKS(2000));
	}

}