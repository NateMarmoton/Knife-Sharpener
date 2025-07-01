#include <Arduino.h>
#include <CANSAME5x.h>
#include <Adafruit_NeoPixel.h>
#include <SevSeg.h>
// #include <FreeRTOS.h>

#define LED_COUNT 1

#define D1 (13ul)//Seg7_PIN12_
#define pinA (12ul)//Seg7_PIN11_
#define pinF (11ul)//Seg7_PIN10_
#define D2  (10ul)//Seg7_PIN9_
#define D3  (9ul)//Seg7_PIN8_
#define pinB  (6ul)//Seg7_PIN7_
#define D4 (5ul)//Seg7_PIN6_
#define pinE (22ul)//Seg7_PIN5_
#define pinD (21ul)//Seg7_PIN4_
#define pinDP (4ul)//Seg7_PIN3_
#define pinC (1ul)//Seg7_PIN2_
#define pinG (0ul)//Seg7_PIN1_

typedef enum {
	VoltageOpenLoop = 0x00,
	CurrentClosedLoop = 0x01,
	SpeedControlLoop = 0x02,
	PositionControlLoop = 0x03,
	DisableMotor = 0x09,
	EnableMotor = 0x0A,
};

void SetMode(uint8_t mode);
void SetSpeed(int8_t speed);
int addRpmToMovingAverage(int new_rpm);


Adafruit_NeoPixel strip(LED_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
CANSAME5x CAN;
SevSeg sevseg; //Instantiate a seven segment object

uint8_t Red = 0;
uint8_t Green = 0;
uint8_t Blue = 0;

int8_t speed = 0x00;

// Moving average for RPM
#define RPM_BUFFER_SIZE 100  // Adjust this value to change smoothing (higher = more smoothing)
int rpm_buffer[RPM_BUFFER_SIZE];
int rpm_buffer_index = 0;
int rpm_buffer_count = 0;
bool rpm_buffer_filled = false;

void setup()
{
	/* CANBUS INIT */
	pinMode(PIN_CAN_STANDBY, OUTPUT);
	digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY
	pinMode(PIN_CAN_BOOSTEN, OUTPUT);
	digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster

	if (!CAN.begin(1000000)) {
		while (1) delay(10);
	}
	SetSpeed(0x00); // set feedback to 0x00
	SetMode(SpeedControlLoop);


	/* NEOPIXEL INIT */
	pinMode(PIN_NEOPIXEL_POWER, OUTPUT);
	digitalWrite(PIN_NEOPIXEL_POWER, true);
	strip.setPixelColor(0, 10, 0, 0);
	strip.show();


	strip.begin();
	/* SEVEN SEGMENT DISPLAY INIT */
	byte numDigits = 4;
	byte digitPins[] = { D1, D2, D3, D4 };
	byte segmentPins[] = { pinA, pinB, pinD,pinC, pinG, pinF, pinE, pinDP };
	bool resistorsOnSegments = false; // 'false' means resistors are on digit pins
	byte hardwareConfig = COMMON_ANODE; // See README.md for options
	bool updateWithDelays = false; // Default 'false' is Recommended
	bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
	bool disableDecPoint = false; // Use 'true' if your decimal point doesn't exist or isn't connected. Then, you only need to specify 7 segmentPins[]

	sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments,
		updateWithDelays, leadingZeros, disableDecPoint);
	sevseg.blank();
	sevseg.setNumber(8888, 3);
	sevseg.refreshDisplay();



	// Initialize RPM moving average buffer
	for (int i = 0; i < RPM_BUFFER_SIZE; i++) {
		rpm_buffer[i] = 0;
	}



	/* ADC INIT */
	pinMode(PIN_A0, INPUT);




	/* SERIAL INIT */
	// Serial.begin(9600);
	// while (!Serial) {}

	/* INIT COMPLETE UPDATE NEOPIXEL COLOR */
	strip.setPixelColor(0, 0, 10, 0);
	strip.show();
}

void loop()
{
	// put your main code here, to run repeatedly:

	// try to parse packet
	int packetSize = CAN.parsePacket();

	if (packetSize) {
		// received a packet
		// Serial.print("Received ");

		// if (CAN.packetExtended()) {
		// 	Serial.print("extended ");
		// }

		uint8_t ID = CAN.packetId();
		// Serial.print("packet with id 0x");
		// Serial.print(ID, HEX);
		// Serial.print(" and length ");
		// Serial.println(packetSize);


		uint8_t	data[16];
		for (int i = 0; i < packetSize; i++) {
			data[i] = CAN.read();
			// Serial.print(data[i], HEX);
			// Serial.print(" ");
		}

		switch (ID) {

		case 0x98:
		{

			strip.setPixelColor(0, 0, 10, 0);

			int raw_rpm = (int16_t)((data[0] << 8) | data[1]) * 0.01f;
			int averaged_rpm = addRpmToMovingAverage(raw_rpm);

			// Serial.print("RPM: ");
			// Serial.print(rpm);
			// Serial.print("  ");
			uint16_t adc = analogRead(PIN_A0);

			if (adc > 512) {
				speed = (adc - 512) / 7.11;
			}
			else if (adc < 512) {
				speed = 255 - (512 - adc) / 7.11;
			}
			else {
				speed = 0;
			}

			// Serial.print(averaged_rpm);
			// Serial.print("   ");
			SetSpeed(speed);
			sevseg.setNumber(averaged_rpm, 0);


			break;
		}

		default:
			// Serial.println("Unknown packet");
			strip.setPixelColor(0, 10, 0, 0);
			break;
		}

		strip.show();
		Serial.print("\r");
	}
	sevseg.refreshDisplay();
}

void SetMode(uint8_t mode)
{
	CAN.beginPacket(0x105);
	CAN.write(0x00);
	CAN.write(mode);
	CAN.write(0x00);
	CAN.write(0x00);
	CAN.write(0x00);
	CAN.write(0x00);
	CAN.write(0x00);
	CAN.write(0x00);
	CAN.endPacket();
}

void SetSpeed(int8_t speed)
{
	CAN.beginPacket(0x32);
	CAN.write(0x00);
	CAN.write(0x00);
	CAN.write(speed);
	CAN.write(0x00);
	CAN.write(0x00);
	CAN.write(0x00);
	CAN.write(0x00);
	CAN.write(0x00);
	CAN.endPacket();
}

int addRpmToMovingAverage(int new_rpm)
{
	// Add new value to circular buffer
	rpm_buffer[rpm_buffer_index] = new_rpm;

	// Update buffer state
	if (!rpm_buffer_filled) {
		rpm_buffer_count++;
		if (rpm_buffer_count >= RPM_BUFFER_SIZE) {
			rpm_buffer_filled = true;
		}
	}

	// Move to next position (circular)
	rpm_buffer_index = (rpm_buffer_index + 1) % RPM_BUFFER_SIZE;

	// Calculate average
	long sum = 0;
	int count = rpm_buffer_filled ? RPM_BUFFER_SIZE : rpm_buffer_count;

	for (int i = 0; i < count; i++) {
		sum += rpm_buffer[i];
	}

	return sum / count;
}