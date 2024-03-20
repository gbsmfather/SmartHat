/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 
*/


#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <Wire.h>
#include <WiFi.h>
#include <esp_sleep.h>


/*
#include <SPI.h>
#include <Adafruit_FRAM_SPI.h>

uint8_t FRAM_CS = 11;

//Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_CS);  // use hardware SPI

uint8_t FRAM_SCK= 6;
uint8_t FRAM_MISO = 7;
uint8_t FRAM_MOSI = 8;
//Or use software SPI, any pins!
Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_SCK, FRAM_MISO, FRAM_MOSI, FRAM_CS);

uint16_t          addr = 0;
*/

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define BQ25121A_I2C_ADDRESS	0x6A	// I2C address of BQ25121A
#define LSM6DSO32_ADDRESS 		0x6B	// LSM6DSO32 I2C address


// 사용할 I2C 핀을 정의합니다.
#define SDA_PIN					27
#define SCL_PIN					26


#define	STX						0x02

#define	STX_FIELD				0
#define	LEN_FIELD				1
#define	CODE_FIELD				2
#define	DATA_FIELD				3


#define STATE_CHARGING 0
#define STATE_TO_RUN 1
#define STATE_RUNNING 2

#define MAX_COUNT_MEASURE 3

#define SENSIBILITY 15 // 0 ~ 31

BLEServer *pServer = NULL;
BLEService *pService = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

BLEAdvertising *pAdvertising;

uint8_t txValue = 0;
uint8_t txTest[20];

uint8_t Battery_Data, Device_Temp;
uint16_t Thermistor_ADC;
int16_t ThermistorData;


uint8_t Sw_cnt = 0;

uint8_t led_state = 0, charge_state = 0, ble_pair = 0, Tx_Busy, Tx_Retry; 

uint8_t Boot_Check = STATE_CHARGING;

uint16_t Tx_Timer;

uint32_t timer_10ms = 0, timer_1s = 0, timer_1s_old = 0, pair_timer = 0;



const int PG_Pin = 18;
const int ledPin = 19;						// LED Pin 번호
const int SwPin = 21;						// SW Pin 번호
const int Buzzer_Pin = 22;
const int PSM_CD_Pin = 23;
const int INT1_pin = 25;
const int thermistorPin = 34;				// ADC 핀 번호


int ThermistorValue = 0; 			       // 아날로그 값 저장 변수


int8_t accelerometerDataX, accelerometerDataY, accelerometerDataZ;
int8_t gyroscopeDataX, gyroscopeDataY, gyroscopeDataZ;
int8_t gyroscopeRefX, gyroscopeRefY, gyroscopeRefZ;


hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;


volatile uint8_t fullFlag = 0; // FIFO full flag


uint8_t pushSwPinFlag = 0;
uint32_t pushSwPinCount = 0;

uint16_t accelRange = 16;
uint16_t gyroRange = 2000;

#define EVENT_TIMER_5S 0x80
#define EVENT_TIMER_3S 0x40
#define EVENT_TIMER_2S 0x20
#define EVENT_TIMER_1S 0x10
#define EVENT_TIMER_500MS 0x08
#define EVENT_TIMER_300MS 0x04
#define EVENT_TIMER_200MS 0x02
#define EVENT_TIMER_50MS 0x01


uint8_t event_timer_flag = 0;


#define EVENT_HAT_BLE_CONNECT 0x80
#define EVENT_HAT_BLE_DISCONNECT 0x40

uint8_t event_hat_flag = 0;


#define uS_TO_S_FACTOR 1000000
#define TIME_TO_SLEEP 5


// ISR callback for INT1
void INT1_fullEvent_cb()
{
	//fullFlag = 1;
  fullFlag++;
}


void ARDUINO_ISR_ATTR onTimer()
{
	// Increment the counter and set the time of ISR
	portENTER_CRITICAL_ISR(&timerMux);
	isrCounter = isrCounter + 1;
	lastIsrAt = millis();
  timer_10ms++;
	portEXIT_CRITICAL_ISR(&timerMux);
	// Give a semaphore that we can check in the loop
	xSemaphoreGiveFromISR(timerSemaphore, NULL);
	// It is safe to use digitalRead/Write here if you want to toggle an output

  if((timer_10ms % 5) == 0) 			// 100밀리초
  {
    event_timer_flag |= EVENT_TIMER_50MS;
  }

  if((timer_10ms % 20) == 0) 			// 200밀리초
  {
    event_timer_flag |= EVENT_TIMER_200MS;
  }

  if((timer_10ms % 30) == 0) 			// 300밀리초
  {
    event_timer_flag |= EVENT_TIMER_300MS;
  }

  if((timer_10ms % 50) == 0) 			// 500밀리초
  {
    event_timer_flag |= EVENT_TIMER_500MS;
  }

  if((timer_10ms % 100) == 0) 			// 1초
  {
    event_timer_flag |= EVENT_TIMER_1S;
  }

  if((timer_10ms % 200) == 0) 			// 2초
  {
    event_timer_flag |= EVENT_TIMER_2S;
  }

  if((timer_10ms % 300) == 0) 			// 3초
  {
    event_timer_flag |= EVENT_TIMER_3S;
  }

  if((timer_10ms % 500) == 0) 			// 5초
  {
    event_timer_flag |= EVENT_TIMER_5S;
  }
}


uint16_t connectedId = 0;

class MyServerCallbacks: public BLEServerCallbacks
{
	void onConnect(BLEServer* pServer)
	{
		Serial.println("BLE Connected Callback");

    connectedId = pServer->getConnId();

		// 연결된 기기의 주소 얻어오기
		//BLEAddress connectedAddress = pServer->getPeerAddress();
    
//		Serial.print("Connected device address: ");
//		Serial.println(connectedAddress.toString().c_str());

    event_hat_flag |= EVENT_HAT_BLE_CONNECT;

		// 연결 관련 설정
//		esp_bd_addr_t remote_bda = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66}; // 예제 주소, 실제로는 연결하려는 디바이스의 주소로 설정해야 합니다.

		// 연결 관련 설정 업데이트
//		pServer->updateConnParams(remote_bda, 20, 40, 0, 5000); // 최소 간격: 20ms, 최대 간격: 40ms, latency: 0, timeout: 5000ms
	};

  void onDisconnect(BLEServer* pServer)
	{
    event_hat_flag |= EVENT_HAT_BLE_DISCONNECT;

		Serial.println("BLE Disconnected Callback");
	}
};


class MyCallbacks: public BLECharacteristicCallbacks
{
	uint8_t checksum;

	void onWrite(BLECharacteristic *pCharacteristic)
	{
		String rxValue = pCharacteristic->getValue();

		if(rxValue.length() > 0)
		{
/*		
			Serial.println("*********");
			Serial.print("Received Value: ");

			for(int i = 0; i < rxValue.length(); i++)
			{
//				Serial.print(rxValue[i]);
				Serial.print(" 0x");
				Serial.print(rxValue[i], HEX);
			}
			Serial.println("*********");			
*/

//			Tx_Retry = 0;

			checksum = 0;
			for(int i = 0; i < (rxValue.length() - 1); i++)
			{
				checksum ^= rxValue[i];
			}

			if(checksum == rxValue[rxValue.length() - 1])
			{
				if(0x01 == rxValue[3])
				{
					// 체크섬 OK
					Tx_Retry = 0;
				}
				else
				{
					// 응답이 실패로 옴
//					Tx_Retry++;
//					if(Tx_Retry > 2)
					{
						Tx_Retry = 0;
					}
				}
			}
			else
			{
				// 체크섬 Fail
//				Tx_Retry++;
//				if(Tx_Retry > 2)
				{
					Tx_Retry = 0;
				}
			}
		}
	}
};






void initBQ25121A()
{
	// Configure BQ25121A settings
	writeRegister(BQ25121A_I2C_ADDRESS, 0x00, 0x01);
	writeRegister(BQ25121A_I2C_ADDRESS, 0x01, 0x00);
	writeRegister(BQ25121A_I2C_ADDRESS, 0x02, 0x0A);			// 0x88
	writeRegister(BQ25121A_I2C_ADDRESS, 0x03, 0x94);			// 0x80		// 24.03.03 전류값 수정

	writeRegister(BQ25121A_I2C_ADDRESS, 0x04, 0x12);
	writeRegister(BQ25121A_I2C_ADDRESS, 0x05, 0x78);
	writeRegister(BQ25121A_I2C_ADDRESS, 0x06, 0xFF);
	writeRegister(BQ25121A_I2C_ADDRESS, 0x07, 0x2D);			// 0x2C
	
	writeRegister(BQ25121A_I2C_ADDRESS, 0x08, 0x68);  			// 0x68		// 24.03.03 수정
	writeRegister(BQ25121A_I2C_ADDRESS, 0x09, 0x3B);  			// 0x3A		// 24.03.03 수정

	writeRegister(BQ25121A_I2C_ADDRESS, 0x0A, 0x80);
	writeRegister(BQ25121A_I2C_ADDRESS, 0x0B, 0x42);

	// Additional initialization steps can be added based on your requirements
	Serial.println("BQ25121A Initialized");
}


void initlSM6DSO32()
{
	// LSM6DSO32 초기화 및 동작 모드 설정
	writeRegister(LSM6DSO32_ADDRESS, 0x10, 0x60); // CTRL1_XL 레지스터, 0x60은 208Hz, ±4g
	writeRegister(LSM6DSO32_ADDRESS, 0x11, 0x60); // CTRL2_G 레지스터, 0x60은 208Hz, ±250dps

	uint8_t temp;

	temp = readRegister(LSM6DSO32_ADDRESS, 0x57);
	temp = temp | SENSIBILITY;
	writeRegister(LSM6DSO32_ADDRESS, 0x57, temp);

	temp = readRegister(LSM6DSO32_ADDRESS, 0x58);
	temp = temp | ((0x80) | SENSIBILITY);
	writeRegister(LSM6DSO32_ADDRESS, 0x58, temp);

	temp = readRegister(LSM6DSO32_ADDRESS, 0x59);
	temp = temp | SENSIBILITY;
	writeRegister(LSM6DSO32_ADDRESS, 0x59, temp);

	temp = readRegister(LSM6DSO32_ADDRESS, 0x56);
	temp = temp | 0x0E;
	writeRegister(LSM6DSO32_ADDRESS, 0x56, temp);

	temp = readRegister(LSM6DSO32_ADDRESS, 0x5E);
	temp = temp | 0x40;
	writeRegister(LSM6DSO32_ADDRESS, 0x5E, temp);	

  // writeRegister(LSM6DSO32_ADDRESS, 0x0D, 0x0B);

	Serial.println("LSM6DSO32 Initialized");
}

/*
void initFRAM()
{
	if(fram.begin())
	{
		Serial.println("Found SPI FRAM");
	}
	else
	{
		Serial.println("No SPI FRAM found ... check your connections\r\n");
		while (1);
	}

	// Read the first byte
	uint8_t test = fram.read8(0x0);
	Serial.print("Restarted ");
	Serial.print(test);
	Serial.println(" times");

	// Test write ++
	fram.writeEnable(true);
	fram.write8(0x0, test+1);
	fram.writeEnable(false);

	fram.writeEnable(true);
	fram.write(0x1, (uint8_t *)"FTW!", 5);
	fram.writeEnable(false);

	// dump the entire 8K of memory!
	uint8_t value;
	for(uint16_t a = 0; a < 8192; a++)
	{
		value = fram.read8(a);
		if((a % 32) == 0)
		{
			Serial.print("\n 0x");
			Serial.print(a, HEX);
			Serial.print(": ");
		}
		Serial.print("0x"); 
		if(value < 0x1) 
		{
			Serial.print('0');
			Serial.print(value, HEX);
			Serial.print(" ");
		}
	}
}
*/


uint16_t readRegister(uint8_t deviceAddress, uint8_t reg)
{
	Wire.beginTransmission(deviceAddress);
	Wire.write(reg);
	Wire.endTransmission();

	Wire.requestFrom(deviceAddress, 1);

	while(Wire.available() == 0)
	{
		// Wait for data
	}

	return Wire.read();
}


void writeRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data)
{
	Wire.beginTransmission(deviceAddress);
	Wire.write(registerAddress);
	Wire.write(data);
	Wire.endTransmission();
}


int LSM6DSO_readAcceleration(int16_t& x, int16_t& y, int16_t& z)
{
  uint8_t address = 0X28;

  Wire.beginTransmission(LSM6DSO32_ADDRESS);
	Wire.write(address);
	Wire.endTransmission();    

  Wire.requestFrom(LSM6DSO32_ADDRESS, (uint8_t)6);
  uint8_t xlg = Wire.read();
  uint8_t xhg = Wire.read();
  uint8_t ylg = Wire.read();
  uint8_t yhg = Wire.read();
  uint8_t zlg = Wire.read();
  uint8_t zhg = Wire.read();

  // combine high and low bytes
  int16_t ax = (int16_t)(xhg << 8 | xlg);
  int16_t ay = (int16_t)(yhg << 8 | ylg);
  int16_t az = (int16_t)(zhg << 8 | zlg);

  x = ax;
  y = ay;
  z = az;

  // x = (int8_t)(ax / 256);
  // y = (int8_t)(ay / 256);
  // z = (int8_t)(az / 256);

	return 1;
}


int LSM6DSO_readGyroscope(int16_t& x, int16_t& y, int16_t& z)
{
  uint8_t address = 0X22;

  Wire.beginTransmission(LSM6DSO32_ADDRESS);
	Wire.write(address);
	Wire.endTransmission();    

  Wire.requestFrom(LSM6DSO32_ADDRESS, (uint8_t)6);
  uint8_t xlg = Wire.read();
  uint8_t xhg = Wire.read();
  uint8_t ylg = Wire.read();
  uint8_t yhg = Wire.read();
  uint8_t zlg = Wire.read();
  uint8_t zhg = Wire.read();

  // combine high and low bytes
  int16_t gx = (int16_t)(xhg << 8 | xlg);
  int16_t gy = (int16_t)(yhg << 8 | ylg);
  int16_t gz = (int16_t)(zhg << 8 | zlg);

  x = gx;
  y = gy;
  z = gz;

  // x = (int8_t)(gx / 256);
  // y = (int8_t)(gy / 256);
  // z = (int8_t)(gz / 256);

	return 1;
}

float calcAccel( int16_t input )
{
	float output = (float)input * 0.061 * (accelRange >> 1) / 1000;
	return output;
}

float calcGyro( int16_t input )
{
	uint8_t gyroRangeDivisor = gyroRange / 125;
	if ( gyroRange == 245 ) {
		gyroRangeDivisor = 2;
	}

	float output = (float)input * 4.375 * (gyroRangeDivisor) / 1000;
	return output;
}

int LSM6DSO_readTemperature(int& temperature_deg)
{
  uint8_t address = 0X28;

  Wire.beginTransmission(LSM6DSO32_ADDRESS);
	Wire.write(address);
	Wire.endTransmission();    

  Wire.requestFrom(LSM6DSO32_ADDRESS, (uint8_t)2);
  uint8_t deglg = Wire.read();
  uint8_t deghg = Wire.read();
  
  // combine high and low bytes
  int16_t temperature_raw = (int16_t)(deghg << 8 | deglg);

  /* Convert to °C. */
	static int const TEMPERATURE_LSB_per_DEG = 256;
	static int const TEMPERATURE_OFFSET_DEG = 25;

	temperature_deg = static_cast<int>((static_cast<float>(temperature_raw) / TEMPERATURE_LSB_per_DEG) + TEMPERATURE_OFFSET_DEG);

	return 1;
}


void DataMake(void)
{
	uint8_t	length, i, checksum;

	length = 11;

  txTest[STX_FIELD] = STX;											// STX					0
  txTest[LEN_FIELD] = length;											// LEN					1
  txTest[CODE_FIELD] = 0x21;											// CMD					2
  txTest[DATA_FIELD] = ThermistorData >> 8;							// Thermistor High Data		3
  txTest[DATA_FIELD + 1] = ThermistorData;							// Thermistor Low Data		4    
  txTest[DATA_FIELD + 2] = Device_Temp;								// Device Temp			5
  txTest[DATA_FIELD + 3] = accelerometerDataX;						// ACC_X[0]				6
  txTest[DATA_FIELD + 4] = accelerometerDataY;						// ACC_Y[0]				7
  txTest[DATA_FIELD + 5] = accelerometerDataZ;						// ACC_Z[0]				8
  txTest[DATA_FIELD + 6] = gyroscopeDataX; // gyroscopeDataX; // abs(gyroscopeDataX-gyroscopeRefX);							// AXIS_X[0]			9
  txTest[DATA_FIELD + 7] = gyroscopeDataY; // gyroscopeDataY; // abs(gyroscopeDataY-gyroscopeRefY);							// AXIS_Y[0]			10
  txTest[DATA_FIELD + 8] = gyroscopeDataZ; // gyroscopeDataZ; // abs(gyroscopeDataZ-gyroscopeRefZ);							// AXIS_Z[0]			11
  txTest[DATA_FIELD + 9] = Battery_Data; 							// Battery				12

	for(i = 0; i < (length + 2); i++)
	{
		checksum ^= txTest[i];
	}
    txTest[length + 2] = checksum;   									// XOR
}


void activateReset()
{
	Serial.println("Reset");
	delay(100);

	ESP.restart();
	
	// EN_SYS_OUT Disable
//	writeRegister(BQ25121A_I2C_ADDRESS, 0x06, 0x7F);
}


// Pin 토글 함수
void togglePin(int pin)
{
	// 현재 핀의 상태 읽기
	int currentState = digitalRead(pin);

	// 핀의 상태 반전 (토글)
	digitalWrite(pin, !currentState);
}


void resetBleService() {
  // BLE 초기화
	BLEDevice::init("SMART HAT");

	// BLE 서버 생성 및 이벤트 핸들러 등록
	pServer = BLEDevice::createServer();
	pServer->setCallbacks(new MyServerCallbacks());

	// GATT 서비스 생성
	pService = pServer->createService(SERVICE_UUID);

	// 서비스에 특성 추가
	pTxCharacteristic = pService->createCharacteristic
	(
		CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY
	);

	pTxCharacteristic->addDescriptor(new BLE2902());

	BLECharacteristic * pRxCharacteristic = pService->createCharacteristic
	(
		CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE
	);

	pRxCharacteristic->setCallbacks(new MyCallbacks());

	// GATT 서비스 시작
	// pService->start();
/*
	pAdvertising = BLEDevice::getAdvertising();
	// advertising 간격 설정 (100ms - 1000ms 범위 내에서 설정 가능)
	pAdvertising->setMinInterval(500);  // 최소 간격: 500ms
	pAdvertising->setMaxInterval(1000); // 최대 간격: 1000ms
*/
	// // BLE 서버 시작
	// pServer->getAdvertising()->start();
	// Serial.println("Waiting a client connection to notify...");
}

void startBleAdvertising() {
  // GATT 서비스 시작
	pService->start();

  // BLE 서버 시작
	pServer->getAdvertising()->start();
	Serial.println("Waiting a client connection to notify...");
}

void stopBleAdvertising() {
  // BLE 서버 시작
	pServer->getAdvertising()->stop();
	Serial.println("stop Advertsing...");

  pService->stop();
}

void measureBattery() {
  writeRegister(BQ25121A_I2C_ADDRESS, 0x0A, 0x80);
  Battery_Data = readRegister(BQ25121A_I2C_ADDRESS, 0x0A);

  if(Battery_Data >= 0x7c)
  {
    Battery_Data = 100;
  }
  else if(Battery_Data <= 0x4c)
  {
    Battery_Data = 0;
  }

  if(Battery_Data >> 2 < 0x12) {
    digitalWrite(PSM_CD_Pin, LOW); // 충전 진행
  }

  Serial.print("Battery : ");
  Serial.println(Battery_Data);
}

void measureAcceleration() {
  // 가속도 및 자이로 데이터 읽기
  float x, y, z;
  int16_t temp_accX, temp_accY, temp_accZ;
  

  LSM6DSO_readAcceleration(temp_accX, temp_accY, temp_accZ);

  int8_t _accDataX = (int8_t) (calcAccel(temp_accX) + 0.5) * 30;
  int8_t _accDataY = (int8_t) (calcAccel(temp_accY) + 0.5) * 30;
  int8_t _accDataZ = (int8_t) (calcAccel(temp_accZ) + 0.5) * 30;

  accelerometerDataX = (accelerometerDataX * (MAX_COUNT_MEASURE-1) + _accDataX) / MAX_COUNT_MEASURE;
  accelerometerDataY = (accelerometerDataY * (MAX_COUNT_MEASURE-1) + _accDataY) / MAX_COUNT_MEASURE;
  accelerometerDataZ = (accelerometerDataZ * (MAX_COUNT_MEASURE-1) + _accDataZ) / MAX_COUNT_MEASURE;
}

void measureGyroscope() {
  float gx, gy, gz;
  int16_t temp_gyroX, temp_gyroY, temp_gyroZ;
  int8_t calc_gyroX, calc_gyroY, calc_gyroZ;

  LSM6DSO_readGyroscope(temp_gyroX, temp_gyroY, temp_gyroZ);

  int8_t _calc_gyroX = (int8_t) (calcGyro(temp_gyroX) / 2293.3 * 255);
  int8_t _calc_gyroY = (int8_t) (calcGyro(temp_gyroY) / 2293.3 * 255);
  int8_t _calc_gyroZ = (int8_t) (calcGyro(temp_gyroZ) / 2293.3 * 255);

  gyroscopeDataX = (gyroscopeDataX * (MAX_COUNT_MEASURE-1) + _calc_gyroX) / MAX_COUNT_MEASURE;
  gyroscopeDataY = (gyroscopeDataY * (MAX_COUNT_MEASURE-1) + _calc_gyroY) / MAX_COUNT_MEASURE;
  gyroscopeDataZ = (gyroscopeDataZ * (MAX_COUNT_MEASURE-1) + _calc_gyroZ) / MAX_COUNT_MEASURE;
}

void measureDeviceTemp() {
  int temperature_int = 0;
  LSM6DSO_readTemperature(temperature_int);
  Device_Temp = temperature_int;
}

void measureThermistor() {
  // 아날로그 값을 읽어옴
  Thermistor_ADC = analogRead(thermistorPin);
//		Serial.print("ADC Value: ");
//		Serial.println(Thermistor_ADC);

  ThermistorData = (int16_t) ((0.03012 * Thermistor_ADC - 30.6717) * 10);
}

void measureCharging() {
  uint8_t temp;
  temp = readRegister(BQ25121A_I2C_ADDRESS, 0x00);
  temp = (temp & 0xC0) >> 6;
  if(temp == 1)
  {
    // 충전 중인 상태
    charge_state = 1;
  }
  else if(temp == 0)
  {
    // 충전 케이블 해제
    charge_state = 0;
  }
  else if(temp == 2)
  {
    // 완충
    charge_state = 2;		
  }
}

void measureSensors() {
  measureBattery();

//		Serial.print("ISR : ");
//		Serial.println(fullFlag);
//		fullFlag = 0;

  //measureAcceleration();

  // measureGyroscope();

  measureDeviceTemp();

//		Serial.print(Device_Temp);
//		Serial.println(" °C");
//		delay(10);		

  measureThermistor();

  measureCharging();
}


void setup()
{
  Serial.begin(9600);
	delay(1000);
	
	pinMode(PG_Pin, INPUT_PULLUP);	
	pinMode(SwPin, INPUT_PULLUP);

	pinMode(ledPin, OUTPUT);	
	pinMode(PSM_CD_Pin, OUTPUT);
	pinMode(Buzzer_Pin, OUTPUT);
	digitalWrite(PSM_CD_Pin, HIGH);
  	
	Wire.begin(SDA_PIN, SCL_PIN); 

	// Interrupt pin settings
	pinMode(INT1_pin, INPUT);
	attachInterrupt(digitalPinToInterrupt(INT1_pin), INT1_fullEvent_cb, RISING); // attach watermark event to INT1 input pin

	WiFi.disconnect();
	WiFi.mode(WIFI_OFF);
	delay(100);

  // esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  
  // esp_deep_sleep_start();

	// Initialize BQ25121A
	initBQ25121A();

	// Initialize lSM6DSO32
	initlSM6DSO32();

//	initFRAM();

	// ESP32의 속도를 80MHz로 설정
	setCpuFrequencyMhz(80);


	digitalWrite(ledPin, HIGH);
	delay(50);
	digitalWrite(ledPin, LOW);
	delay(50);	
	digitalWrite(ledPin, HIGH);
	delay(50);	
	digitalWrite(ledPin, LOW);

  resetBleService();

  // BLE 전송 출력값 변경
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N0);

  // BLE 파워 조정 값
  // ESP_PWR_LVL_N12 = 0, /*!< Corresponding to -12dbm */
  // ESP_PWR_LVL_N9 = 1, /*!< Corresponding to -9dbm */
  // ESP_PWR_LVL_N6 = 2, /*!< Corresponding to -6dbm */
  // ESP_PWR_LVL_N3 = 3, /*!< Corresponding to -3dbm */
  // ESP_PWR_LVL_N0 = 4, /*!< Corresponding to 0dbm */
  // ESP_PWR_LVL_P3 = 5, /*!< Corresponding to +3dbm */
  // ESP_PWR_LVL_P6 = 6, /*!< Corresponding to +6dbm */
  // ESP_PWR_LVL_P9 = 7, /*!< Corresponding to +9dbm */
  // ESP_PWR_LVL_N14 = ESP_PWR_LVL_N12, /*!< Backward compatibility! Setting to -14dbm will actually result to -12dbm */
  // ESP_PWR_LVL_N11 = ESP_PWR_LVL_N9, /*!< Backward compatibility! Setting to -11dbm will actually result to -9dbm */
  // ESP_PWR_LVL_N8 = ESP_PWR_LVL_N6, /*!< Backward compatibility! Setting to -8dbm will actually result to -6dbm */
  // ESP_PWR_LVL_N5 = ESP_PWR_LVL_N3, /*!< Backward compatibility! Setting to -5dbm will actually result to -3dbm */
  // ESP_PWR_LVL_N2 = ESP_PWR_LVL_N0, /*!< Backward compatibility! Setting to -2dbm will actually result to 0dbm */
  // ESP_PWR_LVL_P1 = ESP_PWR_LVL_P3, /*!< Backward compatibility! Setting to +1dbm will actually result to +3dbm */
  // ESP_PWR_LVL_P4 = ESP_PWR_LVL_P6, /*!< Backward compatibility! Setting to +4dbm will actually result to +6dbm */
  // ESP_PWR_LVL_P7 = ESP_PWR_LVL_P9, /*!< Backward compatibility! Setting to +7dbm will actually result to +9dbm */



	ble_pair = 1;
	pair_timer = 10;				// 10초


	// Create semaphore to inform us when the timer has fired
	timerSemaphore = xSemaphoreCreateBinary();
	
	// Set timer frequency to 1Mhz
	timer = timerBegin(1000000);
	
	// Attach onTimer function to our timer.
	timerAttachInterrupt(timer, &onTimer);
	
	// Set alarm to call onTimer function every second (value in microseconds).
	// Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
	timerAlarm(timer, 10000, true, 0);				// 10ms
//	timerAlarm(timer, 1000000, true, 0);	

	Serial.println(fullFlag);

	Boot_Check = STATE_TO_RUN;
}


void loop()
{
  if(event_hat_flag & EVENT_HAT_BLE_CONNECT) {
    event_hat_flag &= ~EVENT_HAT_BLE_CONNECT;

    deviceConnected = true;
    timer_10ms = 0;
    event_timer_flag = 0;

    measureGyroscope();

    gyroscopeRefX = gyroscopeDataX;
    gyroscopeRefY = gyroscopeDataY;
    gyroscopeRefZ = gyroscopeDataZ;
  }

  else if(event_hat_flag & EVENT_HAT_BLE_DISCONNECT) {
    event_hat_flag &= ~EVENT_HAT_BLE_DISCONNECT;

    deviceConnected = false;

    Boot_Check = STATE_TO_RUN;
  }

  if (Boot_Check == STATE_CHARGING) { // 충전 상태 
    if(event_timer_flag & EVENT_TIMER_1S) {
      event_timer_flag &= ~EVENT_TIMER_1S;

      measureCharging();
      
      if(charge_state == 1) { // 충전 중인 상태
        togglePin(ledPin);
      }
      else if(charge_state == 2) { // 완충
        digitalWrite(ledPin, HIGH);
      }
    }

    if(digitalRead(PG_Pin) == HIGH) {
      digitalWrite(PSM_CD_Pin, HIGH); // 충전 해제 

      delay(100); // 디바운스를 위한 짧은 지연
      //activateReset();

      Boot_Check = STATE_TO_RUN;		
    }
  }
  else if (Boot_Check == STATE_TO_RUN) { // 준비
    digitalWrite(ledPin, LOW);
    timer_10ms = 0;
    event_timer_flag = 0;
    event_hat_flag = 0;
    fullFlag = 0;

    accelerometerDataX = 0;
    accelerometerDataY = 0;
    accelerometerDataZ = 0;

    gyroscopeDataX = 0;
    gyroscopeDataY = 0;
    gyroscopeDataZ = 0;

    if(!deviceConnected) {
      startBleAdvertising();
    }
    
    Boot_Check = STATE_RUNNING;
  }
  else if (Boot_Check == STATE_RUNNING) { // 충전 해제 완료
    if(digitalRead(PG_Pin) == LOW) {
      
      digitalWrite(PSM_CD_Pin, LOW); // 충전 진행

      // Boot_Check = STATE_CHARGING;

      // if(deviceConnected) {
      //   pServer->disconnect(connectedId);
      //   stopBleAdvertising();
      // }      

      // timer_10ms = 0;
      // event_timer_flag = 0;

      // if(event_timer_flag & EVENT_TIMER_1S) {
      //   event_timer_flag &= ~EVENT_TIMER_1S;

      //   measureCharging();
        
      //   // if(charge_state == 1) { // 충전 중인 상태
      //   //   togglePin(ledPin);
      //   // }
      //   // else if(charge_state == 2) { // 완충
      //   //   digitalWrite(ledPin, HIGH);
      //   // }
      // }
    }
    else {
      digitalWrite(PSM_CD_Pin, HIGH); // 충전 해제 
    }
    
    if(digitalRead(SwPin) == LOW) {
      pushSwPinFlag = 1;
      pushSwPinCount++;
      if(pushSwPinCount > 96000) {
        activateReset();
      }
    }
    else {
      if(pushSwPinFlag == 1) {
        pushSwPinFlag = 0;
        pushSwPinCount = 0;

        if(fullFlag) {
          fullFlag = 0;
          digitalWrite(Buzzer_Pin, LOW);
          Serial.println("Shork Clear");
        }
      }
    }

    if(fullFlag)
    {
      digitalWrite(Buzzer_Pin, HIGH);
      delay(2);
      digitalWrite(Buzzer_Pin, LOW);		
    }

    if(deviceConnected) {
      if(event_timer_flag & EVENT_TIMER_50MS) {
        event_timer_flag &= ~EVENT_TIMER_50MS;

        measureAcceleration();
        measureGyroscope();
      }

      if(event_timer_flag & EVENT_TIMER_1S) {
        event_timer_flag &= ~EVENT_TIMER_1S;
        
        digitalWrite(ledPin, HIGH);
        
        measureSensors();

        DataMake();
        pTxCharacteristic->setValue(txTest, (txTest[LEN_FIELD] + 3));
        // Notify를 통해 데이터 전송		
        pTxCharacteristic->notify();
        Serial.println("BLE TX DATA");
        Tx_Busy = 1;
        Tx_Timer = 10;
        Tx_Retry++;
        if(Tx_Retry > 0) {
          Tx_Retry = 0;
        }

        if(Battery_Data <= 0x4c) {
          digitalWrite(ledPin, HIGH);
        }
        else {
          digitalWrite(ledPin, LOW);
        }
        timer_10ms = 0;

        if(digitalRead(PG_Pin) == LOW) {
          if(charge_state == 2) { // 완충
            digitalWrite(ledPin, HIGH);
          }
        }
      }
    }
    else {
      if(event_timer_flag & EVENT_TIMER_1S) {
        event_timer_flag &= ~EVENT_TIMER_1S;

        measureCharging();
      }

      if(event_timer_flag & EVENT_TIMER_300MS) {
        event_timer_flag &= ~EVENT_TIMER_300MS;

        togglePin(ledPin);

        if(digitalRead(PG_Pin) == LOW) {
          if(charge_state == 2) { // 완충
            digitalWrite(ledPin, HIGH);
          }
        }
      }
    }
  }
  else { // UNKNOWN STATE
  }

	

	
	

		


	


// 	if(((1 == ble_pair) || (2 == ble_pair)) && (0 == pair_timer))
// 	{
// 		digitalWrite(ledPin, LOW);		// GPIO 핀을 LOW로 설정

// 		ble_pair = 0;

// 		// 특성을 삭제하여 RX 비활성화
// //		pServer->getAdvertising()->stop();
// 		Serial.println("Pairing END");

// 		// 모뎀 Sleep 모드 초기화
// //		esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_ON);
// 		esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_ON);

// 		esp_sleep_enable_timer_wakeup(2e6);

// 		// Modem Sleep 모드로 진입
// 		Serial.println("Sleep Check");
		
// 		esp_light_sleep_start();    
// 	}

  
	

// 	if((Tx_Busy == 1) && (Tx_Timer == 0))
// 	{
// 		if(Tx_Retry != 0)
// 		{
// 			Tx_Busy = 0;
// 		}
// 		else
// 		{
// 			Tx_Busy = 2;

// 			if(fullFlag)
// 			{
// 				Tx_Timer = 10;			// 100ms
// 			}
// 			else
// 			{
// 				Tx_Timer = 200;			// 2초
// /*
// 				// 모뎀 Sleep 모드 초기화
// //				esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_ON);
// 				esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_ON);
				
// 				// Modem Sleep 모드로 진입
// 				esp_sleep_enable_timer_wakeup(2e6);		// 2000ms
// //				esp_sleep_enable_timer_wakeup(5e5);		// 500ms
// 				Tx_Timer = 0;
// 				Serial.println("TX Sleep Start");
	
// 				esp_light_sleep_start();
				
// 				deviceConnected = 0;
// 				// BLE 모듈을 활성화하여 다시 동작시킴	
// 				Serial.println("TX Wakeup Start");
// */				
// 			}
// 		}
// 	}

// 	if((Tx_Busy == 2) && (Tx_Timer == 0))
// 	{
// 		Tx_Busy = 0;
// 	}

// 	// 커넥션 대기 시간 웨이팅 구현
// 	if((Tx_Busy == 3) && (Tx_Timer == 0))
// 	{
// 		Tx_Busy = 0;
// 	}	


// 	// disconnecting
// 	if(!deviceConnected && oldDeviceConnected)
// 	{
// 		delay(500); // give the bluetooth stack the chance to get things ready

// 		if(1 == charge_state)
// 		{
// 			ble_pair = 0;
// 		}
		
// 		Serial.println("BLE Disconnected");

// 		ble_pair = 2;
// 		pair_timer = 20;				// 20초
// 		pServer->startAdvertising(); // restart advertising

// 		oldDeviceConnected = deviceConnected;		 
// 	}

// 	// connecting
// 	if(deviceConnected && !oldDeviceConnected)
// 	{
// 		// 특성을 삭제하여 RX 비활성화
// //		pServer->getAdvertising()->stop();
	
// 		// do stuff here on connecting
// 		oldDeviceConnected = deviceConnected;

// 		if(pair_timer)
// 		{
// 			pair_timer = 0;
// 		}
// 	}

// 	if((0 == ble_pair) && (0 == deviceConnected) && (digitalRead(SwPin) == HIGH))
// 	{
// 		// 모뎀 Sleep 모드 초기화
// //		esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_ON);
// 		esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_ON);

// 		esp_sleep_enable_timer_wakeup(2e6);

// 		// Modem Sleep 모드로 진입
// 		Serial.println("Sleep Start");
// 		delay(10);
// 		esp_light_sleep_start(); 

// 		Serial.println("Wakeup Start");
// 		delay(10);
// 	}

// 	if(LOW == digitalRead(PG_Pin))
// 	{
// 		togglePin(ledPin);
// 	} 

// 	if(ble_pair == 0)
// 	{
// 		if((timer_10ms % 100) == 0)				// 1초
// 		{
// 			if(charge_state == 2)
// 			{
// 				digitalWrite(ledPin, HIGH);
// 			}
// 			else
// 			{
// 				digitalWrite(ledPin, LOW);		
// 			}
// 		}
// 	}
// 	else
// 	{
// 		if(ble_pair == 1)
// 		{
// 			if((timer_10ms % 10) == 0)				// 100ms
// 			{
// 				togglePin(ledPin);
// 			}
// 		}
// 	}
}
