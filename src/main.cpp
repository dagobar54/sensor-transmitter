#include <Arduino.h>

#define SERIAL_DEBUG 1
//Проблемы
//Настройка мощности передатчика и приемника
//Прием от нескольких передатчиков

#include <LowPower.h>
#include "Wire.h"
#include "DS3231.h" // скачать https://github.com/jarzebski/Arduino-DS3231
#include <SPI.h>
#include <BME280I2C.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "home_meteo.h"
#if defined(SERIAL_DEBUG) 
#include "printf.h"
#endif

	// Макросы для работы с портами  скорость и место
#define SetOutput(port,bit)       DDR ## port |= _BV(bit)
#define SetInput(port,bit)        DDR ## port &= ~_BV(bit)
#define SetBit(port,bit)          PORT ## port |= _BV(bit)
#define ClearBit(port,bit)        PORT ## port &= ~_BV(bit)
#define WritePort(port,bit,value) PORT ## port = (PORT ## port & ~_BV(bit)) | ((value & 1) << bit)
#define ReadPort(port,bit)        (PIN ## port >> bit) & 1
#define PullUp(port,bit)          { SetInput(port,bit); SetBit(port,bit); }
#define Release(port,bit)         { SetInput(port,bit); ClearBit(port,bit); }

#define pinRF24 4
DS3231 clock;
RF24 radio(9,8);  
BME280I2C::Settings settings(
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::OSR_X1, 
   BME280::Mode_Forced,
   BME280::StandbyTime_1000ms,
   BME280::Filter_Off,
   BME280::SpiEnable_False,
   BME280I2C::I2CAddr_0x76 // I2C address. I2C specific.
);

BME280I2C bme(settings);

void setTransmitState(transmit_states state);
void FailAnalyzer();
void AckPayloadAnalizer();
void NoneCommandAction();
void wakeUp();
void PackDataToStruct();
#if defined(SERIAL_DEBUG)
void  serial_sensors(String s);
#endif
bool RealSend();
bool SendMeteoToRadioFast();
void GetMeteoData();
long readVcc();

const uint64_t address[2] = {   0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL  };
meteo_data_struct sensors;
server_ack ackData;

float T,P,H,Ph;
uint32_t measurement_time; 
long int exchange_counter = 0; 
long int long longer_counter = 0;

transmit_states transmit_state = start_state; 

const char statusStr[6][20] =  {"start_state",
  "connecting_state",
  "connected_state",
  "lost_connectState",
  "disconnected_state",
  "test_state"};
  
RTCAlarmTime at;
bool flagTransmit =true;
bool flagMeteo = true;
volatile bool flagWakeUp = true;
#if defined(SERIAL_DEBUG)
char str1[100] = {0,};
char* str;
#endif
const uint32_t max_exchange_interval = 1000;
unsigned long round_tripDelay;


long int vcc;
void setup()
{
	#if defined(SERIAL_DEBUG)
	str=&str1[0];
	#endif
  
	ADCSRA &= ~(1 << ADEN); // Отключаем АЦП
	//set_sleep_mode(SLEEP_MODE_PWR_DOWN); //Устанавливаем интересующий нас режим
	cli(); // Временно запрещаем обработку прерываний
	//sleep_enable();
	// Отключаем детектор пониженного напряжения питания
	MCUCR != ((1 << BODS) | (1 << BODSE));
	MCUCR &= ~(1 << BODSE);
	CLKPR = 0x80;    // Разрешаем изменение значения делителя
	CLKPR = 1;
	sei(); // Разрешаем обработку прерываний
  
	#if defined(SERIAL_DEBUG) 
	Serial.begin(115200);
	#endif
	delay(2000);
	clock.begin();
	clock.enableOutput(false);
	//clock.setDateTime(__DATE__, __TIME__);
	clock.armAlarm1(false); clock.clearAlarm1();
	clock.armAlarm2(false); clock.clearAlarm2();
	clock.setAlarm1(0, 0, 0, 0, DS3231_EVERY_SECOND);
	clock.setAlarm2(0, 0, 0,  DS3231_EVERY_MINUTE);
  
	#if defined(SERIAL_DEBUG) 
	RTCDateTime dt = clock.getDateTime();
	clock.dtFormat("H:m:s",dt,str1);
	Serial.print(F("Start data: "));
	Serial.println(str1);
	//Serial.print(dt.unixtime);
	//Serial.print(F("   ");
	//Serial.print(dt.year);   Serial.print(F("-");
	//Serial.print(dt.month);  Serial.print(F("-");
	//Serial.print(dt.day);    Serial.print(F(" ");
	//Serial.print(dt.hour);   Serial.print(F(":");
	//Serial.print(dt.minute); Serial.print(F(":");
	//Serial.print(dt.second); Serial.println(F("");
	delay(50);
	#endif
	pinMode(pinRF24,INPUT); 
	//set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	
	#if defined(SERIAL_DEBUG) 
	while(!Serial) {} // Wait
		printf_begin();
	Serial.println("Start radio meteo sensor");
	#endif
  
	Wire.begin();
	#if defined(SERIAL_DEBUG) 
	//while(!bme.begin())
	//{
		bme.begin();
	//	Serial.println("Could not find BME280I2C sensor!");
		delay(1000);
	//}
	#else
	bme.begin();
	#endif
	// Change some settings before using.
	settings.tempOSR = BME280::OSR_X4;
	bme.setSettings(settings);
   
    #if defined(SERIAL_DEBUG) 
	switch(bme.chipModel())
	{
		case BME280::ChipModel_BME280:
			Serial.println("Found BME280 sensor! Success.");
			break;
		case BME280::ChipModel_BMP280:
			Serial.println("Found BMP280 sensor! No Humidity available.");
			break;
		default:
			Serial.println("Found UNKNOWN sensor! Error!");
	}
	#endif
	Wire.end();

	// Setup and configure rf radio
	radio.begin();  
	radio.setPALevel(RF24_PA_HIGH);
	radio.setDataRate(RF24_2MBPS);
	radio.setChannel(106);
  
	radio.enableDynamicPayloads();                    // Ack payloads are dynamic payloads
	radio.enableDynamicAck();
	radio.enableAckPayload();                         // We will be using the Ack Payload feature, so please enable it
	//radio.setAutoAck(true);

	radio.openWritingPipe(address[0]);             // communicate back and forth.  One listens on it, the other talks to it.
	radio.openReadingPipe(1,address[1]); 
	
	#if defined(SERIAL_DEBUG) 
	radio.printDetails();  
	Serial.println(sizeof(meteo)); 
	Serial.println(sizeof(meteo_data_struct));  
	Serial.println(sizeof(server_ack)); 
	#endif
	delay(200);
	GetMeteoData();
	attachInterrupt(0, wakeUp, FALLING); 
}

void loop()
{
	if (flagWakeUp)	{
		if (clock.isAlarm2(true)){
			flagMeteo = true;
			flagTransmit = true;
		}
		if (clock.isAlarm1(true)){
			flagTransmit = true;
			flagMeteo = true;    
			}
		flagWakeUp= false;
		}
  
	if (flagMeteo){
		//Serial.println("Meteo");
		GetMeteoData();
		flagMeteo = false;
		}
	if (flagTransmit){

		SetOutput(B,5);     // slk - выход сделать
		SetOutput(B,3);     // mosi - выход сделать
		SPI.begin();        // старт spi заново
		radio.powerUp();
		PackDataToStruct();
		bool ex_success = SendMeteoToRadioFast();
		if (!ex_success){//нет ответа
			//exchange_counter++;
      		FailAnalyzer();
			}
		else{//ответ получен
			//exchange_counter++;
			AckPayloadAnalizer();    
			}
		flagTransmit = false;
		}
  
	attachInterrupt(0, wakeUp, FALLING);
	radio.powerDown();  // выключаем чип но это не все!!!
	SPI.end();          // выключаем SPI
	//PullUp(B,6); 
	PullUp(B,5);        // slk - сделать входом и подтянуть к 3.3 вольтам
	PullUp(B,3);        // mosi - сделать входом и подтянуть к 3.3 вольтам

	LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}
void setTransmitState(transmit_states state)
{
    #if defined(SERIAL_DEBUG)
	Serial.print("Power:");
    Serial.print(radio.getPALevel());
    Serial.print(" Move from to state:");
    Serial.print(transmit_state);
	delay(50);
    #endif
	if (transmit_state != state)
	{
		switch(state)
		{
			case start_state: 
				transmit_state=start_state;
				clock.armAlarm1(true);
				break;
			case connecting_state:
        		//radio.setPALevel(RF24_PA_MAX);
				transmit_state=connecting_state;
				clock.armAlarm1(true);
				break;
			case connected_state:
				transmit_state=connected_state;
				clock.armAlarm1(false);
				break;
			case lost_connectState:
				transmit_state=lost_connectState;
        		//radio.setPALevel(RF24_PA_MAX);
				clock.armAlarm1(true);
				break;
			case disconnected_state: 
				transmit_state=disconnected_state;
				clock.armAlarm1(false);
				break;
			case test_state:
				transmit_state=test_state;
				clock.armAlarm1(true);
				break;   
		}
		#if defined(SERIAL_DEBUG)
		Serial.print("-");
		Serial.print(transmit_state);
		Serial.println(statusStr[transmit_state]);
		delay(50);
		#endif
		exchange_counter = 0;
		longer_counter = 0;
	}
}

void FailAnalyzer()
{
   if ((transmit_state != disconnected_state) && (transmit_state != lost_connectState)
    && (transmit_state != test_state)){
        setTransmitState(lost_connectState);
   }
   if ((transmit_state == lost_connectState)&&(exchange_counter >=20)){
        exchange_counter = 0;
        setTransmitState(disconnected_state);      
   }
}
void AckPayloadAnalizer()
{
	//Serial.print("\r\npower before analyzer=");
	//Serial.print(radio.getPALevel());
	//Serial.print(" command=");
	//Serial.println(ackData.command);
	mode_commands m = ackData.command; 
	switch(m)
	{
		case none_command :
			NoneCommandAction();
			break;
		case start_command :
			setTransmitState(start_state);
			break;
		case set_time:
			uint32_t dt = ackData.server_time;
			clock.setDateTime(dt+65);
			break; 
		case test_command:
			setTransmitState(test_state);
			break; 
		default:
			break;
	}

	//Serial.print("\r\npower after analyzer=");
	//Serial.println(radio.getPALevel());

}
void NoneCommandAction()
{

  uint32_t dt;
  switch(transmit_state)
  {
	case   start_state:
		if (exchange_counter==3){
			setTransmitState(connecting_state);
		}
		break;
	case connecting_state:
		if (exchange_counter==10)
		{
			setTransmitState(connected_state);
			longer_counter = 0;
			exchange_counter = 0;
		} 
		break;
	case connected_state:
		if (exchange_counter<10){   
			if (exchange_counter==3){
				dt = ackData.server_time % 60;
				#if defined(SERIAL_DEBUG) 
				uint32_t dtc = clock.getDateTime().unixtime;
				unixtimeToString(dtc,str);
				Serial.print("Correct time client = ");
				Serial.println(str);  
				#endif          
				uint32_t unixT = 55- dt % 60;
				if (unixT != 0)
				{ 
					clock.setDateTime(ackData.server_time+65); 
				}
				}
			}
		#if defined(SERIAL_DEBUG) 
		dt = ackData.server_time;
		unixtimeToString(dt,str);
		Serial.println(str);
		dt = clock.getDateTime().unixtime;
		unixtimeToString(dt,str);
		Serial.println(str);
		delay(50);
		#endif
	  
		break;
	case lost_connectState:
		setTransmitState(connecting_state);
		exchange_counter = 0;
		longer_counter = 0;
		break;
	case disconnected_state:
		setTransmitState(connecting_state);
		exchange_counter = 0;
		longer_counter = 0;
		break;
	case test_state:
		break; 
	}
}
void wakeUp()
{
	flagWakeUp = true;    
	detachInterrupt(0);
}
void PackDataToStruct()
{
	sensors.query = none_q;
 	sensors.client_time = clock.getDateTime().unixtime;
	sensors.state=transmit_state;
	sensors.power = radio.getPALevel();
	sensors.meteo_data.measurement_time = measurement_time;
	sensors.meteo_data.T = (int32_t)(T*10.0);
	sensors.meteo_data.P = (int32_t)(P*10.0);
	sensors.meteo_data.H = (int32_t)(H*10.0);
  	sensors.type_of_data = data_meteoOutDoor;
	sensors.vcc = round(vcc /100.0);
	sensors.vc1 = exchange_counter;
	#if defined(SERIAL_DEBUG)
		serial_sensors("from pack");
	#endif
}

#if defined(SERIAL_DEBUG) 
void  serial_sensors(String s)
{
	Serial.print(s);
    Serial.print(":  T=");
    float t = ((float)sensors.meteo_data.T) / 10.0;
    Serial.print(t,1);
    float p = ((float)sensors.meteo_data.P) / 10.0;
    Serial.print("   P=");
    Serial.print(p,1);
    Serial.print("-");
    Serial.print(Ph,1);
    Serial.print("   H=");
    float h = sensors.meteo_data.H / 10.0;
    Serial.print(h,1);
    Serial.print("    ");
    Serial.print(statusStr[transmit_state]);
    Serial.print(" power=");
    Serial.print(sensors.power);
	//Serial.print(radio.getPALevel());
    Serial.print("    ");
    uint32_t dt = sensors.meteo_data.measurement_time;
    unixtimeToString(dt,str);
    Serial.print(str);
    Serial.print("  Vcc=");
    Serial.print(sensors.vcc);
    Serial.print("  counter=");
    Serial.println(exchange_counter);
    delay(50);
}
#endif

bool RealSend(  )
{
  //Serial.println(*round_tripDelay);
  bool tx,fail,rx;
  do{
    unsigned long time = micros();
    radio.startWrite(&sensors, sizeof(meteo_data_struct) ,0);
    //uint32_t started_waiting_at = micros();               // Set up a timeout period, get the current microseconds
    while(digitalRead(pinRF24)!=0)   {}
    radio.whatHappened(tx,fail,rx); 
    round_tripDelay= micros()-time;
    #if defined(SERIAL_DEBUG) 
    Serial.print(tx);
    Serial.print(F("-"));
    Serial.print(fail);
    Serial.print(F("-"));
    Serial.print(rx);
    Serial.print(F(" round-trip delay: "));
    Serial.println(round_tripDelay);
    #endif
  }while (tx & !rx);
  return !fail;
}
bool SendMeteoToRadioFast()
{
	//radio.powerUp();
  	bool res =RealSend();
	if (res){
		radio.flush_tx();
		sensors.round_tripDelay = round_tripDelay; 
    	ackData = {0,};
		uint8_t s = sizeof(server_ack);//radio.getDynamicPayloadSize();
		radio.read(&ackData,s);
		#if defined(SERIAL_DEBUG) 
		Serial.print(F("SendFast:OK"));
		Serial.print(F(" command="));
		Serial.print(ackData.command);
		Serial.print(F("   server time="));
		unixtimeToString(ackData.server_time,str);
		Serial.println(str);
		delay(50);
		#endif
	}
	else {
		sensors.round_tripDelay = round_tripDelay; 
		radio.txStandBy();
		#if defined(SERIAL_DEBUG) 
		Serial.print(F("Send:fail"));
		Serial.print(F(" round delay: "));
		Serial.print(round_tripDelay);
		Serial.println(F(" microseconds"));
		delay(50);
		#endif
	}
	exchange_counter++;
	radio.powerDown();
	return res;
}
void GetMeteoData()
{
	float temp(NAN), hum(NAN), pres(NAN);

	BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
	BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  	bme.read(pres, temp, hum, tempUnit, presUnit);
	//bme.read(&_P, &_T, &_H, tempUnit, presUnit);
	//Serial.println(pres);
	P= pres/100;
	T = temp;
	H = hum;
	Ph = pres /133.3224;
	//vcc = readVcc();
	measurement_time=clock.getDateTime().unixtime;
}
long readVcc() {
	// Read 1.1V reference against AVcc
	// set the reference to Vcc and the measurement to the internal 1.1V reference
	#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
		ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
		ADMUX = _BV(MUX5) | _BV(MUX0);
	#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
		ADMUX = _BV(MUX3) | _BV(MUX2);
	#else
		ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#endif  
	delay(20);
	//delay(75); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Start conversion
	while (bit_is_set(ADCSRA,ADSC)); // measuring
	uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
	uint8_t high = ADCH; // unlocks both
	long result = (high<<8) | low;

	result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
	return result; // Vcc in millivolts
}