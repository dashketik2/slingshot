#include <adk.h>
#include "HX711.h"
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

#include <avr/wdt.h>

USB Usb;
ADK adk(&Usb, "TKJElectronics", // Manufacturer Name
              "ArduinoBlinkLED", // Model Name
              "Example sketch for the USB Host Shield", // Description (user-visible string)
              "1.0", // Version
              "http://www.tkjelectronics.dk/uploads/ArduinoBlinkLED.apk", // URL (web page to visit if no installed apps support the accessory)
              "123456789"); // Serial Number (optional)
   //mini
#define DT 3
#define CLK 2
#define LASER 4
#define BAT A3

#define MAX_BUT 840
#define MIN_BUT 790

 /* uno
#define DT A1
#define CLK A0
#define LASER 8
#define BAT A2

#define MAX_BUT 840
#define MIN_BUT 790
 */

uint32_t timer, timer1;
bool connected;
byte msg[7]; 
unsigned int data_ = 0;
int data_sign = 0;
boolean flag = 0;
HX711 scale(DT, CLK); 

unsigned int threshold = 40;
bool laser_flag = false;
unsigned long time_laser = 0;
unsigned int butt_sum = 0;
int but_pct = 0;

void setup() {

  but_pct = (MAX_BUT - MIN_BUT);

  pinMode(LASER, OUTPUT);
  pinMode(BAT, INPUT);
  analogReference(INTERNAL);

  Serial.begin(115200);


  #if !defined(__MIPSEL__)
    while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  #endif
  if (Usb.Init() == -1) {
    Serial.print("\r\nOSCOKIRQ failed to assert");
    while (1); // halt
  }
  //pinMode(LED, OUTPUT);
  Serial.print("\r\nStarted");

  scale.set_scale(2280.f);                 
  scale.tare(); 
  watchdogSetup();

}

void loop() {


  Usb.Task();

  if (adk.isReady()) {
    if (!connected) {
      connected = true;
      Serial.println(F("\r\nConnected to accessory"));

      pack(msg, true, analogRead(BAT), 2);
      adk.SndData(sizeof(msg), (uint8_t*)&msg);

    }


    

    uint8_t msg_r[7];
    uint16_t len = sizeof(msg_r);
    uint8_t rcode = adk.RcvData(&len, msg_r);
    //Serial.print(msg_r[0]);
    if (!rcode) {
        
        
        Serial.println(msg_r[0], HEX);
        Serial.println(msg_r[1], HEX);
        Serial.println(msg_r[2], HEX);
        Serial.println(msg_r[3], HEX);
        Serial.println(msg_r[4], HEX);
        Serial.println(msg_r[5], HEX);
        Serial.println(msg_r[6], HEX);

      Serial.print(F("\r\nData Packet: "));
      //Serial.print(msg_r[0]);

      if (msg_r[0] == 0x55 && msg_r[1] == 0xBB && msg_r[6] == (msg_r[4] ^ msg_r[5])) {//(msg_r[0] == 0x55 && msg_r[1] == 0xBB && msg_r[6] == (msg_r[4] ^ msg_r[5])) { // BB 55 2 0 0 1C 1C  Сигнатура 0x55, 0xBB, 2 байта длина, потом 2 байта данных, потом crc 1 байт.
        threshold = msg_r[4] << 8;
        threshold += msg_r[5];
        Serial.println(threshold);

      }

    }
	
    if (millis() - timer >= 10000) { //send data from battery
      timer = millis();
      butt_sum = 0;
      for (int i = 0; i <10; i++) {
        butt_sum += analogRead(BAT);
        Serial.println(butt_sum);

      }

      pack(msg, true, (butt_sum * 100 / (but_pct * 10)), 2);
      adk.SndData(sizeof(msg), (uint8_t*)&msg);

    }

    if (millis() - timer1 >= 50) { // Send data 
      timer1 = millis();

      data_sign = scale.get_units();
      if (data_sign < 0)
        data_ = 0;
      else
        data_ = data_sign;

      if (laser_flag == false && data_ > threshold) {
        laser_flag = true;
        time_laser = millis();
        digitalWrite(LASER, HIGH);
      }


      if (laser_flag == true && data_ < threshold) {
        laser_flag = false;
        digitalWrite(LASER, LOW);
      }

      if (millis() - time_laser >= 10000)
        digitalWrite(LASER, LOW);

      pack(msg, false, data_ , 2);//
      rcode = adk.SndData(sizeof(msg), (uint8_t*)&msg);
      delay(250);
      wdt_reset();

    }

  } else {
    if (connected) {
      connected = false;
      Serial.print(F("\r\nDisconnected from accessory"));
      
    }
  }
}

void pack(byte * package, boolean command, unsigned int data, unsigned int length) { // command: 1 - AA 55 battery  0 - 55 AA tension sensor
	if (command) {
  	package[0] = 0xAA;
		package[1] = 0x55;
	} else {
		package[0] = 0x55;
		package[1] = 0xAA;
	}


  package[2] = length >> 8;
  package[3] = length & 255;

	package[4] = data >> 8;    //TODO: add loop for data more then 2 bytes !!!
	package[5] = data & 255;  

  package[6] = crc(package, 4, 5);
}

byte crc(byte *arr, byte num_start, byte num_end) { //num_start, num_end порядковые номера элементов массива
  byte crc = arr[num_start];
  for(int i = 1; i <= (num_end - num_start); i++) {
    crc ^= arr[i + num_start];
  }
  return crc;
}
//TODO: crc function for data array


void watchdogSetup(void)
{
  cli();       // disable all interrupts
  wdt_reset(); // reset the WDT timer
  /*
   WDTCSR configuration:
   WDIE = 1: Interrupt Enable
   WDE = 1 :Reset Enable
   WDP3 = 0 :For 2000ms Time-out
   WDP2 = 1 :For 2000ms Time-out
   WDP1 = 1 :For 2000ms Time-out
   WDP0 = 1 :For 2000ms Time-out
  */
  // Enter Watchdog Configuration mode:
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // Set Watchdog settings:
  WDTCSR = (1<<WDIE) | (1<<WDE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);
  sei();
}
