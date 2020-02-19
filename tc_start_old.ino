//  ---------------- TC Start ------------------------

// Sensor 

#define sensPIN 2

// Buttons

//#define btnPIN 3
#define btn0PIN A2
#define btn1PIN 7
#define btn2PIN 8

// Display

#define dispSCLK 4
#define dispRCLK 5
#define dispDIO 6 

// Beeper

#define beepPIN A3
#define beepGND A4

// Batery

#define batvPIN A1

// Radio

//--------------------- НАСТРОЙКИ ----------------------
#define CH_AMOUNT 3   // число каналов
#define CH_NUM 0x60   // номер канала (должен совпадать)

#define SEND_DEBUG_MODE 0   // выводить отправленное  и принятое


// уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
#define SIG_POWER RF24_PA_MIN

// скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
// должна быть одинакова на приёмнике и передатчике!
// при самой низкой скорости имеем самую высокую чувствительность и дальность!!
// ВНИМАНИЕ!!! enableAckPayload НЕ РАБОТАЕТ НА СКОРОСТИ 250 kbps!
#define SIG_SPEED RF24_1MBPS
//--------------------- НАСТРОЙКИ ----------------------

//--------------------- БИБЛИОТЕКИ ----------------------
//#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
RF24 radio(9, 10);   // "создать" модуль на пинах 9 и 10 для НАНО/УНО
//RF24 radio(9, 53); // для МЕГИ
//--------------------- БИБЛИОТЕКИ ----------------------






//--------------------- ПЕРЕМЕННЫЕ ----------------------
byte pipeNo;
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; // возможные номера труб

volatile unsigned int sens_tmt = 1000;
volatile unsigned int cur_racer = 1;

volatile unsigned long sens_last_released, btn_last_released, bat_v_last_msrd, dbl_btn_released = 0;
unsigned long last_get_request, timer_started, result_time;

volatile unsigned long transmit_data[CH_AMOUNT];  // массив пересылаемых данных
unsigned long recieved_data; // массив принятых данных

volatile byte state = 0;
byte menu = 0;
bool menu_entered = false;

byte dispBuffer[4];
bool dispBufferDot[4] = {false, false, false, false};

unsigned long bat_v = 0;
double bat_v_k = 0.005017845;

// Protocol

//	1	Timer	
//		1	start	13	racer number
//		2	cancel	-	curent racer
//	2	Settings	
//	3	Get
//	4	Ping


void setup() {
	pinMode(sensPIN, INPUT_PULLUP);
	attachInterrupt(0, onSensor, FALLING);

	//pinMode(btnPIN, INPUT_PULLUP);
	//attachInterrupt(1, onBtn, FALLING);

	//attachInterrupt(1, onBtn, CHANGE);

	pinMode(btn0PIN, INPUT_PULLUP);
	pinMode(btn1PIN, INPUT_PULLUP);
	pinMode(btn2PIN, INPUT_PULLUP);

	pinMode(dispRCLK, OUTPUT);
	pinMode(dispSCLK, OUTPUT);
	pinMode(dispDIO, OUTPUT);  

	pinMode(beepPIN, OUTPUT);
	pinMode(beepGND, OUTPUT);
	digitalWrite(beepGND, LOW);

	pinMode(batvPIN, INPUT);

	radioSetup();

	Serial.begin(9600);
	Serial.println(" ==== TC Start ==== ");

	setDisplay(11,22,13,12); // Race
	tone(beepPIN, 9050,300);
}

void loop() {

	if(digitalRead(sensPIN)==LOW) {
		if(millis() - btn_last_released > 200) {
			onBtn(0);
			btn_last_released = millis();
		}
	} else if(digitalRead(btn1PIN)==LOW && digitalRead(btn2PIN)==HIGH) {
		if(millis() - btn_last_released > 200) {
			onBtn(2);
			btn_last_released = millis();
		}
	} else if(digitalRead(btn2PIN)==LOW && digitalRead(btn1PIN)==HIGH) {
		if(millis() - btn_last_released > 200) {
			onBtn(1);
			btn_last_released = millis();
		}
	} else if(digitalRead(btn2PIN)==LOW && digitalRead(btn1PIN)==LOW) {
		if(dbl_btn_released > 0 && millis() - dbl_btn_released > 3000) {
			onBtn(12);
			dbl_btn_released = 0;
		} else {
			if(!dbl_btn_released) {
				dbl_btn_released = millis();	
			}
		}
	} else {
		dbl_btn_released = 0;
	}

	if(state == 0) {
		if(menu_entered) {
			if(menu==0) {

			} else if (menu==1) {
				setDisplay(14,21,15,16); // sddsd
			} else if (menu==2) {
				getBatV(true);
			}
		}
	} else if(state == 2) {
		if(millis() - last_get_request > 500) {
			send(3,1,cur_racer);
			last_get_request = millis();
		}
		if(result_time>0) {
			setDisplayTime(result_time);
		} else {
			setDisplayTime(millis() - timer_started);
		}
	}
	drawDisplay();
}

void onSensor() {
	if(millis() - sens_last_released < sens_tmt) {return false;}
	if (state == 1) { // Ожидание
		send(1,1,cur_racer);
		Serial.print("Sensor released\n");
		state = 2;
		timer_started = millis();
		tone(beepPIN, 9050,300);
		result_time = 0;
	}
	sens_last_released = millis();
}

void onBtn(byte n) {

	if(state == 0) { // Меню
		if(n==0) {// Нажата кнопка 0
			if(menu<2) {
				menu++;
			} else {
				menu = 0;
			}
			if(menu==0) { 
				setDisplay(11,22,13,12); // Race
			} else if (menu==1) {
				setDisplay(19,20,14,21); // Ping
			} else if (menu==2) { 
				getBatV(true);
			}
		} else if (n==1) {// Нажата кнопка 1

		} else if (n==2) {// Нажата кнопка 2
			if(menu==0) { 
				state = 1;
				setCurRacer(cur_racer);
			} else if (menu==1) {
				
			} else if (menu==2) { 
				
			}
		}
	} else if (state == 1) { // Ожидание
		if(n==1) { // Нажата кнопка 1
			if(cur_racer > 1) {
				cur_racer--;
				setCurRacer(cur_racer);
				Serial.print("Set racer:\t");Serial.print(cur_racer);Serial.print("\n");
			} else {
				state = 0;
				setDisplay(11,22,13,12); // Race
			}
		} else if(n==2) { // Нажата кнопка 2
			cur_racer++;
			setCurRacer(cur_racer);
			Serial.print("Set racer:\t");Serial.print(cur_racer);Serial.print("\n");
		}
	} else if (state == 2) { // Заезд
		if (n==2) { // Нажата кнопка 2
			Serial.print("Cancel race:\t");Serial.print(cur_racer);Serial.print("\n");
			send(1,2,0);
			setDisplay(13,14,16,15);
			result_time = 0;
			state = 1;
		}
	}

	if(n==12) { // нажаты 2 кнопки 3 сек.
		Serial.print("Reset");Serial.print("\n");
		setCurRacer(1);
		state = 1;
		result_time = 0;
		send(2,2,0);
		setDisplay(11,12,17,18);

	}

}

void setCurRacer(unsigned int n) {
	cur_racer = n;
	setDisplay(11,0,0,cur_racer);
	send(2,1,cur_racer);
}

void onRecieved(long unsigned data) { // Пришел ответ
    if(SEND_DEBUG_MODE) {Serial.print("> Recieved:\t");Serial.print(data);Serial.print("\n");}
	if(state == 2) { // Идет заезд
		if(data>128) { // Пришел результат заезда
		    Serial.print("Result:");Serial.print("\t");Serial.print(cur_racer);Serial.print(":\t");Serial.print(0.001*data,3);Serial.print("\n\n");
    		result_time = data;
    		setDisplayTime(result_time);
    		tone(beepPIN, 3000,100); delay(150);
    		tone(beepPIN, 3000,100); delay(150);
    		tone(beepPIN, 3000,100); delay(150);
		    state = 1;
		} else { // Пришла команда

		}
	}
}

void send(long unsigned object, long unsigned method, long unsigned data) {
	transmit_data[0] = object;
	transmit_data[1] = method;
	transmit_data[2] = data;
	if(radio.write(&transmit_data, sizeof(transmit_data))) {
		if(object != 3 && SEND_DEBUG_MODE) {Serial.print("> Sended:\t");Serial.print(object);Serial.print("\t");Serial.print(method);Serial.print("\t");Serial.print(data);Serial.print("\n");}
	    if (!radio.available()) {                                  // если получаем пустой ответ
	    } else {
	      while (radio.available() ) {                    // если в ответе что-то есть
        	radio.read(&recieved_data, sizeof(recieved_data));    // читаем
	        onRecieved(recieved_data);
	      }
    	}
	};
}

void radioSetup() {
  radio.begin();              // активировать модуль
  radio.setAutoAck(1);        // режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 15);    // (время между попыткой достучаться, число попыток)
  radio.enableAckPayload();   // разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32);   // размер пакета, в байтах
  radio.openWritingPipe(address[0]);   // мы - труба 0, открываем канал для передачи данных
  radio.setChannel(CH_NUM);            // выбираем канал (в котором нет шумов!)
  radio.setPALevel(SIG_POWER);         // уровень мощности передатчика
  radio.setDataRate(SIG_SPEED);        // скорость обмена
  // должна быть одинакова на приёмнике и передатчике!
  // при самой низкой скорости имеем самую высокую чувствительность и дальность!!

  radio.powerUp();         // начать работу
  radio.stopListening();   // не слушаем радиоэфир, мы передатчик
}

void setDisplay(int unsigned d0, int unsigned d1, int unsigned d2, int unsigned d3){
	dispBufferDot[0] = false; dispBufferDot[1] = false; dispBufferDot[2] = false; dispBufferDot[3] = false;
	dispBuffer[0] = d0;
	dispBuffer[1] = d1;
	dispBuffer[2] = d2;
	dispBuffer[3] = d3;
}

void setDisplayInt(int unsigned number) {
	dispBufferDot[0] = false; dispBufferDot[1] = false; dispBufferDot[2] = false; dispBufferDot[3] = false;
	dispBuffer[0] = int(number/1000);
	dispBuffer[1] = int(number/100)-10*dispBuffer[0];
	dispBuffer[2] = int(number/10)-10*int(number/100);
	dispBuffer[3] = int(number)-10*int(number/10);
	dispBufferDot[1] = true;
}

void setDisplayTime(long unsigned ms){
	int unsigned d0,d1,d2,d3 = 0;
	dispBufferDot[0] = false; dispBufferDot[1] = false; dispBufferDot[2] = false; dispBufferDot[3] = false;
	if(ms < 10000) {
		d0 = int(ms/1000);
		d1 = int(ms/100)-10*d0;
		d2 = int(ms/10)-10*int(ms/100);
		d3 = int(ms)-10*int(ms/10);
		dispBufferDot[0] = true;
	} else if (10000 <= ms && ms < 60000) {
		d0 = int(ms/10000);
		d1 = int(ms/1000) - 10*int(ms/10000);
		d2 = int(ms/100) - 10*int(ms/1000);
		d3 = int(ms/10) - 10*int(ms/100);
		dispBufferDot[1] = true;
	} else if (ms >= 60000) {
		d0 = 0;
		d1 = 0;
		d2 = 0;
		d3 = 0;
		dispBufferDot[1] = true;
	}
	dispBuffer[0] = d0;
	dispBuffer[1] = d1;
	dispBuffer[2] = d2;
	dispBuffer[3] = d3;
	//dispBuffer[0] |= (1 << 4); 
	//dispBuffer[0] &= ~(1 << 0);
}

void drawDisplay(){
  
  byte digit[23] = {  // маска для 7 сигментного индикатора  
		0b11000000, // 0
		0b11111001, // 1
		0b10100100, // 2
		0b10110000, // 3
		0b10011001, // 4
		0b10010010, // 5
		0b10000010, // 6
		0b11111000, // 7
		0b10000000, // 8
		0b10010000, // 9 

		0b11111111, //	10	Пусто 
		0b11001110, //	11	r
		0b10000110, //	12	E
		0b11000110, //	13	c
		0b11001000, //	14	n
		0b10100001, //	15	d
		0b11000111, //	16	L
		0b10010010, //	17	S
		0b10000111, //	18	t
		0b10001100, //	19	P
		0b11111001, //	20	I
		0b10000010, //	21	G
		0b10001000, //	22	A

   
  };

  byte chr[4] = { // маска для разряда
      0b00001000,  
      0b00000100,  
      0b00000010,  
      0b00000001  
  };

  byte todisp;

  // отправляем в цикле по два байта в сдвиговые регистры
  for(byte i = 0; i <= 3; i++){ 
    digitalWrite(dispRCLK, LOW); // открываем защелку
    	if(dispBufferDot[i]) {
     		todisp = digit[dispBuffer[i]] &= ~(1 << 7);
    	} else {
    		todisp = digit[dispBuffer[i]] |= (1 << 7);
    	}
    	shiftOut(dispDIO, dispSCLK, MSBFIRST, todisp);  // отправляем байт с "числом"
    	shiftOut(dispDIO, dispSCLK, MSBFIRST, chr[i]);   // включаем разряд
    digitalWrite(dispRCLK, HIGH); // защелкиваем регистры
    //delay(1); // ждем немного перед отправкой следующего "числа"
  }  
}


int getBatV(bool to_disp) {
	if(millis() - bat_v_last_msrd < 100) return;
	if(bat_v == 0) {
		bat_v = analogRead(batvPIN);
	} else {
		bat_v = (analogRead(batvPIN) + 9*bat_v)/10;
	}
	bat_v_last_msrd = millis();
	if(to_disp) {
		setDisplayInt(int(bat_v*bat_v_k*100));
	}
    //Serial.print("v:");Serial.print("\t");Serial.print(bat_v*0.00151);Serial.print("\t");Serial.print(bat_v*bat_v_k);Serial.print("\n");

}