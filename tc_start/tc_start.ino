//  ---------------- TC Start ------------------------

// Sensor 
#define sensPIN A4
#define laser_sensPIN A5
#define laserPIN 6

// Buttons
#define btn1PIN A2
#define btn2PIN 2
#define btn3PIN 3

// Display
#define disp_clkPIN 5
#define disp_dioPIN 4

/************** БУКВЫ И СИМВОЛЫ *****************/
#define _A 0x77
#define _B 0x7f
#define _C 0x39
#define _D 0x3f
#define _E 0x79
#define _F 0x71
#define _G 0x3d
#define _H 0x76
#define _J 0x1e
#define _L 0x38
#define _N 0x37
#define _O 0x3f
#define _P 0x73
#define _S 0x6d
#define _U 0x3e
#define _Y 0x6e
#define _a 0x5f
#define _b 0x7c
#define _c 0x58
#define _d 0x5e
#define _e 0x7b
#define _f 0x71
#define _h 0x74
#define _i 0x10
#define _j 0x0e
#define _l 0x06
#define _n 0x54
#define _o 0x5c
#define _q 0x67
#define _r 0b00110001
#define _t 0x78
#define _u 0x1c
#define _y 0x6e
#define _- 0x40
#define __ 0x08
#define _= 0x48
#define _empty 0x00

#define _0 0x3f
#define _1 0x06
#define _2 0x5b
#define _3 0x4f
#define _4 0x66
#define _5 0x6d
#define _6 0x7d
#define _7 0x07
#define _8 0x7f
#define _9 0x6f
/************** БУКВЫ И СИМВОЛЫ *****************/


// Beeper
#define beepPIN A3
#define beepGND A4

// Batery
#define batvPIN A1

// ВT
const int GRX_PIN = 8;
const int GTX_PIN = 7;


//--------------------- НАСТРОЙКИ ----------------------
#define CH_AMOUNT 3   // число каналов
#define CH_NUM 0x0c //0x60   // номер канала (должен совпадать)
#define SEND_DEBUG_MODE 0   // выводить отправленное  и принятое
#define BTN_TMT 50   // антидребезг кнопок
#define BTN_LONGPRESS_TIME 1000   // антидребезг кнопок
#define PING_TMT 500   // интервал пинга
#define LASER true   // использование лазера
//#define MUTE true   // заглушить звуки

byte MODE = 0;   // режим работы 	
	//	0 - круговой с одним устройством, 
	//	1 - круговой с одним устройством, 
//#define SAVE_RESULTS true   // сохранение результатов в память
bool SAVE_RESULTS = false;   // сохранение результатов в память
unsigned int  SENSOR_MA_VALUES_NUM, new_sensor_ma_values_num; // ширина окна для скольязего среднего датчика
unsigned int  SENSOR_TRUE_LEVEL, new_sensor_true_level; // уровень с которого срабатывает датчик
unsigned int LAPS_NUMBER, new_laps_number; // количество кругов
unsigned int SENSOR_TIMEOUT, new_sensor_timeout; // игнорирует повторные срабатывания дачтика втечение таймаута
bool MUTE;   // заглушить звуки


#define EEPROM_OFFSET 32   // сдвиг памяти - размер области для служебных данных
byte SENSOR_MODE = 0;   // режим датчика (FALLING/RISING/CHANGE)
#define SENSOR_ON_INTERRUPT false   // Датчик висит на прерывании
// уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
//#define SIG_POWER RF24_PA_MIN
#define SIG_POWER RF24_PA_LOW
// скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
// должна быть одинакова на приёмнике и передатчике!
// при самой низкой скорости имеем самую высокую чувствительность и дальность!!
// ВНИМАНИЕ!!! enableAckPayload НЕ РАБОТАЕТ НА СКОРОСТИ 250 kbps!
#define SIG_SPEED RF24_1MBPS
//--------------------- НАСТРОЙКИ ----------------------

//--------------------- БИБЛИОТЕКИ ----------------------
// #include <SPI.h>
// #include "nRF24L01.h"
// #include "RF24.h"
// #include "printf.h"
// RF24 radio(9, 10);   // "создать" модуль на пинах 9 и 10 для НАНО/УНО
//RF24 radio(9, 53); // для МЕГИ
#include <EEPROM.h>

// BT SoftwareSerial

char divider = ' ';
char ending = ';';
const char *headers[]  = {
	"getresults",
	"getinputs",
	"getconfig",
	"race",
	"save",
	"mode",
	"laps",
	"level",
	"cancel",
	"scal",
	"batr",
	"mute",
	"stimeout",
	"<", 
	"^", 
	">", 
	"help",
};
enum names {
	GET_RESULTS, 
	GET_INPUTS, 
	GET_CONFIG, 
	RACE, 
	SAVE, 
	_MODE, 
	LAPS, 
	LEVEL, 
	CANCEL, 
	SCAL, 
	BATR, 
	_MUTE, 
	_STIMEOUT, 
	BUTTON1, 
	BUTTON2, 
	BUTTON3, 
	HELP, 
};
names thisName;
byte headers_am = sizeof(headers) / 2;
uint32_t prsTimer;
String prsValue = "";
String prsHeader = "";
enum stages {WAIT, HEADER, GOT_HEADER, VALUE, SUCCESS};
stages parseStage = WAIT;
boolean recievedFlag;

#include <SoftwareSerial.h>
SoftwareSerial BTSerial(GRX_PIN, GTX_PIN);



// Дисплей
#include <TM1637Display.h>
TM1637Display display(disp_clkPIN, disp_dioPIN);

//--------------------- БИБЛИОТЕКИ ----------------------

//--------------------- ПЕРЕМЕННЫЕ ----------------------
byte pipeNo;
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; // возможные номера труб

//volatile unsigned int SENSOR_TIMEOUT = 1000;
volatile unsigned int cur_racer = 1;
volatile unsigned int sensor_value = 0;
unsigned int laps_counter = 0;
unsigned int best_lap_racer = 0, best_racer = 0;

volatile unsigned long sens_last_released, btn_last_released, bat_v_last_msrd, dbl_btn_released;
unsigned long last_ping, timer_started, result_time, cur_lap_time, cur_time, best_lap_time = 0, best_time = 0, ping_time, last_test_time = 0, inputvals_last_showed = 0, serial_last_read = 0;
unsigned int test_send_counter = 0;
unsigned long display_blocked = 0;

volatile unsigned long transmit_data[CH_AMOUNT];  // массив пересылаемых данных
unsigned long recieved_data, test_data; // массив принятых данных

volatile byte state = 0;
byte menu = 0;
bool menu_entered = false;
byte menu_deep = 0;

unsigned long bat_v = 0;
double bat_v_k = 0.005017845;

byte rssi;
int trnsmtd_pack = 1, failed_pack;
unsigned long RSSI_timer;

struct Row {
  unsigned int r;
  unsigned long t;
};
int unsigned last_result_num, cur_result_to_read = 9999;



// Protocol
//	1	Timer	
//		1	start	13	racer number
//		2	cancel	-	curent racer
//	2	Settings	
//	3	Get
//	4	Empty ping
//	5	Confirm
//	6	Testing
//		1	start	13	racer number

void setup() {

	Serial.begin(9600); //Serial.setTimeout(100);
	BTSerial.begin(19200);  BTSerial.setTimeout(100);

	log("\n ==== TC Start ==== \n");

	readSettings();

	pinMode(sensPIN, INPUT_PULLUP);

	if(SENSOR_ON_INTERRUPT) {
		if(SENSOR_MODE == 0) {
			attachInterrupt(0, onSensor, FALLING);
		} else if (SENSOR_MODE == 1) {
			attachInterrupt(0, onSensor, RISING);
		}
	}

	pinMode(btn1PIN, INPUT_PULLUP);
	pinMode(btn2PIN, INPUT_PULLUP);
	pinMode(btn3PIN, INPUT_PULLUP);

	pinMode(beepPIN, OUTPUT);
	pinMode(beepGND, OUTPUT);
	digitalWrite(beepGND, LOW);

	pinMode(batvPIN, INPUT);

	if(LASER) {
		pinMode(laserPIN, OUTPUT);  
		digitalWrite(laserPIN, LOW);
		pinMode(laser_sensPIN, INPUT);  
	}


	// radioSetup();
	// if(MODE==0) radioOff(); // Если включен круговой режим, гасим радио

	display.setBrightness(7);
	displayWord(_r,_A,_C,_E);

	beep( 3500,100);
	delay(150);
	beep( 3000,100);
	delay(150);
	beep( 2500,100);
	delay(150);

}

void loop() {
	checkForSensor();
	handler();
	getBatV();
	//ping();

	parsingSeparate();
	SerialRouter();

}

void handler(byte event_code = 0, long unsigned data = 0) {
	// Events codes:
	// 	1	Btn1
	// 	2	Btn2
	// 	3	Btn3
	// 	4	Sensor
	// 	5	Recieved
	// 	10	BTSerial command

	// States
	//	0	Menu
	//	1	Race

	if(event_code==0) event_code = checkForEvents();
	//if(event_code) {log("\n"); log(event_code);}

	if(event_code == 1  || event_code == 3) beep(140,10);
	if( event_code == 2 ) beep(400,20);

	if(state == 0) { // Состояние - меню
		if(event_code == 1) { // Нажата  кнопка 1
			if(!menu_entered) {
				if(menu<=0) {
					menu = 16;
				} else {
					menu--;
				}
			}
		} else if(event_code == 2) { // Нажата  кнопка 2
			if(state!=1 && !(state==0 && menu == 1)) {
				menu_entered = !menu_entered;	
			}
		} else if(event_code == 3) { // Нажата  кнопка 3
			if(!menu_entered) {
				if(menu>=16) {
					menu = 0;
				} else {
					menu++;
				}
			}
		}

		if(menu == 0) { // Заезд
			if(!menu_entered) {
				displayWord(_r,_A,_C,_E); // Race
			} else {
				displayCurRacer(cur_racer);
				if(event_code == 1) {
					if(cur_racer>1) {
						cur_racer--;
						setCurRacer(cur_racer);						
					}
				} else if(event_code == 3) {
					if(cur_racer<999) {
						cur_racer++;
						setCurRacer(cur_racer);
					}
				} else if(event_code == 4) {
					//send(1,1,cur_racer);
					state = 1; // go to state 1
					timer_started = millis();
					result_time = 0;
					laps_counter = 0;
					beep( 9050,300);
					log("\nr "); log(cur_racer); log(" started");
				} else if (event_code == 10) {
					if(data) setCurRacer(data);
				}
			}
		} else if (menu == 1) { // Вывод всех сохраненных результатов на дисплей
			if(!menu_entered) {
				displayWord(_r,_S,_L,_t); // RSLT
				if(event_code == 2) {
					menu_entered = true;
					menu_deep = 0;
				}
			} else {
				unsigned int write_to_addr = EEPROM.get(0, write_to_addr);
				if(write_to_addr==0) {
					displayWord(__,__,_N,_O); // __NO
					if(event_code == 2) {
						menu_deep = 0;
						menu_entered = false;

					}
					return;
				}

				last_result_num = write_to_addr/sizeof(Row)-1;
				if(cur_result_to_read == 9999) {
					cur_result_to_read = last_result_num;
				}

				// log(write_to_addr);
				// log(" ");
				// log(cur_result_to_read);
				// log(" ");
				// log(last_result_num);
				// log("\n");

				if(event_code == 1) {
					if(cur_result_to_read<=0) {
						cur_result_to_read = last_result_num;
					} else {
						cur_result_to_read--;
					}
				} else if (event_code == 3) {
					if(cur_result_to_read>=last_result_num) {
						cur_result_to_read = 0;
					} else {
						cur_result_to_read++;
					}
				}

				Row stored_row;				
				stored_row = readResult(cur_result_to_read);

				if(menu_deep==0) {
					displayCurRacer(stored_row.r);
					if(event_code == 2) {
						menu_deep = 1;
					}
				} else {
					displayCurRacer(stored_row.t);
					if(event_code == 2) {
						menu_deep = 0;
						menu_entered = false;
					}
				}
				
			}
		} else if (menu == 2) { // Вывод всех сохраненных результатов в порт
			if(!menu_entered) {
				displayWord(_F,_L,_S,_H); // FLSH
			} else {
				printAllResults();
				delay(500);
				menu_entered = 0;
			}
		} else if (menu == 3) { // Стирание результатов
			if(!menu_entered) {
				displayWord(_C,_L,_r,__); // CLr_
			} else {
				displayWord(_y,__,__,_n); // y__n
				if(event_code == 1) {
					menu_entered = 0;
				} else if(event_code == 3) {
					clearResults();
					delay(500);
					menu_entered = 0;
				}				
			}
		} else if (menu == 4) { // Включение/выключение сохранения результатов
			if(!menu_entered) {
				displayWord(_S,_A,_u,_E); // Race
			} else {
				if(event_code == 1 || event_code == 3) {
					SAVE_RESULTS = !SAVE_RESULTS;
					EEPROM.put(2, SAVE_RESULTS);
				}
				if(event_code == 11) {
					log("\nSAVE_RESULTS: "); log(SAVE_RESULTS); log("\n");
				}
				if(event_code == 12) {
					SAVE_RESULTS = data;
					EEPROM.put(2, SAVE_RESULTS);
					log("\nSet SAVE_RESULTS: "); log(SAVE_RESULTS); log("\n");
				}
				if(SAVE_RESULTS==1) {
					displayWord(__,_y,_E,_S); // _YES
				} else {
					displayWord(__,__,_N,_O); // __NO
				}
			}
		} else if (menu == 5) { // Время отклика
			if(!menu_entered) {
				displayWord(_P,_i,_N,_G); // Ping
			} else {
				displayInt(int(ping_time));
			}
		} else if (menu == 6) { // Количество успешно отправленных пакетов
			if(!menu_entered) {
				displayWord(_L,_O,_S,_t); // Lost
			} else {
				displayInt(int(rssi));
			}
		} else if (menu == 7) { // Тестирование
			if(!menu_entered) {
				displayWord(_t,_E,_S,_t); // tESt
			} else {
				if(millis()-last_test_time > 300) {
					//send(6,1,10+test_send_counter);
					test_send_counter++;
					last_test_time = millis();
				}
				if(event_code == 5) {// Принято сообщение
					test_data = data;
					log("Recieved testing:");log(" ");log(data);log("\n");
				}
				displayTime(test_data);
			}
		} else if (menu == 8) { // Напряжение питания
			if(!menu_entered) {
				displayWord(_B,_A,_t,_r); // Batery
			} else {
				displayInt(1000*bat_v*bat_v_k);
				if(event_code == 20) {
					log("\nBatery voltage:"); log(" ");  BTSerial.print(bat_v*bat_v_k,3);  log("\n");
				}
			}
		} else if (menu == 9) { // Режим датчика
			if(!menu_entered) {
				displayWord(_S,_E,_N,_S); // SENS
			} else {
				if(event_code == 1) {
					if(SENSOR_MODE <= 0) {
						SENSOR_MODE = 1;
					} else {
						SENSOR_MODE--;
					}
					EEPROM.put(3, SENSOR_MODE);
				} else if (event_code == 3) {
					if(SENSOR_MODE >= 1) {
						SENSOR_MODE = 0;
					} else {
						SENSOR_MODE++;
					}
					EEPROM.put(3, SENSOR_MODE);
				}
				if(SENSOR_MODE==0) {
					displayWord(_F,_A,_i,_L); // FALL
				} else if (SENSOR_MODE==1) {
					displayWord(_r,_i,_S,_E); // RISE
				}
				
			}
		} else if (menu == 10) { // Режим круг/прямой
			if(!menu_entered) {
				displayWord(_n,_O,_D,_E); // MODE
			} else {
				if(event_code == 1 || event_code == 3) {
					MODE = !MODE;
					EEPROM.put(4, MODE);
					//log(MODE);log("\n");
				}
				if(event_code == 13) {
					log("\nMODE: "); log(MODE); if(MODE==0) {log(" (LAPS)");} else {log(" (LINE)");} log("\n");
				}
				if(event_code == 14) {
					MODE = data;
					EEPROM.put(4, MODE);
					log("\nSet MODE: "); log(MODE); if(MODE==0) {log(" (LAPS)");} else {log(" (LINE)");} log("\n");
				}
				if(MODE==0) {
					displayWord(_L,_A,_P,_S); // LAPS
					//radioOff();
				} else if (MODE==1) {
					displayWord(_L,_i,_N,_E); // LINE
					//radioOn();
				}
			}
		} else if (menu == 11) { // Количество кругов
			if(!menu_entered) {
				displayWord(_L,_A,_P,_N); // LAPN
				if(event_code == 2 && new_laps_number != LAPS_NUMBER) {
					LAPS_NUMBER = new_laps_number;
					EEPROM.put(9, LAPS_NUMBER);
					beep( 3000,100); delay(150);
				}
				new_laps_number = LAPS_NUMBER;
			} else {
				if(event_code == 1) {
					if(new_laps_number>0) new_laps_number--;
				}
				if(event_code == 3) {
					if(new_laps_number<255) new_laps_number++;
				}
				displayInt(new_laps_number); 
				if(event_code == 15) {
					displayInt(LAPS_NUMBER); 
					log("\nLAPS_NUMBER: "); log(LAPS_NUMBER); log("\n");
				}
				if(event_code == 16) {
					LAPS_NUMBER = data;
					EEPROM.put(9, LAPS_NUMBER);
					displayInt(LAPS_NUMBER); 
					log("\nSet LAPS_NUMBER: "); log(LAPS_NUMBER); log("\n");
					//beep( 3000,100); delay(150);
				}
			}

		} else if (menu == 12) { // SENSOR_MA_VALUES_NUM
			if(!menu_entered) {
				displayWord(_S,_A,_U,_G); // SAVG
				if(event_code == 2 && new_sensor_ma_values_num != SENSOR_MA_VALUES_NUM) {
					SENSOR_MA_VALUES_NUM = new_sensor_ma_values_num;
					EEPROM.put(5, SENSOR_MA_VALUES_NUM);
					beep( 3000,100); delay(150);
				}
				new_sensor_ma_values_num = SENSOR_MA_VALUES_NUM;
			} else {
				if(event_code == 1) {
					if(new_sensor_ma_values_num>2) new_sensor_ma_values_num--;
				}
				if(event_code == 3) {
					if(new_sensor_ma_values_num<255) new_sensor_ma_values_num++;
				}
				displayInt(new_sensor_ma_values_num); 
			}
		} else if (menu == 13) { // SENSOR_TRUE_LEVEL
			if(!menu_entered) {
				displayWord(_S,_L,_u,_L); // SLVL
				if(event_code == 2 && new_sensor_true_level != SENSOR_TRUE_LEVEL) {
					SENSOR_TRUE_LEVEL = new_sensor_true_level;
					EEPROM.put(7, SENSOR_TRUE_LEVEL);
					beep( 3000,100); delay(150);
				}
				new_sensor_true_level = SENSOR_TRUE_LEVEL;
			} else {
				if(event_code == 1) {
					if(new_sensor_true_level>0) new_sensor_true_level--;
				}
				if(event_code == 3) {
					if(new_sensor_true_level<255) new_sensor_true_level++;
				}
				displayInt(new_sensor_true_level); 
			}
			if(event_code == 3) {
				if(new_sensor_true_level<255) new_sensor_true_level++;
				displayInt(new_sensor_true_level); 
			}
			if(event_code == 17) {
				displayInt(SENSOR_TRUE_LEVEL); 
				log("\nSENSOR_TRUE_LEVEL: "); log(SENSOR_TRUE_LEVEL); log("\n");
			}
			if(event_code == 18) {
				SENSOR_TRUE_LEVEL = data;
				EEPROM.put(7, SENSOR_TRUE_LEVEL);
				displayInt(SENSOR_TRUE_LEVEL); 
				log("\nSet SENSOR_TRUE_LEVEL: "); log(SENSOR_TRUE_LEVEL); log("\n");
			}
		} else if (menu == 14) { // SENSOR_TIMEOUT
			if(!menu_entered) {
				displayWord(_S,_t,_n,_t); // Stnt
				if(event_code == 2 && new_sensor_timeout != SENSOR_TIMEOUT) {
					SENSOR_TIMEOUT = new_sensor_timeout;
					EEPROM.put(12, SENSOR_TIMEOUT);
					beep( 3000,100); delay(150);
				}
				new_sensor_timeout = SENSOR_TIMEOUT;
			} else {
				if(event_code == 1) {
					if(new_sensor_timeout>0) new_sensor_timeout--;
				}
				if(event_code == 3) {
					if(new_sensor_timeout<100000) new_sensor_timeout++;
				}
				displayInt(new_sensor_timeout); 
			}
			if(event_code == 3) {
				if(new_sensor_timeout<100000) new_sensor_timeout++;
				displayInt(new_sensor_timeout); 
			}
			if(event_code == 23) {
				displayInt(SENSOR_TIMEOUT); 
				log("\nSENSOR_TIMEOUT: "); log(SENSOR_TIMEOUT); log("\n");
			}
			if(event_code == 24) {
				SENSOR_TIMEOUT = data;
				EEPROM.put(12, SENSOR_TIMEOUT);
				displayInt(SENSOR_TIMEOUT); 
				log("\nSet SENSOR_TIMEOUT: "); log(SENSOR_TIMEOUT); log("\n");
			}
		} else if (menu == 15) { // Включение/выключение звука
			if(!menu_entered) {
				displayWord(_n,_U,_t,_E); // MUTE
			} else {
				if(event_code == 1 || event_code == 3) {
					MUTE = !MUTE;
					EEPROM.put(11, MUTE);
				}
				if(event_code == 21) {
					log("\nMUTE: "); log(MUTE); log("\n");
				}
				if(event_code == 22) {
					MUTE = data;
					EEPROM.put(11, MUTE);
					log("\nSet MUTE: "); log(MUTE); log("\n");
				}
				if(MUTE==1) {
					displayWord(__,_y,_E,_S); // _YES
				} else {
					displayWord(__,__,_N,_O); // __NO
				}
			}
		} else if (menu == 16) { // Настройка датчика
			if(!menu_entered) {
				displayWord(_S,_C,_A,_L);  // SCAL
			} else {
				displayInt(sensor_value); 
			}
		}
	} else if(state == 1) { // Состояние - заезд
		if(event_code == 2 || event_code == 40) { // Нажата средняя кнопка
			log("Cancel race: ");log(cur_racer);log("\n");
			//send(1,2,0);
			displayWord(_C,_N,_C,_L); // Cancel
			result_time = 0;
			laps_counter = 0;
			cur_time = 0;
			beep( 3000,200); delay(250);
			beep( 3000,200); delay(250);
			state = 0;
			menu_entered = 0;
		// } else if(event_code == 5) { // Принято сообщение
		// 	if(data>128) { // Пришел результат заезда
		// 	    log("\nResult: ");log(cur_racer);log(": ");Serial.print(0.001*data,3);BTSerial.print(0.001*data,3);log("\n\n");
		// 		result_time = data;
		// 		displayTime(result_time);
		// 		beep( 3000,100); delay(150);
		// 		beep( 3000,100); delay(150);
		// 		beep( 3000,100); delay(150);
		// 		if(SAVE_RESULTS) writeResult(cur_racer,result_time);
		// 		unsigned int write_to_addr = EEPROM.get(0, write_to_addr);
		// 		last_result_num = write_to_addr/sizeof(Row)-1;
		// 		cur_result_to_read = last_result_num;
		// 	    state = 2;
		// 	} else { // Пришла команда
		// 	}

			// best_lap_racer
			// best_racer
			// best_lap_time
			// best_time
		} else if (event_code == 4) { // Сработал датчик
			if(MODE==0) { // В круговом режиме -  проехан круг
				//displayWord(__,__,__,__);
				beep( 4000,40); delay(60);
				beep( 4000,40); 
				display_blocked = millis();
				laps_counter++;
				cur_lap_time = millis() - timer_started - cur_time;
				cur_time = millis() - timer_started;
				displayInt(laps_counter);

			    log("\nLap ");log(laps_counter);log(": ");millisToTime(cur_time);log(" ");millisToTime(cur_lap_time);
				if(best_lap_time==0) {
					best_lap_racer = cur_racer;
					best_lap_time = cur_lap_time;
				} else if (cur_lap_time<best_lap_time) {
					best_lap_racer = cur_racer;
					best_lap_time = cur_lap_time;
				    log(" Best lap result!");
					delay(100);
					beep( 523,130); delay(140); // До
					beep( 587,130); delay(140); // Ре
					beep( 659,130); delay(140); // Ми
					beep( 880,300); delay(300);
					beep( 987,400); 
				}

				if(laps_counter>=LAPS_NUMBER) { // проехан последний круг
					//result_time = millis() - timer_started;
					result_time = cur_time;
				    log("\nResult:");log(" ");log(cur_racer);log(": ");millisToTime(result_time);
					laps_counter = 0;
					displayTime(result_time);
					beep( 3000,100); delay(150);
					beep( 3000,100); delay(150);
					beep( 3000,100); delay(150);

					if(best_time==0) {
						best_racer = cur_racer;
						best_time = result_time;
					} else if (result_time<best_time) {
						best_racer = cur_racer;
						best_time = result_time;
					    log(" Best result!");
						delay(100);
						beep( 659,130); delay(140);
						beep( 659,130); delay(140);
						beep( 659,130); delay(140);
						beep( 523,600); 
					}
					log("\n\n");

					if(SAVE_RESULTS) writeResult(cur_racer,result_time);
					unsigned int write_to_addr = EEPROM.get(0, write_to_addr);
					last_result_num = write_to_addr/sizeof(Row)-1;
					cur_result_to_read = last_result_num;
					cur_time = 0;
					state = 2;

				}
			}
		}
		if((millis() - display_blocked) >= 1000) displayTime(millis() - timer_started);

	} else if(state == 2) { // Состояние - отображение результатов
		if(result_time>0) {
			displayTime(result_time);
		}
		if(event_code == 1 || event_code == 2 || event_code == 3) {
			state = 0;
		} else if(event_code == 4) {
			//send(1,1,cur_racer);
			state = 1;
			timer_started = millis();
			beep( 9050,300);
			result_time = 0;
			laps_counter = 0;
			log("\nr "); log(cur_racer); log(" started");
		}

	}

	if(LASER && event_code) {
		if(menu_entered && (menu == 0 || menu == 16)) { // Включаем лазер при входе в пункт меню "гонка" или "настройка датчика"
			digitalWrite(laserPIN, HIGH);
		} else {
			digitalWrite(laserPIN, LOW);
		}
	}

	//// old dysp setDisplay(0,0,0,event_code);
}


bool btn1_pressed = false;
bool btn2_pressed = false;
bool btn3_pressed = false;
long unsigned btn_pressed_time = 0;

byte checkForEvents() {
	if(millis() - btn_last_released < BTN_TMT) return 0;
	bool btn1 = (digitalRead(btn1PIN)==LOW);
	bool btn2 = (digitalRead(btn2PIN)==LOW);
	bool btn3 = (digitalRead(btn3PIN)==LOW);

	if(btn1|btn2|btn3 && btn_pressed_time == 0) {
		btn_pressed_time = millis();
	}

	if(!btn1) {
		if(btn1_pressed) {
			btn1_pressed = btn1;
			btn_last_released = millis();
			return 1;
		}
	} else {
		if(millis() - btn_pressed_time > BTN_LONGPRESS_TIME) {
			btn_last_released = millis();
			return 1;
		}
	}
	btn1_pressed = btn1;

	if(!btn2) {
		if(btn2_pressed) {
			btn2_pressed = btn3;
			btn_last_released = millis();
			return 2;
		}
	} else {
		if(millis() - btn_pressed_time > BTN_LONGPRESS_TIME) {
			btn_last_released = millis();
			return 2;
		}
	}
	btn2_pressed = btn2;

	if(!btn3) {
		if(btn3_pressed) {
			btn3_pressed = btn3;
			btn_last_released = millis();
			return 3;
		}
	} else {
		if(millis() - btn_pressed_time > BTN_LONGPRESS_TIME) {
			btn_last_released = millis();
			return 3;
		}
	}
	btn3_pressed = btn3;

	if(!(btn1|btn2|btn3)) {
		btn_pressed_time = 0;
	}


	return 0;
}

bool sensor_released = false;
bool laser_state;

void checkForSensor() {
	if(SENSOR_ON_INTERRUPT) return;
	
	bool sens = false;
	if(LASER) {
	    sensor_value = ((SENSOR_MA_VALUES_NUM-1)*sensor_value + analogRead(laser_sensPIN))/SENSOR_MA_VALUES_NUM;
		if(sensor_value>=SENSOR_TRUE_LEVEL) {
			laser_state = true;
			//log("\n"); log(laser_sensPIN); log(" "); log(sensor_value);
		} else {
			if(laser_state) sens = true;
			laser_state = false;
		}
	} else {
		sensor_value = digitalRead(sensPIN);
		sens = (sensor_value==LOW);
	}

	if(SENSOR_MODE == 0) {
		if(!sens) {
			if(sensor_released) {
				sensor_released = sens;
				onSensor();
			}
		}
	} else {
		if(sens) {
			if(!sensor_released) {
				sensor_released = sens;
				onSensor();
			}
		}
	}
	sensor_released = sens;
}

void onSensor() {
	if(millis() - sens_last_released < SENSOR_TIMEOUT) {return false;}
	sens_last_released = millis();
	handler(4);
	//log("Sensor released\n");
}

void onRecieved(long unsigned data) {
    if(SEND_DEBUG_MODE) {log("> Recieved: ");log(data);log("\n");}
    //send(5,1,cur_racer);
    handler(5, data);
}

void setCurRacer(unsigned int n) {
	cur_racer = n;
	displayCurRacer(cur_racer);
	log("Set racer: ");log(cur_racer);log("\n");
	//send(2,1,cur_racer);
}

void getBatV() {
	if(millis() - bat_v_last_msrd < 100) return;
	if(bat_v == 0) {
		bat_v = analogRead(batvPIN);
	} else {
		bat_v = (analogRead(batvPIN) + 9*bat_v)/10;
	}
	bat_v_last_msrd = millis();
}

void getInputValues() {
	if(millis() - inputvals_last_showed < 1000) return;
	log("\nInput values\n");
	log("A0: "); log(analogRead(A0)); log("\n");
	log("A1: "); log(analogRead(A1)); log("\n");
	log("A2: "); log(analogRead(A2)); log("\n");
	log("A3: "); log(analogRead(A3)); log("\n");
	log("A4: "); log(analogRead(A4)); log("\n");
	log("A5 (laser_sensPIN): "); log(analogRead(A5)); log("\n");
	log("A6: "); log(analogRead(A6)); log("\n");
	log("A7: "); log(analogRead(A7)); log("\n");

	log("D0: "); log(digitalRead(0)); log("\n");
	log("D1: "); log(digitalRead(1)); log("\n");
	log("D2 (sensPIN): "); log(digitalRead(2)); log("\n");
	log("D3: "); log(digitalRead(3)); log("\n");
	log("D4: "); log(digitalRead(4)); log("\n");
	log("D5: "); log(digitalRead(5)); log("\n");
	log("D6 (laserPIN ON/OFF): "); log(digitalRead(6)); log("\n");
	log("D7: "); log(digitalRead(7)); log("\n");
	log("D8: "); log(digitalRead(8)); log("\n");
	log("D9: "); log(digitalRead(9)); log("\n");
	log("D10: "); log(digitalRead(10)); log("\n");
	log("D11: "); log(digitalRead(11)); log("\n");
	log("D12: "); log(digitalRead(12)); log("\n");
	log("D13: "); log(digitalRead(13)); log("\n");

	inputvals_last_showed = millis();
}

// void ping() {
// 	if(millis() - last_ping < PING_TMT) return;
// 	if(state==1) {
// 		send(3,1,cur_racer);
// 	} else {
// 		send(4,1,cur_racer);
// 	}
// 	last_ping = millis();
// }


// void send(long unsigned object, long unsigned method, long unsigned data) {
// 	if(MODE == 0) return;
// 	transmit_data[0] = object;
// 	transmit_data[1] = method;
// 	transmit_data[2] = data;
// 	long unsigned last_tx_time = micros();
// 	if(radio.write(&transmit_data, sizeof(transmit_data))) {
// 		if(object != 3 && SEND_DEBUG_MODE) {log("> Sended: ");log(object);log(" ");log(method);log(" ");log(data);log("\n");}
// 	    trnsmtd_pack++;
// 		if(ping_time == 0) {
// 			ping_time = micros() - last_tx_time;
// 		} else {
// 			ping_time = (micros() - last_tx_time + 4*ping_time)/5;
// 		}
// 	    if (!radio.available()) {                                  // если получаем пустой ответ
// 	    } else {
// 	      while (radio.available() ) {                    // если в ответе что-то есть
//         	radio.read(&recieved_data, sizeof(recieved_data));    // читаем
// 	        onRecieved(recieved_data);
// 	      }
//     	}
// 	} else {
//     	failed_pack++;
// 	}

// 	if (millis() - RSSI_timer > 5000) {                        // таймер RSSI
// 	    // рассчёт качества связи (0 - 100%) на основе числа ошибок и числа успешных передач
// 	    rssi = (1 - ((float)failed_pack / trnsmtd_pack)) * 100;  

// 	    // сбросить значения
// 	    failed_pack = 0;
// 	    trnsmtd_pack = 0;
// 	    RSSI_timer = millis();
// 	}

// }


// void radioSetup() {
//   radio.begin();              // активировать модуль
//   radio.setAutoAck(1);        // режим подтверждения приёма, 1 вкл 0 выкл
//   radio.setRetries(0, 15);    // (время между попыткой достучаться, число попыток)
//   radio.enableAckPayload();   // разрешить отсылку данных в ответ на входящий сигнал
//   radio.setPayloadSize(32);   // размер пакета, в байтах
//   radio.openWritingPipe(address[0]);   // мы - труба 0, открываем канал для передачи данных
//   radio.setChannel(CH_NUM);            // выбираем канал (в котором нет шумов!)
//   radio.setPALevel(SIG_POWER);         // уровень мощности передатчика
//   radio.setDataRate(SIG_SPEED);        // скорость обмена
//   // должна быть одинакова на приёмнике и передатчике!
//   // при самой низкой скорости имеем самую высокую чувствительность и дальность!!

//   radio.powerUp();         // начать работу
//   radio.stopListening();   // не слушаем радиоэфир, мы передатчик
// }

// void radioOn() {
// 	radio.powerUp();
// }

// void radioOff() {
// 	radio.powerDown();
// }



struct Row readResult(unsigned int i) {
	Row row;
	EEPROM.get(EEPROM_OFFSET + i*sizeof(Row), row);
	return row;
}

struct Row readLastResult() {
	unsigned int write_to_addr = EEPROM.get(0, write_to_addr);
	Row row;
	EEPROM.get(EEPROM_OFFSET + write_to_addr-sizeof(Row), row);
	return row;
}

void printAllResults(int r=0) {
	unsigned int write_to_addr = EEPROM.get(0, write_to_addr);
	if(write_to_addr==0) return;
	Row stored_row; 
	for(int i=0; i*sizeof(Row)<=(write_to_addr-sizeof(Row)); i++) {
	    stored_row = readResult(i);
	    if(r==0 || stored_row.r==r) {
			log(stored_row.r);log(" ");millisToTime(stored_row.t);log("\n");
	    }
		if(i>999) return;
	}
}

void clearResults() {
	EEPROM.put(0,0);
	log("Results cleared");log("\n");
}

int unsigned writeResult(int unsigned racer, long unsigned time) {
	Row row;
	int unsigned write_to_addr;
	row.r = racer;
	row.t = time;
	EEPROM.get(0, write_to_addr);
	EEPROM.put(EEPROM_OFFSET + write_to_addr, row);
	write_to_addr += sizeof(Row);
	EEPROM.put(0, write_to_addr);
	log("[Saved]\n");
	return write_to_addr;
}

void readSettings() {

	SAVE_RESULTS = EEPROM.get(2, SAVE_RESULTS);
	//if(SAVE_RESULTS != true || SAVE_RESULTS != false) SAVE_RESULTS = false;
	log("SAVE_RESULTS: ");
	log(SAVE_RESULTS);
	log("\n");

	SENSOR_MODE = EEPROM.get(3, SENSOR_MODE);
	//if(SENSOR_MODE != true || SENSOR_MODE != false) SENSOR_MODE = false;
	log("SENSOR_MODE: ");
	log(SENSOR_MODE);
	log("\n");
	
	MODE = EEPROM.get(4, MODE);
	//if(MODE != 1 || MODE != 0) MODE = 0;
	MODE = 0; // temp

	log("MODE: ");
	log(MODE);
	log("\n");

	SENSOR_MA_VALUES_NUM = EEPROM.get(5, SENSOR_MA_VALUES_NUM);
	//if(SENSOR_MA_VALUES_NUM != 1 || SENSOR_MA_VALUES_NUM != 0) SENSOR_MA_VALUES_NUM = 0;
	log("SENSOR_MA_VALUES_NUM: ");
	log(SENSOR_MA_VALUES_NUM);
	log("\n");

	SENSOR_TRUE_LEVEL = EEPROM.get(7, SENSOR_TRUE_LEVEL);
	//if(SENSOR_TRUE_LEVEL != 1 || SENSOR_TRUE_LEVEL != 0) SENSOR_TRUE_LEVEL = 0;
	log("SENSOR_TRUE_LEVEL: ");
	log(SENSOR_TRUE_LEVEL);
	log("\n");


	LAPS_NUMBER = EEPROM.get(9, LAPS_NUMBER);
	//if(LAPS_NUMBER != 1 || LAPS_NUMBER != 0) LAPS_NUMBER = 0;
	log("LAPS_NUMBER: ");
	log(LAPS_NUMBER);
	log("\n");

	MUTE = EEPROM.get(11, MUTE);
	//if(MUTE != true || MUTE != false) MUTE = false;
	log("MUTE: ");
	log(MUTE);
	log("\n");

	SENSOR_TIMEOUT = EEPROM.get(12, SENSOR_TIMEOUT);
	//if(SENSOR_TIMEOUT != 1000 || SENSOR_TIMEOUT != 0) SENSOR_TIMEOUT = 1000;
	log("SENSOR_TIMEOUT: ");
	log(SENSOR_TIMEOUT);
	log("\n");

	log("\n");
}

template<typename T>
T log(T text) {
	Serial.print(text);
	BTSerial.print(text);
}


void displayCurRacer(int unsigned number) {
  uint8_t data[] = { _r, 0x00, 0x00, 0x00 }; 
  data[1] = display.encodeDigit(int(number/100)-10*int(number/1000));
  data[2] = display.encodeDigit(int(number/10)-10*int(number/100));
  data[3] = display.encodeDigit(int(number)-10*int(number/10));
  display.setSegments(data, 4, 0);
}

void displayInt(int unsigned number) {
  uint8_t data[] = { 0x00, 0x00, 0x00, 0x00 }; 
  data[0] = display.encodeDigit(int(number/1000)-10*int(number/10000));
  data[1] = display.encodeDigit(int(number/100)-10*int(number/1000));
  data[2] = display.encodeDigit(int(number/10)-10*int(number/100));
  data[3] = display.encodeDigit(int(number)-10*int(number/10));
  display.setSegments(data, 4, 0);
}

void displayTime(long unsigned ms){
  int unsigned d0,d1,d2,d3,m,s = 0;
  uint8_t data[] = { 0b00110001, 0x00, 0x00, 0x00 }; 
  if(ms < 10000) {
    d0 = int(ms/1000);
    d1 = int(ms/100)-10*d0;
    d2 = int(ms/10)-10*int(ms/100);
    d3 = int(ms)-10*int(ms/10);
  } else if (10000 <= ms && ms < 60000) {
    d0 = int(ms/10000);
    d1 = int(ms/1000) - 10*int(ms/10000);
    d2 = int(ms/100) - 10*int(ms/1000);
    d3 = int(ms/10) - 10*int(ms/100);
  } else if (ms >= 60000) {
    s = int(ms/1000);
    m = int(s/60);
    d0 = int(m/10);
    d1 = m - 10*d0;
    d2 = int((s-m*60)/10);
    d3 = s-m*60 - 10*d2;
  }
  data[0] = display.encodeDigit(d0);
  data[1] = display.encodeDigit(d1);
  data[2] = display.encodeDigit(d2);
  data[3] = display.encodeDigit(d3);
  if(ms<10000) {
    data[1] &= ~(1 << 7);
  } else {
    data[1] |= (1 << 7);
  }
  display.setSegments(data, 4, 0);
}

void displayWord(uint8_t d1,uint8_t d2,uint8_t d3,uint8_t d4) {
  uint8_t data[] = {d1,d2,d3,d4}; 
  display.setSegments(data);
}

void millisToTime(long unsigned time){
  int unsigned h_,m_,s_,m,s,ms = 0;
  s_ = int(time/1000);
  m_ = int(time/60000);
  h_ = int(time/3600000);
  ms = time-s_*1000;
  s = s_-m_*60;
  m = m_-h_*60;
  log(m<10?"0":"");log(m);log(":");log(s<10?"0":"");log(s);log(".");log(ms<10?"0":"");log(ms<100?"0":"");log(ms);
  //return (m<10?"0":"")+String(m)+":"+(s<10?"0":"")+String(s)+"."+(ms<10?"0":"")+(ms<100?"0":"")+String(ms);
  //return (m<10?"0":"")+String(m)+":"+(s<10?"0":"")+String(s)+"."+(ms<10?"0":"")+(ms<100?"0":"")+String(ms);
}

void parsingSeparate() {
	if (BTSerial.available() > 0) {
		if (parseStage == WAIT) {
			parseStage = HEADER;
			prsHeader = "";
			prsValue = "";
		}
		if (parseStage == GOT_HEADER) parseStage = VALUE;
		char incoming = (char)BTSerial.read();
		if (incoming == divider) {
			parseStage = GOT_HEADER;
		} else if (incoming == ending) {
			parseStage = SUCCESS;
		}
		if (parseStage == HEADER) {
			prsHeader += incoming;
		}
		else if (parseStage == VALUE) prsValue += incoming;
		prsTimer = millis();
	}
	if (parseStage == SUCCESS) {
		for (byte i = 0; i < headers_am; i++) { if (prsHeader == headers[i]) { thisName = i; } } recievedFlag = true; parseStage = WAIT; } if ((millis() - prsTimer > 10) && (parseStage != WAIT)) {  // таймаут
		parseStage = WAIT;
	}
	// if (parseStage == HEADER) {
	// 	if (millis() - prsTimer > 10 ) {
	// 		parseStage == SUCCESS;
	// 	}
	// }
}

void SerialRouter() {
	if (recievedFlag) {
		recievedFlag = false;
		if(thisName == GET_RESULTS && state !=1) {
			if(prsValue.toInt()==0) {
				log("\nResults all"); log("\n\n");
				printAllResults(); log("\n");
			} else {
				log("\nResults for racer "); log(prsValue); log("\n\n");
				printAllResults(prsValue.toInt()); log("\n");
			}
		} else if (thisName == GET_INPUTS && state !=1) {
			getInputValues();
		} else if (thisName == GET_CONFIG && state !=1) {
			readSettings();
		} else if (thisName == RACE && state !=1) {
			menu=0;
			menu_entered=(prsValue.toInt()>0);
			handler(10, prsValue.toInt());
		} else if (thisName == SAVE && state !=1) {
			menu=4;
			menu_entered=true;
			if(prsValue == "") {
				handler(11, 0);
			} else {
				handler(12, 1*(prsValue.toInt()>0));
			}
		} else if (thisName == _MUTE && state !=1) {
			menu=15;
			menu_entered=true;
			if(prsValue == "") {
				handler(21, 0);
			} else {
				handler(22, 1*(prsValue.toInt()>0));
			}
		} else if (thisName == _MODE && state !=1) {
			menu=10;
			menu_entered=true;
			if(prsValue == "") {
				handler(13, 0);
			} else {
				handler(14, 1*(prsValue.toInt()>0));
			}
		} else if (thisName == LAPS && state !=1) {
			menu=11;
			menu_entered=true;
			if(prsValue == "") {
				handler(15, 0);
			} else {
				handler(16, prsValue.toInt());
			}
		} else if (thisName == _STIMEOUT && state !=1) {
			menu=14;
			menu_entered=true;
			if(prsValue == "") {
				handler(23, 0);
			} else {
				handler(24, prsValue.toInt());
			}
		} else if (thisName == LEVEL && state !=1) {
			menu=13;
			if(prsValue == "") {
				handler(17, 0);
			} else {
				handler(18, prsValue.toInt());
			}
		} else if (thisName == CANCEL) {
			handler(40);
		} else if (thisName == SCAL && state !=1) {
			menu=16;
			menu_entered=true;
			handler(19);
		} else if (thisName == BATR && state !=1) {
			menu=8;
			menu_entered=true;
			handler(20);
		} else if (thisName == BUTTON1) {
			handler(1);
		} else if (thisName == BUTTON2) {
			handler(2);
		} else if (thisName == BUTTON3) {
			handler(3);
		} else if (thisName == HELP && state !=1) {
			printHelp();
		}
		thisName = '0'; prsValue=""; parseStage = WAIT;
	}
}

void beep(unsigned int freq, unsigned int duration) {
	if(MUTE) return;
	tone(beepPIN, freq,duration);
}

void printHelp() {
	log("\n<,^,> : release 1,2,3 Buttons");
	log("\ngetresults <1-255>: returns result for racer/all");
	log("\ngetinputs : returns all inputs values");
	log("\ngetconfig : returns config");
	log("\nrace <1-255>: go to race mode and set racer");
	log("\nsave <1-0>: get/set saving results to memory option");
	//log("\nmode <1/0>: get/set Line or Laps mode");
	log("\nlaps <1-255>: get/set laps number (for Laps mode) ");
	log("\nlevel <1-1024>: get/set sensor TRUE level ");
	log("\nstimeout <0-10000>: get/set sensor timeout in milliseconds");
	log("\ncancel : cancel race for curet racer (if it started)");
	log("\nscal : go to sensor calibration mode");
	log("\nbatr : returns batery voltage");
	log("\nmute <1-0>: get/set muting sounds");
	log("\nhelp : print this^");
	log("\n");


}
