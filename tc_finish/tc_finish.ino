//  ---------------- TC Finish ------------------------

// Sensor 

#define sensPIN 2

// Beeper

#define beepPIN 8

// Batery
#define batvPIN A1

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

// Radio

//--------------------- НАСТРОЙКИ ----------------------
#define CH_AMOUNT 3  // число каналов
#define CH_NUM 0x0c //0x60   // номер канала (должен совпадать)

#define SEND_DEBUG_MODE 0   // выводить отправленное  и принятое
byte SENSOR_MODE = 0;   // режим датчика (FALLING/RISING/CHANGE)
#define SENSOR_ON_INTERRUPT false   // Датчик висит на прерывании
#define TIME_DELAY 22   // поправка

// уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
#define SIG_POWER RF24_PA_MIN
//#define SIG_POWER RF24_PA_HIGH

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

#include <Arduino.h>
#include <TM1637Display.h>
TM1637Display display(disp_clkPIN, disp_dioPIN);

//--------------------- БИБЛИОТЕКИ ----------------------

//--------------------- ПЕРЕМЕННЫЕ ----------------------
byte pipeNo;
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; // возможные номера труб

volatile unsigned long sens_last_released;
volatile unsigned int sens_tmt = 1000;

unsigned long transmit_data;  // массив пересылаемых данных
unsigned long recieved_data[CH_AMOUNT]; // массив принятых данных

int unsigned cur_racer, rx_object, rx_method, rx_data;
volatile byte state = 0;
volatile unsigned long timer_started, timer_finished, result_time, bat_v_last_msrd;

unsigned long bat_v = 0;
//double bat_v_k = 0.005017845;
double bat_v_k = 0.005215686;

void setup() {
  pinMode(sensPIN, INPUT_PULLUP);
  if(SENSOR_ON_INTERRUPT) {
    if(SENSOR_MODE == 0) {
      attachInterrupt(0, onSensor, FALLING);
    } else if (SENSOR_MODE == 1) {
      attachInterrupt(0, onSensor, RISING);
    }
  }

  radioSetup();

  pinMode(beepPIN, OUTPUT);
  pinMode(batvPIN, INPUT);


  Serial.begin(9600);
  Serial.println(" ====TC Finish ====");

  display.setBrightness(7);

  displayWord(_H,_E,_L,_L);
  delay(500);
}

void loop() {
  checkForSensor();
  handler();
  getBatV();
  listen();
}

void handler(byte event_code = 0, long unsigned data = 0) {
  if(cur_racer==0)   display.showNumberDec(bat_v*bat_v_k*100);

  if(event_code == 5) {
    rx_object = recieved_data[0];
    rx_method = recieved_data[1];
    rx_data = recieved_data[2];
  }
  
  if(event_code==0) event_code = checkForEvents();

  if(state == 2) { // В режиме отображения результатов
    if(event_code>0) { // любое событие
      if(rx_object != 4) state = 0;  // , кроме пинга,  вызывает переход в режим ожидания
    }
  }

  if(state == 0) { // Ожидание | Показ результтаов
    if(event_code == 5) {
      if(rx_object == 1) { // Таймер
        if(rx_method == 1) { // Старт
          cur_racer = rx_data;
          timer_started = millis();
          state = 1;
          Serial.print("Timer started:");Serial.print("\t");Serial.print(cur_racer);Serial.print("\n");
        }
      } else if (rx_object == 2)  { // Настройка
        if(rx_method == 1) { // Смена номера участника
          cur_racer = rx_data;
          Serial.print("Set racer:\t");Serial.print(cur_racer);Serial.print("\n");
          displayCurRacer(cur_racer);
        }
      } else if (rx_object == 3)  { // Get
        if(rx_method == 1) { // запрос результатов заезда
          send(result_time); 
        }
      } else if (rx_object == 4)  { // Ping
        if(rx_method == 1) { 
          if(state == 0) {
            if(cur_racer == 0) {
              cur_racer = rx_data;
              displayCurRacer(cur_racer);
            } else {
              cur_racer = rx_data;  
            }
          } else if(state == 1) {
            cur_racer = rx_data;
            displayCurRacer(cur_racer);
          }
        }
      } else if (rx_object == 6)  { // Testing
        if(rx_method == 1) { 
          //displayWord(_t,_E,_S,_t);
          Serial.print("Testing recieved: ");Serial.print("\t");Serial.print(rx_data);Serial.print("\n");
          delay(100);
          send(100*rx_data); 
          display.showNumberDec(rx_data);
        }
      }     
    }
  } else if(state == 1) { // Заезд
    displayTime(millis()-timer_started);
    if(event_code == 4) {
      timer_finished = millis();
      result_time = timer_finished - timer_started - TIME_DELAY;
      Serial.print("Result:");Serial.print("\t");Serial.print(cur_racer);Serial.print(":\t");Serial.print(0.001*result_time,3);Serial.print("\n\n");
      send(result_time); 
      state = 2;
    } else if (event_code == 5) {
      if (rx_object == 1)  { // Таймер
        if (rx_method == 2) { // Отмена заезда
          timer_started = 0;
          timer_finished = 0;
          result_time = 0; 
          state = 0; 
          displayWord(_C,_N,_L,_d);
          Serial.print("Race canceled:");Serial.print("\t");Serial.print(cur_racer);Serial.print("\n");
        }
      }
    }
  } else if(state == 2) { // Показ результатов
    displayTime(result_time);
  }

}

void listen() {
  while (radio.available(&pipeNo)) {                                 // слушаем эфир
    radio.read( &recieved_data, sizeof(recieved_data));              // чиатем входящий сигнал
    onRecieved(recieved_data);
  }  
}

byte checkForEvents() {
  return 0;
}

bool sensor_released = false;
void checkForSensor() {
  if(SENSOR_ON_INTERRUPT) return;
  bool sens = (digitalRead(sensPIN)==LOW);
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
  if(millis() - sens_last_released < sens_tmt) {return false;}
  sens_last_released = millis();
  handler(4);
  Serial.print("Sensor released\n");
}

void onRecieved(long unsigned data[CH_AMOUNT]) {
    if(SEND_DEBUG_MODE) {Serial.print("> Recieved:\t");Serial.print(data[0]);Serial.print("\t");Serial.print(data[1]);Serial.print("\t");Serial.print(data[2]);Serial.print("\n");}
    handler(5, data);
}

void send(long unsigned data) {
  transmit_data = data;
  radio.writeAckPayload(pipeNo, &transmit_data, sizeof(transmit_data));
  if(SEND_DEBUG_MODE) {Serial.print("> Sended:\t");Serial.print(data);Serial.print("\n");}
}


void radioSetup() {         // настройка радио
  radio.begin();               // активировать модуль
  radio.setAutoAck(1);         // режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 15);     // (время между попыткой достучаться, число попыток)
  radio.enableAckPayload();    // разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32);    // размер пакета, байт
  radio.openReadingPipe(1, address[0]);   // хотим слушать трубу 0
  radio.setChannel(CH_NUM);               // выбираем канал (в котором нет шумов!)
  radio.setPALevel(SIG_POWER);            // уровень мощности передатчика
  radio.setDataRate(SIG_SPEED);           // скорость обмена
  // должна быть одинакова на приёмнике и передатчике!
  // при самой низкой скорости имеем самую высокую чувствительность и дальность!!

  radio.powerUp();         // начать работу
  radio.startListening();  // начинаем слушать эфир, мы приёмный модуль
}

void getBatV() {
  if(millis() - bat_v_last_msrd < 100) return;
  if(bat_v == 0) {
    bat_v = analogRead(batvPIN);
  } else {
    bat_v = (analogRead(batvPIN) + 9*bat_v)/10;
  }
  bat_v_last_msrd = millis();
  //Serial.print("> Bat V:\t");Serial.print(bat_v);Serial.print("\n");
}

void displayCurRacer(int unsigned number) {
  uint8_t data[] = { _r, 0x00, 0x00, 0x00 }; 
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



