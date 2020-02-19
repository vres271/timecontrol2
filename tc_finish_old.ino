//  ---------------- TC Finish ------------------------

// Sensor 

#define sensPIN 2

// Beeper

#define beepPIN 8

// Radio

//--------------------- НАСТРОЙКИ ----------------------
#define CH_AMOUNT 3  // число каналов
#define CH_NUM 0x60   // номер канала (должен совпадать)

#define SEND_DEBUG_MODE 0   // выводить отправленное  и принятое

// уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
//#define SIG_POWER RF24_PA_MIN
#define SIG_POWER RF24_PA_HIGH

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

volatile unsigned long sens_last_released;
volatile unsigned int sens_tmt = 2000;

unsigned long transmit_data;  // массив пересылаемых данных
unsigned long recieved_data[CH_AMOUNT]; // массив принятых данных

int unsigned cur_racer, rx_object, rx_method, rx_data;
volatile byte state = 0;
volatile unsigned long timer_started, timer_finished, result_time;
//volatile bool result_recieve_confirmed = false;

void setup() {
  pinMode(sensPIN, INPUT_PULLUP);
  attachInterrupt(0, onSensor, FALLING);

  radioSetup();

  pinMode(beepPIN, OUTPUT);

  Serial.begin(9600);
  Serial.println(" ====TC Finish ====");

  //tone(beepPIN, 3000,1000); delay(1000); //noTone(beepPIN);
  //tone(beepPIN, 9050,1000); delay(1000); //noTone(beepPIN);
}

void loop() {
  while (radio.available(&pipeNo)) {                                 // слушаем эфир
    radio.read( &recieved_data, sizeof(recieved_data));              // чиатем входящий сигнал
    if(SEND_DEBUG_MODE) {Serial.print("> Recieved:\t");Serial.print(recieved_data[0]);Serial.print("\t");Serial.print(recieved_data[1]);Serial.print("\t");Serial.print(recieved_data[2]);Serial.print("\n");}

    rx_object = recieved_data[0];
    rx_method = recieved_data[1];
    rx_data = recieved_data[2];


    // Protocol
    //  1 Timer 
    //    1 start 13  racer number
    //    2 cancel  - curent racer
    //  2   
    // if(rx_object == 5) { // Подтверждение получения результатов стартовым модулем
    //   if(rx_method == 1) {
    //     result_recieve_confirmed = true;
    //   }
    // }
    if(state == 0) { // Ожидание
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
        } else if(rx_method == 2) { // Reset
          cur_racer = 1;
          timer_started = 0;
          timer_finished = 0;
          result_time = 0; 
          state = 0; 
          Serial.print("Reset");Serial.print("\n");
        }
      } else if (rx_object == 3)  { // Get
        if(rx_method == 1) { // запрос результатов заезда
          send(result_time); 
        }
      } else if (rx_object == 6)  { // Get
        if(rx_method == 1) { // запрос результатов заезда
          Serial.print("Testing recieved: ");Serial.print("\t");Serial.print(rx_data);Serial.print("\n");
          delay(100);
          send(100*rx_data); 
        }
      }
    } else if (state == 1) { // Заезд
      if (rx_object == 1)  { // Таймер
        if (rx_method == 2) { // Отмена заезда
          timer_started = 0;
          timer_finished = 0;
          result_time = 0; 
          state = 0; 
          Serial.print("Race canceled:");Serial.print("\t");Serial.print(cur_racer);Serial.print("\n");
        }
      }
    }
  }
}

void onSensor() {
  if(millis() < (sens_last_released + sens_tmt)) return false;
  sens_last_released = millis();
  Serial.print("Sensor released\n");

  if(state == 1) { // Заезд
    timer_finished = millis();
    result_time = timer_finished - timer_started;
    Serial.print("Result:");Serial.print("\t");Serial.print(cur_racer);Serial.print(":\t");Serial.print(0.001*result_time,3);Serial.print("\n\n");
    //result_recieve_confirmed = false;
    send(result_time); 
    state = 0;
  }
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
