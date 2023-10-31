#include <Servo.h>                       // подключение библиотеки для управления двигателями
#include "I2Cdev.h"                      // библиотека для работы с шиной I2C
#include "MPU6050_6Axis_MotionApps20.h"  // библиотека для работы с MPU
#include "GyverFilters.h"                // библиотека фильтров

#define BUFFER_SIZE 100                  // буфер для работы с MPU
#define PIN_TRIG 4                       // присваивание пина 4 выходу TRIG датчика высоты
#define PIN_ECHO 6                       // присваивание пина 6 выходу ECHO датчика высоты

int static_thrust = 1050;                // значение базового сигнала для двигателей (контролеры двигателей принимают значения порядком от 800 до 2000)
int MIN_thrust = 920;                    // минимально допустимый сигнал двигателя
int MAX_thrust = 1500;                   // максимально допустимый сигнал двигателя
int eng1_thrust;                         // значение сигнала для левого двигателя с регулированием
int eng2_thrust;                         // значение сигнала для правого двигателя с регулированием
int high_thrust;                         // значение сигнала с регулированием по высоте
int pot_angle;                           // значение требуемого угла с потенциометра
int pot_high;                            // значение требуемой высоты с потенциометра

float high;                              // значение высоты 
float mpu_angle;                         // значение угла с MPU
float error_high;                        // значение ошибки регулирования высоты
float error_angle;                       // значение ошибки регулирования угла
float prev_error_high = 0;               // значение предыдущей ошибки высоты
float prev_error_angle = 0;              // значение предыдущей ошибки высоты
float dt = 0.01;                         // период вычислений 

volatile bool flag_stop = false;         // переменная флаг для отслеживания аварийной остановки
uint8_t fifoBuffer[45];                  // буфер

// переменные для расчёта угла :
Quaternion q;
VectorFloat gravity;
float ypr[3];                            // массив для записи значений углов (в радианах)

static uint32_t timer1;                  // переменная таймера для выдержки периода регулирования
static uint32_t timer2;                  // переменная таймера для DMP MPU
static uint32_t timer3;                  // переменная таймера для отправки данных в UART

//объекты :
MPU6050 mpu;                             // объект MPU
GMedian <10, int> filter;                // медианный фильтр для фильтрации задающего угла ( <размер окна, тип данных> )
Servo engine1;                           // левый двигатель
Servo engine2;                           // правый двигатель

void setup() {  
  Serial.begin(9600);                    // настройка скорости для порта UART
  Wire.begin();                          // старт шины I2C
  Wire.setClock(1000000UL);              // разгон шины на максимум
  pinMode(A0,INPUT);                     // определение входа 1-го потенциометра (задающее устройство угла)
  pinMode(A1,INPUT);                     // определение входа 2-го потенциометра (задающее устройство высоты)
  pinMode(2,INPUT_PULLUP);               // определение входа на пин D2 для конопки аварийной остановки
  pinMode(PIN_TRIG, OUTPUT);             // определение TRIG выхода датчика высоты
  pinMode(PIN_ECHO, INPUT);              // определение ECHO входа датчика высоты
  engine1.attach(3);                     // назначение пина для левого двигателя
  engine2.attach(5);                     // назначение пина для правого двигателя
  attachInterrupt(0,stop_engine,RISING); // определение прерывания на D2 при смене с HIGH на LOW
  
  // инициализация DMP(обработчик движений в MPU6050) :
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
}

// функция основного цикла алгоритма
void loop() {
  
  // блок работы регуляторов высоты и угла
  if(millis() - timer1 >= 10){                       // выдержка периода регулирования по таймеру (dt)
    pot_high  = map(analogRead(A1),0,1024,10,42);    // получение требуемого значения высоты с потенциометра 
    pot_angle = map(analogRead(A0),0,1024,-10,+21);  // получение требуемого угла с потенциометра
    pot_angle = filter.filtered(pot_angle);          // фильтрация сигнала с потенциометра
    high = get_high();                               // получение фактического значения высоты с датчика
    mpu_angle = get_angle();                         // получение фактического угла с MPU
    error_high = pot_high - high;                    // расчет ошибки регулирования высоты
    error_angle = pot_angle - mpu_angle;             // расчет ошибки регулирования угла
    PID_high();                                      // расчет ПИД-регулироемого сигнала высоты на двигателей

    // работа ПИД регулятора угла с разными коэффициентами для разной высоты (работает грубо, необходим более точный адаптивный регулятор)
    if(high >= 10 and high < 15){
      PID_angle(5,0,1,1);                            // расчет ПИД-регулироемого сигнала угла на левый двигатель
      PID_angle(5.5,0,1,2);                          // расчет ПИД-регулироемого сигнала угла на правый двигатель
    }
    else if(high >= 15 and high < 35){
      PID_angle(7.0,0,1.1,1);                        // расчет ПИД-регулироемого сигнала угла на левый двигатель
      PID_angle(7.5,0,1.1,2);                        // расчет ПИД-регулироемого сигнала угла на правый двигатель
    }

    else if(high >= 35 and high <= 42){
      PID_angle(9.5,0,1.2,1);                         // расчет ПИД-регулироемого сигнала угла на левый двигатель
      PID_angle(10.0,0,1.2,2);                        // расчет ПИД-регулироемого сигнала угла на правый двигатель
    }
     
    prev_error_high = error_high;                     // запоминание ошибки высоты
    prev_error_angle = error_angle;                   // запоминание ошибки угла
    timer1 = millis();                                // зброс таймера для периода регулирования 
  }

  // ограничение минимума и максимума для сигнала на двигатели
  eng1_thrust = constrain(eng1_thrust,MIN_thrust,MAX_thrust);
  eng2_thrust = constrain(eng2_thrust,MIN_thrust,MAX_thrust);
  
  // подача сигнала на двигатели с проверкой на аварийную остановку
  // в штатном режиме :
  if(flag_stop == false){
    engine1.write(eng1_thrust);
    engine2.write(eng2_thrust);
  }
  // при аварийной остановке (сброс тяги на 0) :
  else{
    engine1.write(800);
    engine2.write(800);
  }

  // блок вывода данных в порт UART
  if(millis() - timer3 >= 200){ 
    Serial.println("ОШИБКА УГЛА: " + String(error_angle)); 
    Serial.println("ОШИБКА ВЫСОТЫ : "  + String(error_high));
    Serial.println("ЛЕВЫЙ : " + String(eng1_thrust)); 
    Serial.println("ПРАВЫЙ : "  + String(eng2_thrust));
    Serial.println();                               
    timer3 = millis();  // сброс таймера
  }
 // send_data(); // отправка данных в порт UART для отображения в AlainPlot
}

// функция установки флага остановки (вызывается при прерывании с кнопки)
void stop_engine(){
  flag_stop = true;
}

// функция получения высоты подёма 
float get_high(){
  
  float duration;  // значение высоты высоты в сантиметрах
  
  // Генерация короткого импульса длительностью 2-5 микросекунд.
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIG, HIGH);

  // Выставив высокий уровень сигнала, ждем около 10 микросекунд. В этот момент датчик будет посылать сигналы с частотой 40 КГц.
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  // расчет высоты по времени задержки акустического сигнала на эхолокаторе. (29.1 - константа для датчика HC-SR04)
  duration = (pulseIn(PIN_ECHO, HIGH) / 2) / 29.1; 
  return(duration);
}

// функция для получения углов с MPU
float get_angle(){
  if (millis() - timer2 >= 11) {                      // таймер на 11 мс отлова момента отправки данных с DMP
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      // расчёты угла :
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      timer2 = millis();                              // сброс таймера
    }
  }
  return(degrees(ypr[2]) + 3);                        // возврат значения угла в градусах
}
 
// функция ПИД регулятора высоты
void PID_high(){
  
  float P = 0;    // пропорциональная составляющая регулятора
  float I = 0;    // интегральная составляющая регулятора
  float D = 0;    // дифференциальная составляющая регулятора
  float Kp = 25;  // пропорциональный коеффициент
  float Ki = 0;   // интегральный коеффициент
  float Kd = 0;   // дифференциальный коеффициент
  
  P = error_high * Kp;                             // расчет пропорциональной составляющей
  I = (I + error_high * dt) * Ki;                  // расчет интегрирующей составляющей
  I = constrain(I,0,20);                           // ограничение интегральной составляющей 
  D = ((error_high - prev_error_high) / dt) * Kd;  // расчет дфференциальной составляющей  
  high_thrust =  P + I + D;                        // расчет регулированого значения тяги
}

// функция ПИД регулятора для угла  < П-коеф., И-коеф., Д-коеф., номер двигателя >
void PID_angle(float Kp,float Ki,float Kd, int eng_numb){
  
  float P = 0;  // пропорциональная составляющая регулятора
  float I = 0;  // интегральная составляющая регулятора
  float D = 0;  // дифференциальная составляющая регулятора
 
  P = error_angle * Kp;                              // расчет пропорциональной составляющей
  I = (I + error_angle ) * dt * Ki;                  // расчет интегрирующей составляющей
  I = constrain(I,0,20);                             // ограничение интегральной составляющей 
  D = ((error_angle - prev_error_angle) / dt) * Kd;  // расчет дфференциальной составляющей
  
  // установка отрегулированых значений для двигателей (с прибавкой по высоте)
  switch(eng_numb){
    case 1 :
      eng1_thrust = static_thrust + P + I + D + high_thrust;
      break;
    case 2 :
      eng2_thrust = static_thrust - P - I - D + high_thrust;
      break;    
  }
}

// функция отправки данных в порт для отображения в AlainPlot
void send_data() {
  union {
    float fl;         // сюда записываем выходное значение
    uint8_t bytes[4]; // отсюда получаем готовый к отправке массив байт
  } 
  float_to_bytes;
  Serial.write(253);
 
 //Здесь передаются данные в данной версии ПО
  float_to_bytes.fl = pot_high;  // левый график входного значения высоты
  //float_to_bytes.fl = pot_angle  // левый график входного значения угла
  Serial.write(float_to_bytes.bytes[0]);
  Serial.write(float_to_bytes.bytes[1]);
  Serial.write(float_to_bytes.bytes[2]);
  Serial.write(float_to_bytes.bytes[3]);

  float_to_bytes.fl = high;      // правый график выходного значения высоты
  //float_to_bytes.fl = mpu_angle; // правый график выходного значения угла
  Serial.write(float_to_bytes.bytes[0]);
  Serial.write(float_to_bytes.bytes[1]);
  Serial.write(float_to_bytes.bytes[2]);
  Serial.write(float_to_bytes.bytes[3]);
  Serial.write(0);   // флаг завершения ;
  Serial.write(254);
  //if(accident_flag)  while(1) { };
}
