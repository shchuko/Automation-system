/*-----------------------------------------------*/
/*ДИРЕКТИВА #include - ПОДКЛЮЧЕНИЕ ДОПОЛНИТЕЛЬНЫХ ФАЙЛОВ*/
#include <avr/wdt.h>  //для работы сторожевого таймера
#include <avr/pgmspace.h> //для работы PROGMEM
#include <LiquidCrystal_I2C.h> //для работы дисплея 1602 I2C
#include <DHT.h>  //для работы сенсора температуры и влажности
#include <DS1302.h> //для работы часов реального времени
#include <SPI.h>  //для обмена данными по шине SPI
#include <RF24.h> //для работы приёмопередатчика nRF24L01
#include <nRF24L01.h> //вспомогательная библиотека для nRF24L01
#include <EEPROM.h> //для работы с EEPROM
//#include <IRremote.h>

/*-----------------------------------------------*/
/*НАСТРОЙКИ*/

/*МОДУЛЬ ТЕМПЕРАТУРЫ И ВЛАЖНОСТИ DHTxx*/
#define DHT_PIN 7  //пин передачи данных
#define DHT_TYPE DHT11 //тип сенсора (DHT11 || DHT22 || DHT21)
#define DHT_INTERVAL 60000  //интервал опроса сенсора
#define DHT_TEMPERATURE_ERROR -3.5  //погрешность при измерении температуры

/*I2C ДИСПЛЕЙ*/
#define ROW_COUNT 2 //количество рядов дисплея
#define COLS_COUNT 16 //количество колонок дисплея
#define DISP_ADDR 0x27 //адрес дисплея (в шестнадцатеричной системе счисления 0xXX)
#define LDR_PIN A7  //пин подключения фоторезистора освещенности
#define DISPLGHTNESS_PIN 6 //пин подключения светодиода подсветки дисплея

/*СДВИГОВЫЙ РЕГИСТР 74HC595*/
#define DATA_REG_PIN 14 //пин передачи данных
#define LATCH_REG_PIN 15  //пин-"защелка"
#define CLOCK_REG_PIN 16  //пин синхронизации
#define OUTPUTS_REG_COUNT 8 //количество выходов регистра (0..64)

/*МИКРОСХЕМА ЧАСОВ РЕАЛЬНОГО ВРЕМЕНИ DS1302*/
#define CLCPIN_CE 2 //DS1302::CE - 5 вывод микросхемы 
#define CLCPIN_IO 3 //DS1302::I/O - 6 вывод микросхемы 
#define CLCPIN_SCLK 4 //DS1302::SCLK - 7 вывод микросхемы

/*МОДУЛЬ 2.4GHz NRF24L01*/
#define RF_CE_PIN 9
#define RF_CSN_PIN 10
#define RADIO_CHANNEL 0x6A
#define RADIO_REQUEST_INTERVAL 400

#define READING_PIPE_NUM 0
#define WRITING_PIPE_NUM 1

#define CENTRAL_BOARD_ADDRESS 0
#define RECEIVE_DATA_PIPE 1

#define ADDRESS 5

#define RF_DATARESET_TIME 10000 //время, в течение которого данные сохраняются, если связь потеряна (мс)
#define RF_NOTLISTEN_TIME 800

/*ЗУММЕР*/
#define BUZZER_PIN 5 //пин подключения 

/*АНАЛОГОВЫЕ КНОПКИ*/
#define KEYS_PIN A6 //пин подключения кнопок
#define KEYS_UNBOUNCE_TIME 35 //интервал времени (мс) опроса для защиты от дребезга
#define KEYS_COUNT 7 //количество кнопок
#define KEYS_ERROR 40 //погрешность сдвига - прибавить к значению с АЦП
#define KEYS_BIT_SHIFT 5 //количество бит, на которые необходимо сдвинуть значение с АЦП вправо(с учетом погрешности) для устранения дребезга

/*ДОПОЛНИТЕЛЬНЫЕ НАСТРОЙКИ*/
#define BAUDRATE 9600 //скорость последовательного порта
#define BUZZER_PIN 5  //пин подключения зуммера
#define BLUETOOTH_ENABLE_PIN 8//пин управления питанием модуля HC-06 Bluetooth (1 - вкл, 0 - выкл)
#define ALARMS_COUNT 4  //количество будтльников
#define IR_PIN 17 //пин подключения ИК-приемника
#define NIGHTMODE_EEPROM_ADDRESS 1000

//переменные для вывода данных на дисплей
const char CelsiumDegree[8] PROGMEM = {0x18, 0x18, 0x06, 0x08, 0x08, 0x08, 0x06, 0x00}; //*C
const char ThermometerOut[8] PROGMEM = {0x04, 0x0A, 0x0E, 0x0E, 0x0E, 0x1F, 0x1F, 0x0E};  //закрашенный термометр
const char ThermometerIns[8] PROGMEM = {0x04, 0x0A, 0x0A, 0x0A, 0x0E, 0x1F, 0x1F, 0x0E}; //пустой термомер
const char HumidityIns[8] PROGMEM = {0x04, 0x0A, 0x0A, 0x11, 0x11, 0x11, 0x0E, 0x00}; //пустая капля
const char HumidityOut[8] PROGMEM = {0x04, 0x0E, 0x0E, 0x1F, 0x1F, 0x1F, 0x0E, 0x00}; //закрашенная капля

const char Bell[8] PROGMEM = {0x04, 0x0E, 0x0E, 0x0E, 0x1F, 0x00, 0x04, 0x00};  //колокольчик
const char Antenna[8] PROGMEM = {0x0F, 0x05, 0x03, 0x01, 0x15, 0x09, 0x15, 0x00}; //0x00, 0x1F, 0x15, 0x0E, 0x04, 0x04, 0x04, 0x00}; //антенна

const char DownArrow[8] PROGMEM = {0x00, 0x04, 0x04, 0x04, 0x15, 0x0E, 0x04, 0x00}; //стрелка вниз
const char UpArrow[8] PROGMEM = {0x00, 0x04, 0x0E, 0x15, 0x04, 0x04, 0x04, 0x00}; //стрелка вверх

const char Check[8] PROGMEM = {0x00, 0x01, 0x03, 0x16, 0x1C, 0x08, 0x00, 0x00}; //галочка
const char Cross[8] PROGMEM = {0x00, 0x00, 0x1B, 0x0E, 0x04, 0x0E, 0x1B, 0x00}; //крестик
const char Clock[8] PROGMEM = {0x00, 0x00, 0x0E, 0x15, 0x17, 0x11, 0x0E, 0x00}; //часы

const char Hot[8] PROGMEM = {0x00, 0x00, 0x0E, 0x1F, 0x1B, 0x1F, 0x0E, 0x00};  //стремительное потепление - жаркое солнце
const char Sunny[8] PROGMEM = {0x00, 0x00, 0x0E, 0x11, 0x15, 0x11, 0x0E, 0x00}; //потепление  - солнце
const char Normal[8] PROGMEM = {0x0C, 0x1E, 0x00, 0x0E, 0x11, 0x15, 0x11, 0x0E};//погода не изменится - облачно с прояснениями
const char Rainy[8] PROGMEM = {0x00, 0x0E, 0x1F, 0x15, 0x04, 0x04, 0x0C, 0x0C};  //ухудшение погоды
const char Storm[8] PROGMEM = {0x0C, 0x1E, 0x1F, 0x00, 0x08, 0x08, 0x0C, 0x04}; //стремительное ухудшение погоды
/*-----------------------------------------------*/
/*НЕМНОЖКО ООП*/

struct Data_meteo //структура данных, генерируемых на метеомодуле
{
  byte address = 0;
  byte command = 0;
  byte check_byte = 0; //байт проверки качества передачи данных
  byte connection = 0; //Побитовая кодировка дежурной информации:
  /*  0b [0][0][0][0][0][0][0][0]
          ^  ^  ^  ^  ^  ^  ^  ^
          P  M  A  H  Y  W  1  0
      P - Панель управления (>> 7)
      M - Метеомодуль (>> 6)
      A - Панель актуаторов  (>> 5)
      H - Хаб - центральный контроллер (>> 4)
      Y - Флаг високосного года (>> 3)
      W - Warning! (>> 2)
      1 - Индивдуальный бит передачи данных 1 (>> 1)
      0 - Индивидуальный бит передачи данных 0 (>> 0)
  */
  int  temperature; //для передачи температуры на улице
  byte humidity; //для передачи уровня влажносити на улице (относ.)
  long pressure; //для передачи текущего значения атмосферного давления
  byte illumination; //для передачи текущего уровня освещенности (0..100)
  byte forecast; //для передачи прогноза погоды
  /* 0 - гарантированное улучшение погоды
     1 - возможно улучшение погоды
     2 - погода не изменится
     3 - возможно ухудшение погоды
     4 - гарантированно ухудшение погоды
  */
  int dPressure; //тенденция изменения атмосферного давления за 3 часа
  bool forecast_flag = 0;
};

struct Data_from_panel //структура данных, генерируемых на панели управления
{
  byte address = 0;
  byte command = 0;
  byte check_byte = 0;  //байт проверки связи
  byte connection = 0; //Побитовая кодировка дежурной информации:
  /*  0b [0][0][0][0][0][0][0][0]
   *      ^  ^  ^  ^  ^  ^  ^  ^
   *      P  M  A  H  Y  W  1  0
   *  P - Панель управления (>> 7)
   *  M - Метеомодуль (>> 6)
   *  A - Панель актуаторов  (>> 5)
   *  H - Хаб - центральный контроллер (>> 4)
   *  Y - Флаг високосного года (>> 3)
   *  W - Warning! (>> 2)
   *  1 - Индивдуальный бит передачи данных 1 (>> 1)
   *  0 - Индивидуальный бит передачи данных 0 (>> 0)
   */
 
  int temperature;  //температура с датчика на панели управления
  byte humidity;  //влажность с датчика на панели управления
  byte illumination;  //уровень освещенности с датчика на панели управления 

  byte second; //секунда
  byte minute; //минута
  byte hour; //час
  byte date; //день
  byte month; //месяц
  byte day; 
};

struct Data_panel_act //структура данных с настройками актуатора
{
  byte address = 0;
  byte command = 0;
  byte check_byte = 0; //байт проверки связи
  byte count; //количество актуаторов
  byte num; //номер актуатора
  bool enable_flag; //состояние актуатора, передаваемое на панель акт.
  bool state; //состояние актуратора, получаемое с панели акт.
  byte mode;  //режим работы

  int  temperature; //температура срабатывания
  bool t_sensor; //тип сенсора температуры
  bool t_mode; //режим срабатывания от значения температуры

  byte humidity;  //влажность срабатывания
  bool h_sensor; //тип сенсора влажности
  bool h_mode; //режим срабатывания от уровня влажности

  byte illumination;  //освещенность срабатывания
  bool i_sensor; //тип сенсора освещенности
  bool i_mode; //режим срабатывания от уровня освещенности

  byte hour = 12; //час срабатывания
  byte minute = 30; //минута срабатывания
  byte repeat_days = 0b10000000;  //дни повтора срабатывания
};

enum Forecast { //перечисление: погода
  HOT    = 0, //стремительное улучшение
  SUNNY  = 1, //улучшение
  NORMAL = 2, //погода не изменится
  RAINY  = 3, //ухудшение
  STORM  = 4  //стремительное ухудшение
};

class SR_74HC595
/*Предназначен для работы со сдвиговым регистром 74HC595
   реализовано на скорую руку, требует доработки
   задейструет 3 цифровых пина
   управляет 1 регистром
      для изменения данного параметра раскомментировать
      требуемое количество памяти
        максимум - 64 бита - 64 выхода - 8 регистров, включенных последовательно
   Enable() включает запись в регистр(ы), все выходы в 0
   Disable() выключает запись в регистр(ы), все выходы в 0
   Write() аналогично digitalWrite(), нумерация пинов с 0
   getState() возвращает состояние выхода регистра, имеет 2 реалтзации (перегружена)
   getEnable() возвращает состояние возможности записи в регистр
*/
{
  private:
    byte      data_pin;  //пин передачи данных
    byte      latch_pin; //пин-защелка
    byte      clock_pin; //пин синхронизации
    bool      enable; //включить/выключить регистр

    //меньше 8 выходов
    uint8_t   states;  //8 бит патями для хранения состояний выходов регистра

    //от 8 до 16 выходов
    //  uint16_t states;  //16 бит патями для хранения состояний выходов регистра
    //  outputs_count 16; //количество выходов регистра

    //от 16 до 32 выходов
    //  uint32_t states;  //32 бита патями для хранения состояний выходов регистра
    //  outputs_count 32; //количество выходов регистра

    //от 32 до 64 выходов
    //  uint64_t states;  //64 бита патями для хранения состояний выходов регистра
    //  outputs_count 64; //количество выходов регистра

    void regClear() //функция сброса всех выводов регистра в "0"
    {
      states = 0;  //перезаписать сохраненные состояния выходов

      digitalWrite(latch_pin, LOW); //отключить защиту от записи в регистр
      //for (byte i = 0; i < outputs_count; i += 8) //для количества выходов более 8
      shiftOut(data_pin, clock_pin, MSBFIRST, 0); //отправить обнуленный байт данных в регистр
      digitalWrite(latch_pin, HIGH);  //включить защиту от записи в регистр
    }

  public:
    void Write()  //метод не принимает переменные
    {
      if (enable)
        regClear(); //перевод всех выходов регистра в 0
    }
    
    void Write(byte pin, bool state)  //метод принимает (byte номер_пина [0..8], bool состояние_пина)
    {
      if (enable && (states >> pin & (1 << 0)) != state) //если бит состояния не совпадает с новым состоянием и сдвиговый регистр активен
      {
        if (state)  //если новое состояние - "1"
          states |= (1 << pin);  //обновить переменную памяти - битовый сдвиг маски влево, исключающее побитовое ИЛИ
        else  //если новое состояние - "0"
          states &= ~(1 << pin);  //обновить переменную памяти - битовый сдвиг маски влево, побитовое инвертирование маски, побитовое И
        digitalWrite(latch_pin, LOW);   //отключить защиту от записи в регистр

        //раскомментировать две последующие строки, если выводов больше 8
        //for (byte i = 0; i < outputs_count; i += 8) //запись в регистры побайтно
        //  shiftOut(data_pin, clock_pin, MSBFIRST, states >> i);
        //закомментировать последующую строку, если выходов больше 8
        shiftOut(data_pin, clock_pin, MSBFIRST, states);   //отправить обнуленный байт данных в регистр начиная с младшего бита
        digitalWrite(latch_pin, HIGH);  //включить защиту от записи в регистр
      }
    }

    void WriteByte(byte _byte)
    {
      digitalWrite(latch_pin, LOW);   //отключить защиту от записи в регистр
      shiftOut(data_pin, clock_pin, MSBFIRST, _byte);   //отправить обнуленный байт данных в регистр начиная с младшего бита
      digitalWrite(latch_pin, HIGH);  //включить защиту от записи в регистр
      states = _byte;
    }
    bool getState(byte pin) //принимает byte номер_пина
    {
      return (states >> pin) & (1 << 0); //вернуть состояние пина регистра
    }

    void getState(byte pin, bool &pin_state) //принимает byte номер_пина, bool &состояние_пина (ссылка на переменную)
    {
      pin_state = (states >> pin) & (1 << 0);  //изменить значение переменной состояние_пина
      //в сооответствии с состоянием выхода регистра
    }

    void Enable() //включить регистр
    {
      regClear(); //перевести все выходы регистра в 0
      enable = 1; //установить флаг активности регистра
    }

    void Disable()  //отключить регистр
    {
      regClear(); //перевести все выходы регистра в 0
      enable = 0; //обнулить флаг активности регистра
    }

    bool getEnable() //возвращает состояние регистра (включен - 1, выключен - 0)
    {
      return enable; //вернуть значение флага активности
    }

    SR_74HC595 (byte _data_pin, byte _latch_pin, byte _clock_pin) //конструкор класса
    //принимает byte номер_пина_данных, byte номер_пина_защелки, byte номер_пина_синхронизации
    {
      //сохранить полученные значения в поля объекта:
      data_pin = _data_pin;    //номер_пина_данных
      latch_pin = _latch_pin;  //номер_пина_защелки
      clock_pin = _clock_pin;  //номер_пина_синхронизации
      pinMode(data_pin, OUTPUT); //установить пин номер_пина_данных как выход
      pinMode(latch_pin, OUTPUT); //установить пин номер_пина_защелки как выход
      pinMode(clock_pin, OUTPUT); //установить пин номер_пина_синхронизации как выход
      Enable(); //включить регистр
    }
};

class Buzzer
/* Предназначен работы с зуммером
   В зуммер должен быть встроен генератор звуковой частоты
   Режимы работы:
    включить
    отключить
    включить на промежуток времени
    досрочно отключить до истечения времени
    периодически включать и отключать с задынным интервалом времени
      возможно изменение интервала по ходу выполнения переключений
    работает с ПРЕРЫВАНИЯМИ ПО ТАЙМЕРУ, 1 прерывание - 1 миллисекунда
      Пример настройки для таймера 1:
      Buzzer buzz(3); //зуммер buzz на пине 3
      void setup() {
        TCCR1B |= (1 << 3); //установка режима таймера: сброс при совпадении
        OCR1A = 0xF9; //запись значения в регистр сравнения (249)
        TIMSK1 |= (1 << OCIE1A);  //разрешить прерывание при совпадении с регистром A (OCR1A)
        TCCR1B |= (0 <<CS12) | (1 << CS11) | (1 << CS10); //усановить предделитель на 64
        sei();  //разрешаем прерывания
       }
       ISR(TIMER1_COMPA_vect)  //обработчик прерываний таймера 1 при совпадении с регистром A - вызывается раз в 1 мс
                    {
           buzz.ISR_check(); //проверка зуммера buzz
        }
*/
{
  private:
    byte                          pin; //нопер пина подключения
    bool                          enable; //состояние зуммера
    unsigned long                 beep_interval = 0; //интервал периодических включений
    bool                          beep_interval_flag = 0; //флаг акивности периодических включений
    unsigned long                 beep_millis_time; //длительность времени однократного включения
    bool                          beep_millis_flag = 0;  //флаг активности однократного включения
    volatile unsigned long int    counter; //счетчик миллисекунд (для прерываний)

  public:
    void Enable() //включить зуммер
    {
      digitalWrite(pin, HIGH);
      enable = 1;
    }

    void Disable()  //выключить зуммер
    {
      digitalWrite(pin, LOW);
      enable = 0;
    }

    void setState(bool state)
    {
      enable = state;
      digitalWrite(pin, enable);
    }

    bool getState()
    {
      return enable;
    }

    void intervalEnable(unsigned long beep_interval)  //активировать периодические включения зуммера (интервал передать в функцию)
    {
      if (!beep_interval_flag) //защита от повторного запуска
      {
        this->beep_interval = beep_interval;
        digitalWrite(pin, HIGH);  //включить зуммер
        cli();  //запретить прерывания
        beep_interval_flag = 1; //активировать периодические включения
        counter = 0;  //обнулить счетчик
        if (beep_millis_flag) //если акивно однократное включение
        {
          beep_millis_time = 0; //отключаем
          beep_millis_flag = 0; //обнуляем время однократного включения
        }
        sei();  //разрешить прерывания
      }
    }

    void intervalUpdate(unsigned long beep_interval) //изменение интервала "на ходу" (интервал передать в функцию)
    {
      if (beep_interval_flag) //если акивны периодические переключения зуммера
      {
        cli();  //запрещаем прерывания
        this->beep_interval = beep_interval;  //обновляем интервал
        counter = 0;  //обнулить счетчик
        sei();  //разрешаем прерывания
      }
    }

    void intervalDisable() //деактивировать периодические включения зуммера
    {
      cli();  //запретить прерывания
      beep_interval_flag = 0; //деактивировать периодические включения
      counter = 0;  //обнулить счетчик
      sei();  //разрешить прерывания
      digitalWrite(pin, LOW); //отключить зуммер
      beep_interval = 0;
    }

    void millisEnable(unsigned long beep_millis_time) //однократно включить зуммер на beep_millis_time миллисекунд
    {
      /* Сработает только если:
          предыдущий вызов был отработан (прошло beep_millis_time мс)
          предварительно вызвано Disable_millis()
          периодические включения зуммера не активны
         Проверка данных условий:
      */
      if (!beep_interval_flag && !beep_millis_flag)
      {
        digitalWrite(pin, HIGH);  //включить зуммер
        this->beep_millis_time = beep_millis_time;  //сохранить полученное временя однократного включения
        beep_millis_flag = 1;
      }
    }

    void millisDisable() //отключить однократно включенный зуммер
    //даже если beep_millis_time не истекло
    {
      cli();  //запрещаем прерывания
      counter = 0;  //обнуляем счетчик
      beep_millis_time = 0; //деактивируем однокрытные включения
      beep_millis_flag = 0; //обнуляем время однократного включения
      digitalWrite(pin, LOW);  //выключить зуммер
      sei(); //разрешаем прерывания
    }

    bool ISR_check() //Функция для обработчика прерываний - ВЫЗЫВАТЬ В ОБРАБОТЧИКЕ ПРЕРЫВАНИЙ! (интервал - 1 мс) ISR(...) {}
    // Возвращает 1 - активны периодические включения
    // Возвращает 0 - неактивны периодические включения
    {
      if (beep_interval_flag)  //проверка активности периодических включений
      { //если активны
        counter++;  //инкремент счетчика (+1 мс)
        if (counter >= beep_interval)   //если досчитал до интервала периодических включений
        {
          counter = 0;  //сброс счетчик
          digitalWrite(pin, !digitalRead(pin)); //инвертирование состояние зуммера
        }
        return 1; //вернуть 1
      } else {  //если не активны
        if (beep_millis_flag)  //проверка активности включения на промежуток времени
        {
          counter++;  //инкремент счетчика
          if (counter >= beep_millis_time)  //если прошло beep_millis_time времени
          {
            digitalWrite(pin, LOW); //отключаем зуммер
            beep_millis_time = 0; //обнуляем время однократного включения
            beep_millis_flag = 0; //обнуляем флаг однократного включения
            counter = 0;  //обнуляем счетчик
          }
        }
        return 0; //вернуть 0
      }
    }

    bool Interval_State()
    {
      return beep_interval_flag;
    }

    Buzzer(byte _pin)//, int beep_interval) //конструктор класса
    {
      pin = _pin;  //сохранить полученное значение
      //this->beep_interval = beep_interval;  //сохранить полученное значение
      pinMode(pin, OUTPUT); //установить пин зуммера как выход
    }
};

class Analog_Keys
/* Предназначен для работы с аналоговыми кнопками, подключенными к одному аналоговому входу
    ПРИ ВЫЗОВЕ analogRead() в loop()
      необходимо перевести флаг analog_read_process = 1
      после выхова analogRead()
      вернуть флаг analog_read_process = 1
          -- мера предосторожности из-за вызова analogRead() в обработчике прерываний
    очень сырой, оптимизирован под схему с некорректно рассчитанными номиналами резисоров
    используется АЦП микроконтроллера
    для работы необходимо указать bit_shift и error
      для 7 кнопок использован bit_shift = 5
                               error = 20
      рекомендуется использовать error = 0, >0 при нестабильном поведении
*/
{
  private:
    byte            pin; //пин подключения (аналоговый!)
    byte            count; //число подключенных кнопок
    byte            *voltage; //значения c АЦП  при нажатии кнопок ((analogRead(X) + 20) >> 5)
    byte            bit_shift;  //количество бит для сдвига вправо
    byte            error;  //погрешность (прибавляется к значению с АЦП перед сдвигом на bit_shift)
    byte            millis_unbounce; //время проверки нажатия (в миллисекундах) - защита от дребезга
    volatile int    value = voltage[count];  //устойчивое значение с АЦП
    bool            analog_read_process = 0; //флаг для предовращения наложения вызывов analogRead()

  public:
    void analogReadProcessBegin()
    {
      analog_read_process = 1;
    }

    void analogReadProcessEnd()
    {
      analog_read_process = 0;
    }

    void ISR_IDC_KeyValue() //Функция для обработчика прерываний - ВЫЗЫВАТЬ В ОБРАБОТЧИКЕ ПРЕРЫВАНИЙ! (интервал - 1 мс) ISR(...) {}
    {
      if (!analog_read_process) //если не АЦП не активен (не производится analogRead() где-либо еще)
      {
        int actual_value = (analogRead(pin) + error) >> bit_shift; //считать текущее значение ((..+20)>>5) - компенсация погрешности АЦП
        static int previous_value = voltage[count]; //предыдущее значение
        static byte millis_counter = 0; //счетчик времени
        if (actual_value != voltage[count]) //если кнопка нажата
        {
          if (actual_value == previous_value) //если значение совпало с предыдущим
          {
            millis_counter++; //увеличить счетчик
            if (millis_counter > millis_unbounce) //если прошло достаточно времени (антидребезг)
            { //=> полученное состояние устойчиво
              previous_value = voltage[count];  //обнулить предыдущее значение
              millis_counter = 0; //обнулить счетчик времени
              value = actual_value; //сохранить устойчивое состояние
            }
          } else {  //если значения не совпали
            previous_value = actual_value; //сохранить новое значение
          }
        } else {  //если кнопка не нажата
          millis_counter = 0; //обнулить счетчик
          previous_value = voltage[count]; //обнулить предыдущее значение
          value = voltage[count]; //сохраняем устойчивое состояние - все кнопки отпущены
        }
      }
    }

    byte Get() //возвращает номер нажатой кнопки
    {
      static int previous_value = 0;  //предыдущее значение
      cli();  //запретить прерывание
      byte infunc_value = value; //сохранить текущее значение
      sei();  //разрешить прерывания
      if (infunc_value != previous_value) //если значение не совпадает с предыдущим
      {
        for (byte i = 0; i < count; i++)  //в цикле проверяем
        {
          if (infunc_value == voltage[i]) //существует ли такая кнопка
          { //если существует
            previous_value = infunc_value;  //сохраняем полученное значение как предыдущее
            return i + 1; //возвращаем номер кнопки (выход из функции)
          }
        } //конец тела цикла
      }
      previous_value = infunc_value; //сохраняем полученное значение как предыдущее
      return 0; //возвращаем 0 - кнопка не нажата
    }

    Analog_Keys(byte _pin, byte _count, int _voltage[], byte _millis_unbounce, byte _bit_shift, byte _error)  //конструктор класса
    {
      pin = _pin;  //пин подключения
      count = _count;  //количество кнопок
      voltage = new byte[_count + 1];  //инициализировать указатель
      millis_unbounce = _millis_unbounce; //интервал времени (мс) опроса для защиты от дребезга
      bit_shift = _bit_shift;  //количество бит, на которые необходимо сдвинуть значение с АЦП (с учетом погрешности) для устранения дребезга
      error = _error;  //погрешность сдвига
      for (byte i = 0; i < count + 1; i++)  //в цикле: сохранить значения полученные с АЦП при
      { //нажатии кнопок со сдвигом и с учетом погрешности
        voltage[i] = (_voltage[i] + error) >> _bit_shift;  //сохранить
      }
    }

    ~Analog_Keys()  //так как мы выделяли память, ее необходимо освободить
    { //хотя этот деструктор никогда не будет вызван
      delete[] voltage; //но все же освободим, на всякий случай
    }
};

class Time_Correspondence  //класс - обработчик совпадения времени
{
  private:
    byte repeat_days = 0b10000000;  //дни повтора - редактировать методом (1 << n), для проверки включения повтора обратиться к разряду (1 << 7)
    bool enable = 0;  //флаг акивности
    byte hour = 12; //час - стандартно - 12
    byte minute = 30; //минута - стандартно - 30
    bool interal_enable = 0; //флаг активности для работы внутри класса

    bool state; //флаг активности сигнала
    bool resetting_flag = 0;

    bool pre_disable = 0;  //флаг ручного отключения сигнала

  public:
  
     void setEEPROM(byte addr)
      {
       EEPROM.update(addr + 511, repeat_days);
       addr++;
       EEPROM.update(addr + 511, enable);
       addr++;
       EEPROM.update(addr + 511, hour);
       addr++;
       EEPROM.update(addr + 511, minute);
       addr++;
      }

      void getEEPROM(byte &addr)
      {
       repeat_days = EEPROM.read(addr + 511);
       addr++;
       enable = EEPROM.read(addr + 511);
       addr++;
       hour = EEPROM.read(addr + 511);
       addr++;
       minute = EEPROM.read(addr + 511);
       addr++;
      }
    

    bool getState()  //возвращает состояние флага сигнала
    {
      return state;
    }

    bool getEnable() //возвращает состояния флага акивности
    {
      return enable;
    }

    byte getWeek()  //возвращает repeat_days
    {
      return repeat_days;
    }

    bool getDay(byte day_num)  //возвращает флаг дня недели
    {
      return (repeat_days >> (day_num - 1)) & (1 << 0);
    }

    byte getHour()  //возвращает часы
    {
      return hour;
    }

    byte getMinute()  //возвращает минуты
    {
      return minute;
    }

    void setEnable(bool _enable)  //устанавливает флаг акивности
    {
      enable = _enable;
      pre_disable = 0;  //флаг ручного отключения в 0
    }

    void setHour(byte hour) //устанавливает часы
    {
      this->hour = hour;
    }

    void setMinute(byte minute) //устанавлтвает минуты
    {
      this->minute = minute;
    }

    void setDay(byte day_num, bool flag) //устанавливает флаг дня недели
    {
      if ( flag )
        repeat_days |= (1 << (day_num - 1));
      else
        repeat_days &= ~(1 << (day_num - 1));
    }

    void setWeek(byte repeat_config)  //записать повторы в repeat_days в виде 1 байта (например 0b00100101)
    {
      repeat_days = repeat_config;
    }

    void Set(byte _hour, byte _minute, byte _repeat_days, bool _enable) //устанавливает параметры работы
    {
      hour = _hour;  //часы
      minute = _minute;  //минуты
      repeat_days = _repeat_days;  //дни повтора
      enable = _enable; //cостояние обработчика совпадения времени
      pre_disable = 0;  //флаг ручного отключения в 0
    }

    void Set(byte _hour, byte _minute, bool _enable)
    {
      Set(_hour, _minute, (1 << 7), _enable);
    }

    void Reset()  //сброс до стандартных значений
    {
      hour = 12; //час - стандартно - 12
      minute = 30; //минута - стандартно - 30
      repeat_days = 0b10000000;  //стандартно - повтор отключен
      enable = 0;  //флаг акивности - стандартно - 0
      pre_disable = 0;  //флаг ручного отключения в 0
    }

    bool Check(byte _hour, byte _minute, byte _day, bool manual_disable)  //проверка состояния
    {
      if ( enable && !pre_disable                       //если обработчик совпадения времени активек и сигнал не отключен вручную
           && ( getDay(8) || getDay(_day) )             //если повтор по дням отключен или совпал день
           && hour == _hour && minute == _minute )      //если совпало время
      {
        state = 1;  //переводим состояние флага активности сигнала в 1
      }

      bool previous_state = state;  //переменная для хранения флага активности сигнала до его изменения

      if ( state && !pre_disable && ( minute != _minute || manual_disable ) )  //если флаг сигнала == 1 И не было ручного отключения
        //И (время_когда_сигнал_активен не совпадает с время::минуты ИЛИ была нажата кнопка)
      {
        state = 0;  //перевести состояние флага сигнала в 0
        pre_disable = 1;  //флаг ручного отключения
        //previous_state = state;
        if ( getDay(8) )  //если повтор по дням отключен
        {
          enable = 0; //отключить обработчик совпадения времени
        }
      }

      if ( pre_disable &&  minute != _minute )
        pre_disable = 0;  //флаг ручного отключения в 0

      return previous_state;  //вернуть состояние сигнала
    }

    //конструкторы
    Time_Correspondence(byte hour,
                        byte minute,
                        byte repeat_days,
                        bool enable)
    {
      Set(hour, minute, repeat_days, enable);
    }

    Time_Correspondence(byte hour,
                        byte minute,
                        byte repeat_days)
    {
      Set(hour, minute, repeat_days, 0);
    }

    Time_Correspondence()
    {

    }
};

class Display1602_Menu
{
  private:
    LiquidCrystal_I2C* lcd;
    Time_Correspondence* Alarms;
    byte alarms_count;
    DS1302* rtc;

    Data_panel_act act_device;
    byte act_request_num = 1;
    Time_Correspondence act_device_sched = Time_Correspondence(12, 30, 0b10000000, 1);

    Time* real_time = NULL;

    bool* alarm_enable_flag = NULL;
    bool* connection_flag = NULL;
    bool* night_mode = NULL;
    byte* humidity_inside = NULL;
    byte* humidity_outside = NULL;
    int*  temperature_inside = NULL;
    int*  temperature_outside = NULL;
    unsigned int* pressure = NULL;
    int* pressure_change = NULL;
    bool* forecast_flag = NULL;
    bool* nigth_mode = NULL;
    bool* signalisation = NULL;
    Forecast* weather_forecast = NULL;

    byte display_menu[3] = {0, 0, 0};

    volatile unsigned int  display_timer = 0; //счетчик времени отображения сообщения
    volatile bool          display_timer_flag = 0;  //флаг активности работы счетчика времени
    volatile bool          display_timer_escape = 0;  //флаг выхода из экрана (прекращения вывода сообщения)

    const char space  = ' ';    //пробел
    const char zero   = '0';    //ноль
    const char colon  = ':';    //двоеточие
    const char sharp  = '#';    //решетка
    const char pct    = '%';    //знак процента

    const char week_day[7][4] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"}; //день недели

    byte char_created = 255; //номер дисплея, для которого созданы кастомные символы

    String AddChar(uint8_t value, char set_in)  //добавление символа перед числом, если оно одноразрядное
    {
      if (value == 0)
        return String(set_in) + String(set_in);
      else if (value < 10)
        return set_in + String(value, DEC);
      else
        return String(value, DEC);
    }

    void Clear()  //очистка дисплея
    {
      lcd->home();
      lcd->print("                ");
      lcd->setCursor(0, 1);
      lcd->print("                ");
    }

    void ClearLine(byte line)  //очистка строки дисплея
    {
      lcd->setCursor(0, line);
      lcd->print("                ");
      lcd->home();
    }

    bool MsgTimer(unsigned int interval_millis)  //вывод информации на дисплей с задержкой interval_millis
    {
      cli();  //запретить прерывания
      if (!display_timer_flag)  //если таймер дисплея неактивен
        display_timer_flag = 1; //активировать
      if (display_timer >= interval_millis) //если счетчик превысил заданный интервал
      {
        display_timer_flag = 0; //деактевировать таймер дисплея
        display_timer = 0;  //обнулить счетчик
        display_timer_escape = 1;   //флаг выхода перевести в 1
      }
      bool escape = display_timer_escape; //сохраняет значение флага выхода
      sei();  //разрешить прерывания
      return escape;  //вернуть значение флага выхода
    }

    void rstMsgTimer() //сброс таймера в начало отсчета
    {
      cli();
      display_timer = 0;
      sei();
    }

    void disableMsgTimer() //отключение таймера
    {
      cli();
      display_timer = 0;
      display_timer_flag = 0; //деактевировать таймер дисплея
      display_timer = 0;  //обнулить счетчик
      display_timer_escape = 1;   //флаг выхода перевести в 1
      sei();
    }

    void workSpace(byte &key)
    {
      if (char_created != display_menu[0])  //если символы для экрана не созданы
      { //создать
        lcd->createChar(0, Bell);  //колокольчик
        lcd->createChar(1, HumidityIns); //незакрашенная капелька
        lcd->createChar(2, ThermometerIns);  //незакрашенный термометр
        lcd->createChar(3, ThermometerOut);  //закрашенный термометр
        lcd->createChar(4, CelsiumDegree); //градусы по Цельсию
        char_created = display_menu[0]; //символы созданы для экрана 1
      }
      if (!key) //если все кнопки отпущены
      {
        lcd->home(); //курсор в начало дисплея
        lcd->print(AddChar(real_time->hr, zero)); //вывести время::часы
        lcd->print(colon); //вывести двоеточие
        lcd->print(AddChar(real_time->min, zero));  //вывести время::минуты
        lcd->print(space); //вывести пробел
        //вывести день недели:
        lcd->print((char)week_day[real_time->day - 1][0]);  //1 символ дня недели
        lcd->print((char)week_day[real_time->day - 1][1]);  //2 символ дня недели
        lcd->print((char)week_day[real_time->day - 1][2]);  //3 символ дня недели
        lcd->print(space); //вывести пробел
        if (!*alarm_enable_flag) //если все будильники отключены
          lcd->print(space); //вывести пробел
        lcd->print(AddChar(real_time->date, zero)); //вывести время::число_месяца
        lcd->print('/'); //вывести '/'
        lcd->print(AddChar(real_time->mon, zero));  //вывести время::месяц
        if (*alarm_enable_flag)  //если какой-либо из будильников включен
          lcd->write(0); //вывести символ: колокольчик
        else  //иначе
          lcd->print(space); //вывести пробел
        lcd->setCursor(0, 1);  //курсор на 0 символ 1 строки
        if (*connection_flag)  //если подключение активно
        {
          switch (*weather_forecast)  //переключатель: погода
          { //создаем соответствующий символ прогноза погоды
            case HOT:
              lcd->createChar(5, Hot);
              break;
            case SUNNY:
              lcd->createChar(5, Sunny);
              break;
            case NORMAL:
              lcd->createChar(5, Normal);
              break;
            case RAINY:
              lcd->createChar(5, Rainy);
              break;
            case STORM:
              lcd->createChar(5, Storm);
              break;
          }
        } else //если не активно
        {
          lcd->createChar(5, Antenna); //создаем символ перечеркнутой антенны
        }
        lcd->setCursor(0, 1);  //курсор на 0 символ 1 строки
        if (!*connection_flag) //если соединение неактивно
          lcd->print("   "); //выводим пропуск
        lcd->write(1); //вывести символ: незакрашенная капля
        lcd->print(*humidity_inside); //вывести влажность в помещении
        lcd->print(pct); //вывести '%'
        if (*humidity_inside < 10) //если влажность меньше 10
          lcd->print(space); //вывести пробел
        lcd->print(space); //вывести пробел
        lcd->write(2); //вывести символ: незакрашенный термометр
        lcd->print(*temperature_inside);  //вывести температуру в помещении
        lcd->write(4); //вывеси символ: градусы по Цельсию
        if (*temperature_inside < 10) //если температура в помещении меньше 10*С
          lcd->print(space); //вывести пробел
        lcd->print(space); //вывести пробел
        if (!*connection_flag) //если соединение неактивно
          lcd->print("   "); //выводим пропуск
        else if (*connection_flag)  //если соединение активно
        {
          lcd->write(3); //закрашенный термометр
          if (*temperature_outside > 0)  //если температура на улице больше 0
            lcd->print('+'); //вывести '+'
          lcd->print(*temperature_outside); //вывести температуру на улице
          lcd->write(4); //вывести символ: градусы по Цельсию
          if (abs(*temperature_outside) < 10)  //если температура на улице по модулю меньше 10
          {
            lcd->print(space); //вывести пробел
            if (*temperature_outside == 0) //если температура на улице равна 0
              lcd->print(space); //вывести пробел
          }
        }
        lcd->setCursor(15, 1); //курсор на последний символ строки 1
        lcd->write(5); //вывести символ: погода или антенна
      } else //если кнопка была нажата
      {
        char_created = 255; //сбрасываем номер дисплея, для которого созданы символы
        Clear();
        switch (key)  //переключатель: кнопка
        {
          case 1: //нажата кнопка 1
            display_menu[0] = 1;  //переключиться на дисплей 1
            break;
          case 2: //нажата кнопка 2
            if ( *connection_flag )
              display_menu[0] = 2;  //переключиться на дисплей 2
            break;
          case 3: //нажата кнопка 3
            display_menu[0] = 3;  //переключиться на дисплей 3
            break;
          case 4: //нажата кнопка 4
            display_menu[0] = 4;  //переключиться на дисплей 4
            //изменить состояния активности Bluetooth
            digitalWrite(BLUETOOTH_ENABLE_PIN, !digitalRead(BLUETOOTH_ENABLE_PIN));
            break;
          case 5: //нажата кнопка 5
            if ( *connection_flag )
              display_menu[0] = 5;  //переключиться на дисплей 5
            break;
          case 6: //нажата кнопка 6
            if ( *connection_flag )
            {
              *signalisation = !(*signalisation);
              display_menu[0] = 6;  //переключиться на дисплей 6
            }
            break;
          case 7: //нажата кнопка 7
            *night_mode = !*night_mode;
            display_menu[0] = 7;  //переключиться на дисплей 7
            break;
        } //конец тела переключателя
      }
    }
    
    void schedSpace(byte &key, Time_Correspondence* Devices_schedule, byte count, bool type, bool* type_state,  byte* menu, byte* itr, byte* num)
    {
      static byte i = 0;
      if (type)
        i = 0;
      static Time_Correspondence Edit;  //объект для временного хранения текущих параметров будильника
      static byte cursor_pos = 5; //позиция курсора стрелок
      static bool if_saved = 0; //сохранены изменения? 1 - да, 0 - нет
      enum AlarmField { //перечесление полей редактирования времени
        REPEAT_EN = 0, //повтор
        MON = 1,  //понедельник ...
        TUE = 2,
        WED = 3,
        THU = 4,
        FRI = 5,
        SAT = 6,
        SUN = 7,  //... воскресенье
        HOURS = 9,  //часы
        MINUTES = 10  //минуты
      };
      static AlarmField edit = HOURS; //начало редактирования: часы

      if (char_created != 1)  //если символы для экрана не созданы
      { //создать
        if (!type)
          lcd->createChar(0, Bell);  //колокольчик
        else
          lcd->createChar(0, Clock); //часы
        lcd->createChar(1, UpArrow); //стрелка вверх
        lcd->createChar(2, DownArrow);  //стрелка вниз
        lcd->createChar(3, Cross);  //крестик
        lcd->createChar(4, Check); //галочка
        char_created = 1; //символы созданы для экрана расписаний
        Clear();
      }
      
      if (display_menu[2] == 0) //главный экран будильника
      {
        if (key == 0)
        {
          lcd->home();
          lcd->write(0);
          if (!type)
            lcd->print(i + 1);
          else
            lcd->print(*num);
          lcd->print(colon);
          lcd->setCursor(5, 0);
          lcd->print(AddChar(Devices_schedule[i].getHour(), zero));
          lcd->print(colon);
          lcd->print(AddChar(Devices_schedule[i].getMinute(), zero));
          lcd->setCursor(13, 0);
          
          switch (type)
          {
            case 0:
              if (Devices_schedule[i].getEnable())
                lcd->print(" ON");
              else
                lcd->print("OFF");
             break;
            case 1:
              if (*type_state)
                lcd->print("ENB");
              else
                lcd->print("DIS");
             break;
          }

          if (Devices_schedule[i].getDay(8))
          {
            lcd->setCursor(0, 1);
            lcd->print(" Repeat disable ");
          }
          else
          {
            lcd->setCursor(0, 1);
            lcd->print(space);
            lcd->print('M');
            lcd->setCursor(3, 1);
            lcd->print('T');
            lcd->setCursor(5, 1);
            lcd->print('W');
            lcd->setCursor(7, 1);
            lcd->print('T');
            lcd->setCursor(9, 1);
            lcd->print('F');
            lcd->setCursor(11, 1);
            lcd->print('S');
            lcd->setCursor(13, 1);
            lcd->print('S');
            byte cursor_temporary_position = 2;
            for (byte j = 1; j < 8; j++)
            {
              lcd->setCursor(cursor_temporary_position, 1);
              if (Devices_schedule[i].getDay(j))
                lcd->write(4);
              else
                lcd->write(3);
              cursor_temporary_position += 2;
            }
            lcd->setCursor(15, 1);
            lcd->print(space);
          }
        } else
        {
          switch (key)
          {
            case 1:
              if (!type)
              {
                if (i == count - 1)
                  i = 0;
                else
                  i++;
              } else {
                if (*itr == count)
                  *itr = 1;
                else
                  *itr++;
              }
              break;
            case 2:
              if (type)
                *type_state = !(*type_state);
              else {
                Devices_schedule[i].setEnable(!Devices_schedule[i].getEnable());
                Devices_schedule[i].setEEPROM(i * 4);
              }
              break;
            case 3:
              Clear();
              Edit = Devices_schedule[i];
              display_menu[2] = 1;
              break;
            case 4:
              Devices_schedule[i].Reset();
              if (!type)
                 Devices_schedule[i].setEEPROM(i * 4);
              break;
            case 5:
              if (type)
              {
                *menu = 0;
              }
              break;
            case 6:
              Clear();
              if ( !type )
              {
                i = 0;
                display_menu[0] = 0;
                display_menu[1] = 0;
                display_menu[2] = 0;
              }
              char_created = 255; //сброс номера экрана для генерации символов
              break;
            case 7:
              if ( type )
              {
                *num = 0;
                *itr = 1;
              }
              Clear();
              i = 0;
              display_menu[0] = 0;
              display_menu[1] = 0;
              display_menu[2] = 0;
              char_created = 255; //сброс номера экрана для генерации символов
              break;
          }
        }
      } else if (display_menu[2] == 1) //редактирование будильника - часть 1
      {
        if (key == 0)
        {
          lcd->home();
          lcd->write(0);
          if (!type)
            lcd->print(i + 1);
          else
            lcd->print(*itr);
          lcd->print(colon);
          lcd->setCursor(5, 0);
          lcd->print(AddChar(Edit.getHour(), zero));
          lcd->print(colon);
          lcd->print(AddChar(Edit.getMinute(), zero));
          lcd->setCursor(11, 0);
          lcd->print("REP:");
          if (Edit.getDay(8))
            lcd->write(3);
          else
            lcd->write(4);
          lcd->setCursor(cursor_pos, 1);
          lcd->write(1);
          lcd->write(1);
          if (edit == REPEAT_EN)
            lcd->write(1);

        } else
        {
          switch (key)
          {
            case 3:
              ClearLine(1);
              if (edit == HOURS)
              {
                edit = MINUTES;
                cursor_pos = 8;
              } else if (edit == MINUTES)
              {
                edit = REPEAT_EN;
                cursor_pos = 11;
              } else if (edit == REPEAT_EN)
              {
                if (Edit.getDay(8))
                {
                  edit = HOURS;
                  cursor_pos = 5;
                } else
                {
                  cursor_pos = 1;
                  edit = MON;
                  ClearLine(0);
                  display_menu[2] = 2;
                }
              }
              break;
            case 4:
              switch (edit)
              {
                case HOURS:
                  if (Edit.getHour() == 0)
                    Edit.setHour(23);
                  else
                    Edit.setHour(Edit.getHour() - 1);
                  break;
                case MINUTES:
                  if (Edit.getMinute() == 0)
                    Edit.setMinute(59);
                  else
                    Edit.setMinute(Edit.getMinute() - 1);
                  break;
                case REPEAT_EN:
                  if (Edit.getDay(8))
                  {
                    Edit.setWeek(0);
                  } else {
                    Edit.setWeek(1 << 7);
                  }
                  break;
              }
              break;
            case 5:
              switch (edit)
              {
                case HOURS:
                  if (Edit.getHour() == 23)
                    Edit.setHour(0);
                  else
                    Edit.setHour(Edit.getHour() + 1);
                  break;
                case MINUTES:
                  if (Edit.getMinute() == 59)
                    Edit.setMinute(0);
                  else
                    Edit.setMinute(Edit.getMinute() + 1);
                  break;
                case REPEAT_EN:
                  if (Edit.getDay(8))
                  {
                    Edit.setWeek(0);
                  } else {
                    Edit.setWeek(1 << 7);
                  }
                  break;
              }
              break;
            case 6:
              Devices_schedule[i] = Edit;
              if_saved = 1;
              display_menu[2] = 3;
              Clear();
              break;
            case 7:
              if_saved = 0;
              display_menu[2] = 3;
              Clear();
              break;
          }
        }
      } else if (display_menu[2] == 2) //редактирование будильника - часть 2
      {
        if (key == 0)
        {
          lcd->setCursor(cursor_pos, 0);
          lcd->write(2);
          lcd->setCursor(0, 1);
          lcd->print(space);
          lcd->print('M');
          lcd->setCursor(3, 1);
          lcd->print('T');
          lcd->setCursor(5, 1);
          lcd->print('W');
          lcd->setCursor(7, 1);
          lcd->print('T');
          lcd->setCursor(9, 1);
          lcd->print('F');
          lcd->setCursor(11, 1);
          lcd->print('S');
          lcd->setCursor(13, 1);
          lcd->print('S');
          byte cursor_temporary_position = 2;
          for (byte j = 1; j < 8; j++)
          {
            lcd->setCursor(cursor_temporary_position, 1);
            if (Edit.getDay(j))
              lcd->write(4);
            else
              lcd->write(3);
            cursor_temporary_position += 2;
          }
          lcd->setCursor(15, 1);
          lcd->print(space);
        } else
        {

          switch (key)
          {
            case 3:
              ClearLine(0);
              if (edit == SUN)
              {
                Clear();
                edit = HOURS;
                cursor_pos = 5;
                display_menu[2] = 1;

              } else {
                edit = edit + 1;
                cursor_pos += 2;
              }
              break;
            case 4:
              Edit.setDay(edit, !Edit.getDay(edit));
              break;
            case 5:
              Edit.setDay(edit, !Edit.getDay(edit));
              break;
            case 6:
              Devices_schedule[i] = Edit;
              if_saved = 1;
              display_menu[2] = 3;
              Clear();
              break;
            case 7:
              if_saved = 0;
              display_menu[2] = 3;
              Clear();
              break;
          }
        }
      } else if (display_menu[2] == 3) //редактирование будильника - вывод сообщения об окончании редактирования
      {
        if (!type)
          if (if_saved == 1)  //если изменения сохранены
          {
            lcd->home();  //курсор в начало
            lcd->print("Alarm ");  //вывод соотв. сообщения
            lcd->write(0);
            lcd->print(i + 1);
            lcd->print(" set    ");
            lcd->setCursor(0, 1);
            lcd->print("    successfully");
          }
          else  //иначе - не сохранены
          {
            lcd->home();
            lcd->print("Changes  haven't");  //вывод соотв. сообщения
            lcd->setCursor(5, 1);
            lcd->print("saved");
          }
        //устанавливить задержку дисплея 1200 мс
        if ( type || MsgTimer(1750) )  //если флаг выхода активен
        {
          //сбросить флаги
          byte b = Devices_schedule[i].getWeek();
          if (!b & ~(1 << 7))
            Devices_schedule[i].setDay(8, 1);
          if (if_saved)
            Devices_schedule[i].setEnable(1);
          if (!type)
                 Devices_schedule[i].setEEPROM(i * 4);
          if_saved = 0;
          display_timer_escape = 0;
          cursor_pos = 5; //курсор на позицию 0
          edit = HOURS; //редактирование:часы
          if (!type)
            display_menu[0] = 1;
          else
            display_menu[0] = 5;
          display_menu[1] = 0;
          display_menu[2] = 0;
          Clear();
        }
      }
    }

    void meteoSpace(byte &key)
    {
      if (char_created != display_menu[0])  //если символы для экрана не созданы
      { //создать
        lcd->createChar(0, HumidityIns);  //колокольчик
        lcd->createChar(1, HumidityOut); //незакрашенная капелька
        lcd->createChar(2, ThermometerIns);  //незакрашенный термометр
        lcd->createChar(3, ThermometerOut);  //закрашенный термометр
        lcd->createChar(4, CelsiumDegree); //градусы по Цельсию
        char_created = display_menu[0]; //символы созданы для экрана 1
      }
      if (display_menu[1] == 0) //общая метеоинформация
      {
        switch (key)
        {
          case 0:
            switch (*weather_forecast) //см. пред. экран
            {
              case HOT:
                lcd->createChar(5, Hot);
                break;
              case SUNNY:
                lcd->createChar(5, Sunny);
                break;
              case NORMAL:
                lcd->createChar(5, Normal);
                break;
              case RAINY:
                lcd->createChar(5, Rainy);
                break;
              case STORM:
                lcd->createChar(5, Storm);
                break;
            }
            lcd->home(); //курсор в начало
            lcd->write(0); //вывести символ: капля
            lcd->print(AddChar(*humidity_inside, zero));  //вывести влажность в помещении
            lcd->print(pct);
            lcd->print(space);
            lcd->print('P');
            lcd->print(colon);
            lcd->print(*pressure);
            lcd->setCursor(10, 0);
            lcd->print("mmHG");
            lcd->print(space);
            lcd->write(5);
            lcd->setCursor(0, 1);
            lcd->write(1);
            lcd->print(AddChar(*humidity_outside, zero));
            lcd->print(pct);
            lcd->print(space);
            lcd->write(2);
            if (*temperature_inside > 0)  //если температура в помещении больше 0
              lcd->print('+'); //вывести '+'
            lcd->print(*temperature_inside); //вывести температуру в помещении
            lcd->write(4); //вывести символ: градусы по Цельсию
            if (abs(*temperature_inside) < 10)  //если температура в помещении по модулю меньше 10
            {
              lcd->print(space); //вывести пробел
            }
            lcd->print(space);
            lcd->write(3);
            if (*temperature_outside > 0)  //если температура на улице больше 0
              lcd->print('+'); //вывести '+'
            lcd->print(*temperature_outside); //вывести температуру на улице
            lcd->write(4); //вывести символ: градусы по Цельсию
            if (abs(*temperature_outside) < 10)  //если температура на улице по модулю меньше 10
            {
              lcd->print(space); //вывести пробел
              if (*temperature_outside == 0) //если температура на улице равна 0
                lcd->print(space); //вывести пробел
            }
            break;
          case 1:
            Clear();
            display_menu[1] = 1;
            break;
          case 2:
            Clear();
            display_menu[1] = 2;
            break;
          case 3:
            Clear();
            display_menu[1] = 3;
            break;
          case 6: case 7:
            Clear();
            display_menu[0] = 0;
            display_menu[1] = 0;
            display_menu[2] = 0;
            break;
        }
      } else if (display_menu[1] == 1) //сведения об атмосферном давлении
      {
        switch (key)
        {
          case 0:
            lcd->home();
            lcd->print("Pressure ");
            lcd->print(*pressure);
            lcd->setCursor(12, 0);
            lcd->print("mmHG");
            lcd->setCursor(0, 1);
            lcd->print("Tend:");
            if (*pressure_change > 0)
              lcd->print('+');
            lcd->print(*pressure_change);
            lcd->print("Pa/3h");
            if ( !(*forecast_flag) || forecast_flag == NULL )
              lcd->createChar(5, Cross);
            else
              lcd->createChar(5, Check);
            lcd->setCursor(15, 1);
            lcd->write(5);
            break;
          case 2:
            Clear();
            display_menu[1] = 2;
            break;
          case 6:
            Clear();
            display_menu[1] = 0;
            break;
          case 7:
            Clear();
            display_menu[0] = 0;
            display_menu[1] = 0;
            display_menu[2] = 0;
            break;
        }
      } else if (display_menu[1] == 2) //сведения о влажности воздуха
      {
        switch (key)
        {
          case 0:
            lcd->home();
            lcd->write(0);
            lcd->print("Ins:");
            lcd->print(AddChar(*humidity_inside, zero));
            lcd->print(pct);
            lcd->print(space);
            lcd->print("Out:");
            lcd->print(AddChar(*humidity_outside, zero));
            lcd->print(pct);
            lcd->setCursor(0, 1);
            if (*humidity_inside >= 40 && *humidity_inside <= 60)
              lcd->print("NORMAL HUMIDITY ");
            else if (*humidity_inside < 40 && *humidity_inside < *humidity_outside || *humidity_inside > 60 && *humidity_inside > *humidity_outside)
              lcd->print("OPEN THE WINDOW ");
            else if (*humidity_inside < 40 && *humidity_inside > *humidity_outside || *humidity_inside > 60 && *humidity_inside < *humidity_outside)
              lcd->print("CLOSE THE WINDOW ");
            else if (*humidity_inside == *humidity_outside)
              if (*humidity_inside < 40)
                lcd->print("LOW HUMIDITY    ");
              else if (*humidity_inside > 60)
                lcd->print("HIGH HUMIDITY   ");
            break;
          case 1:
            Clear();
            display_menu[1] = 1;
            break;
          case 6:
            Clear();
            display_menu[1] = 0;
            break;
          case 7:
            Clear();
            display_menu[0] = 0;
            display_menu[1] = 0;
            display_menu[2] = 0;
            break;
        }
      } 
    }

    void clcEditSpace(byte &key)
    {
      static bool read_time_flag = 0; //флаг чтения времени
      static Time readed_time = *real_time;  //объект для временного хранения текущего времени
      static byte cursor_pos = 0; //позиция курсора стрелок
      static bool if_saved = 0; //сохранены изменения? 1 - да, 0 - нет
      enum ClockField { //перечесление полей редактирования времени
        HOURS = 0,  //часы
        MINUTES = 1,  //минуты
        DAY = 2,  //день недели
        YEAR = 3, //год
        MONTH = 4, //месяц
        DATE = 5  //день месяца
      };
      static ClockField edit = HOURS; //начало редактирования: часы

      if (!read_time_flag) //если readed_time не инициализирован
      {
        readed_time = *real_time;  //заполняем его текущим временем
        read_time_flag = 1; //флаг чтения времени: прочитано
      }

      if (char_created != display_menu[0])  //если символы для экрана не созданы
      { //создать
        lcd->createChar(0, UpArrow);  //стрелка вверх
        char_created = display_menu[0]; //символы созданы для экрана 3
      }

      if (display_menu[1] == 0) //редактирование часов - 1 часть
      {
        switch (key)
        {
          case 0:
            lcd->home(); //курсор в начало дисплея
            lcd->print(AddChar(readed_time.hr, zero)); //вывести время::часы
            lcd->print(colon); //вывести двоеточие
            lcd->print(AddChar(readed_time.min, zero));  //вывести время::минуты
            lcd->print(space); //вывести пробел
            lcd->print(space); //вывести пробел
            //вывести день недели:
            lcd->print((char)week_day[readed_time.day - 1][0]);  //1 символ дня недели
            lcd->print((char)week_day[readed_time.day - 1][1]);  //2 символ дня недели
            lcd->print((char)week_day[readed_time.day - 1][2]);  //3 символ дня недели
            lcd->print(space); //вывести пробел
            lcd->print(space); //вывести пробел
            lcd->print(2000 + readed_time.yr);  //вывести год
            lcd->setCursor(cursor_pos, 1); //курсор на cursor_pos строки 1
            lcd->write(0); //вывести символ: стрелка вверх
            lcd->write(0); //вывести символ: стрелка вверх
            if (edit == DAY)  //если редактируем день недели
              lcd->write(0); //вывести символ: стрелка вверх
            if (edit == YEAR) //если редактируем год
            {
              lcd->write(0); //вывести символ: стрелка вверх
              lcd->write(0); //вывести символ: стрелка вверх
            }
            break;
          case 3: //нажата кнопка 3 - переключение поля редактирования
            ClearLine(1);  //очистить первую строку
            if (edit == HOURS)  //если редактирование часов
            {
              cursor_pos = 3; //курсор на позицию 3
              edit = MINUTES; //переключить редакирование на минуты
            }
            else if (edit == MINUTES) //если редактирование часов
            {
              cursor_pos = 7; //курсор на позицию 7
              edit = DAY; //переключить редакирование на день недели
            }
            else if (edit == DAY) //если редактирование дня недели
            {
              cursor_pos = 12;  //курсор на позицию 7
              edit = YEAR;  //переключить редакирование на год
            }
            else if (edit == YEAR)  //если редактирование года
            {
              ClearLine(0);  //очистить строку 0 дисплея
              cursor_pos = 6; //курсор на позицию 6
              edit = MONTH; //переключить редакирование на месяцы
              if (readed_time.yr % 4 != 0 && readed_time.mon == 2 && readed_time.date == 29)
                readed_time.date--;
              display_menu[1] = 1;  //переключение на [3][1][0] - второй экран редактирования времени
            }
            break;
          case 4: //нажата кнопка 4 - уменьшения значения в поле редактирования
            switch (edit) ///переключатель: редактирование
            {
              case HOURS: //редактирование часов
                if (readed_time.hr == 0)  //если время::часы == 0
                  readed_time.hr = 23;  //время::часы присвоить 23
                else //иначе
                  readed_time.hr--; //декремент время::часы
                break;
              //далее аналогично
              case MINUTES:
                if (readed_time.min == 0)
                  readed_time.min = 59;
                else
                  readed_time.min--;
                break;
              case DAY:
                if (readed_time.day == 1)
                  readed_time.day = 7;
                else
                  readed_time.day--;
                break;
              case YEAR:
                if (readed_time.yr == 0)
                  readed_time.yr = 99;
                else
                  readed_time.yr--;
                break;
            }
            break;
          case 5: //нажата кнопка 5 - увеличение значения в поле редактирования (аналогично кнопке 4)
            switch (edit)
            {
              case HOURS:
                if (readed_time.hr == 23)
                  readed_time.hr = 0;
                else
                  readed_time.hr++;
                break;
              case MINUTES:
                if (readed_time.min == 59)
                  readed_time.min = 0;
                else
                  readed_time.min++;
                break;
              case DAY:
                if (readed_time.day == 7)
                  readed_time.day = 1;
                else
                  readed_time.day++;
                break;
              case YEAR:
                if (readed_time.yr == 99)
                  readed_time.yr = 0;
                else
                  readed_time.yr++;
                break;
            }
            break;
          case 6: //нажата кнопка 6 - выйти и сохранить
            rtc->time(readed_time);  //записать время в память микросхемы DS1302
            if_saved = 1; //изменения сохранены
            Clear();  //очистка дисплея
            //вернуться к экрану [0][0][0]
            display_menu[1] = 3;  //переключение на экран [3][3][0] - вывод сообщения пользователю
            break;
          case 7: //нажата кнопка 7 - выйти без сохранения
            //аналогична кнопке 6, кроме трех первых операторов
            if_saved = 0; //изменения не сохранены
            Clear();
            display_menu[1] = 3;
            break;
        } //конец переключателя
      } else if (display_menu[1] == 1) //редактирование часов - 2 часть
      {
        switch (key)
        {
          case 0:
            lcd->home(); //курсор в начало
            lcd->print("Month:");
            lcd->print(AddChar(readed_time.mon, zero)); //вывести время::месяц
            lcd->print(space); //вывести пробел
            lcd->print(" Day:");
            lcd->print(AddChar(readed_time.date, zero));  //вывести время::число_месяца
            lcd->setCursor(cursor_pos, 1);
            lcd->write(0); //вывести символ: стрелка вверх
            lcd->write(0); //вывести символ: стрелка вверх
            break;
          case 3: //нажата кнопка 3 - переключение поля редактирования
            ClearLine(1); //очистить строку 1 дисплея
            if (edit == MONTH)  //если редактирование месяца
            {
              cursor_pos = 14;  //курсор на позицию 14
              edit = DATE;  //переключить редактирование на день месяца
            }
            else if (edit == DATE)  //если редактирование дня месяца
            {
              ClearLine(0);  //очистить строку 0 дисплея
              edit = HOURS; //переключить режим редактирования на часы
              cursor_pos = 0; //курсор на позицию 0
              display_menu[1] = 0;  //переключение на [3][1][0] - первый экран редактирования времени
            }
            break;
          case 4: //нажата кнопка 4 - уменьшение значения в поле редактирования
            {
              switch (edit) //переключатель: редактирование
              {
                case MONTH: //редактирование месяца
                  if (readed_time.mon == 1) //если время::месяц == 1 (январь)
                    readed_time.mon = 12; //время::месяц = 12 (декабрь)
                  else  //иначе
                    readed_time.mon--;  //декремент месяца
                  break;
                case DATE:  //редактирование дня месяца
                  //byte a - максимальное количество дней в месяце с учетом високосного года
                  byte  a = 30 * ((readed_time.mon + 1 * (readed_time.mon > 7))  % 2 == 0) * (readed_time.mon != 2); //30
                  a += 31 * ((readed_time.mon + 1 * (readed_time.mon > 7)) % 2 == 1) * (readed_time.mon != 2);  //31
                  a += (28 + 1 * (readed_time.yr % 4 == 0)) * (readed_time.mon == 2); //28 или 29
                  //далее аналогично редактированию месяца
                  if (readed_time.date == 1)
                    readed_time.date = a;
                  else
                    readed_time.date--;
                  break;
              }
            }
            break;
          case 5: //нажата кнопка 5 - увеличение значения в поле редактирования
            switch (edit)
            {
              //далее аналогично кнопке 4 с заменой условия и декремента на инкремент
              case MONTH:
                if (readed_time.mon == 12)
                  readed_time.mon = 1;
                else
                  readed_time.mon++;
                break;
              case DATE:
                byte a = 30 * ((readed_time.mon + 1 * (readed_time.mon > 7)) % 2 == 0) * (readed_time.mon != 2);
                a += 31 * ((readed_time.mon + 1 * (readed_time.mon > 7)) % 2 == 1) * (readed_time.mon != 2);
                a += (readed_time.mon == 2) * (28 + 1 * (readed_time.yr % 4 == 0));
                if (readed_time.date == a)
                  readed_time.date = 1;
                else
                  readed_time.date++;
                break;
            }
            break;
          case 6: //нажата кнопка 6 - выйти и сохранить
            rtc->time(readed_time);  //записать время в память микросхемы DS1302
            if_saved = 1; //изменения сохранены
            Clear();  //очистка дисплея
            //вернуться к экрану [0][0][0]
            display_menu[1] = 3;  //переключение на экран [3][3][0] - вывод сообщения пользователю
            break;
          case 7: //нажата кнопка 7 - выйти без сохранения
            //аналогична кнопке 6, кроме трех первых операторов
            if_saved = 0; //изменения не сохранены
            Clear();
            display_menu[1] = 3;
            break;
        } //конец переключателя
      } else if (display_menu[1] == 3) //меню вывода сообщения пользователю
      {
        lcd->home();  //курсор в начало
        if (if_saved == 1)  //если изменения сохранены
        {
          lcd->print("Time set      ");  //вывод соотв. сообщения
          lcd->setCursor(0, 1);
          lcd->print("    successfully");
        }
        else  //иначе - не сохранены
        {
          lcd->print("Changes  haven't");  //вывод соотв. сообщения
          lcd->setCursor(5, 1);
          lcd->print("saved");
        }
        //устанавливить задержку дисплея 1200 мс
        if (MsgTimer(1500))  //если флаг выхода активен
        {
          //сбросить флаги
          if_saved = 0;
          read_time_flag = 0;
          display_timer_escape = 0;
          cursor_pos = 0; //курсор на позицию 0
          edit = HOURS; //редактирование:часы
          char_created = 255; //сброс номера экрана для генерации символов
          //переключение на экран [0][0][0] - главный
          display_menu[0] = 0;
          display_menu[1] = 0;
          display_menu[2] = 0;
        }
      }
    }

    void bluetoothMessageSpace()
    {
      lcd->home();
      lcd->print("Bluetooth:      ");
      lcd->setCursor(0, 1);
      if (digitalRead(BLUETOOTH_ENABLE_PIN))
        lcd->print("          Enable");
      else
        lcd->print("         Disable");
      if (MsgTimer(1500))  //если флаг выхода активен
      {
        display_timer_escape = 0;
        //переключение на экран [0][0][0] - главный
        display_menu[0] = 0;
        display_menu[1] = 0;
        display_menu[2] = 0;
      }
    }

    void remoteSpaceManual(byte &key)
    {
      if (display_menu[2] == 0)
      {
        switch (key)
        {
          case 0:
            lcd->home();
            lcd->print(sharp);
            lcd->print(act_device.num);
            lcd->print(colon);
            switch (act_device.enable_flag)
            {
              case 1:
                lcd->print("  Enable     ");
                break;
              case 0:
                lcd->print("  Disable    ");
            }
            lcd->setCursor(0, 1);
            lcd->print("  Manual  mode  ");
            break;
          case 1:
            Clear();
            if (act_request_num == act_device.count)
              act_request_num = 1;
            else
              act_request_num++;
            break;
          case 2:
            act_device.enable_flag = !act_device.enable_flag;
            break;
          case 3:
            Clear();
            display_menu[2] = 1;
            break;
          case 7:
            Clear();
            display_menu[0] = 0;
            display_menu[1] = 0;
            display_menu[2] = 0;
            act_device.num = 0;
            break;
        }
      } else if (display_menu[2] == 1)
      {
        static byte cursor_pos = 0;
        switch (key)
        {
          case 0:
            lcd->home();
            lcd->print("MANUAL  ");
            lcd->write(2);
            lcd->print(space);
            lcd->write(3);
            lcd->print(space);
            lcd->write(4);
            lcd->print(space);
            lcd->write(6);
            lcd->print(space);
            lcd->setCursor(cursor_pos, 1);
            if (cursor_pos == 0)
              for (byte j = 0; j < 6; j++)
                lcd->write(1);
            else
              lcd->write(1);
            break;
          case 3:
            ClearLine(1);
            if (cursor_pos == 0)
              cursor_pos = 8;
            else if (cursor_pos == 14)
              cursor_pos = 0;
            else
              cursor_pos += 2;
            break;
          case 4: case 5: case 6:
            Clear();
            if (cursor_pos != 0)
              if ((cursor_pos - 8) / 2 + 1 == 4)
              {
                //Devices_sched[i].setEnable(1);
                act_device.mode = 4;
              } else {
                act_device.mode = (cursor_pos - 8) / 2 + 1;
              }
            else
              act_device.mode = 0;
            display_menu[2] = 0;
            cursor_pos = 0;
            break;
          case 7:
          
            cursor_pos = 0;
            act_device.num = 0;
            Clear();
            display_menu[0] = 0;
            display_menu[1] = 0;
            display_menu[2] = 0;
            break;
        }
      }
    }

    void remoteSpaceTemperature(byte &key)
    {
      static bool sensor_type = 0;
      static bool sensor_mode = 0;
      static int temperature = 0;
      if (display_menu[2] == 0)
        switch (key)
        {
          case 0:
            lcd->home();
            lcd->print(sharp);
            lcd->print(act_device.num);
            lcd->print(colon);
            lcd->setCursor(5, 0);
            if (act_device.enable_flag)
              lcd->print("Enable ");
            else
              lcd->print("Disable");
            lcd->setCursor(0, 1);
            if (act_device.t_sensor)
              lcd->print("Outside:");
            else
              lcd->print("Inside: ");
            lcd->setCursor(9, 1);
            lcd->write(2);
            if (act_device.temperature > 0)
              lcd->print('+');
            if (act_device.temperature != 0)
              lcd->print(AddChar(act_device.temperature, zero));
            else
              lcd->print(0);
            lcd->write(5);
            lcd->print(space);
            lcd->setCursor(15, 1);
            lcd->write(act_device.t_mode);
            break;
          case 1:
            Clear();
            if (act_request_num == act_device.count)
              act_request_num = 1;
            else
              act_request_num++;
            break;
          case 3:
            Clear();
            display_menu[2] = 1;
            temperature = act_device.temperature;
            sensor_type = act_device.t_sensor;
            sensor_mode = act_device.t_mode;
            break;
          case 4:
            Clear();
            act_device.temperature = 25;
            act_device.t_sensor = 0;
            act_device.t_mode = 0;
            act_device.mode = 0;
            break;
          case 5:
            Clear();
            act_device.mode = 0;
            break;
          case 7:
            Clear();
            act_device.num = 0;
            display_menu[0] = 0;
            display_menu[1] = 0;
            display_menu[2] = 0;
            break;
        }
      else if (display_menu[2] == 1)
        switch (key)
        {
          case 0:
            lcd->setCursor(0, 0);
            lcd->write(2);
            lcd->print(act_device.num);
            lcd->print(colon);
            lcd->setCursor(6, 0);
            lcd->print("Edit");
            lcd->setCursor(2, 1);
            if (sensor_type)
              lcd->print("Outside");
            else
              lcd->print("Inside ");
            lcd->setCursor(10, 1);
            if (temperature > 0)
              lcd->print('+');
            if (temperature != 0)
              lcd->print(AddChar(temperature, zero));
            else
              lcd->print(0);
            lcd->write(5);
            lcd->setCursor(15, 1);
            lcd->write(sensor_mode);
            break;
          case 2:
            sensor_type = !sensor_type;
            break;
          case 3:
            sensor_mode = !sensor_mode;
            break;
          case 4:
            if (temperature != -35)
              temperature--;
            break;
          case 5:
            if (temperature != 35)
              temperature++;
            break;
          case 6:
            act_device.temperature = temperature;
            act_device.t_mode = sensor_mode;
            act_device.t_sensor = sensor_type;
            Clear();
            display_menu[2] = 0;
            break;
          case 7:
            Clear();
            display_menu[2] = 0;
            break;
        }
    }

    void remoteSpaceHumidity(byte &key)
    {
      static bool sensor_type = 0;
      static bool sensor_mode = 0;
      static byte humidity = 0;
      if (display_menu[2] == 0)
        switch ( key )
        {
          case 0:
            lcd->home();
            lcd->print(sharp);
            lcd->print(act_device.num);
            lcd->print(colon);
            lcd->setCursor(5, 0);
            if ( act_device.enable_flag )
              lcd->print("Enable ");
            else
              lcd->print("Disable");
            lcd->setCursor(0, 1);
            if ( act_device.h_sensor )
              lcd->print("Outside:");
            else
              lcd->print("Inside: ");
            lcd->setCursor(9, 1);
            lcd->write(3);
            //if (act_device.humidity != 0)
            lcd->print(AddChar(act_device.humidity, zero));
            //else
            //  lcd->print(0);
            lcd->print(pct);
            lcd->print(space);
            lcd->setCursor(15, 1);
            lcd->write(act_device.h_mode);
            break;
          case 1:
            Clear();
            if (act_request_num == act_device.count)
              act_request_num = 1;
            else
              act_request_num++;
            break;
          case 3:
            Clear();
            display_menu[2] = 1;
            humidity = act_device.humidity;
            sensor_type = act_device.h_sensor;
            sensor_mode = act_device.h_mode;
            break;
          case 4:
            Clear();
            act_device.humidity = 50;
            act_device.h_mode = 0;
            act_device.h_sensor = 0;
            act_device.mode = 0;
            break;
          case 5:
            Clear();
            act_device.mode = 0;
            break;
          case 7:
            Clear();
            act_device.num = 0;
            display_menu[0] = 0;
            display_menu[1] = 0;
            display_menu[2] = 0;
            break;
        }
      else if (display_menu[2] == 1)
        switch (key)
        {
          case 0:
            lcd->setCursor(0, 0);
            lcd->write(3);
            lcd->print(act_device.num);
            lcd->print(colon);
            lcd->setCursor(6, 0);
            lcd->print("Edit");
            lcd->setCursor(2, 1);
            if (sensor_type)
              lcd->print("Outside");
            else
              lcd->print("Inside ");
            lcd->setCursor(10, 1);
            if (humidity != 0)
              lcd->print(AddChar(humidity, zero));
            else
              lcd->print(0);
            lcd->print(pct);
            lcd->setCursor(15, 1);
            lcd->write(sensor_mode);
            break;
          case 2:

            sensor_type = !sensor_type;
            break;
          case 3:
            sensor_mode = !sensor_mode;
            break;
          case 4:
            if (humidity != 10)
              humidity--;
            break;
          case 5:
            if (humidity != 90)
              humidity++;
            break;
          case 6:
            act_device.humidity = humidity;
            act_device.h_mode = sensor_mode;
            act_device.h_sensor = sensor_type;
            Clear();
            display_menu[2] = 0;
            break;
          case 7:
            Clear();
            display_menu[2] = 0;
            break;
        }
    }

    void remoteSpaceIllumination(byte &key)
    {
      static bool sensor_type = 0;
      static bool sensor_mode = 0;
      static byte illumination = 0;
      if (display_menu[2] == 0)
        switch (key)
        {
          case 0:
            lcd->home();
            lcd->print(sharp);
            lcd->print(act_device.num);
            lcd->print(colon);
            lcd->setCursor(5, 0);
            if ( act_device.enable_flag )
              lcd->print("Enable ");
            else
              lcd->print("Disable");
            lcd->setCursor(0, 1);
            if ( act_device.i_sensor )
              lcd->print("Outside:");
            else
              lcd->print("Inside: ");
            lcd->setCursor(9, 1);
            lcd->write(4);
            if ( act_device.illumination != 0 )
              lcd->print(AddChar(act_device.illumination, zero));
            else
              lcd->print(0);
            lcd->print(pct);
            lcd->print(space);
            lcd->setCursor(15, 1);
            lcd->write(act_device.i_mode);
            break;
          case 1:
            Clear();
            if ( act_request_num == act_device.count )
              act_request_num = 1;
            else
              act_request_num++;
            break;
          case 3:
            Clear();
            display_menu[2] = 1;
            illumination = act_device.illumination;
            sensor_type = act_device.i_sensor;
            sensor_mode = act_device.i_mode;
            break;
          case 4:
            Clear();
            act_device.illumination = 50;
            act_device.i_sensor = 0;
            act_device.i_mode = 0;
            act_device.mode = 0;
            break;
          case 5:
            Clear();
            act_device.mode = 0;
            break;
          case 7:
            Clear();
            act_device.num = 0;
            display_menu[0] = 0;
            display_menu[1] = 0;
            display_menu[2] = 0;
            break;
        }
      else if (display_menu[2] == 1)
        switch (key)
        {
          case 0:
            lcd->setCursor(0, 0);
            lcd->write(4);
            lcd->print(act_device.num);
            lcd->print(colon);
            lcd->setCursor(6, 0);
            lcd->print("Edit");
            lcd->setCursor(2, 1);
            if (sensor_type)
              lcd->print("Outside");
            else
              lcd->print("Inside ");
            lcd->setCursor(10, 1);
            if (illumination != 0)
              lcd->print(AddChar(illumination, zero));
            else
              lcd->print(0);
            lcd->print(pct);
            lcd->setCursor(15, 1);
            lcd->write(sensor_mode);
            break;
          case 2:
            sensor_type = !sensor_type;
            break;
          case 3:
            sensor_mode = !sensor_mode;
            break;
          case 4:
            if (illumination != 10)
              illumination--;
            break;
          case 5:
            if (illumination != 90)
              illumination++;
            break;
          case 6:
            act_device.illumination = illumination;
            act_device.i_mode = sensor_mode;
            act_device.i_sensor = sensor_type;
            Clear();
            display_menu[2] = 0;
            break;
          case 7:
            Clear();
            display_menu[2] = 0;
            break;
        }
    }

    void remoteSpace(byte &key)
    {
      if (char_created != display_menu[0] && display_menu[1] != 4)  //если символы для экрана не созданы
      { //создать
        lcd->createChar(0, DownArrow);  //стрелка вниз
        lcd->createChar(1, UpArrow); //стрелка вверх
        lcd->createChar(2, ThermometerIns);  //незакрашенный термометр
        lcd->createChar(3, HumidityOut);  //закрашенная капелька
        lcd->createChar(4, Sunny);  //солнце
        lcd->createChar(5, CelsiumDegree); //градусы по Цельсию
        lcd->createChar(6, Clock);  //часы
        char_created = 5; //символы созданы для экрана 5
      }

      display_menu[1] = act_device.mode;
      switch (display_menu[1])
      {
        case 0:
          remoteSpaceManual(key);
          break;
        case 1:
          remoteSpaceTemperature(key);
          break;
        case 2:
          remoteSpaceHumidity(key);
          break;
        case 3:
          remoteSpaceIllumination(key);
          break;
        case 4:
          schedSpace(key, &act_device_sched, act_device.count, 1, &act_device.enable_flag, &act_device.mode, &act_request_num, &act_device.num);
          break;
      }
    }

    void signalisationSpace()
    {
      lcd->home();
      lcd->print("Signalisation:  ");
      lcd->setCursor(0, 1);
      
      if (*signalisation)
        lcd->print("          Enable");
      else
        lcd->print("         Disable");
        
      if (MsgTimer(1500))  //если флаг выхода активен
      {
        display_timer_escape = 0;
        //переключение на экран [0][0][0] - главный
        display_menu[0] = 0;
        display_menu[1] = 0;
        display_menu[2] = 0;
      }
    }

    void nightModeMessageSpace()
    {
      lcd->home();
      lcd->print("Night mode:        ");
      lcd->setCursor(0, 1);
      if (*night_mode)
        lcd->print("          Enable");
      else
        lcd->print("         Disable");
      if (MsgTimer(1500))  //если флаг выхода активен
      {
        display_timer_escape = 0;
        //переключение на экран [0][0][0] - главный
        display_menu[0] = 0;
        display_menu[1] = 0;
        display_menu[2] = 0;
      }
    }

  public:
    Display1602_Menu (LiquidCrystal_I2C *_lcd, DS1302 *_rtc,
                      Time_Correspondence *_Alarms, byte _alarms_count)
    {
      lcd = _lcd;
      rtc = _rtc;
      Alarms = _Alarms;
      alarms_count = _alarms_count;
    }

    void ISR_DisplayTimer() //обработчик задержки вывода сообщения на дисплей
    { //ВЫЗЫВАТЬ В ОБРАБОТЧИКЕ ПРЕРЫВАНИЙ (интервал - 1 мс)
      if (display_timer_flag)
      {
        display_timer++;
      }
    }


    void Switcher(byte key)
    {
      switch (display_menu[0])
      {
        case 0:
          workSpace(key);
          break;
        case 1:
          schedSpace(key, Alarms, alarms_count, 0, NULL, NULL, NULL, NULL);
          break;
        case 2:
          if ( *connection_flag )
          {
            meteoSpace(key);
          } else {
            display_menu[0] = 0;
            display_menu[1] = 0;
            display_menu[2] = 0;
          }
          break;
        case 3:
          clcEditSpace(key);
          break;
        case 4:
          bluetoothMessageSpace();
          break;
        case 5:
          if ( *connection_flag )
          {
            remoteSpace(key);
          } else {
            display_menu[0] = 0;
            display_menu[1] = 0;
            display_menu[2] = 0;
          } 
          break;
        case 6:
          signalisationSpace();
          break;
        case 7:
          nightModeMessageSpace();
          break;
      }
    }

    byte isActData(byte key)
    {
      /*Serial.print(display_menu[0]);
        Serial.print(" - ");
        Serial.print(display_menu[1]);
        Serial.print(" - ");
        Serial.print(display_menu[2]);
        Serial.print(" - ");
        Serial.print(key);
        Serial.print(" - ");*/
      if ( display_menu[0] == 0 && key == 5 ) //если при нажатии кнопки будет переход к дисплею упр. актуатор.
      {
        //Serial.println("Act request 1");
        return 1; //запрос данных об актуаторе
      }
      else if ( display_menu[0] == 5 )  //если дисплей упр. акт.
      {
        if ( display_menu[2] == 0 && key == 1 ) //если не режим редактирования и нажата кнопка 1
        {
          //Serial.println("Act request 2");
          return 1; //запрос данных об актуаторе
        }
        else if ( display_menu[2] == 0 && key == 6  )  //если не режим редактирования и будет произведено сохранение данных
        {
          return 2; //готовность к отправке данных об актуаторе
        } else {
          //Serial.println("N/O");
          return 0; //вернуть 0
        }
      }
      else //иначе
      {
        //Serial.println("N/O");
        return 0; //вернуть 0
      }
    }

    void actDeviceUpdate(Data_panel_act& _act_device)
    {
      act_device = _act_device;
      act_request_num = act_device.num;
      act_device_sched.setHour(act_device.hour);
      act_device_sched.setMinute(act_device.minute);
      act_device_sched.setWeek(act_device.repeat_days);
      
      //Serial.print("ACT >> SETTINGS: ");
      //Serial.println(act_device.num);
      //Serial.print("H ");
      //Serial.println(act_device_sched.getHour());
      //Serial.print("M ");
      //Serial.println(act_device_sched.getMinute());
      //Serial.print("W ");
      //Serial.println(act_device_sched.getWeek(), BIN);
    }

    Data_panel_act actDeviceGet()
    {

      act_device.hour = act_device_sched.getHour();
      act_device.minute = act_device_sched.getMinute();
      act_device.repeat_days = act_device_sched.getWeek();
      //Serial.print("ACT << SETTINGS: ");
      //Serial.println(act_device.num);
      //Serial.print("H ");
      //Serial.println(act_device.hour);
      //Serial.print("M ");
      //Serial.println(act_device.minute);
      //Serial.print("W ");
      //Serial.println(act_device.repeat_days, BIN);
      return act_device;
    }

    byte getActNumber()
    {
      if (act_request_num == act_device.count || display_menu[0] == 0)
        act_request_num = 0;
      if ( act_request_num - act_device.num > 1 )
      {
        act_request_num = act_device.num;
        Serial.println("Breaking num");
      }
        
      return act_request_num + 1;
    }
    
    void begin(int* temp_ins,  byte* hum_ins,
               int* temp_outs, byte* hum_outs,
               byte* illum_outs, byte* illum_ins,
               int* _pressure, int* _pressure_change,
               Forecast* forecast, bool* _forecast_flag,
               bool* connect_flag, bool* alarm_flag,
               bool* _night_mode, Time* _real_time,
               bool* _signalisation)
    {
      humidity_inside       = hum_ins;
      humidity_outside      = hum_outs;
      temperature_inside    = temp_ins;
      temperature_outside   = temp_outs;
      pressure              = _pressure;
      pressure_change       = _pressure_change;
      forecast_flag         = _forecast_flag;
      weather_forecast      = forecast;
      alarm_enable_flag     = alarm_flag;
      connection_flag       = connect_flag;
      night_mode            = _night_mode;
      signalisation         = _signalisation;
      real_time             = _real_time;
    }

};

/*-----------------------------------------------*/
/*БЛОК ГЛОБАЛЬНЫХ ПЕРЕМЕННЫХ*/

//переменные для работы с сенсором DHTxx
volatile unsigned int   DHT_update_counter    = 0; //счетчик времени для опроса сенсора DHTxx

volatile unsigned int   _radio_free_counter  = 0;
volatile unsigned int   _radio_request_counter = 0;
volatile byte           act_request = 0;

//переменные для хранения метеопоказаний
int       temp_outs = 0; //температура вне помещения
byte      hum_outs = 0; //относительная влажность вне помещения
byte      illum_outs = 0; //уровень освещенности вне помещения
int       pressure = 0; //атмосферное давление
int       pressure_change = 0; //изменение атмосферного давления за 3 часа
Forecast  weather_forecast = 2; //переменная для хранения прогноза погоды
bool      forecast_flag = 0;

//флаги
bool      night_mode = 0;  //ночной режим (отключение LED индикации и звуков)
bool      connect_flag = 0;
bool      alarm_flag = 0;
bool      signalisation = 0;
//значения, полученные с АЦП при нажатии кнопок
//+ дополнительное значение, когда кнопка не нажата
int keys_voltage[KEYS_COUNT + 1] = {0, 92, 240, 387, 562, 703, 860, 1023};

const uint64_t pipes[] = { //адреса ("трубы") обмена данными
  0xFFFFFFFFF0LL,  //адрес 0
  0xFFFFFFFFF1LL,  //адрес 1
  0xFFFFFFFFF2LL,  //адрес 2
  0xFFFFFFFFF3LL,  //адрес 3
  0xFFFFFFFFF4LL,  //адрес 4
  0xFFFFFFFFF5LL   //адрес 5
};

/*-----------------------------------------------*/
/*СОЗДАНИЕ ОБЪЕКТОВ*/

Data_meteo received_data; //данные, принимаемые с хаба в станд. реж.
Data_panel_act received_data_act;   //данные, принимаемые с хаба
//при запросе настроек актуатора

Data_from_panel transmit_data;  //данные, отправляемые на хаб в станд. реж.
Data_panel_act  transmit_data_act;  //данные настроей актуатора, передаваемые
//на хаб при завершении редакт.
Time real_time;

LiquidCrystal_I2C lcd(DISP_ADDR, COLS_COUNT, ROW_COUNT);  //создаем объект lcd для ввода/вывода данных на I2C дисплей

SR_74HC595 reg(DATA_REG_PIN, LATCH_REG_PIN, CLOCK_REG_PIN); //создаем объект reg для управления выводами сдвигового регистра 74HC595

Buzzer buzz(BUZZER_PIN);  //создаем объект buzz - зуммер

DHT dht(DHT_PIN, DHT_TYPE); //создаем объект dht - сенсор DHTxx

Analog_Keys keys(KEYS_PIN, KEYS_COUNT, keys_voltage, KEYS_UNBOUNCE_TIME, KEYS_BIT_SHIFT, KEYS_ERROR); //создаем объект keys - аналоговые кнопки

DS1302 rtc(CLCPIN_CE, CLCPIN_IO, CLCPIN_SCLK); //создаем объект rtc - часы реального времени DS1302

RF24 radio(RF_CE_PIN, RF_CSN_PIN);

Time_Correspondence Alarms[ALARMS_COUNT];

Display1602_Menu Display(&lcd, &rtc, Alarms, ALARMS_COUNT);

//IRrecv irrecv(IR_PIN);

//decode_results results;
/*-----------------------------------------------*/
/*ФУНКЦИИ*/

int illuminationCorrectRead(byte pin)
{
  int illumination = 0;
  for (byte i = 0; i < 10; i++)
    illumination += analogRead(pin);
  return illumination / 10;
}

void lcd_Start()  //включает в себя все функции, необходимые для запуска дисплея
{
  lcd.init(); //инициализировать дисплей
  lcd.backlight(); //включить подсветку дисплея

  keys.analogReadProcessBegin(); //будем получать значение с АЦП
  transmit_data.illumination = map(illuminationCorrectRead(LDR_PIN), 0, 1023, 0, 100); //ПОЛУЧЕНИЕ ЗНАЧЕНИИЯ С АЦП!
  keys.analogReadProcessEnd();  //значение с АЦП получено

  analogWrite(DISPLGHTNESS_PIN, map(transmit_data.illumination, 0, 100, 55, 255)); //автонастройка яркости дисплея
}

void bootlogo() //анимация загрузки
{
  lcd.setCursor(0, 0); //установить курсор в начало ряда 0
  lcd.print(F("                 ")); //очищаем дисплей
  lcd.setCursor(0, 1); //установить курсор в начало ряда 0
  lcd.print(F("                 ")); //очищаем дисплей
  
  lcd.setCursor(0, 0); //установить курсор в начало ряда 0
  lcd.print(F("Starting  system")); //вывод приветствия
  lcd.setCursor(0, 1); //установить курсор в начало ряда 1
  lcd.print(F("     v1.0.3    ")); //версия программы*/
  delay(700);  //задержка 1000 мс для чтения сообщения пользователем
  buzz.millisEnable(25); //проверка зуммера
  for (int i = 3; i >= 0; i--)  //светодиодные индикаторы - "анимация"
    {
    reg.Write(i, 1);
    reg.Write(7 - i, 1);
    delay(200);
    }
  delay(300);
  reg.Write();
  
  lcd.setCursor(0, 0); //установить курсор в начало ряда 0
  lcd.print(F("                 ")); //очищаем дисплей
  lcd.setCursor(0, 1); //установить курсор в начало ряда 0
  lcd.print(F("                 ")); //очищаем дисплей
}

void timer1_Setup() //настройки таймера 1
{
  TCCR1B |= (1 << 3); //установка режима таймера: сброс при совпадении
  OCR1A = 0xF9; //запись значения в регистр сравнения (249)
  TIMSK1 |= (1 << OCIE1A);  //разрешить прерывание при совпадении с регистром A (OCR1A)
  TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10); //усановить предделитель на 64
  sei();  //разрешаем прерывания
}

void DHTread() //чтение информации с DHTxx
{
  cli();  //запретить прерывания
  unsigned int DHT_update_counter_mem = DHT_update_counter; //сохранить текущее значение счетчика
  sei();  //разрешить прерывания

  if (DHT_update_counter_mem >= DHT_INTERVAL) //если время счетчика превышает или равен интервалу опроса сенсора
  {
    cli();  //запретить прерывания
    DHT_update_counter = 0; //обнулить счетчик
    sei();  //разрешить прерывания
    transmit_data.humidity = round(dht.readHumidity());  //считать значение влажности и округлить до целых
    transmit_data.temperature = round(dht.readTemperature() + (DHT_TEMPERATURE_ERROR));  //считать значение температуры и округлить до целых
  }
}

/*-----------------------------------------------*/
/*ОБРАБОТЧИКИ ПРЕРЫВАНИЙ*/

ISR(TIMER1_COMPA_vect)  //обработчик прерываний таймера 1 при совпадении с регистром A - вызывается раз в 1 мс
{
  keys.ISR_IDC_KeyValue();
  if (!buzz.ISR_check()) //проверка зуммера buzz - если периодические включения акивны
    DHT_update_counter++; //счетчик не инкременируется => датчик не опрашивается => отсутствуют задержки
  Display.ISR_DisplayTimer();
  _radio_free_counter++;
  
  if ( act_request != 0 )
    _radio_request_counter++;
  else if ( _radio_request_counter !=  0 )
    _radio_request_counter = 0;
}

/*-----------------------------------------------*/

void setup() //код выполняется однркратно при запуске программы
{
  //wdt_enable (WDTO_8S); //настройка сторожевого таймера на 8 секунд
  wdt_disable();
  timer1_Setup(); //настройка таймера
  
  dht.begin();  //инициализация сенсора DHTxx

  Serial.begin(BAUDRATE); //открыть последовательный порт с заданной скоростью
  
  radio.begin();  //запуск модуля радио nRF24l01
  //настройки модуля nRF:
  radio.powerUp();  //включить радиомодуль
  delay(100); //задержка для включения
  radio.setChannel(RADIO_CHANNEL); //канал передачи данных
  radio.setRetries(15, 30); //косличество попыток отправки данных и временной интервал между попытками
  radio.setDataRate(RF24_1MBPS);  //установить скорость передачи данных 1 мбит/с
  radio.setPALevel(RF24_PA_MAX);  //установить максимальный уровень мощности передатчика
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(1);//разрешить приемнику отправлять подтверждение получения
  radio.enableDynamicPayloads();  //включить пакеты динамического размера
  radio.enableAckPayload(); //включить режим передачи пользовательских данных при подтверждении получения
  //radio.enableDynamicAck(); //for ALL pipes? Чтобы можно было вкл\выкл получение ACK?
  
  radio.openReadingPipe(RECEIVE_DATA_PIPE, pipes[READING_PIPE_NUM]); //открыть трубу для приёма данных
  
  radio.openWritingPipe(pipes[WRITING_PIPE_NUM]);  //открыть трубу отправки даных
  radio.startListening(); //слушать эфир
  
  lcd_Start();  //настройки дисплея
  bootlogo(); //бутлого
  
  real_time = rtc.time();  //сохраняем текущее время
  transmit_data.humidity = round(dht.readHumidity());  //считать значение влажности и округлить до целых
  transmit_data.temperature = round(dht.readTemperature() + DHT_TEMPERATURE_ERROR);  //считать значение температуры, прибавить погрешность и округлить до целых
  //rtc.halt(0); //запуск тактового генератора микросхемы часов - помог при сбоях работы микросхемы
  pinMode(BLUETOOTH_ENABLE_PIN, OUTPUT);  //пин управления питанием модуля Bluetooth как цифровой выход
  
  byte addr = 0;
  Alarms[0].getEEPROM(addr);
  Alarms[1].getEEPROM(addr);
  Alarms[2].getEEPROM(addr);
  Alarms[3].getEEPROM(addr);
  night_mode = EEPROM.read(NIGHTMODE_EEPROM_ADDRESS);
  //забиваем данными все указатели в объекте меню дисплея:
  Display.begin( &transmit_data.temperature, &transmit_data.humidity, //температура, влажность внутри помещения
                 &temp_outs, &hum_outs,                               //температура, влажность вне помещения
                 &illum_outs, &transmit_data.illumination,            //освещенность внутри, вне помещения
                 &pressure, &pressure_change,                         //давление, изменение давления за 3 часа
                 &weather_forecast, &forecast_flag,                   //прогноз погоды, флаг актуальности прогноза
                 &connect_flag, &alarm_flag,                          //флаги активности соединения, будильника
                 &night_mode, &real_time,                             //флаг ночного режима, объект с текущем временем
                 &signalisation);                                     //флаг сингнализиции                      
  transmit_data.address = ADDRESS;
  
  
  //irrecv.enableIRIn();
  //wdt_reset();  //сброс сторожевого таймера
}

void loop() //бесконечный цикл
{
  cli();  //запретить прерывания

  static bool free_interval_flag = 0;
  static bool radio_listen_flag = 1;
  
  if ( free_interval_flag && _radio_free_counter >= RF_NOTLISTEN_TIME )
  {
    _radio_free_counter = 0;
    free_interval_flag = 0;
    radio_listen_flag = 1;
    radio.startListening();
  }

  bool data_request_flag = 0;
  
  if ( act_request != 0 && _radio_request_counter > RADIO_REQUEST_INTERVAL )
  {
    _radio_request_counter = 0;
    data_request_flag = 1;
  } else if ( act_request == 0 )
    data_request_flag = 1;

    
  sei();  //разрешить прерывания
  DHTread();  //считать данные с сенсора DHTxx
  /*if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    irrecv.resume(); // Receive the next value
    }*/
  real_time = rtc.time(); //переменная для хранения текущего времени
  
  byte pressed_key = keys.Get(); //переменная для хранения текущего состояния кнопок

  byte serialRead = 255;
  
  while ( Serial.available() > 0 )
  {
    serialRead = Serial.read() - '0';
    Serial.println(serialRead);
  }
  
  if (serialRead == 0) {
    if (!connect_flag)
      Serial.println("No connection!");
    Serial.println("Temperature");
    Serial.print("-Inside:");
    Serial.print(transmit_data.temperature);
    Serial.println("*C");
    if (connect_flag)
    {
      Serial.print("-Outside: ");
      Serial.print(temp_outs);
      Serial.println("*C");
    }
    
    Serial.println("Humidity");
    Serial.print("-Inside:");
    Serial.print(transmit_data.humidity);
    Serial.println("%");
    if (connect_flag)
    {
      Serial.print("-Outside: ");
      Serial.print(hum_outs);
      Serial.println("%");
      Serial.print("Pressure: ");
      Serial.print(pressure);
      Serial.println("mmHG");
      Serial.print("Tend: ");
      Serial.print(pressure_change);
      Serial.print("Pa/3H");
      if (forecast_flag)
        Serial.println(" - actual");
      else
        Serial.println(" - no actual");
    }
    Serial.println();
  } else if (serialRead < 8)
  {
    pressed_key = serialRead;
  }
  
  if ( pressed_key != 0 ) //если отключен тихий режим была нажата кнопка
    buzz.millisEnable(10); //пикуть зуммером
  
  keys.analogReadProcessBegin(); //будем получать значение с АЦП
  transmit_data.illumination = map(illuminationCorrectRead(LDR_PIN), 0, 1023, 0, 100); //ПОЛУЧЕНИЕ ЗНАЧЕНИИЯ С АЦП!
  keys.analogReadProcessEnd();  //значение с АЦП получено

  bool buzzer_flag = 0; //флаг управления зуммером по будильнику
  alarm_flag = 0;
  for ( byte i = 0; i < ALARMS_COUNT; i++ ) //перебрать все будильники в цикле
  {
    if ( Alarms[i].Check(real_time.hr,  //если сигнал активен
                         real_time.min,
                         real_time.day,
                         (bool)pressed_key) )
      buzzer_flag = 1;  //перевести флаг зуммера в 1
    if ( Alarms[i].getEnable() )  //если какой-либо будильник включен
      alarm_flag = 1; //перевести флаг активности (индикации) будильников в 1
  }

  if ( buzzer_flag )// && !danger_signaling) //если флаг сигнала зуммера - 1
  {
    buzz.intervalEnable(300); //включить зуммер с интервалом 300 мс
  } else {  //иначе
    buzz.intervalDisable(); //отключить зуммер
  }

  //запись текущего времени в структуру данных, передаваемых по радиоканалу
  transmit_data.hour = real_time.hr;    //часы
  transmit_data.minute = real_time.min; //минуты
  transmit_data.second = real_time.sec;  //секунды
  transmit_data.date = real_time.date;  //день месяца
  transmit_data.month = real_time.mon;  //месяц
  transmit_data.day = real_time.day;    //день
  transmit_data.connection |= ((1 << 3) * (real_time.yr % 4 == 0)); //високосный год

  //переменная хранения запроса объекта "Меню дисплея"
  static byte data_request_mode = Display.isActData(pressed_key);
  static bool data_request_success = 0;
  
  if ( data_request_success )
    data_request_mode = Display.isActData(pressed_key);

  if ( data_request_flag )
  {
    if ( data_request_mode == 1 )
    { 
      //Serial.print("Act data request: ");
      static struct {
        byte address;
        byte command;
      } data;
      
      data.address = CENTRAL_BOARD_ADDRESS;
      
      if ( act_request != 1 )
      {
        
        data.command = (1 << 4);  //запрос данных об актуаторе
        data.command |= Display.getActNumber();
      }
      //Serial.println( (data.command & 0b00001111) );
      
      radio.stopListening();
      radio.write(&data, sizeof(data));
      radio.startListening();
      
      //data_request_success = 1;
      act_request = 1;
      
      cli();
      _radio_free_counter = 0;
      sei();
      
      free_interval_flag = 0;
      radio_listen_flag = 1;
      
      radio.flush_rx();
      
    } else if ( data_request_mode == 2 )
    {
      //Serial.println("Act data sent");
      if ( act_request != 2 )
        transmit_data_act = Display.actDeviceGet();
      transmit_data_act.address = CENTRAL_BOARD_ADDRESS;
      transmit_data_act.command = 0b00100000;
      radio.stopListening();
      radio.write(&transmit_data_act, sizeof(transmit_data_act));
      radio.startListening();
      
      //data_request_success = 1;
      act_request = 2;
      
      cli();
      _radio_free_counter = 0;
      sei();
      
      free_interval_flag = 0;
      radio_listen_flag = 1;
      
      radio.flush_rx();
    } else if ( data_request_mode == 0 )
    {
      data_request_success = 1;
    }
}
  

  //счетчик времени с момента последнего успешного приема данных
  static unsigned long display_connection_counter = millis();

  byte pipeNO = 255;
  if (  radio.available( &pipeNO ) )
    if ( pipeNO == RECEIVE_DATA_PIPE )
    {
      struct {
        byte address = 255;
        byte command = 255;
      } data;

      //Serial.println("Reading some data");
      radio.read(&data, sizeof(&data));
      
      if ( data.address == ADDRESS )
      {
        //Serial.println("My address");

        switch ( data.command )
        {
          case 0:
            radio.read( &received_data, sizeof(received_data) );
            
            transmit_data.check_byte = received_data.check_byte + 1;
            
            radio.writeAckPayload(pipeNO, &transmit_data, sizeof(transmit_data));
            
            //Serial.println("Updating Data");
            //обновить метеоданные
            temp_outs = received_data.temperature;
            hum_outs = received_data.humidity;
            illum_outs = received_data.illumination;
            pressure = received_data.pressure;
            pressure_change = received_data.dPressure;
            weather_forecast = (Forecast)received_data.forecast;
            forecast_flag = received_data.forecast_flag;

            if ( act_request == 1 )
            {
              //Serial.println("Retry");
              data_request_success = 0;
              data_request_mode = 1;
              //act_request = 1;
              
            } else if ( act_request == 2 )
            {
              if ( !((received_data.connection >> 0)&(1 << 0)) )
              { 
                data_request_success = 0;
                data_request_mode = 2;
              } else {
                data_request_success = 0;
                data_request_mode = 1;
              } 
            } else {
              data_request_success = 1;
            }
              
              
            connect_flag = 1;
            display_connection_counter = millis();
            
            radio.flush_rx();
            break;
            
           case 1:
            radio.read(&received_data_act, sizeof(received_data_act));
            
            transmit_data.check_byte = received_data_act.check_byte + 1;
            
            radio.writeAckPayload(pipeNO, &transmit_data, sizeof(transmit_data));

            //Serial.print("READING ACT DATA__");
            //Serial.println(received_data_act.num);
            
            connect_flag = 1;
            
            data_request_success = 1;
            data_request_mode = 0;
            act_request = 0;
            
            Display.actDeviceUpdate(received_data_act);
            
            display_connection_counter = millis();
            
            radio.flush_rx();
            break;
        }
        
        
      } else {

        radio.flush_rx();
        cli();
        _radio_free_counter = 0;
        sei();
        free_interval_flag = 1;
        
        if ( (!free_interval_flag) != radio_listen_flag)
        {
          radio_listen_flag = !free_interval_flag;
          if ( !radio_listen_flag )
          {
            radio.flush_rx();
            radio.flush_tx();
            radio.stopListening();   
          }
        }
      }
    } else {
      radio.flush_rx();
    }

 
  if ( millis() - display_connection_counter >= RF_DATARESET_TIME ) 
  {
    connect_flag = 0; //флаг успешной передачи в 0
    data_request_success = 1;
    data_request_mode = 0;
    act_request = 0;
  }

  
  if ( buzzer_flag ) //если сигнал будильника не работает
  {
    Display.Switcher(0);  //управляем меню дисплея
  } else {
    Display.Switcher(pressed_key);  //управляем меню дисплея
  }

  static bool night_mode_previous = night_mode;
  if ( night_mode != night_mode_previous )
  {
    EEPROM.update( NIGHTMODE_EEPROM_ADDRESS, night_mode );
    night_mode_previous = night_mode;
  }
  if ( connect_flag )
  {
    received_data.connection |= (1 << 7);
    if ( !night_mode )
      reg.WriteByte(received_data.connection);
    else if ( night_mode && transmit_data.illumination > 25 )
      reg.WriteByte(received_data.connection);
    else if ( night_mode && transmit_data.illumination < 20 )
       reg.Write();
  } else {
      reg.Write();
  }
  analogWrite(DISPLGHTNESS_PIN, map(transmit_data.illumination, 0, 100, 45, 255)); //автонастройка яркости дисплея
  //wdt_reset();  //сброс сторожевого таймера
}

