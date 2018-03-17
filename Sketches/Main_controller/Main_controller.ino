#include <avr/wdt.h>  //для работы сторожевого таймера
#include <avr/pgmspace.h> //для работы PROGMEM
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EEPROM.h> //для работы с EEPROM


/*МОДУЛЬ 2.4GHz NRF24L01*/
#define RF_CE_PIN 9
#define RF_CSN_PIN 10
#define RADIO_CHANNEL 0x6A

#define BOARD_ADDRESS 0
#define WRITING_PIPE_NUM 0

#define SWITCH_ADDRESS_TIME 2500
#define RADIO_PAUSE_TIME 700

#define CPANEL_READING_PIPE_NUM 1 
#define CPANEL_LISTEN_PIPE 1 
#define CPANEL_ADDR 5
#define CPANEL_NUMBER 1
#define CPANEL_UPDATE_TIME 20000

//#define READING_PIPE_NUM_1
//#define READING_PIPE_NUM_2

#define METEO_ADDR 7
#define METEO_NUMBER 0
#define METEO_UPDATE_TIME 20000

#define ACTUATORS_ADDR 3
#define ACTUATORS_NUMBER 2
#define ACTUATORS_UPDATE_TIME 20000

#define MAX_TRANSMIT_NUMBER 2

/*НАСТРОЙКИ ИНДИКАТОРНЫХ СВЕТОДИОДОВ*/
#define POWER_LED_PIN 14  //пин подключения светодиода работы
#define WARNING_LED_PIN 15 //пин подключения светодиода ошибки
#define LED_PIN_1 5  //пин подключения светодиода 1
#define LED_PIN_2 4  //пин подключения светодиода 2
#define LED_PIN_3 3  //пин подключения светодиода 3
#define LED_PIN_4 7  //пин подключения светодиода 4

#define LED_BLINK_ON_INTERVAL 100  //время активной фазы мигания светодиода (мс)
#define LED_BLINK_OFF_INTERVAL 2000 //время  
#define LED_WARNING_INTERVAL 1500

/*ЗУММЕР*/
#define BUZZER_PIN 6 //пин подключения 

/*ДОПОЛНИТЕЛЬНЫЕ НАСТРОЙКИ*/
#define BAUDRATE 9600
#define ACT_COUNT 4

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
  byte forecast = 2; //для передачи прогноза погоды
  /* 0 - гарантированное улучшение погоды
     1 - возможно улучшение погоды
     2 - погода не изменится
     3 - возможно ухудшение погоды
     4 - гарантированно ухудшение погоды
  */
  int dPressure; //тенденция изменения атмосферного давления за 3 часа
  bool forecast_flag = 1;
};

struct Data_from_panel //структура данных, генерируемых на панели управления
{
  byte address = 0;
  byte command = 0;
  byte check_byte = 0;  //байт проверки связи
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

struct Data_panel_set_act
{
  byte address = 0;
  byte command = 0;
  byte number  = 0;
};

struct Data_actuators
{
  byte address = 0;
  byte command = 0;
  byte check_byte = 0; //байт проверки качества передачи данных
  byte connection = 0; //Побитовая кодировка дежурной информации:
  /*  0b [0][0][0][0][0][0][0][0]
          ^  ^  ^  ^  ^  ^  ^  ^
          A  M  С  M        Y  W
      A - Панель актуаторов
      M - Метеомодуль
      С - Панель управления
      M - Срабатывание датчика движения
      Y - флаг високосного года
      W - Warning!
  */
  byte actuators_states = 0xFF;
  byte act_manual_flags = 0;
  byte moisure_sensor = 0;
  //текущее время
  byte second = 0; //секунда
  byte minute = 0; //мин
  byte hour = 0;   //часы
  byte date = 0;   //число месяца
  byte month = 0;  //месяц
};

struct Data_time
{
  byte address = 0;
  byte command = 0;
  byte check_byte = 0; //байт проверки качества передачи данных
  byte connection = 0; //Побитовая кодировка дежурной информации:
  /*  0b [0][0][0][0][0][0][0][0]
          ^  ^  ^  ^  ^  ^  ^  ^
          A  M  С           Y  W
      A - Панель актуаторов (>> 7)
      M - Метеомодуль (>> 6)
      С - Панель управления (>> 5)
      Y - флаг високосного года (>> 1)
      W - Warning! (>> 0)
  */

  byte second = 0; //секунда
  byte minute = 0; //мин
  byte hour = 0;   //часы
  byte date = 1;   //число месяца
  byte month = 1;  //месяц
};

class SystemTime  //класс обработчика системного времени
{
  private:
    uint16_t  millis_counter; //счетчик миллисекунд
    volatile uint8_t minute;         //минута  (0..59)
    volatile uint8_t hour;           //час     (0..23)
    volatile uint8_t date;           //день месяца (1..max_date)
    volatile uint8_t month;          //месяц (1..12)
    uint8_t          year;           //флаг високосного года (0/1)

    const uint8_t max_date[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; //максммальное значение дня месяца

    bool begin_flag = 0;  //флаг активности работы
    bool mode_flag = 0;   //флаг режима времени (0 - модульное время, 1 - общесистемное время)

  public:

    SystemTime()  //конструктор
    { //обнуление всех счетчиков и флагов
      millis_counter = 0;
      minute  = 0;
      hour    = 0;
      date    = 1;
      month   = 1;
      year    = 0;
      begin_flag = 0;
      mode_flag  = 0;
    }

    void begin()  //функция запуска часов
    {
      cli();
      begin_flag = 1; //флаг активности работы в 1
      sei();
    }

    void set(byte _second, byte _minute, byte _hour, byte _date, byte _month, bool _year)
    {
      cli();
      //запись новых значений времени
      millis_counter  = _second * 1000;
      minute          = _minute;
      hour            = _hour;
      date            = _date;
      month           = _month;
      year            = _year;
      mode_flag       = 1; //флаг режима времени в состояние "общесистемное"
      sei();
    }

    void Interrupt_func() //функция для вызова в обработчике прерываний
    {
      if (begin_flag) //если часы запущены
      {
        millis_counter++; //инкремент счетчика

        if (millis_counter >= 60000) //если досчитали до 60000 мс - 1 минута
        {
          millis_counter = 0;  //обнулить счетчик мс
          minute++; //инкремент счетчика минут

          if (minute >= 60) //если досчитали до 60 минут
          {
            minute = 0; //обнулить счетчик минут
            hour++; //инкремент счетчика часов

            if (hour >= 24) //если досчитали до 24 ч
            {
              hour = 0; //обнулить счетчик часов
              date++; //инеремент счетчика даты

              //если день месяца превысил допустимый диапозон
              if ( date > (max_date[month - 1] + year * (month == 2)) )
              {
                date = 1; //установить счетчик дня месяца в 1
                month++;  //инкремент счетчика месяца

                if (month >= 13)  //если месяц превысил допустисый диапазон
                {
                  month = 1;  //обнулить счетчик месяца
                  year = 0; //обнулить год в состояние "невисокосный"
                }
              }
            }
          }
        }
      }
    }

    byte getSec()
    {
      cli();
      byte _second = millis_counter / 1000;
      sei();
      return _second;
    }
    byte getMin() //функция вывода текущего значения минут
    {
      cli();
      byte _minute = minute;
      sei();
      return _minute;
    }

    byte getHour() //функция вывода текущего значения часов
    {
      cli();
      byte _hour = hour;
      sei();
      return _hour;
    }

    byte getDate() //функция вывода текущего дня месяца
    {
      cli();
      byte _date = date;
      sei();
      return _date;
    }

    byte getMonth() //функция вывода текущего месяца
    {
      cli();
      byte _month = month;
      sei();
    }

    bool getMode()
    {
      cli();
      byte _mode_flag = mode_flag;
      sei();
      return _mode_flag;
    }

    void getTime(byte& _second, byte& _minute, byte& _hour, byte& _date, byte& _month, bool& _year, bool& _mode_flag)
    {
      cli();
      _second = millis_counter / 1000;
      _minute = minute;
      _hour = hour;
      _date = date;
      _month = month;
      _year = year;
      _mode_flag = mode_flag;
      sei();
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

      void getEEPROM(byte addr)
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

class Actuator
{
  private:
    Time_Correspondence *Schedule;

    enum Enable_mode {
      MANUAL = 0,
      TEMPERATURE = 1,
      HUMIDITY = 2,
      ILLUMINATION = 3,
      SCHEDULE = 4
    };

    Enable_mode mode = MANUAL; //режим работы

    boolean enable_flag = 0; //флаг активности работы

    int  temperature = 25; //температура переключения
    bool t_sensor = 0;  //сенсор: 0 - внури помещения, 1 - на улице
    bool t_mode = 0;  //режим переключения: 0 - включение при температуре ниже, 1 - при температуре выше
    const byte t_gisteresis = 2;

    byte humidity = 45; //влажность переключения
    bool h_sensor = 0; //сенсор: 0 - внури помещения, 1 - на улице
    bool h_mode = 0; //режим переключения: 0 - включение при температуре ниже, 1 - при температуре выше
    const byte h_gisteresis = 5;

    byte illumination = 50;  //освещенность переключения
    bool i_sensor = 0; //сенсор: 0 - внури помещения, 1 - на улице
    bool i_mode = 0; //режим переключения: 0 - включение при температуре ниже, 1 - при температуре выше
    const byte i_gisteresis = 15;

    int  *t_ins     = NULL;
    byte *h_ins     = NULL;
    byte *i_ins     = NULL;
    bool *ins_flag  = NULL;

    int  *t_outs    = NULL;
    byte *h_outs    = NULL;
    byte *i_outs    = NULL;
    bool *outs_flag = NULL;

    byte *hour      = NULL;
    byte *minute    = NULL;
    byte *day       = NULL;
    //можно было бы создать tempalate, но выигрыша по памяти не будет, поэтому выбран более наглядный вариант

    template <typename T>
    bool sensorChooseMode(
      bool sensor,
      bool mode_flag,
      T fixed_val,
      T val_inside,
      bool val_inside_flag,
      T val_outside,
      bool val_outside_flag,
      byte gisteresis)
    {
      T changing_val;
      bool flag;
      switch (sensor)
      {
        case 0:
          changing_val = val_inside;
          flag = val_inside_flag;
          break;

        case 1:
          changing_val = val_outside;
          flag = val_outside_flag;
          break;
      }
      if (flag)
        switch ( mode_flag )
        {
          case 0:
            if ( changing_val <= fixed_val )
              return 1;
            else if ( changing_val >= fixed_val + gisteresis )
              return 0;
            break;
          case 1:
            if ( changing_val >= fixed_val )
              return 1;
            else if ( changing_val <= fixed_val - gisteresis )
              return 0;
            break;
        }
      else
        return enable_flag;
    }

  public:

      void setEEPROM(byte addr)
      {
       EEPROM.update(addr + 511, getMode());
       addr++;
       EEPROM.update(addr + 511, enable_flag);
       addr++;
       EEPROM.update(addr + 511, (temperature >> 8) );
       addr++;
       EEPROM.update(addr + 511, (temperature & 0xFF) );
       addr++;
       EEPROM.update(addr + 511, t_sensor);
       addr++;
       EEPROM.update(addr + 511, t_mode);
       addr++;
       EEPROM.update(addr + 511, humidity);
       addr++;
       EEPROM.update(addr + 511, h_sensor);
       addr++;
       EEPROM.update(addr + 511, h_mode);
       addr++;
       EEPROM.update(addr + 511, illumination);
       addr++;
       EEPROM.update(addr + 511, i_sensor);
       addr++;
       EEPROM.update(addr + 511, i_mode);
       addr++;
      }

      void getEEPROM(byte addr)
      {
       setMode(EEPROM.read(addr + 511));
       addr++;
       enable_flag = EEPROM.read(addr + 511);
       addr++;
       temperature = 0;
       temperature = (EEPROM.read(addr + 511) >> 8);
       addr++;
       temperature |= EEPROM.read(addr + 511);
       addr++;
       t_sensor = EEPROM.read(addr + 511);
       addr++;
       t_mode = EEPROM.read(addr + 511);
       addr++;
       humidity = EEPROM.read(addr + 511);
       addr++;
       h_sensor = EEPROM.read(addr + 511);
       addr++;
       h_mode = EEPROM.read(addr + 511);
       addr++;
       illumination = EEPROM.read(addr + 511);
       addr++;
       i_sensor = EEPROM.read(addr + 511);
       addr++;
       i_mode = EEPROM.read(addr + 511);
       addr++;
      }

    Actuator (Time_Correspondence* _Schedule) //конструктор
    {
      Schedule = _Schedule;
    }

    void begin( int*  _t_ins,  byte* _h_ins,  byte* _i_ins,   bool* _ins_flag,
                int*  _t_outs, byte* _h_outs, byte* _i_outs,  bool* _outs_flag,
                byte* _hour,   byte* _minute, byte* _day )
    {
      t_ins     = _t_ins;
      h_ins     = _h_ins;
      i_ins     = _i_ins;
      ins_flag  = _ins_flag;

      t_outs    = _t_outs;
      h_outs    = _h_outs;
      i_outs    = _i_outs;
      outs_flag = _outs_flag;

      hour      = _hour;
      minute    = _minute;
      day       = _day;

    }

    void ForLoop(bool& time_flag) //для бесконечного цикла программы, проверка состояний
    {
      switch ( mode )
      {
        case TEMPERATURE:

          enable_flag = sensorChooseMode<int>(t_sensor, t_mode, temperature, *t_ins, *ins_flag, *t_outs, *outs_flag, t_gisteresis);
          break;
        case HUMIDITY:
          enable_flag = sensorChooseMode<byte>(h_sensor, h_mode, humidity, *h_ins, *ins_flag, *h_outs, *outs_flag, h_gisteresis);
          break;
        case ILLUMINATION:
          enable_flag = sensorChooseMode<byte>(i_sensor, i_mode, illumination, *i_ins, *ins_flag, *i_outs, *outs_flag, i_gisteresis);
          break;
        case SCHEDULE:
          //Serial.println("<<<<<<<<CHRKING");
          if ( !Schedule->getEnable() )
          {
            Schedule->setEnable(1);
            Serial.print("Enable change");
          }
          if (time_flag)
          {
            //Serial.println(Schedule->getEnable());
            if (Schedule->Check(*hour, *minute, *day, 1))
            {
              enable_flag = !enable_flag;
              if ( !Schedule->getEnable() )
                mode = MANUAL;
              Serial.println("CHECKING>>>>>>");
            } 
          }
          break;
      }
    }

    template <class T>
    void getSettings(T *Get)
    {
      Get->enable_flag = enable_flag;
      Get->mode = mode;

      Get->temperature = temperature;
      Get->t_sensor = t_sensor;
      Get->t_mode = t_mode;

      Get->humidity = humidity;
      Get->h_sensor = h_sensor;
      Get->h_mode = h_mode;

      Get->illumination = illumination;
      Get->i_sensor = i_sensor;
      Get->i_mode = i_mode;

      Get->hour = Schedule->getHour();
      Get->minute = Schedule->getMinute();
      Get->repeat_days = Schedule->getWeek();
    }

    template <class T>
    void setSettings(T *Set)
    {
      enable_flag = Set->enable_flag;
      mode = Set->mode;

      temperature = Set->temperature;
      t_sensor    = Set->t_sensor;
      t_mode      = Set->t_mode;

      humidity  = Set->humidity;
      h_sensor  = Set->h_sensor;
      h_mode    = Set->h_mode;

      illumination  = Set->illumination;
      i_sensor      = Set->i_sensor;
      i_mode        = Set->i_mode;

      Schedule->setHour(Set->hour);
      Schedule->setMinute(Set->minute);
      Schedule->setWeek(Set->repeat_days);
      
    }

    bool getEnable()
    {
      return (bool)enable_flag;
    }

    void Enable()
    {
      mode = MANUAL;
      enable_flag = 1;
    }

    void Disable()
    {
      enable_flag = 0;
      if (mode == SCHEDULE && !Schedule->getDay(8))
        mode = MANUAL;
    }


    void setEnable(bool _enable_flag)
    {
      enable_flag = _enable_flag;
    }

    void setMode_M()
    {
      mode = MANUAL;
    }

    void setMode_T()
    {
      mode = TEMPERATURE;
    }

    void setMode_H()
    {
      mode = HUMIDITY;
    }

    void setMode_I()
    {
      mode = ILLUMINATION;
    }

    void setEnable_S()
    {
      mode = SCHEDULE;
    }

    void setMode(byte _mode)
    {
      switch (_mode)
      {
        case 0:
          mode = MANUAL;
          break;
        case 1:
          mode = TEMPERATURE;
          break;
        case 2:
          mode = HUMIDITY;
          break;
        case 3:
          mode = ILLUMINATION;
          break;
        case 4:
          mode = SCHEDULE;
          break;
      }
      mode = _mode;
    }

    byte getMode()
    {
      switch (mode)
      {
        case MANUAL:
          return 0;
          break;
        case TEMPERATURE:
          return 1;
          break;
        case HUMIDITY:
          return 2;
          break;
        case ILLUMINATION:
          return 3;
          break;
        case SCHEDULE:
          return 4;
          break;
      }
    }

    void rstSchedule()  //сброс настроек расписания
    {
      Schedule->Reset();
    }

    void rstTemp() //сброс настроек темперауры
    {
      temperature = 25;
      t_sensor = 0;
      t_mode = 0;
    }

    int getTemp()
    {
      return temperature;
    }

    bool getT_Mode()
    {
      return t_mode;
    }

    bool getT_Sensor()
    {
      return t_sensor;
    }

    void setTemp(int _temperature)
    {
      temperature = _temperature;
    }

    void setT_Mode(bool flag)
    {
      t_mode = flag;
    }

    void setT_Sensor(bool flag)
    {
      t_sensor = flag;
    }

    void rstHumid()  //сброс настроек влажности
    {
      humidity = 50;
      h_sensor = 0;
      h_mode = 0;
    }

    byte getHumid()
    {
      return humidity;
    }

    bool getH_Mode()
    {
      return h_mode;
    }

    bool getH_Sensor()
    {
      return h_sensor;
    }

    void setHumid(byte _humidity)
    {
      humidity = _humidity;
    }

    void setH_Mode(bool flag)
    {
      h_mode = flag;
    }

    void setH_Sensor(bool flag)
    {
      h_sensor = flag;
    }

    void rstIllum() //сброс настроек темперауры
    {
      illumination = 50;
      i_mode = 0;
      i_sensor = 0;
    }

    void setIllum(byte _illumunation)
    {
      illumination = _illumunation;
    }

    void setI_Mode(bool flag)
    {
      i_mode = flag;
    }

    void setI_Sensor(bool flag)
    {
      i_sensor = flag;
    }

    byte getIllum()
    {
      return illumination;
    }

    bool getI_Mode()
    {
      return i_mode;
    }

    bool getI_Sensor()
    {
      return i_sensor;
    }
};


const uint64_t pipes[6] = { //адреса ("трубы") обмена данными
  0xFFFFFFFFF0LL,  //адрес 0
  0xFFFFFFFFF1LL,  //адрес 1
  0xFFFFFFFFF2LL,  //адрес 2
  0xFFFFFFFFF3LL,  //адрес 3
  0xFFFFFFFFF4LL,  //адрес 4
  0xFFFFFFFFF5LL   //адрес 5
};

int  temp_outs = 0; //температура вне помещения
byte hum_outs = 0; //относительная влажность вне помещения
byte illum_outs = 0; //уровень освещенности вне помещения
bool outs_flag = 0; //флаг "свежести" данных

int  temp_ins = 0; //температура внутри помещения
byte hum_ins = 0; //относительная влажность внутри помещения
byte illum_ins = 0; //уровень освещенности внутри помещения
bool ins_flag = 0;  //флаг "свежести" данных

byte moisure = 0; //влага

int  pressure = 0; //атмосферное давление
int  pressure_change = 0; //изменение атмосферного давления за 3 часа
byte forecast = 2;
bool forecast_flag = 1;

byte data_received_flags[5] = { 0, 0, 1, 0, 0 };  //флаги приёма новых данных
bool update_request_flag[6] = { 0, 0, 0, 0, 0, 0 }; //флаги запросов обновления
bool system_reboot_flag = 0;


byte second = 0;
byte minute = 0;
byte hour   = 0;
byte date   = 0;
byte day    = 0;
byte month  = 0;
bool year_flag = 0;
bool time_flag = 0;

byte indication_byte = 0;

//счетчики времени с последнего приема данных
volatile unsigned long _meteo_request_counter = 0; //с метеомодуля
volatile unsigned long _actuators_request_counter = 0; //с модуля актуаторов
volatile unsigned long _cpanel_request_counter = 0; //с панели управления

volatile unsigned int   _led1_blink_counter         = 0;
volatile unsigned int   _led2_blink_counter         = 0;
volatile unsigned int   _led3_blink_counter         = 0;
volatile unsigned int   _led4_blink_counter         = 0;
volatile unsigned int   _led_power_blink_counter    = 0;
volatile unsigned int   _led_warning_blink_counter  = 0;

bool actuators_connection = 0;
bool flower_request = 0;

volatile unsigned int _radio_transmit_switch_counter = 0;
byte                  transmitting_mode = 0;

byte                  cpanel_sent_mode = 0;
byte                  actuators_sent_mode = 0;
volatile unsigned int _radio_transmit_pause_counter = 0;
SystemTime real_time;

//структуры данных для радиообмена
Data_meteo        received_meteo_data;  //данные, принятые с метеомодуля
Data_time         transmit_meteo_data;  //данные, отправляемые на метеомодуль (время)
//-----
Data_actuators    received_act_data;  //данные, принятые с панели актуаторов
Data_actuators    transmit_act_data;  //данные, отправляемые на панель актуаторов

//данные, отправляемые на панель управления
Data_meteo        transmit_panel_data;        //в стандартном режиме
Data_panel_act    transmit_panel_act_data;    //в режиме настройки актуатора

//данные, принимаеиые с панели управления
Data_from_panel   received_panel_data; //в стандартном режиме
Data_panel_act    received_panel_act_data; //в режиме настройки актуатора
                                            

/*СОЗДАНИЕ ОБЪЕКТОВ*/
RF24 radio(RF_CE_PIN, RF_CSN_PIN);  //создать объект - модуль связи nRF

Time_Correspondence Act_schedule[ACT_COUNT];

Actuator Act_device[ACT_COUNT] = {
  Actuator(&Act_schedule[0]),
  Actuator(&Act_schedule[1]),
  Actuator(&Act_schedule[2]),
  Actuator(&Act_schedule[3])
};

/*ФУНКЦИИ*/

void radioRecieve() //функция для обработчика внешних прерываний по команде радиомодуля
{
  byte currPipeNum;
  
  if ( radio.available( &currPipeNum ) )
  {
    Serial.println("GET SOME DATA");
    if ( currPipeNum == CPANEL_LISTEN_PIPE )
    {
      struct {
        byte address;
        byte command;
      } data;
      radio.read(&data, sizeof(data));
      
      if ( data.address == BOARD_ADDRESS )
      {

        if ( (data.command >> 4) == 2 )
        {
          radio.read(&received_panel_act_data, sizeof(received_panel_act_data));

          Serial.print(printTime(second, minute, hour, date, month));
          Serial.print("Control panel: Actuator #");
          Serial.print( received_panel_act_data.num );
          Serial.println(" update request");
          Act_device[received_panel_act_data.num - 1].setSettings(&received_panel_act_data);
          
          Act_device[received_panel_act_data.num - 1].setEEPROM( (received_panel_act_data.num - 1) * 12 );
          Act_schedule[received_panel_act_data.num - 1].setEEPROM( (received_panel_act_data.num - 1) * 4 + ACT_COUNT * 12 );
          
          cpanel_sent_mode = 2;
        } else if ( ( data.command >> 4 ) == 1 )
        {
          Serial.print(printTime(second, minute, hour, date, month));
          Serial.print("Control panel: Actuator #");
          Serial.print( (data.command & (0b00001111)) );
          Serial.println(" data request");
          _radio_transmit_switch_counter = 0;
          transmitting_mode = 1;
          cpanel_sent_mode = 1;

          Act_device[(data.command & (0b00001111)) - 1].getSettings(&transmit_panel_act_data);
          transmit_panel_act_data.count = ACT_COUNT;
          transmit_panel_act_data.num = (data.command & (0b00001111));
        } 
      }
    } 
  }
}

void leds_init()
{
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(LED_PIN_3, OUTPUT);
  pinMode(LED_PIN_4, OUTPUT);
  pinMode(WARNING_LED_PIN, OUTPUT);
  pinMode(POWER_LED_PIN, OUTPUT);

  digitalWrite(LED_PIN_1, 1);
  digitalWrite(LED_PIN_2, 1);
  digitalWrite(LED_PIN_3, 1);
  digitalWrite(LED_PIN_4, 1);
  digitalWrite(WARNING_LED_PIN, 1);
  digitalWrite(POWER_LED_PIN, 1);
  delay(500);
  
  digitalWrite(LED_PIN_1, 0);
  digitalWrite(LED_PIN_2, 0);
  digitalWrite(LED_PIN_3, 0);
  digitalWrite(LED_PIN_4, 0);
  digitalWrite(WARNING_LED_PIN, 0);
  delay(500);
}

void timer1_Setup() //настройки таймера 1
{
  TCCR1B |= (1 << 3); //установка режима таймера: сброс при совпадении
  OCR1A = 0xF9; //запись значения в регистр сравнения (249)
  TIMSK1 |= (1 << OCIE1A);  //разрешить прерывание при совпадении с регистром A (OCR1A)
  TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10); //усановить предделитель на 64
  sei();  //разрешаем прерывания
}

ISR(TIMER1_COMPA_vect)  //обработчик прерываний таймера 1 при совпадении с регистром A - вызывается раз в 1 мс
{

  _meteo_request_counter++;
  _actuators_request_counter++;
  _cpanel_request_counter++;

  _radio_transmit_switch_counter++;
  _radio_transmit_pause_counter++;
  _led1_blink_counter++;
  _led2_blink_counter++;
  _led3_blink_counter++;
  _led4_blink_counter++;
  _led_power_blink_counter++;
  _led_warning_blink_counter++;
  
  real_time.Interrupt_func();
}

String AddChar(uint8_t value, char set_in)  //добавление символа перед числом, если оно одноразрядное
{
  if (value == 0)
    return String(set_in) + String(set_in);
  else if (value < 10)
    return set_in + String(value, DEC);
  else
    return String(value, DEC);
}

String printTime(byte second, byte minutes, byte hours, byte date, byte month)
{
  String mode = "";
  if (!time_flag)
    mode = " No Sync!";
  return '[' + AddChar(date, '0') + '/' + AddChar(month, '0') + ' '
         + AddChar(hours, '0') + ':' + AddChar(minutes, '0') + ':' + AddChar(second, '0') + mode + ']' + ' ';
}

void ledBlink(byte pin, 
              unsigned int &counter_value, 
              volatile unsigned int &counter, 
              unsigned int on_interval, 
              unsigned int off_interval)
{
  if (digitalRead(pin))
    if (counter_value >= on_interval)
    {
      cli();
      counter = 0;
      sei();
      digitalWrite(pin, 0);
    } else { }
  else if (counter_value >= off_interval)
    {
      cli();
      counter = 0;
      sei();
      digitalWrite(pin, 1);
    } else { }
}

void setup() {
  timer1_Setup();
  Serial.begin(BAUDRATE);
  Serial.println("------------------");
  Serial.println(" --SMART HOUSE--");
  Serial.println("   -MAINBOARD-");
  Serial.println("------------------");
  leds_init();

  radio.begin();
  delay(100);
  radio.powerUp();
  delay(100);
  radio.setChannel(RADIO_CHANNEL);
  radio.setRetries(15, 30);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setCRCLength(RF24_CRC_16);

  radio.setAutoAck(1);
  radio.enableDynamicPayloads();
  radio.enableAckPayload(); 

  radio.openWritingPipe(pipes[WRITING_PIPE_NUM]); // 0 is SYSTEM, no reading

  radio.openReadingPipe(CPANEL_LISTEN_PIPE, pipes[CPANEL_READING_PIPE_NUM]);

  //чтение данных из энергонезависимой памяти
  byte a = 0;
  Act_device[a].getEEPROM( (a) * 12 );
  Act_schedule[a].setEEPROM( (a) * 4 + ACT_COUNT * 12 );
  a++;
  Act_device[a].getEEPROM( (a) * 12 );
  Act_schedule[a].setEEPROM( (a) * 4 + ACT_COUNT * 12 );
  a++;
  Act_device[a].getEEPROM( (a) * 12 );
  Act_schedule[a].setEEPROM( (a) * 4 + ACT_COUNT * 12 );
  a++;
  Act_device[a].getEEPROM( (a) * 12 );
  Act_schedule[a].setEEPROM( (a) * 4 + ACT_COUNT * 12 );
  
  real_time.begin();  //запуск счетчика системного времени
  real_time.getTime(second, minute, hour, date, month, year_flag, time_flag);

  for (byte i = 0; i < ACT_COUNT; i++)
    Act_device[i].begin( &temp_ins,  &hum_ins,  &illum_ins,  &ins_flag,
                         &temp_outs, &hum_outs, &illum_outs,  &outs_flag,
                         &hour,      &minute,   &day );

  attachInterrupt(0, radioRecieve, FALLING);
  radio.startListening();
  cli();
  _radio_transmit_switch_counter = 0;
  sei();
}


void loop()
{  
  cli();

  unsigned long meteo_request_counter       = _meteo_request_counter;
  unsigned long actuators_request_counter   = _actuators_request_counter;
  unsigned long cpanel_request_counter      = _cpanel_request_counter;

  unsigned int radio_transmit_switch_counter  = _radio_transmit_switch_counter;
  unsigned int radio_transmit_pause_counter   = _radio_transmit_pause_counter;

  unsigned int   led1_blink_counter         = _led1_blink_counter;
  unsigned int   led2_blink_counter         = _led2_blink_counter;
  unsigned int   led3_blink_counter         = _led3_blink_counter;
  unsigned int   led4_blink_counter         = _led4_blink_counter;
  unsigned int   led_power_blink_counter    = _led_power_blink_counter;
  unsigned int   led_warning_blink_counter  = _led_warning_blink_counter;

  bool radio_write_flag = 0;
  if ( radio_transmit_pause_counter >= RADIO_PAUSE_TIME )
  {
    _radio_transmit_pause_counter = 0;
    radio_write_flag = 1;
  }

  static byte transmitting_mode = 0;
  if ( radio_transmit_switch_counter >=  SWITCH_ADDRESS_TIME )
  {
    transmitting_mode++;
    if (transmitting_mode == (MAX_TRANSMIT_NUMBER + 1) )
      transmitting_mode = 0;

    _radio_transmit_switch_counter = 0;

    _radio_transmit_pause_counter = 0;
    radio_write_flag = 1;
    
    radio.flush_rx();
  }

  real_time.getTime(second, minute, hour, date, month, year_flag, time_flag);

  sei();
          
  static bool answer_meteo_flag     = 1;
  static bool answer_cpanel_flag    = 1;
  static bool answer_actuators_flag = 1;

  if ( radio_write_flag )
    if ( transmitting_mode == METEO_NUMBER )
    {
      if ( answer_meteo_flag )
      {
        transmit_meteo_data.check_byte = random(253) + 1;
        answer_meteo_flag = 0;
      }
      
      radio.stopListening();
      transmit_meteo_data.address = METEO_ADDR;
      
      transmit_meteo_data.month       = month;
      transmit_meteo_data.date        = date;
      transmit_meteo_data.connection  |= (1 << 3) * year_flag;
      transmit_meteo_data.hour        = hour;
      transmit_meteo_data.minute      = minute;
      transmit_meteo_data.second      = second;

      radio.flush_rx();
      radio.flush_tx();

      cli();
      if ( radio.write( &transmit_meteo_data, sizeof(transmit_meteo_data)) )
      {
        if ( radio.available() )
        {
          byte address = 255;
          radio.read(&address, sizeof(address));

          if (address == METEO_ADDR)
          {
            radio.read(&received_meteo_data, sizeof(received_meteo_data));
            sei();
            answer_meteo_flag = transmit_meteo_data.check_byte == received_meteo_data.check_byte - 1;
          } else {
            sei();
            answer_meteo_flag = 0;
            radio.flush_rx();
          }

          if ( answer_meteo_flag )
          {
            Serial.print(printTime(second, minute, hour, date, month));
            Serial.println("Meteo update success");
            hum_outs          = received_meteo_data.humidity;
            illum_outs        = received_meteo_data.illumination;
            pressure          = received_meteo_data.pressure;
            pressure_change   = received_meteo_data.dPressure;
            forecast          = received_meteo_data.forecast;
            forecast_flag     = received_meteo_data.forecast_flag;
            outs_flag         = 1;
            transmit_panel_data = received_meteo_data;

            cli();
            _meteo_request_counter = 0;
            _radio_transmit_switch_counter = 2000;
            sei();
          }
        } 
      } 

      radio.startListening();

    } else if ( transmitting_mode == CPANEL_NUMBER )
    {
      radio.stopListening();

      if ( answer_cpanel_flag )
      {
        transmit_panel_data.check_byte = random(253) + 1;
        answer_cpanel_flag = 0;
      }

      transmit_panel_data.address     = CPANEL_ADDR;
      transmit_panel_act_data.address = CPANEL_ADDR;
      
      transmit_panel_act_data.command = 1;

      transmit_panel_data.connection = ((actuators_connection << 5) | (outs_flag << 6) | (flower_request << 1));
      
      radio.flush_rx();
      radio.flush_tx();
      
      bool wrt_success;

      if ( cpanel_sent_mode == 2 )
        transmit_panel_data.connection |= (1 << 0);
      else
        transmit_panel_data.connection &= ~(1 << 0);
      
      cli();
      if ( cpanel_sent_mode == 1 )
      {
        wrt_success = radio.write( &transmit_panel_act_data, sizeof(transmit_panel_act_data) );
        //Serial.println("Writing ACT");
      } else {
        wrt_success = radio.write( &transmit_panel_data, sizeof(transmit_panel_data) );
        //Serial.println("Writing BASIC");
      }

      if ( wrt_success )
      {
        if ( radio.available() )
        {
          byte address = 0;
          radio.read(&address, sizeof(address));

          if ( address == CPANEL_ADDR )
          {
            radio.read(&received_panel_data, sizeof(received_panel_data));
            sei();
            
            if ( cpanel_sent_mode == 1 )
              answer_cpanel_flag = transmit_panel_act_data.check_byte == received_panel_data.check_byte - 1;
            else
              answer_cpanel_flag = transmit_panel_data.check_byte == received_panel_data.check_byte - 1;
              
          } else {
            sei();
            answer_cpanel_flag = 0;
          }
          
          if ( answer_cpanel_flag )
          {
            real_time.set( received_panel_data.second,
                           received_panel_data.minute,
                           received_panel_data.hour,
                           received_panel_data.date,
                           received_panel_data.month,
                           (received_panel_data.connection >> 3) & (1 << 0) );
            day = received_panel_data.day;

            temp_ins = received_panel_data.temperature;
            hum_ins = received_panel_data.humidity;
            illum_ins = received_panel_data.illumination;
            ins_flag = 1;
            
            real_time.getTime(second, minute, hour, date, month, year_flag, time_flag);
            
            if ( cpanel_sent_mode == 1 )
            {
              cpanel_sent_mode = 0;
              Serial.print(printTime(second, minute, hour, date, month));
              Serial.println("Actuator data sent success");
            } else if ( cpanel_sent_mode == 2 )
            {
              cpanel_sent_mode = 0;
              Serial.print(printTime(second, minute, hour, date, month));
              Serial.println("Actuator data update success");
            }

            Serial.print(printTime(second, minute, hour, date, month));
            Serial.println("System time: synchronized");

            cli();
            _cpanel_request_counter = 0;
            _radio_transmit_switch_counter = 2000;
            sei();
          }

        }
      } 
      
      radio.startListening();
      
    } else if ( transmitting_mode == ACTUATORS_NUMBER )
    {
      radio.stopListening();

      if ( answer_actuators_flag )
      {
        transmit_act_data.check_byte = random(253) + 1;
        answer_actuators_flag = 0;
      }

      transmit_act_data.address = ACTUATORS_ADDR;
      transmit_act_data.command = 1;
      
      transmit_act_data.actuators_states = 0;
      for (byte i = 0; i < ACT_COUNT; i++)
        transmit_act_data.actuators_states |= ( Act_device[i].getEnable() << i );
        
      radio.flush_rx();
      radio.flush_tx();

      if ( actuators_sent_mode == 1 )
        transmit_act_data.connection |= (1 << 0);
      else
        transmit_act_data.connection &= ~(1 << 0);
        
      cli();
      if ( radio.write(&transmit_act_data, sizeof(transmit_act_data)) )
      {
        if ( radio.available() )
        {

          byte address = 0;
          radio.read(&address, sizeof(address));

          if (address == ACTUATORS_ADDR)
          {
            radio.read(&received_act_data, sizeof(received_act_data));
            sei();

            answer_actuators_flag = transmit_act_data.check_byte == received_act_data.check_byte - 1;

          } else {
            sei();
            answer_actuators_flag = 0;
          }
          
          if ( answer_actuators_flag )
          {

            Serial.print(printTime(second, minute, hour, date, month));
            Serial.println("Actuators update success");
            moisure = received_act_data.moisure_sensor;
            actuators_connection = 1;
            if ( moisure < 125 )
              flower_request = 1;
            else if ( moisure > 140 )
              flower_request = 0;
              
            cli();
            _actuators_request_counter = 0;
            _radio_transmit_switch_counter = 2000;
            sei();
          }
        } 
      }
    } 
    
  for ( byte i = 0; i < ACT_COUNT; i++ )
    Act_device[i].ForLoop( time_flag ); //для бесконечного цикла программы, проверка состояний

  if ( cpanel_request_counter >= CPANEL_UPDATE_TIME )
  {
    ins_flag = 0;
  }
  if ( meteo_request_counter >= METEO_UPDATE_TIME )
  {
    outs_flag = 0;
  }
  
  if (actuators_request_counter >= ACTUATORS_UPDATE_TIME )
  {
    actuators_connection = 0;
  }

  if ( ins_flag )
    digitalWrite(LED_PIN_1, 0);
  else 
    digitalWrite(LED_PIN_1, 1);
            
   if ( outs_flag )
      digitalWrite(LED_PIN_2, 0);
   else
      digitalWrite(LED_PIN_2, 1);
            
  if ( actuators_connection )
    digitalWrite(LED_PIN_3, 0);
  else 
    digitalWrite(LED_PIN_3, 1);
  
  if (time_flag)
    ledBlink(WARNING_LED_PIN,
            led_power_blink_counter, 
            _led_power_blink_counter, 
            LED_BLINK_ON_INTERVAL, 
            LED_BLINK_OFF_INTERVAL);
  else 
    ledBlink(WARNING_LED_PIN,
            led_power_blink_counter, 
            _led_power_blink_counter, 
            LED_WARNING_INTERVAL, 
            LED_WARNING_INTERVAL);
            
  
}
