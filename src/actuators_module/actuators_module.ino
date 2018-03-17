/*-----------------------------------------------*/
/*ДИРЕКТИВА #include - ПОДКЛЮЧЕНИЕ ДОПОЛНИТЕЛЬНЫХ ФАЙЛОВ*/
#include <avr/wdt.h>  //для работы сторожевого таймера
#include <SPI.h>  //библиотека для работы шины SPI
#include <RF24.h> //библиотека модуля nRF
#include <nRF24L01.h> //библиотека модуля nRF
#include <Bounce2.h>  //библиотека антидребезга кнопок
#include <EEPROM.h> //библиотека для работы с энергонезависимой памятью

/*-----------------------------------------------*/
/*НАСТРОЙКИ*/


/*МОДУЛЬ 2.4GHz NRF24L01*/
#define RF_CE_PIN 9 
#define RF_CSN_PIN 10
#define RADIO_CHANNEL 0x6A

#define READING_PIPE_NUM 0
//#define WRITING_PIPE_NUM 2

//#define CENTRAL_BOARD_ADDRESS 0
#define RECEIVE_DATA_PIPE 1

#define ADDRESS 3

#define RF_DATARESET_TIME 7000 //время, в течение которого данные сохраняются, если связь потеряна (мс)
#define RF_NOTLISTEN_TIME 800

#define ACT_PIN_1 2 //пин подключения имполнителя 1
#define ACT_PIN_2 3 //пин подключения имполнителя 2
#define ACT_PIN_3 4 //пин подключения имполнителя 3
#define ACT_PIN_4 5 //пин подключения имполнителя 4
#define ACK_UPDATE_INTERVAL 5000  //интервал времени обновления выходов
#define EEPROM_ACT_ADDR 512 //адрес байта энергонезависимой памяти для записи состояний исполнителей

#define KEY_PIN_1 17  //пин подключения тактовой кнопки 1
#define KEY_PIN_2 16  //пин подключения тактовой кнопки 2
#define KEY_PIN_3 15  //пин подключения тактовой кнопки 1
#define KEY_PIN_4 14  //пин подключения тактовой кнопки 4

//#define INPUT INPUT_PULLUP  //раскомментировать, если у кнопок нет внешних подтягтвающих резисторов

/*ДОПОЛНИТЕЛЬНЫЕ НАСТРОЙКИ*/
#define BAUDRATE 9600 //скорость последовательного порта

struct Data_actuators 
{
  byte address = 0;
  byte command = 0;
  byte check_byte = 0; //байт проверки качества передачи данных
  byte connection = 0; //Побитовая кодировка дежурной информации:
  /*  0b [0][0][0][0][0][0][0][0]
   *      ^  ^  ^  ^  ^  ^  ^  ^
   *      A  M  С  M        Y  W
   *  A - Панель актуаторов
   *  M - Метеомодуль
   *  С - Панель управления
   *  M - Срабатывание датчика движения
   *  Y - флаг високосного года
   *  W - Warning!
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

class SystemTime  //класс обработчика системного времени
{
  private:
    volatile uint16_t millis_counter; //счетчик миллисекунд
    volatile uint8_t  minute;         //минута  (0..59)
    volatile uint8_t  hour;           //час     (0..23)
    volatile uint8_t  date;           //день месяца (1..max_date)
    volatile uint8_t  month;          //месяц (1..12)
    uint8_t           year;           //флаг високосного года (0/1)

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

const uint64_t pipes[6] = { //адреса ("трубы") обмена данными
  0xFFFFFFFFF0LL,  //адрес 0
  0xFFFFFFFFF1LL,  //адрес 1
  0xFFFFFFFFF2LL,  //адрес 2
  0xFFFFFFFFF3LL,  //адрес 3
  0xFFFFFFFFF4LL,  //адрес 4
  0xFFFFFFFFF5LL   //адрес 5
};

volatile unsigned long  _radio_free_interval    = 0;  
volatile unsigned long  _act_update_counter     = 0;  //счетчик времени обновления состояний исполнительных устройств

byte second = 0;
byte minute = 0;
byte hour   = 0;
byte date   = 0;
byte month  = 0;
bool year_flag = 0;
bool time_flag = 0;

const String line = "------------------";

/*-----------------------------------------------*/
/*СОЗДАНИЕ ОБЪЕКТОВ*/

RF24 radio(RF_CE_PIN, RF_CSN_PIN);  //создать объект "модуль радиосвязи RF24"

SystemTime real_time;

Data_actuators transmit_data; //объект отправляемых данных
Data_actuators received_data; //объект принимаемых данных
  
//Bounce debouncer[4];  //массив объектов "тактовая кнопка с защитой от дребезга"

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
  _radio_free_interval++; 
  _act_update_counter++; 
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

void(* resetFunc) (void) = 0; //resetFunc() - функция программного сброса системы методом перевода стека в нулевой адрес

void reboot() { //функция перезагрузки сиситемы
  wdt_disable(); //отключить сторожевой таймер
  wdt_enable(WDTO_15MS);  //установить сторожевой таймар на 15 мс
  while (1) {}  //бесконечный цикл - ждём перезагрузки через 15 мс
}

void setup() { 
  //настройка пинов подключения исполнителей как выход
  transmit_data.actuators_states = EEPROM.read(EEPROM_ACT_ADDR);
  //transmit_data.act_manual_flags = 0; //EEPROM.read(EEPROM_ACT_ADDR + 1);
  //EEPROM.write(EEPROM_ACT_ADDR + 1, 0);
  pinMode(ACT_PIN_1, OUTPUT); 
  digitalWrite(ACT_PIN_1, (transmit_data.actuators_states >> 0) & (1 << 0));
  pinMode(ACT_PIN_2, OUTPUT);
  digitalWrite(ACT_PIN_2, (transmit_data.actuators_states >> 1) & (1 << 0));
  pinMode(ACT_PIN_3, OUTPUT);
  digitalWrite(ACT_PIN_3, (transmit_data.actuators_states >> 2) & (1 << 0));
  pinMode(ACT_PIN_4, OUTPUT);
  digitalWrite(ACT_PIN_4, (transmit_data.actuators_states >> 3) & (1 << 0));

  //установка пинов подключения тактовых кнопок как вход
  //настройка защиты от дребезга
  /*pinMode(KEY_PIN_1, INPUT);
  debouncer[0].attach(KEY_PIN_1);
  debouncer[0].interval(5);
   
  pinMode(KEY_PIN_2, INPUT);
  debouncer[1].attach(KEY_PIN_2);
  debouncer[1].interval(5); 
  
  pinMode(KEY_PIN_3, INPUT);
  debouncer[2].attach(KEY_PIN_3);
  debouncer[2].interval(5);
  
  pinMode(KEY_PIN_4, INPUT);
  debouncer[3].attach(KEY_PIN_4);
  debouncer[3].interval(5);*/

  //конфигурирование таймеров
  timer1_Setup();
  wdt_disable();
  
  Serial.begin(BAUDRATE); //открыть серийный порт
  
  Serial.println(line);
  Serial.println(" --SMART  HOUSE--");
  Serial.println("-ACTUATORS MODULE-");
  Serial.println(line);

  //конфигурирование радиомодуля
  Serial.print("nRF-radio setup (required ~200 ms)... ");
  radio.begin();  //запуск модуля радио nRF24l01
  //настройки модуля nRF:
  delay(100); //задержка для запуска модуля
  radio.powerUp();  //включение полного питания молуря
  delay(100); //задержка для запуска модуля
  radio.setChannel(RADIO_CHANNEL); //канал передачи данных 
  radio.setRetries(15, 30); //косличество попыток отправки данных и временной интервал между попытками
  radio.setDataRate(RF24_1MBPS);  //установить скорость передачи данных 1 мБит/с
  radio.setPALevel(RF24_PA_MAX);  //установить максимальный уровень мощности передатчика
  radio.setCRCLength(RF24_CRC_16); 
  
  radio.setAutoAck(1);//разрешить приемнику отправлять подтверждение получения
  radio.enableDynamicPayloads();
  radio.enableAckPayload(); //включить режим передачи пользовательских данных при подтверждении получения
  //radio.enableDynamicAck(); //for ALL pipes? Чтобы можно было вкл\выкл получение ACK?      !!!!!!!!!!!!!!!!!!!!!!!!!!!!
  
  radio.openReadingPipe(RECEIVE_DATA_PIPE, pipes[READING_PIPE_NUM]); //открыть трубу для приема данных
//radio.openWritingPipe(pipes[WRITING_PIPE_NUM]);  //открыть трубу отправки даных
  radio.startListening(); //слушать эфир
  
  Serial.println("DONE");
  Serial.println(line);
  Serial.println("Delay 300ms...");
  delay(300);
  
  Serial.println(line);
  Serial.println("SETUP SUCCESSFULL!");
  Serial.println(line);

  real_time.begin();  //запуск счетчика системного времени
  real_time.getTime(second, minute, hour, date, month, year_flag, time_flag);
  

}

void loop() {
  //Serial.println("AAA");
  cli();
  unsigned long   act_update_counter = _act_update_counter; 
  real_time.getTime(second, minute, hour, date, month, year_flag, time_flag);
  
  static bool free_interval_flag = 0;
  static bool radio_listen_flag = 1;
  
  if ( free_interval_flag && _radio_free_interval > RF_NOTLISTEN_TIME )
  {
    //Serial.println("Reset");
    _radio_free_interval = 0;
    free_interval_flag = 0;
    radio_listen_flag = 1;
    radio.startListening();
  }
  
  sei();

  static byte act_have_changed = 0;
  transmit_data.moisure_sensor = map( analogRead(A7), 0, 1023, 0, 255 );
 
 /* for (byte i = 0; i < 4; i++)
  {
    debouncer[i].update();
    if (debouncer[i].fell())
    {
      transmit_data.act_manual_flags |= (1 << i);
      if (!((act_have_changed >> i) & (1 << 0)))
      {
        Serial.print(printTime(second, minute, hour, date, month));
        Serial.print("Manual changing #");
        Serial.print(i + 1);
        Serial.print(" power socket: ");
        if ((transmit_data.actuators_states >> i) & (1 << 0))
        {
          transmit_data.actuators_states &= ~(1 << i);
          Serial.println("Enable");
        } else {
          transmit_data.actuators_states |= (1 << i);
          Serial.println("Disable");
        }
        EEPROM.write(EEPROM_ACT_ADDR + 1, transmit_data.act_manual_flags);
        EEPROM.write(EEPROM_ACT_ADDR, transmit_data.actuators_states);
        act_have_changed |= (1 << i);   
      }
    }
  }*/
  
  if (act_update_counter >= ACK_UPDATE_INTERVAL)
  {
    act_have_changed = 0;
    static byte previous_states = transmit_data.actuators_states;
    if (previous_states != transmit_data.actuators_states)
    {
      Serial.print(printTime(second, minute, hour, date, month));
      Serial.println("Updating power sockets states...");
      digitalWrite(ACT_PIN_1, (transmit_data.actuators_states >> 0) & (1 << 0));
      digitalWrite(ACT_PIN_2, (transmit_data.actuators_states >> 1) & (1 << 0));
      digitalWrite(ACT_PIN_3, (transmit_data.actuators_states >> 2) & (1 << 0));
      digitalWrite(ACT_PIN_4, (transmit_data.actuators_states >> 3) & (1 << 0));
      previous_states = transmit_data.actuators_states;
      //EEPROM.write(EEPROM_ACT_ADDR, transmit_data.actuators_states);
    }
    cli();
    _act_update_counter = 0;
    sei();
  }
  
  //счетчик времени с момента последнего успешного приема данных
  static unsigned long display_connection_counter = millis();

  byte pipeNO = 255;
  if (  radio.available( &pipeNO ) )
    if ( pipeNO == 1 )
    {
      struct data_receive
      {
        byte address = 255;
        byte command = 255;
      } data;

      //Serial.println("Reading some data");
      radio.read(&data, sizeof(&data));
      
      if ( data.address == ADDRESS )
      {
        Serial.println("My address");
        
          radio.read(&received_data, sizeof(received_data));
          received_data.check_byte++;
          //Serial.println(
          transmit_data.check_byte = received_data.check_byte;
          if ( !(transmit_data.actuators_states == received_data.actuators_states) )
          {
            transmit_data = received_data;
            EEPROM.write(EEPROM_ACT_ADDR, transmit_data.actuators_states);
            Serial.println("Updating data");
          }
          transmit_data.address = ADDRESS;
          radio.writeAckPayload(pipeNO, &transmit_data, sizeof(transmit_data));
            
          display_connection_counter = millis();
          radio.flush_rx();        
        
      } else {

        //Serial.print("Not my addr ");
        radio.flush_rx();
        cli();
        _radio_free_interval = 0;
        sei();
        free_interval_flag = 1;
        
        if ( (!free_interval_flag) != radio_listen_flag)
        {
          //Serial.println(radio_listen_flag);
          radio_listen_flag = !free_interval_flag;
          //Serial.println("RST");
          if ( !radio_listen_flag )
          {
            //Serial.println("STOP");
            radio.stopListening();   
          }
        } 
      }
    } else {
      Serial.println("Not my pipe");
      radio.flush_rx();
    }

 
  /*if ( millis() - display_connection_counter >= RF_DATARESET_TIME ) //DISPLAY_CONNECTION_INTERVAL + RADIO_INTERVAL_MILLIS) //если досчитал до интервала
  {
    Serial.println(line);
    Serial.print(printTime(second, minute, hour, date, month));
    Serial.print("SENDING DATA TOTALLY FAILED: ");
    //Serial.print(printTime(second, minute, hour, date, month));
    Serial.println("RESETTING...");
    Serial.println(line);
    Serial.println();
    delay(100);
    //reboot();
    resetFunc();
  }*/
  
}
