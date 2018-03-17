/*-----------------------------------------------*/
/*ДИРЕКТИВА #include - ПОДКЛЮЧЕНИЕ ДОПОЛНИТЕЛЬНЫХ ФАЙЛОВ*/
#include <avr/wdt.h>  //для работы сторожевого таймера
#include <DHT.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Wire.h>              
#include <Adafruit_BMP085.h>

/*-----------------------------------------------*/
/*НАСТРОЙКИ*/

/*АЛГОРИТМ АНАЛИЗА АТМОСФЕРНОГО ДАВЛЕНИЯ ДЛЯ ПРОГНОЗА ПОГОДЫ*/
#define PRESSURE_READ_INTERVAL_MINUTES 10 //интервал снятий показаний давления (в минутах)
#define PRESSURE_INTERVAL_MILLIS (PRESSURE_READ_INTERVAL_MINUTES * 60000) //интервал снятий показаний давления (в мс) - НЕ РЕДАКТИРОВАТЬ
#define PRESSURE_READ_COUNT 9 //количество ячеек памяти для сохранения показаний
#define PRESSURE_FORECAST_HOURS 3 //время в часах, за которое высчитывается изменение давления, передаваемое на головное устройство

/*МОДУЛЬ ТЕМПЕРАТУРЫ И ВЛАЖНОСТИ DHTxx*/
#define DHT_PIN 14  //пин передачи данных
#define DHT_TYPE DHT21 //тип сенсора (DHT11 || DHT22 || DHT21)
#define DHT_TEMPERATURE_ERROR 0  //погрешность при измерении температуры

/*МОДУЛЬ 2.4GHz NRF24L01*/
#define RF_CE_PIN 9
#define RF_CSN_PIN 10
#define RADIO_CHANNEL 0x6A

#define READING_PIPE_NUM 0
#define WRITING_PIPE_NUM 1
//#define CENTRAL_BOARD_ADDRESS 0
#define RECEIVE_DATA_PIPE 1

#define ADDRESS 7

#define RF_DATARESET_TIME 6000 //время, в течение которого данные сохраняются, если связь потеряна (мс)
#define RF_NOTLISTEN_TIME 800
#define BOARD_REBOOT_TIME 60000 * 5

/*НАСТРОЙКИ ИНДИКАТОРНЫХ СВЕТОДИОДОВ*/
#define LED1_PIN 4  //пин подключения светодиода 1
#define LED2_PIN 2  //пин подключения светодиода 2
#define LED3_PIN 3  //пин подключения светодиода 3
#define LED_BLINK_ON_INTERVAL 100  //время активной фазы мигания светодиода (мс)
#define LED_BLINK_OFF_INTERVAL 5000 //время отключения светодиода
#define LED_WARNING_INTERVAL 1500

/*ДОПОЛНИТЕЛЬНЫЕ НАСТРОЙКИ*/
#define BAUDRATE 9600 //скорость последовательного порта
#define LDR_PIN A2  //пин подключения фоторезистора освещенностирадио
#define METEO_INTERVAL_MILLIS 40000 //интервал обновления атмосферных данных
#define ERROR_MSG_INTERVAL 10000 //интервал вывода сообщения об ошибке

struct Data_meteo //структура данных, генерируемых на метеомодуле
{
  byte address = 0;
  byte command = 0;
  byte check_byte = 0; //байт проверки качества передачи данных
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
  int  temperature = 0; //для передачи температуры на улице
  byte humidity = 0; //для передачи уровня влажносити на улице (относ.)
  long pressure = 0; //для передачи текущего значения атмосферного давления
  byte illumination = 0; //для передачи текущего уровня освещенности (0..100)
  byte forecast = 2; //для передачи прогноза погоды
  /* 0 - гарантированное улучшение погоды
     1 - возможно улучшение погоды
     2 - погода не изменится
     3 - возможно ухудшение погоды
     4 - гарантированно ухудшение погоды
  */
  int dPressure = 0; //тенденция изменения атмосферного давления за 3 часа
  bool forecast_flag = 0;
};

struct Data_time 
{
  byte address = 0;
  byte command = 0;
  byte check_byte = 0; //байт проверки качества передачи данных
  byte connection = 0; //Побитовая кодировка дежурной информации:
  /*  0b [0][0][0][0][0][0][0][0]
   *      ^  ^  ^  ^  ^  ^  ^  ^
   *      A  M  С           Y  W
   *  A - Панель актуаторов (>> 7)
   *  M - Метеомодуль (>> 6)
   *  С - Панель управления (>> 5)
   *  Y - флаг високосного года (>> 1)
   *  W - Warning! (>> 0)
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
    volatile uint16_t   millis_counter; //счетчик миллисекунд
    volatile uint8_t    minute;         //минута  (0..59)
    volatile uint8_t    hour;           //час     (0..23)
    volatile uint8_t    date;           //день месяца (1..max_date)
    volatile uint8_t    month;          //месяц (1..12)
    volatile bool       year;           //флаг високосного года (0/1)
    
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

class Pressure_Forecast
//класс для прогнозирования погоды по скорости изменения атмосфорного давления
{
private:
  unsigned long   *pressure = NULL; //указатель для динамического выделения памяти
  double          change_hour; //прогнозируемое изменение давления за 1 час
  byte            measurement_count; //количество измерений, сохраняемых в памяти для анализа
  double          measurement_interval; //интервал между измерениями в минутах
  byte            measurement_update_counter = 1;
  
  void calculate_change()   //расчет прогнозируемого изменения давления
  {
    //используя измерения давления, сохраненные в памяти
    //аппроксимация измерений методом наименьших квадратов
    byte sumx = 0;  
    unsigned long long  sumy = 0;
    unsigned int sumx2 = 0;
    unsigned long sumxy = 0;
    for (byte i = 0; i < measurement_count; i++) {
      sumx += i;
      sumy += pressure[i];
      sumx2 += i * i;
      sumxy += i * pressure[i];
    }
    double a;
    a = measurement_count * sumxy;
    a -= sumx * sumy;
    a = (double)a / (measurement_count * sumx2 - sumx * sumx);
    change_hour = a * 60 / measurement_interval; //прогнозируемое изменение давления за час
  }
  
public:
  void updatePressure(unsigned long _pressure)
  {
    for (int i = 0; i < measurement_count - 1; i++)
     pressure[i] = pressure[i + 1];
    pressure[measurement_count - 1] = _pressure;
    if (measurement_update_counter < measurement_count);
    measurement_update_counter++;
    calculate_change();
  }

  bool isForecast()
  {
    return (measurement_update_counter >= measurement_count);
  }
  double readPressure()
  {
    return pressure[measurement_count - 1];
  }
  
  double hrChangePressure(byte hours)
  {
    return change_hour * hours;
  }
  
  byte forecast()
  {
   double change = hrChangePressure(3);
    if (change > 600)
      return 0;
    if (change >= 170)
      return 1;
    if (change > -170)
      return 2;
    if (change > -600)
      return 3;
    return 4;
  }
  
  Pressure_Forecast(byte _measurement_count,
                    double _measurement_interval)
  {
    pressure = new unsigned long [_measurement_count];
    measurement_count = _measurement_count;
    measurement_interval = _measurement_interval;
  }

  void begin(unsigned long _pressure)
  {
    for (byte i = 0; i < measurement_count; i++)
      pressure[i] = _pressure;
  }
};

const uint64_t pipes[6] = { //адреса "труб" обмена данными
  0xFFFFFFFFF0LL,  //адрес 0
  0xFFFFFFFFF1LL,  //адрес 1
  0xFFFFFFFFF2LL,  //адрес 2
  0xFFFFFFFFF3LL,  //адрес 3
  0xFFFFFFFFF4LL,  //адрес 4
  0xFFFFFFFFF5LL   //адрес 5
};

const double            Pa_to_mmHg = 0.00750062; //коэффициент перевода паскалей в мм рт ст

volatile unsigned int   _led1_blink_counter       = 0;
volatile unsigned int   _led2_blink_counter       = 0;
volatile unsigned int   _led3_blink_counter       = 0;
volatile unsigned long  _meteo_update_counter     = 0;
volatile unsigned long  _system_reboot_counter    = 0;
volatile unsigned long  _pressure_update_counter  = 0;

volatile unsigned int _radio_free_interval = 0;

bool bmp_error_flag = 0;

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

RF24 radio(RF_CE_PIN, RF_CSN_PIN);

SystemTime real_time;

Data_meteo transmit_data;
Data_time received_data;

DHT dht(DHT_PIN, DHT_TYPE); //создаем объект dht - сенсор DHTxx

Adafruit_BMP085 bmp;

Pressure_Forecast meteo(PRESSURE_READ_COUNT, PRESSURE_READ_INTERVAL_MINUTES);

unsigned long bmpCorrectRead()
  {
    unsigned long long p = 0;
    for (byte i = 0; i < 15; i++)
      p += bmp.readPressure();
    return (unsigned long)(p / 15);
  }

int illuminationCorrectRead(byte pin)
{
  int illumination = 0;
  for (byte i = 0; i < 10; i++)
    illumination += analogRead(pin);
  return illumination / 10;
}

void meteoSerialWrite()
{
  Serial.print("-PRESSURE: ");
  Serial.print(transmit_data.pressure);
  Serial.println("mmHG");
  Serial.print("-HUMIDITY: ");
  Serial.print(transmit_data.humidity);
  Serial.println('%');
  Serial.print("-TEMPERATURE: ");
  if (transmit_data.temperature > 0)
    Serial.print('+');
  Serial.print(transmit_data.temperature);
  Serial.println("*C");
  Serial.print("-FORECAST: ");
  if (transmit_data.forecast_flag){
    if (transmit_data.dPressure > 0)
    Serial.print('+');
    Serial.print(transmit_data.dPressure);
    Serial.println("Pa/3h");
  } else 
    Serial.println("NULL");
  Serial.print("-ILLUMINATION: ");
  Serial.print(transmit_data.illumination);
  Serial.println('%');
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

void timer1_Setup() //настройки таймера 1
{
  TCCR1B |= (1 << 3); //установка режима таймера: сброс при совпадении
  OCR1A = 0xF9; //запись значения в регистр сравнения (249)
  TIMSK1 |= (1 << OCIE1A);  //разрешить прерывание при совпадении с регистром A (OCR1A)
  TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10); //усановить предделитель на 64
  sei();  //разрешаем прерывания
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

ISR(TIMER1_COMPA_vect)  //обработчик прерываний таймера 1 при совпадении с регистром A - вызывается раз в 1 мс
{
  _led1_blink_counter++;
  _led2_blink_counter++;
  _led3_blink_counter++;
  _meteo_update_counter++;
  _pressure_update_counter++;
  _system_reboot_counter++;
  _radio_free_interval++;
  real_time.Interrupt_func();
}

void reboot() {
  wdt_disable();  
  wdt_enable(WDTO_15MS);
  while (1) {}
}

void setup() {
  timer1_Setup();
  wdt_disable();
  
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);

  Serial.begin(BAUDRATE); //открыть серийный порт
  
  Serial.println(line);
  Serial.println(" --SMART  HOUSE--");
  Serial.println("  -METEO MODULE-");
  Serial.println(line);

  Serial.println("INDICATION TESTING:");
  Serial.println("-LED1...");
  digitalWrite(LED1_PIN, 1);
  delay(500);
  Serial.println("-LED2...");
  digitalWrite(LED1_PIN, 0);
  digitalWrite(LED2_PIN, 1);
  delay(500);
  Serial.println("-LED3...");
  digitalWrite(LED2_PIN, 0);
  digitalWrite(LED3_PIN, 1);
  delay(500);
  digitalWrite(LED3_PIN, 0);
  Serial.println(line);
  
  Serial.println("Sensors setup:");
  Serial.println("-DHTxx... NO STATUS");
  dht.begin();  //запуск модуля DHTxx
  Serial.print("-BMPxxx... ");
  if (bmp.begin())  //запуск баромента
    Serial.println("OK");
  else
  {
    Serial.println("ERR");
    bmp_error_flag = 1;
    
  }
  Serial.println(line);
  
  Serial.print("Forecast algorithm initialisation... ");
  if (!bmp_error_flag)
    meteo.begin(bmpCorrectRead());  //запуск алгоритма прогнозирования погоды
  
  Serial.println("DONE");
  Serial.println(line);
  
  Serial.print("nRF-radio setup... ");
  radio.begin();  //запуск модуля радио nRF24l01
  //настройки модуля nRF:
  delay(100);
  radio.powerUp();
  delay(100);
  radio.setChannel(RADIO_CHANNEL); //канал передачи данных 
  radio.setRetries(15, 15); //косличество попыток отправки данных и временной интервал между попытками
  radio.setDataRate(RF24_1MBPS);  //установить скорость передачи данных 1 мБит/с
  radio.setPALevel(RF24_PA_MAX);  //установить максимальный уровень мощности передатчика
  radio.setCRCLength(RF24_CRC_16); 
  
  radio.setAutoAck(1);//разрешить приемнику отправлять подтверждение получения
  radio.enableDynamicPayloads();
  radio.enableAckPayload(); //включить режим передачи пользовательских данных при подтверждении получения
  //radio.enableDynamicAck(); //for ALL pipes? Чтобы можно было вкл\выкл получение ACK?
  
  radio.openReadingPipe(RECEIVE_DATA_PIPE, pipes[READING_PIPE_NUM]);
 // radio.openWritingPipe(pipes[WRITING_PIPE_NUM]);  //открыть трубу отправки даных
  radio.startListening(); //слушать эфир
  
  Serial.println("DONE");
  Serial.println(line);
  Serial.println("Delay 300ms...");
  delay(300);
  Serial.println(line);
  
  
  
  Serial.println("Updating: ");
  if (!bmp_error_flag)
    transmit_data.pressure = round(meteo.readPressure() * Pa_to_mmHg);  //сохранить текущее давление для отправки на головное устройство
  else
    transmit_data.pressure = 0;
  transmit_data.humidity = round(dht.readHumidity());  //считать значение влажности и округлить до целых
  transmit_data.temperature = round(dht.readTemperature() + DHT_TEMPERATURE_ERROR);  //считать значение температуры и округлить до целых
  transmit_data.illumination = map(illuminationCorrectRead(LDR_PIN), 0, 1023, 0, 100);
  transmit_data.forecast = 2; //прогноз на 0
  transmit_data.dPressure = 0;  //обнулить изменение давления
  meteoSerialWrite();
  Serial.println(line);
  
  Serial.println("SETUP SUCCESSFULL!");
  Serial.println(line);
  Serial.println();
  
  real_time.begin();  //запуск счетчика системного времени
  real_time.getTime(second, minute, hour, date, month, year_flag, time_flag);
  transmit_data.address = ADDRESS;
  cli();
  _led1_blink_counter = LED_BLINK_OFF_INTERVAL;
  _led2_blink_counter = LED_BLINK_OFF_INTERVAL;
  _led3_blink_counter = LED_BLINK_OFF_INTERVAL;
  _meteo_update_counter = 0;
  _system_reboot_counter = 0;
  _pressure_update_counter = 0;
  sei();
}

void loop() {
  cli();
  unsigned int   led1_blink_counter = _led1_blink_counter;
  unsigned int   led2_blink_counter = _led2_blink_counter;
  unsigned int   led3_blink_counter = _led3_blink_counter;
  
  unsigned long   meteo_update_counter = _meteo_update_counter;
  unsigned long   system_reboot_counter = _system_reboot_counter;
  unsigned long   pressure_update_counter = _pressure_update_counter;

  static bool free_interval_flag = 0;
  static bool radio_listen_flag = 0;
  
  if ( free_interval_flag && _radio_free_interval > RF_NOTLISTEN_TIME )
  {
    _radio_free_interval = 0;
    free_interval_flag = 0;
    radio_listen_flag = 1;
    radio.startListening();
  }
  
  real_time.getTime(second, minute, hour, date, month, year_flag, time_flag);
  sei();
  
  ledBlink(LED1_PIN, led1_blink_counter, _led1_blink_counter, LED_BLINK_ON_INTERVAL, LED_BLINK_OFF_INTERVAL);
  
  if (pressure_update_counter >= PRESSURE_INTERVAL_MILLIS) //проверка времени обновения барометрических данных
  {
    Serial.print(printTime(second, minute, hour, date, month));
    Serial.print("FORECAST UPDATE... ");
    
    if (!bmp_error_flag)
    {
      meteo.updatePressure(bmpCorrectRead()); //сохранить новое значение атмосферного давления в память
      Serial.println("OK");
    } else {
      Serial.println("SENSOR ERROR");
    }
        
    cli();
    _pressure_update_counter = 0; //обновить счетчик
    sei(); 
  }

  if (meteo_update_counter >= METEO_INTERVAL_MILLIS)  //проверка времени обновления атмосферрных данных
  {
    Serial.print(printTime(second, minute, hour, date, month));
    Serial.println("METEO DATA:");
    
    transmit_data.humidity = round(dht.readHumidity());  //считать значение влажности и округлить до целых
    transmit_data.temperature = round(dht.readTemperature() + DHT_TEMPERATURE_ERROR);  //считать значение температуры и округлить до целых  
    transmit_data.forecast = meteo.forecast();  //считать значение прогноза погоды
    transmit_data.dPressure = round(meteo.hrChangePressure(PRESSURE_FORECAST_HOURS)); //считать значение прогноза изменения давления за 3 часа
    transmit_data.forecast_flag = meteo.isForecast();
    if (!bmp_error_flag)
      transmit_data.pressure = round(meteo.readPressure() * Pa_to_mmHg);  //считать атмосферное давление для отправки на головное устройство
    transmit_data.illumination = map(illuminationCorrectRead(LDR_PIN), 0, 1023, 0, 100);   //считать значение уровня освещенности
    
    cli();
    _meteo_update_counter = 0; //обновить счетчик
    sei();
    meteoSerialWrite(); //отправить данные в порт
  }

  static unsigned long display_connection_counter = millis();
  static bool connect_flag = 0;
  
  byte pipeNO = 255;
  if (  radio.available( &pipeNO ) )
    if ( pipeNO == RECEIVE_DATA_PIPE )
    {
      byte address = 255;
      radio.read(&address, sizeof(&address));
      
      if ( address == ADDRESS )
      {        
        radio.read(&received_data, sizeof(received_data));
        
        transmit_data.check_byte = received_data.check_byte + 1;
        
        radio.writeAckPayload(pipeNO, &transmit_data, sizeof(transmit_data));
        
        
        real_time.set( received_data.second, 
                     received_data.minute, 
                     received_data.hour, 
                     received_data.date, 
                     received_data.month, 
                     (received_data.connection >> 3) & (1 << 0) );
                     
        real_time.getTime(second, minute, hour, date, month, year_flag, time_flag);
        Serial.print(printTime(second, minute, hour, date, month));
        Serial.println("Connection success. System time: synchronized");
        
        display_connection_counter = millis();
        connect_flag = 1;
        
        cli();
        _system_reboot_counter = 0;
        sei();
        
        radio.flush_rx();
        
      } else {
        radio.flush_rx();
        cli();
        _radio_free_interval = 0;
        sei();
        free_interval_flag = 1;

        if ( (!free_interval_flag) != radio_listen_flag)
        {
          
          radio_listen_flag = !free_interval_flag;
          if ( !radio_listen_flag )
          {
            radio.stopListening();   
          }
        }
        
      }
    } else {
      Serial.println("Not my pipe");
      radio.flush_rx();
    }

  if ( millis() - display_connection_counter >= RF_DATARESET_TIME ) //DISPLAY_CONNECTION_INTERVAL + RADIO_INTERVAL_MILLIS) //если досчитал до интервала
  {
    connect_flag = 0; //флаг успешной передачи в 0
  }
  
  if (connect_flag)
  {  
    ledBlink(LED2_PIN, led2_blink_counter, _led2_blink_counter, LED_BLINK_ON_INTERVAL, LED_BLINK_OFF_INTERVAL);
  }
  else
  {
    ledBlink(LED2_PIN, led2_blink_counter, _led2_blink_counter, LED_WARNING_INTERVAL, LED_WARNING_INTERVAL);
  }

  transmit_data.connection |= (1 << 4) * bmp_error_flag;
  
  bool error_flag = 0;
  error_flag |= ((received_data.connection >> 2) & (1 << 0));

  static unsigned long error_msg_counter = millis();
  if (millis() - error_msg_counter >= ERROR_MSG_INTERVAL)
  {
    error_msg_counter = millis();
    if ( (transmit_data.connection >> 4) & (1 << 0) )
    {
      Serial.print(printTime(second, minute, hour, date, month));
      Serial.println("Interal system error(s):");
      if (bmp_error_flag)
        Serial.println("-BMPxxx sensor error - please reconnect and reboot module");
    }
    
    if ( (received_data.connection >> 2) & (1 << 0) )
    {
      Serial.print(printTime(second, minute, hour, date, month));
      Serial.println("Exteral system error");
    }
  }
  
  if (error_flag)
    ledBlink(LED3_PIN, led3_blink_counter, _led3_blink_counter, LED_WARNING_INTERVAL, LED_WARNING_INTERVAL);
  else
    ledBlink(LED3_PIN, led3_blink_counter, _led3_blink_counter, LED_BLINK_ON_INTERVAL, LED_BLINK_OFF_INTERVAL);
  
  if (system_reboot_counter >= BOARD_REBOOT_TIME)
  {
    Serial.println(line);
    Serial.print(printTime(second, minute, hour, date, month));
    Serial.print("SENDING DATA TOTALLY FAILED: ");
    Serial.println("REBOOTING...");
    Serial.println(line);
    Serial.println();
    delay(100);
    reboot();
  }
}
