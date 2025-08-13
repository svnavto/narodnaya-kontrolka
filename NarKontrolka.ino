

#include <avr/pgmspace.h>
#include <Bounce2.h>
#include <EEPROM.h>   // подклчаем библиотеку работы с епром
#include "U8g2lib.h"  // подключаем библиотеку работы с дисплеем
#include <PWM.h>    // подключаем библиотеку для ШИМ


// Номера пинов — замени на свои
#define BTN_OK    A0  //  sw1
#define BTN_UP    12  //E1
#define BTN_DOWN  A1  //E2
#define BUZZER_PIN A2 // пищалка
#define PIN_PWR_HOLD 6  // Пин для удержания питания
#define LONG_PRESS_MS  1000

// Другие пины
#define ANALOG_IN      A7  // вольтметр/осциллограф
#define FREQ_IN        2   // частотомер
#define GEN_NEG       10
#define GEN_POS        9
#define FLASH_PIN 8  // Пин фонарика
#define POLARITY_PIN 3          // D3 — вход от оптопары (переполюсовка)
#define PIN_MOSFET_CTRL    7   // Управление MOSFET отключение минусового щупа
#define PULL_PIN 4              // пин подтяжки к VCC
static bool pullOn = false;     // текущее состояние подтяжки (ON/OFF)



#define VOLT_SCALE 0.0332f  // Предрасчитано: 1.1 * 31.3 / 1024
#define DIODE_SCALE ((ADC_REF / 1024.0f / DIVIDER_K) * 1.7f)
// #define DIODE_SCALE (5.681f / 1000.0f)  // если ADC_REF = 1.1, делитель 300к/10к

void voltmetr();
void oscilograf();
void shastotomer();
void generator();
void can_lin_test();
void timers();
void dpkv();
void processButtons(); // Функция обработки длинных и коротких нажатий кнопок
void resetInactivityTimer();   //Функция сброса lastActivityTime при любом нажатии
void diode_test();     // Функция прозвонки цепи и диодов
void checkReversePolarity(); // Функция защиты от переполюсовки

// Глобальная переменная для отслеживания переполюсовки
volatile bool reverseDetected = false;
const unsigned long recoveryDelay = 2000;  // 2 секунды задержка восстановления

// Глобальные константы для всех функций делитель и нопорное напряжение
const float ADC_REF = 1.1;  // Напряжение питания АЦП
const float DIVIDER_K = 10.0 / 310.0;  // Делитель 300к / 10к 

// Глобальная переменная для включения подтяжки + в режиме диоды и режиме осцилограф
static bool pullUser   = false;  // что хочет пользователь (тумблер кнопкой)
static bool pullForced = false;  // режим "диоды" принудительно включает подтяжку
extern volatile bool overVoltageDetected;

// // Глобальные для осцилографа
// byte voltScaleMode = 0;  // 0: 5V, 1:15V, 2:25V, 3:35V, 4:AUTO
// bool oscAutoScale = false;
//float volt_in = 0.0;  // Глобальное напряжение, измеренное вольтметром
static float volt_in = 0.0f;
// Глобальная переменная для отслеживания времени активности
unsigned long lastActivityTime = 0;      // Время последнего действия
bool buzzerWarned = false;               // Флаг, что мы уже пикнули через 2 минуты
unsigned long StopTime = 0;

struct ButtonHandler {
  Bounce btn;
  void (*onShort)();    // короткое нажатие
  void (*onLong)();     // длинное нажатие
  unsigned long pressedAt = 0;
  bool longHandled = false;

  // вот этот конструктор нужен ↓↓↓
  ButtonHandler(Bounce b, void (*s)(), void (*l)()) {
    btn = b;
    onShort = s;
    onLong = l;
  }

  // обязательно пустой конструктор тоже, если вдруг будет использоваться
  ButtonHandler() {}
};



// Прототипы колбэков (можно позже реализовать)
void okShort();   void okLong();
void upShort();   void upLong();
void downShort(); void downLong();


// Объявление массива кнопок
ButtonHandler buttons[] = {
  { Bounce(), okShort,   okLong   }, //sw1
  { Bounce(), upShort,   upLong   }, //E1
  { Bounce(), downShort, downLong }  //E2
  
};

bool flashlightOn = false; // для вкл и вык фонарика
  // глобальные флаги Кнопки 
volatile bool flagBtnOk = false;    //sw1
// volatile bool flagLongOk = false;   //sw1
volatile bool flagBtnUp = false;    //E1
volatile bool flagLongUp = false;   //E1
volatile bool flagBtnDown = false;  //E2





// Заглушки (можно позже заменить на реальные)
void okShort()   { flagBtnOk = true;  } //  sw1
// Выключение устройства при долгом удержании кнопки  sw1
void okLong() { 
 
  if (millis() - StopTime < 5000)return;  // 🔒 Блокируем выключение первые 5 секунды
  tone(BUZZER_PIN, 300, 100);         // звуковой сигнал
  delay(150);                         // короткая задержка для подтверждения
  digitalWrite(PIN_PWR_HOLD, LOW);   // сброс питания
  delay(100);
  while (1);                          // ждём отключения
}
void upShort()   { flagBtnUp = true;  } //E1

void upLong() {flagLongUp = true; } //E1 ← именно этот флаг обрабатывается в voltmetr()


void downShort() { flagBtnDown = true; } //E2
// включение выключение фонарика
void downLong() { //E2
  flashlightOn = !flashlightOn; // переключаем
  digitalWrite(FLASH_PIN, flashlightOn ? HIGH : LOW);
  tone(BUZZER_PIN, flashlightOn ? 1000 : 500, 100); // звук при включении/выключении
 
}  

// Функция сброса lastActivityTime при любом нажатии ДЛЯ ЗАПУСКА ТАЙМЕРА АВТО-ОТКЛЮЧЕНИЯ
void resetInactivityTimer() {
  lastActivityTime = millis();
  buzzerWarned = false;  // Сброс предупреждения
}



// 1) Таймер и коррекция
unsigned long timer;     // для режима
uint8_t      t_offset;   // коррекция из EEPROM

// 2) Частотомер
int   Htime, Ltime;
float Ttime, rtime;
int   pwm, freq;

// 3) Генератор
struct GeneratorState {
  bool positive = false;
  uint16_t frequency = 30;
  uint8_t dutyPercent = 10;
} gen;



// 5) Таймер-импульс
byte n = 1;
byte g = 1;
byte jdem;
byte impyls;
byte led = GEN_NEG;  // GEN_NEG или GEN_POS


// Объекты
// 1) Дисплей
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);

// 2) Кнопки (Bounce2) переменные
Bounce btnUp, btnDown, btnOk; 


// 3) Состояния меню
enum Mode {
  MODE_VOLT, MODE_OSC, MODE_FREQ, MODE_GEN,
  MODE_CANLIN, MODE_TIMER, MODE_DPKV,
  MODE_DIODE,MODE_COUNT
};

// Подписи для каждого режима
// // Отдельные строки
const char str_volt[]     PROGMEM = "VOLTMETER";
const char str_osc[]      PROGMEM = "OSCILLOSCOPE";
const char str_freq[]     PROGMEM = "FREQUENCE";
const char str_gen[]      PROGMEM = "GENERATOR";
const char str_canlin[]   PROGMEM = "CAN LIN TEST";
const char str_timer[]    PROGMEM = "TIMER";
const char str_dpkv[]     PROGMEM = "CRANK SENSOR";
const char str_diode[]    PROGMEM = "DIODE TEST";

// Массив указателей
const char* const captions[] PROGMEM = {
  str_volt,
  str_osc,
  str_freq,
  str_gen,
  str_canlin,
  str_timer,
  str_dpkv,
  str_diode
};

// Стейты приложения
enum AppState { STATE_MENU, STATE_MODE };
static AppState appState = STATE_MENU;

// Текущий пункт меню
static Mode currentMode = MODE_VOLT;



void (*modeHandlers[MODE_COUNT])() = {
  voltmetr,
  oscilograf,
  shastotomer,
  generator,
  can_lin_test,
  timers,
  dpkv,
  diode_test 
};


// Функция обработки длинных и коротких нажатий кнопок
void processButtons() {
  for (auto &h : buttons) {
    h.btn.update();

    // 1) только что нажали ↓
    if (h.btn.fell()) {
      resetInactivityTimer();
      h.pressedAt    = millis();
      h.longHandled  = false;
    }

    // 2) держат кнопку и длинное ещё не отработало ↓
    if (h.btn.read() == LOW
        && !h.longHandled
        && millis() - h.pressedAt >= LONG_PRESS_MS) {
      h.onLong();
      h.longHandled = true;
    }

    // 3) отпустили ↓
    if (h.btn.rose()) {
      // если длинное не случилось — короткое
      if (!h.longHandled) {
        h.onShort();
      }
      h.pressedAt   = 0;
      h.longHandled = false;
    }
  }
}

// Функция отрисовки меню
// Рисует меню: кружки + выделенную подпись
void drawMenu(U8G2 &u8g2, uint8_t selected) {
  u8g2.firstPage();
  do {
    u8g2.clearBuffer();

    // Отрисовка кружков с шагом по ширине дисплея
    uint8_t count = MODE_COUNT;
    uint8_t step = u8g2.getDisplayWidth() / count;
    for (uint8_t i = 0; i < count; ++i) {
      uint8_t x = step / 2 + i * step;
      uint8_t y = 25;
      u8g2.drawCircle(x, y, 3, U8G2_DRAW_ALL);            // внешний круг
      if (i == selected) u8g2.drawDisc(x, y, 2, U8G2_DRAW_ALL); // внутренний заполненный
    }

    // Подпись режима — как в оригинале
    char buf[13];
    // strcpy_P(buf, captions[selected]);
    strcpy_P(buf, (const char*)pgm_read_word(&(captions[selected])));
    u8g2.setFont(u8g2_font_10x20_mr);
    u8g2.setCursor(0, 13);
    u8g2.print(buf);

  } while (u8g2.nextPage());
}


// Функция включения и выключения подтяжки к +
static inline void setPull(bool on) {
  if (on) {
    pinMode(PULL_PIN, OUTPUT);
    digitalWrite(PULL_PIN, HIGH);   // подтяжка к +V (через внешний 10k)
  } else {
    pinMode(PULL_PIN, INPUT);       // Hi-Z — полностью отцепились от линии
  }
  pullOn = on;
}

// Функция теста диодов и прозвонка
void diode_test() {
  static bool firstRun = true;
  static bool overVoltageDetected = false;

  if (firstRun) {
    delay(30);
    firstRun = false;
    // tone(BUZZER_PIN, 800, 200);  // звук при входе
  }

  if (!overVoltageDetected) {
    setPull(true); // подтяжка к VCC через 10k
    // pinMode(4, OUTPUT);
    // digitalWrite(4, HIGH);  // подтяжка к VCC через 10k
  }
   delay(10);
  // === Чтение напряжения ===
  long sum = 0;
  for (int i = 0; i < 20; ++i) {
    sum += analogRead(ANALOG_IN);
    delay(1);
  }
  int adc = sum / 20;
  // float rawVoltage = (adc * ADC_REF / 1024.0f) / DIVIDER_K;
  // float voltage = rawVoltage * 1.6;  // Поправка под твой диод
  float voltage = adc * DIODE_SCALE;


  // === Фильтрация свободного входа (плавающего) ===
  bool floatingInput = (voltage > 2.0f && voltage < 40.2f);
  if (floatingInput) {
    voltage = -1.0f;
  }

   // === Определение статуса ===
   const char* status = "WAIT";
   bool validReading = false;

   if (!overVoltageDetected) {
    if (voltage >= 0 && voltage < 0.05f) {
      status = "SHORT";
      tone(BUZZER_PIN, 1000);
      validReading = true;
    } else if (voltage >= 0 && voltage < 0.15f) {
      status = "DIODE";
      noTone(BUZZER_PIN);
      validReading = true;
    } else if (voltage >= 0 && voltage < 2.5f) {
      status = "OPEN";
      noTone(BUZZER_PIN);
      validReading = true;
    } else {
      noTone(BUZZER_PIN);
    }
  }

  // === Отрисовка ===
  u8g2.firstPage();
  do {
    u8g2.drawTriangle(110, 2, 110, 10, 116, 6);
    u8g2.drawVLine(118, 2, 9);
    u8g2.drawHLine(104, 6, 6);
    u8g2.drawHLine(118, 6, 4);


    u8g2.setFont(u8g2_font_logisoso20_tn);
   char voltageStr[10];
   if (voltage >= 0 && !overVoltageDetected) {
    dtostrf(voltage, 4, 2, voltageStr);  // Преобразуем в строку: 4 символа до точки, 2 после
     strcat(voltageStr, "V");
   } else {
   strcpy(voltageStr, "---");
   }
   int16_t x = (128 - u8g2.getStrWidth(voltageStr)) / 2;
   u8g2.setCursor(x, 30);  // вертикаль = baseline шрифта
   u8g2.print(voltageStr);


  } while (u8g2.nextPage());

  // === Выход по кнопке OK ===
  if (flagBtnOk) {
    flagBtnOk = false;
    pinMode(4, INPUT);
    tone(BUZZER_PIN, 600, 100);
    firstRun = true;
    setPull(false);
    overVoltageDetected = false;
    appState = STATE_MENU;
    drawMenu(u8g2, currentMode);
  }
}



bool loadState = false;

void voltmetr() {
  static float voltage = 0.0f;
  static bool holdMode = false;
  static float heldVoltage = 0.0f;


  // Отключаем ШИМ на D10 (если был)
  TCCR1A &= ~(_BV(COM1B1) | _BV(COM1B0));
  pinMode(GEN_NEG, OUTPUT);
  digitalWrite(GEN_NEG, loadState ? HIGH : LOW);

  // Управление нагрузкой по удержанию UP
  if (flagLongUp) {
    flagLongUp = false;
    loadState = !loadState;
    digitalWrite(GEN_NEG, loadState ? HIGH : LOW);
    tone(BUZZER_PIN, 850, 100);
  }


   if (!holdMode) {
  long sum = 0;
  for (int i = 0; i < 50; i++) sum += analogRead(ANALOG_IN);
  Htime = sum / 50;

  if (Htime >= Ltime + 2 || Htime <= Ltime - 2) Ltime = Htime;

  volt_in = VOLT_SCALE * Ltime;
  }

  // Отрисовка
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_10x20_mr);
    u8g2.setCursor(0, 13);
    u8g2.print("V");

    if (loadState) {
      u8g2.setCursor(0, 32);
      u8g2.print("N");
    }

    // Сигнал о просадке (например, ниже 1.5 В)
    if (volt_in >= 0.5f && volt_in <= 1.5f) {
      // tone(BUZZER_PIN, 700, 150);
    }

    // === Новый шрифт и выравнивание ===
    u8g2.setFont(u8g2_font_logisoso20_tn);
    char voltStr[10];
    dtostrf(volt_in, 4, 1, voltStr);  // формат XX.X
    strcat(voltStr, "V");
    int x = (128 - u8g2.getStrWidth(voltStr)) / 2;
    u8g2.setCursor(x, 32);
    u8g2.print(voltStr);

  } while (u8g2.nextPage());

  if (Ltime <= 3) Ltime = 0;

    // Короткое нажатие E1 (BTN_UP) включает/выключает HOLD
  if (flagBtnUp) {
    flagBtnUp = false;
    holdMode = !holdMode;
    if (holdMode) {
      heldVoltage = voltage;
      tone(BUZZER_PIN, 1200, 100); // сигнал HOLD ON
    } else {
      tone(BUZZER_PIN, 600, 100);  // сигнал HOLD OFF
    }
  }

  // Выход из режима
  if (flagBtnOk) {
    flagBtnOk = false;
    loadState = false;
    tone(BUZZER_PIN, 600, 100);
    digitalWrite(GEN_NEG, LOW);
    appState = STATE_MENU;
    drawMenu(u8g2, currentMode);
  }
}




//////////////////////////////////////////////////////
// Глобальные переменные
// 4) Осциллограф

int  m = 5;
int  w = 100;
// ==== Таймбаза осциллографа ====
uint8_t h = 1;  // 0..5, старт: 10 мс/точку
uint8_t i0_prev = 0; // был static внутри функции — вынеси в глобальные
#define OSC_N 32
#define NPER (sizeof(PERIODS_US)/sizeof(PERIODS_US[0]))
const uint32_t PERIODS_US[] = {

  // 10000,  // 10ms
  5000,   // 5ms
  2000,   // 2ms
  1000,   // 1ms
  500,    // 0.5ms
  200,    // 0.2ms
  100     // 0.1ms (быстро)
};
uint16_t oscData[OSC_N];
uint16_t oscPos = 0;
bool oscReady = false;
uint32_t lastSampleTime = 0;
uint32_t lastOscUpdate = 0;
const uint16_t MIN_SAMPLE_US = 120;


// Функция осциллограф
void oscilograf() {
  // вольтмеир
  long sum = 0;
  for (int i = 0; i < 10; i++) sum += analogRead(ANALOG_IN);
  volt_in = VOLT_SCALE * (sum / 10);

  // === 1. Настройка временной базы ===
  uint32_t time_per_division = PERIODS_US[h];  // Время на одно деление в микросекундах
  uint32_t total_time = time_per_division * 10; // Всего 10 делений на экране
  
  // Рассчитываем интервал между выборками (в микросекундах)
  uint32_t sample_interval = total_time / OSC_N;

   // не уходим в режимы быстрее железа
  if (sample_interval < MIN_SAMPLE_US) {
   // подберите ближайший h, при котором выдерживается лимит
   while (h < NPER-1 && (PERIODS_US[h]*10)/OSC_N < MIN_SAMPLE_US) h++;
  //  while (h > 0 && (PERIODS_US[h] * 10UL) / OSC_N < MIN_SAMPLE_US) h--;  // замедляемся
   time_per_division = PERIODS_US[h];
   total_time = time_per_division * 10;
   sample_interval = total_time / OSC_N;
  }

     // === 2. Сбор данных ===
  static uint32_t last_sample_time = 0;
  uint32_t current_time = micros();

  if (!oscReady && (current_time - last_sample_time >= sample_interval)) {
      oscData[oscPos] = analogRead(ANALOG_IN);
      oscPos++;
      last_sample_time = current_time;
    
      if (oscPos >= OSC_N) {
        oscPos = 0;
        oscReady = true;
    }
  }


  // === 3. Обработка и отображение ===
  if (oscReady) {
    // Находим min и max сигнала
    uint16_t minVal = 1023, maxVal = 0;
    for (uint16_t i = 0; i < OSC_N; i++) {
      if (oscData[i] < minVal) minVal = oscData[i];
      if (oscData[i] > maxVal) maxVal = oscData[i];
    }
    uint16_t range = max(maxVal - minVal, 50); // Минимальный диапазон 50
    
    // Автоматический триггер
    uint16_t trig_level = (maxVal + minVal) / 2;
    uint16_t trig_hyst = max(5, range / 20);
    int16_t trig_pos = -1;
    
    for (uint16_t i = 1; i < OSC_N; i++) {
      if (oscData[i-1] < (trig_level - trig_hyst) && 
          oscData[i] >= (trig_level + trig_hyst)) {
        trig_pos = i;
        break;
      }
    }
    
    uint16_t start_pos = (trig_pos > OSC_N/4) ? (trig_pos - OSC_N/4) : 0;

    // === Отрисовка ===
    u8g2.firstPage();
    do {
      uint8_t w = u8g2.getDisplayWidth();
      uint8_t h = u8g2.getDisplayHeight();
      

      for (uint8_t y = 0; y < h; y += h/5) {
        for (uint8_t x = 0; x < w; x += 8) {
          u8g2.drawPixel(x, y);
        }
      }
      
      // Сигнал
      for (uint16_t x = 1; x < w; x++) {
        uint16_t idx1 = start_pos + (x-1)*OSC_N/w;
        uint16_t idx2 = start_pos + x*OSC_N/w;
        
        idx1 = min(idx1, OSC_N-1);
        idx2 = min(idx2, OSC_N-1);
        
        uint8_t y1 = h - 1 - ((oscData[idx1] - minVal) * (h-1) / range);
        uint8_t y2 = h - 1 - ((oscData[idx2] - minVal) * (h-1) / range);
        
        u8g2.drawLine(x-1, y1, x, y2);
      }
      
      // Отображение параметров
      u8g2.setFont(u8g2_font_10x20_mr);
      u8g2.setCursor(0, 15);
      u8g2.print(volt_in, 1); u8g2.print("V");
      
      u8g2.setFont(u8g2_font_5x8_mr);
      u8g2.setCursor(0, 28);
      u8g2.print(time_per_division / 1000.0, 1); u8g2.print("ms");

      
    } while (u8g2.nextPage());
    
    oscReady = false;
  }

  // === 4. Управление временной базой ===
  if (flagBtnDown) {
    h = (h + 1) % NPER;
    oscPos = 0;
    oscReady = false;
    flagBtnDown = false;
    tone(BUZZER_PIN, 500, 100);
  }

   
  // Управление нагрузкой по удержанию UP 
  if (flagLongUp) {
    flagLongUp = false;
    setPull(!pullOn);                    // тумблер
    tone(BUZZER_PIN, pullOn ? 1000 : 500, 100);
  }

  if (flagBtnOk) {
    appState = STATE_MENU;
    tone(BUZZER_PIN, 600, 100);
    drawMenu(u8g2, currentMode);
    setPull(false);   // на выходе подтяжку гасим
    flagBtnOk = false;
  }
}




// Функция частотомер

void shastotomer() {
  static unsigned long lastCapture = 0;
  const unsigned long interval = 1000;  // мс

  unsigned long now = millis();
  if (now - lastCapture >= interval) {
    lastCapture = now;

    // ограничим ожидание 50 мс, чтобы не виснуть
    Htime = pulseIn(FREQ_IN, HIGH, 50000);
    Ltime = pulseIn(FREQ_IN, LOW, 50000);
    Ttime = Htime + Ltime;
    rtime = Ttime - Ltime;
    pwm = (Ttime > 0) ? int((rtime * 100.0f) / Ttime) : 0;
    freq = (Ttime > 0) ? int(1000000.0f / Ttime) : 0;
  }

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_10x20_mr);
    u8g2.setCursor(0, 13);
    u8g2.print("PWM: ");
    u8g2.print(pwm);
    u8g2.print("%");

    u8g2.setCursor(0, 32);
    u8g2.print("FREQ: ");
    u8g2.print(freq);
    u8g2.print("Hz");
  } while (u8g2.nextPage());

  // выход в меню по OK
  if (flagBtnOk) {
    flagBtnOk = false;
    appState = STATE_MENU;
    drawMenu(u8g2, currentMode);
    tone(BUZZER_PIN, 600, 100);
  }
}


// функция генератор
void generator() {
  // === 1) Долгое удержание Up — меняем полярность ===
  if (flagLongUp) {    
    delay(30);
    flagLongUp = false;
    gen.positive = !gen.positive;  // переключаем полярность
    tone(BUZZER_PIN, 850, 100);    // звук при входе
  }

  // === 2) Короткое нажатие OK — выход в меню ===
  if (flagBtnOk) {
    flagBtnOk = false;
    pwmWrite(GEN_POS, 0);  // отключить D9
    pwmWrite(GEN_NEG, 0);  // отключить D10
    appState = STATE_MENU;
    drawMenu(u8g2, currentMode);
    tone(BUZZER_PIN, 600, 100);  // звук при вЫходе
    return;
  }

  // === 3) Настройка скважности и частоты по кнопкам ===
  if (flagBtnUp) {
    gen.dutyPercent += 5;
    if (gen.dutyPercent > 100) gen.dutyPercent = 0;
    flagBtnUp = false;
    tone(BUZZER_PIN, 500, 100);
  }

  if (flagBtnDown) {
    gen.frequency += 100;
    if (gen.frequency > 2000) gen.frequency = 0;
    flagBtnDown = false;
    tone(BUZZER_PIN, 700, 100);
  }

  // === 4) Отрисовка экрана ===
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_10x20_mr);
    u8g2.setCursor(0, 13);
    u8g2.print(gen.dutyPercent);
    u8g2.print(" PWM ");
    u8g2.print(gen.positive ? "+" : "-");

    u8g2.setCursor(0, 32);
    u8g2.print(gen.frequency);
    u8g2.print(" Hz");
  } while (u8g2.nextPage());

  // === 5) Применяем параметры генерации ===
  uint8_t pin = gen.positive ? GEN_POS : GEN_NEG;
  uint8_t duty = map(gen.dutyPercent, 0, 100, 0, 255);

  // 💡 Отключаем оба пина — чтобы не остался старый активен
  pwmWrite(GEN_POS, 0);  // отключить D9
  pwmWrite(GEN_NEG, 0);  // отключить D10

  // Включаем только нужный
  pwmWrite(pin, duty);
  SetPinFrequency(pin, gen.frequency);
}



// // Функция CAN/LIN тестера с опорой 1.1 В и калибровкой в вольтах
void can_lin_test() {
  // 1) Периодически читаем вход (~200 мс)
  static unsigned long lastRead = 0;
  const unsigned long readInterval = 200;  // мс
  static int adc_raw = -1;                 // -1 = ещё не было замера

  unsigned long now = millis();
  if ((now - lastRead >= readInterval) || (adc_raw < 0)) { // принудительно первый замер
    lastRead = now;
    adc_raw = analogRead(ANALOG_IN);       // читаем СРАЗУ в adc_raw
  }

  if (adc_raw < 0) return;                 // теоретически не дойдём сюда

  float v = adc_raw * VOLT_SCALE;          // твой откалиброванный коэффициент

  // 2) Определяем режим
  const char* label = "<  N/A  >";
  if      (v >= 1.0f && v <= 1.8f)  label = "< CAN L >";
  else if (v >= 2.8f && v <= 3.8f)  label = "< CAN H >";
  else if (v >= 7.0f && v <= 18.0f) label = "<  LIN  >";
  else if (v <= 0.30f)              label = "< TEST  >";

  // 3) Экран
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_10x20_mr);
    u8g2.setCursor(0, 13);
    u8g2.print(F("CAN LIN TEST"));  // F() — чуть экономит SRAM
    u8g2.setCursor(0, 32);
    u8g2.print(label);
  } while (u8g2.nextPage());

  // выход в меню по OK
  if (flagBtnOk) {
    flagBtnOk = false;
    appState = STATE_MENU;
    drawMenu(u8g2, currentMode);
    tone(BUZZER_PIN, 600, 100);
  }
}




// Функция таймер

void timers() {
  delay(30);

  // === 0) Долгое удержание Up — меняем полярность, как в generator() ===
  if (flagLongUp) {
    flagLongUp = false;
    // led — это активный пин полярности (GEN_POS или GEN_NEG)
    led = (led == GEN_POS) ? GEN_NEG : GEN_POS;
    // выключим оба, чтобы не осталось старое состояние
    digitalWrite(GEN_POS, LOW);
    digitalWrite(GEN_NEG, LOW);
    tone(BUZZER_PIN, 850, 100);   // звук при переключении полярности
  }

  // === 1) Периодическое обновление каждую секунду ===
  static unsigned long lastUpdate = 0;
  const unsigned long interval = 1000;  // мс

  if (millis() - lastUpdate >= interval) {
    lastUpdate = millis();

    // 2) Если импульсы закончились, инициализируем подачу
    if (impyls == 0) {
      // на всякий случай гасим оба
      digitalWrite(GEN_POS, LOW);
      digitalWrite(GEN_NEG, LOW);
      jdem   = n + 1;
      impyls = g;
    }

    // 3) Рисуем экран
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_10x20_mr);
      u8g2.setCursor(0, 13);
      u8g2.print(g);
      u8g2.print(F(" IMP   "));
      u8g2.print(n);
      u8g2.print(F(" WAIT"));

      u8g2.setCursor(0, 32);
      if (impyls > 0 && jdem == (n + 1)) {
        // Фаза подачи импульсов
        u8g2.print((led == GEN_POS) ? F("+ IMPULS ") : F("- IMPULS "));
        u8g2.print(impyls);
      } else {
        // Фаза ожидания
        u8g2.print((led == GEN_POS) ? F("+ WAIT   ") : F("- WAIT   "));
        u8g2.print(jdem);
      }
    } while (u8g2.nextPage());

    // 4) Генерация сигнала и звук
    if (impyls > 0 && jdem == (n + 1)) {
      // подаём импульс на выбранной полярности
      tone(BUZZER_PIN, 500, 200);
      // гарантируем, что второй пин погашен
      if (led == GEN_POS) {
        digitalWrite(GEN_NEG, LOW);
        digitalWrite(GEN_POS, HIGH);
      } else {
        digitalWrite(GEN_POS, LOW);
        digitalWrite(GEN_NEG, HIGH);
      }
      impyls--;
    } else {
      // фаза ожидания
      tone(BUZZER_PIN, 900, 50);
      jdem--;
      // в ожидании оба пина LOW
      digitalWrite(GEN_POS, LOW);
      digitalWrite(GEN_NEG, LOW);
    }
  }

  // === Выход в меню по OK ===
  if (flagBtnOk) {
    flagBtnOk = false;
    // гасим оба выхода при выходе
    digitalWrite(GEN_POS, LOW);
    digitalWrite(GEN_NEG, LOW);
    appState = STATE_MENU;
    drawMenu(u8g2, currentMode);
    tone(BUZZER_PIN, 600, 100); // звук при выходе
  }
}


// Функция имитация датчика положения коленвала

enum DPKVState { DPKV_IDLE, DPKV_RUNNING };


void dpkv() {
    static DPKVState state = DPKV_IDLE;
    static unsigned long slotTimer = 0;
    static byte currentElement = 0;     // Текущий элемент (0..totalElements-1)
    static byte phase = 0;              // Фаза внутри элемента (0 или 1)
    static bool tuning = true;
    static byte editMode = 0;
    static byte i = 58, a = 2;          // ZUB и PR
    static int rpm = 1000;              // OB
    static unsigned long slotTime = 0;  // Время одного слота в микросекундах
    static byte totalElements = 0;      // Общее количество элементов

    // === Режим настройки ===
    if (tuning) {
        u8g2.firstPage();
        do {
            u8g2.setFont(u8g2_font_7x14_mr);
            u8g2.setCursor(0, 14);
            u8g2.print("ZUB:");
            u8g2.print(i);
            if (editMode == 0) u8g2.print("<");

            u8g2.setCursor(70, 14);
            u8g2.print("PR:");
            u8g2.print(a);
            if (editMode == 1) u8g2.print("<");

            u8g2.setCursor(0, 30);
            u8g2.print("OB:");
            u8g2.print(rpm);
            if (editMode == 2) u8g2.print("<");

        } while (u8g2.nextPage());

        // Обработка кнопок в режиме настройки
        if (flagBtnUp) {
            flagBtnUp = false;
            if (editMode == 0 && i < 127) i++;
            else if (editMode == 1 && a < 127) a++;
            else if (editMode == 2 && rpm < 7000) rpm += 100;
        }

        if (flagBtnDown) {
            flagBtnDown = false;
            if (editMode == 0 && i > 1) i--;
            else if (editMode == 1 && a > 0) a--;
            else if (editMode == 2 && rpm > 100) rpm -= 100;
        }

        if (flagBtnOk) {
            flagBtnOk = false;
            editMode++;
            if (editMode > 2) {
                tuning = false;
                state = DPKV_RUNNING;
                currentElement = 0;
                phase = 0;
                digitalWrite(GEN_POS, LOW);

                // ПРАВИЛЬНЫЙ РАСЧЕТ
                // Общее количество элементов (зубцы + пропуски)
                totalElements = i + a;
                
                // Время одного оборота в микросекундах
                unsigned long revolutionTime = 6000000000UL / rpm;
                
                // Каждый зубец имеет 2 слота (HIGH и LOW), каждый пропуск имеет 2 слота (LOW и LOW)
                // Таким образом, общее количество слотов: (i + a) * 2
                slotTime = revolutionTime / (totalElements * 2);
                
                // Защита от слишком маленьких значений
                if (slotTime < 50) slotTime = 50; // Минимум 50 мкс

                slotTimer = micros();
                tone(BUZZER_PIN, 1000, 150);
            }
        }
        return;
    }  

    // === Режим генерации ===
    if (state == DPKV_RUNNING) {
        // Обновляем дисплей
        u8g2.firstPage();
        do {
            u8g2.setFont(u8g2_font_7x14_mr);
            u8g2.setCursor(0, 14);
            u8g2.print("ZUB:");
            u8g2.print(i);
            u8g2.print(" PR:");
            u8g2.print(a);
            
            u8g2.setCursor(0, 30);
            u8g2.print("OB:");
            u8g2.print(rpm);
            u8g2.print(" E:");
            u8g2.print(currentElement);
            u8g2.print("/");
            u8g2.print(phase);
            
            u8g2.setCursor(0, 50);
            u8g2.print("SLOT:");
            u8g2.print(slotTime);
            u8g2.print("us");
            
        } while (u8g2.nextPage());

        // Генерация сигнала
        unsigned long currentMicros = micros();
        if (currentMicros - slotTimer >= slotTime) {
            slotTimer = currentMicros;

            // Определяем тип элемента
            bool isTooth = (currentElement < i);

            // Управление выходом
            if (isTooth) {
                // Для зубца: 1-я фаза - HIGH, 2-я фаза - LOW
                digitalWrite(GEN_POS, (phase == 0) ? HIGH : LOW);
            } else {
                // Для пропуска: обе фазы - LOW
                digitalWrite(GEN_POS, LOW);
            }

            // Переход к следующей фазе/элементу
            phase++;
            if (phase > 1) {
                phase = 0;
                currentElement++;
                if (currentElement >= totalElements) {
                    currentElement = 0;  // Начало нового оборота
                }
            }
        }
    }

    // === Обработка выхода из режима ===
    if (flagBtnOk) {
        flagBtnOk = false;
        digitalWrite(GEN_POS, LOW);
        state = DPKV_IDLE;
        tuning = true;
        editMode = 0;
        tone(BUZZER_PIN, 600, 100);
        appState = STATE_MENU;
        drawMenu(u8g2, currentMode);
    }
}







// Функция защиты от переполюсовки
void handleReversePolarity() {
  reverseDetected = true;
}

// Н А С Т Р О Й К А (сетап) //
void setup() { 

pinMode(PIN_PWR_HOLD, OUTPUT);
digitalWrite(PIN_PWR_HOLD, LOW);  // Сначала сбрасываем, на всякий случай

pinMode(BTN_OK, INPUT_PULLUP);
pinMode(BUZZER_PIN, OUTPUT);

// Подождём немного для стабилизации
delay(50);

// Если при включении кнопка нажата — включаем удержание питания
if (digitalRead(BTN_OK) == LOW) {
  delay(1000);  // ждём подтверждение
  if (digitalRead(BTN_OK) == LOW) {
    digitalWrite(PIN_PWR_HOLD, HIGH);  // Включить удержание
    tone(BUZZER_PIN, 800, 100); // звук при входе
  }
}


  // Ускорение АЦП (для Arduino UNO/NANO)
  
  ADCSRA = (ADCSRA & 0xF8) | 0x02; // Делитель 16 (~62.5 кГц)
  t_offset = EEPROM.read(0);
  // === Таймеры для ШИМ (библиотека SafeTimers) ===
  InitTimersSafe();
  // Оптимизация дисплея
  u8g2.setBusClock(400000); // Максимальная частота I2C

  // // === Настройка портов ===
  buttons[0].btn.attach(BTN_OK, INPUT_PULLUP);
  buttons[0].btn.interval(10);
  buttons[0].onShort = okShort;
  buttons[0].onLong  = okLong;

  buttons[1].btn.attach(BTN_UP, INPUT_PULLUP);
  buttons[1].btn.interval(10);
  buttons[1].onShort = upShort;
  buttons[1].onLong  = upLong;

  buttons[2].btn.attach(BTN_DOWN, INPUT_PULLUP);
  buttons[2].btn.interval(10);
  buttons[2].onShort = downShort;
  buttons[2].onLong  = downLong;

  // Инициализация фонарика
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(FLASH_PIN, LOW);
  // Инициализация пищалки
  pinMode(BUZZER_PIN, OUTPUT);
   // Инициализация выхода генератора +
  pinMode(GEN_NEG,   OUTPUT);
  digitalWrite(GEN_NEG, LOW);
  // Инициализация выхода генератора -
  pinMode(GEN_POS,   OUTPUT);
  digitalWrite(GEN_POS, LOW);

  analogReference(INTERNAL);

  // === Инициализация дисплея ===
  u8g2.begin();
  u8g2.setFont(u8g2_font_10x20_mr);

  // Стартовый экран
  u8g2.clearBuffer();
  u8g2.setCursor(20, 25);
  u8g2.print("SVSTARTER");
  u8g2.sendBuffer();
   delay(2000);
  // Мелодия при старте
  tone(BUZZER_PIN, 466, 186);
  delay(186);
  tone(BUZZER_PIN, 2794, 186);
  delay(186);
  noTone(BUZZER_PIN);

  // Отрисовка начального меню
  drawMenu(u8g2, currentMode);

  lastActivityTime = millis();  // Стартовое время активности тля таймера авто-отключения
  
  StopTime = millis(); // Стартовое время активности для задержки кнопки выключения

   // === Инициализация защиты от переполюсовки ===
  pinMode(POLARITY_PIN, INPUT_PULLUP);
  pinMode(PIN_MOSFET_CTRL, OUTPUT);
  digitalWrite(PIN_MOSFET_CTRL, HIGH); // старт включаем мосфет минусового щупа

  // Прерывание на переполюсовку (LOW с оптопары = ошибка)
 attachInterrupt(digitalPinToInterrupt(POLARITY_PIN), handleReversePolarity, FALLING);
 setPull(false);   // стартуем с выключенной подтяжкойк +VCC
 

}

void loop() {

  if (reverseDetected) {
  digitalWrite(PIN_MOSFET_CTRL, LOW);  // моментально отключить
    if (millis() - StopTime > 3000) {
    tone(BUZZER_PIN, 900);  // пищим только если прошло 2 секунды
  }
  while (digitalRead(POLARITY_PIN) == LOW);  // ждать, пока восстановится
  delay(recoveryDelay);                     // задержка восстановления
  digitalWrite(PIN_MOSFET_CTRL, HIGH);      // восстановить питание
  noTone(BUZZER_PIN);
  reverseDetected = false;
}

  
  processButtons(); // функция обработки кнопок

     // === Проверка неактивности ===
   unsigned long now = millis();
   if (now - lastActivityTime >= 3 * 60 * 1000UL && !buzzerWarned) {
     tone(BUZZER_PIN, 800, 150); // Предупредительный пик
     buzzerWarned = true;
   }
   if (now - lastActivityTime >= 5 * 60 * 1000UL) {
    // Выключаем питание
    digitalWrite(PIN_PWR_HOLD, LOW);
    delay(100);
    while (1);  // Ждём отключения
   }  
    

     // Управление меню
    if (appState == STATE_MENU) {
    if (flagBtnUp) {
      currentMode = Mode((currentMode + MODE_COUNT - 1) % MODE_COUNT);
      drawMenu(u8g2, currentMode);
      tone(BUZZER_PIN, 500, 100);
      resetInactivityTimer(); //  Проверка неактивности
      flagBtnUp = false;
    }

    if (flagBtnDown) {
      currentMode = Mode((currentMode + 1) % MODE_COUNT);
      drawMenu(u8g2, currentMode);
      tone(BUZZER_PIN, 500, 100);
      resetInactivityTimer(); //  Проверка неактивности
      flagBtnDown = false;
    }

    if (flagBtnOk) {
      resetInactivityTimer(); //  Проверка неактивности
      appState = STATE_MODE;
      tone(BUZZER_PIN, 1000, 200);
      flagBtnOk = false;
    }

    return;
  }

   switch (currentMode) {
     case MODE_VOLT:    voltmetr();       break;
     case MODE_OSC:     oscilograf();     break;
     case MODE_FREQ:    shastotomer();    break;
     case MODE_GEN:     generator();      break;
     case MODE_CANLIN:  can_lin_test();   break;
     case MODE_TIMER:   timers();         break;
     case MODE_DPKV:    dpkv();           break;
     case MODE_DIODE:   diode_test();     break;
     default: break;
   }

   if (flagBtnOk) {
    appState = STATE_MENU;
    drawMenu(u8g2, currentMode);
    tone(BUZZER_PIN, 600, 100);
    flagBtnOk = false;
   }

}









