#include <avr/pgmspace.h> //Библиотека записывает данные не в SRAM, а во flash-память 
#include <util/delay.h>

#include <Adafruit_RGBLCDShield.h>
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

#define buzzerPin 2
int frequency = 0;
int duration = 0;

#include <AS5048A.h>
AS5048A angleSensor(SS);

#define ENC_LINE_PER_REV     360     // Кол-во линий энкодера на 1 оборот шпинделя
#define MOTOR_Z_STEP_PER_REV 200      // Кол-во шагов на оборот винта Z, продольная
//#define SCREW_Z              150      // Шаг продольного винта Z в сотках, 1.5мм
//#define McSTEP_Z             4        // Микрошаг, ось Z, продольная

// ***** Stepper Motor *****
//Настройка разрядов порта регистра направления порта DDRL на 0-вход или 1-выход.
//Устанавливаем режим состояния портов настроиных как выходы в состояние на выходе лог.1
#define Motor_Init()            DDRL |= B11111111;\
  PORTL |= B11111111;

#define Motor_Z_SetPulse()     PORTL &= ~(1<<0) // Устаналиваем в 0 бит Pin49
#define Motor_Z_RemovePulse()  PORTL |= (1<<0)  // Устаналиваем в 1 бит  Pin49
#define Motor_Z_InvertPulse()  PORTL ^= (1<<0)  // Меняем знычение определённого бита Pin49
#define Read_Z_State           (PINL & (1<<0))  // !!(PINL & (1 << 0))

#define Motor_Z_CW()           PORTL &= ~(1<<6)    //Тактирующий сигнал, сигнал шага по часовой стрелке Pin43 0
#define Motor_Z_CCW()          PORTL |= (1<<6)     //Тактирующий сигнал, сигнал шага против часовой стрелки Pin43 1

#define Motor_Z_Enable()   do {PORTL |= (1<<4); _delay_ms(120);} while(0)   // Потенциальный сигнал, сигнал включения драйвера Pin45 1
// и делаем задержку для инициализации драйвера
#define Motor_Z_Disable()      PORTL &= ~(1<<4)                             // Потенциальный сигнал, сигнал выключения драйвера Pin45 0
#define Read_Z_Ena_State       (PINL & (1<<4)) //Проверка состояния сигнала ENABLE, сигнал включения/выключения драйвера

// ***** MY CONSTANT *****
#define CW               0
#define CCW              1

#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

uint8_t FlagReadSPI = 0;
uint16_t val;
union {
  uint16_t val;
  struct {
    uint8_t lsb;
    uint8_t msb;
  };
} in, out;

void setup() {
  // put your setup code here, to run once:
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);
  SPI_Init();
  sei();
  //PORTB &= ~(1 << PB0); //Установить "0" на линии SS
  in.val = 0b1111111111111111;
  SPDR = 0;
  //PORTB |= (1 << PB0);//Установить "1" на линии SS
  //angleSensor.init();
}

void SPI_Init(void) {
  DDRB |= (1 << 0) | (1 << 1) | (1 << 2);//Настроить выводы SS, SCK, MOSI на выход
  DDRB &= ~(1 << 3);//Настроить вывод MISO на вход
  PORTB |= (1 << PB0);//Установить "1" на линии SS
  SPCR = 0; //Обнулить управляющий регистр SPCR
  SPSR = 0; //Обнулить статусный регистр SPSR
  SPCR |= (1 << SPIE) | (0 << SPE) | (0 << DORD) | (1 << MSTR) | (0 << CPOL) | (1 << CPHA) | (1 << SPR1) | (1 << SPR0);
  /*Режим мастер, F = Fosc / 4
    SPIE - разрешает / запрещает прерывания от модуля SPI.
    SPE – включает / выключает модуль SPI. Если бит установлен в 1, модуль SPI включен.
    DORD – определяет порядок передачи данных. Когда бит установлен в 1, содержимое регистра данных передается младшим битом вперед. Когда бит сброшен, то старшим битом вперед.
    MSTR – определяет режим работы микроконтроллера. Если бит установлен в 1, микроконтроллер работает в режиме Master (ведущий).
    CPOL и CPHA – определяют в каком режиме работает SPI модуль. Требуемый режим работы зависит от используемого периферийного устройства.
    SPI Mode  CPOL  CPHA      Образец
      0         0     0    Ведущий (Восходящий) Край
      1         0     1    Задний (падающий) край
      2         1     0    Ведущий (падающий) край
      3         1     1    Трейлинг (восходящий) край
    SPR1 и SPR0 – определяют частоту тактового сигнала SPI модуля, то есть скорость обмена.
    ~SPI2X SPR1 SPR0  Freq
      1    0     0   fosc/2
      0    0     0   fosc/4
      1    0     1   fosc/8
      0    0     1   fosc/16
      1    1     0   fosc/32
      1    1     1   fosc/64
      0    1     0   fosc/64
      0    1     1   fosc/128
    Максимально возможная скорость обмена всегда указывается в спецификации периферийного устройства.*/

  SPSR |= (0 << SPIF) | (0 << WCOL) | (0 << SPI2X);
  /* F=Fosc/2
    SPIF – флаг прерывания от SPI. Он устанавливается в 1 по окончании передачи байта данных. Если разрешены прерывания модуля
    WCOL- флаг конфликта записи. Флаг устанавливается в 1, если во время передачи данных выполняется попытка записи в регистр данных SPDR.
    SPI2X — бит удвоения скорости обмена. Установка этого разряда в 1 удваивает частоту тактового сигнала SCK.*/
  SPCR |= (1 << SPE); //Включить SPI модуль

}

float RotationRawToAngle(uint16_t DiscreteCode){
  return (DiscreteCode & ~0xC000) * (360.0 / float(0x3FFF));
}

float GetAngularMinutes (float AngleAbsolute){
  return ( AngleAbsolute - int(AngleAbsolute) ) * 60;
}

float GetAngularSeconds (float AngleAbsolute){
  return (GetAngularMinutes(AngleAbsolute) - int(GetAngularMinutes(AngleAbsolute)) ) * 60;
}

inline static uint16_t transfer16(uint16_t data) {

  in.val = data;

  if (!(SPCR & (1 << DORD))) {
    SPDR = in.msb;
    asm volatile("nop"); // See transfer(uint8_t) function
    while (!(SPSR & (1 << SPIF))) ;
    out.msb = SPDR;
    SPDR = in.lsb;
    asm volatile("nop");
    while (!(SPSR & (1 << SPIF))) ;
    out.lsb = SPDR;
  } else {
    SPDR = in.lsb;
    asm volatile("nop");
    while (!(SPSR & (1 << SPIF))) ;
    out.lsb = SPDR;
    SPDR = in.msb;
    asm volatile("nop");
    while (!(SPSR & (1 << SPIF))) ;
    out.msb = SPDR;
  }
  return out.val & ~0xC000;
}

ISR (SPI_STC_vect)
{
  if (!(SPCR & (1 << DORD))) {
    switch (FlagReadSPI) {
      case 0:
        FlagReadSPI = 1;
        PORTB &= ~(1 << PB0); //Установить "0" на линии SS
        SPDR = in.msb;
        //asm volatile("nop"); // See transfer(uint8_t) function
        break;
      case 1:
        //while (!(SPSR & (1 << SPIF))) ;
        out.msb = SPDR;
        FlagReadSPI = 2;
        SPDR = in.lsb;
        //asm volatile("nop");
        break;
      case 2:
        //while (!(SPSR & (1 << SPIF))) ;
        out.lsb = SPDR;
        PORTB |= (1 << PB0);//Установить "1" на линии SS
        val = out.val;
        FlagReadSPI = 1;
        PORTB &= ~(1 << PB0); //Установить "0" на линии SS
        SPDR = in.msb;
        //asm volatile("nop");
        break;
    }
  } else {
    switch (FlagReadSPI) {
      case 0:
        FlagReadSPI = 1;
        PORTB &= ~(1 << PB0); //Установить "0" на линии SS
        SPDR = in.lsb;
        //asm volatile("nop");
        break;
      case 1:
        //while (!(SPSR & (1 << SPIF))) ;
        out.lsb = SPDR;
        FlagReadSPI = 2;
        SPDR = in.msb;
        //asm volatile("nop");
        break;
      case 2:
        // while (!(SPSR & (1 << SPIF))) ;
        out.msb = SPDR;
        PORTB |= (1 << PB0);//Установить "1" на линии SS
        val = out.val;
        FlagReadSPI = 0;
        PORTB &= ~(1 << PB0); //Установить "0" на линии SS
        SPDR = in.lsb;
        //asm volatile("nop");
        break;
    }
  }

  //return ;
}

void loop() {
  // put your main code here, to run repeatedly:
  //uint8_t buttons = lcd.readButtons();
  lcd.clear();
  lcd.setCursor(0, 0);


  //if (tachoReadyFlag == true)
  //{
  //tachoReadyFlag = false;

  // while (1)
  //{
  //PORTB &= ~(1 << PB0); //Установить "0" на линии SS

  //transfer16(0b1111111111111111);
  //PORTB |= (1 << PB0);//Установить "1" на линии SS
  //delay(1);
  //}


  //}


  switch (lcd.readButtons()) {

    case BUTTON_UP:
      lcd.print("UP ");
      lcd.setBacklight(RED);
      break;

    case BUTTON_DOWN:
      lcd.print("DOWN ");
      lcd.setBacklight(BLUE);
      cli();
      sei();
      break;

    case  BUTTON_LEFT:
      lcd.print("LEFT ");
      lcd.setBacklight(GREEN);
      break;

    case BUTTON_RIGHT:
      lcd.print("RIGHT ");
      lcd.setBacklight(TEAL);
      break;

    case BUTTON_SELECT:
      lcd.print("SELECT ");
      lcd.setBacklight(VIOLET);
      break;

    default:

      lcd.setCursor(0, 0);
      //cli();
      lcd.print(RotationRawToAngle(out.val));
      //sei();
      lcd.setCursor(0, 1);
      
      lcd.print("SPI_STC_vect");
  }

}
