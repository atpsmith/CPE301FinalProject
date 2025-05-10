//CPE 301 Final Project
//Aiden Smith

//Libraries
#include <DHT11.h>
#include <LiquidCrystal.h>
#include <Stepper.h>

//UART pointers
volatile unsigned char *myUCSR0A  = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B  = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C  = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0   = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0    = (unsigned char *)0x00C6;
#define RDA 0x80
#define TBE 0x20

//Timer1 registers
volatile unsigned char *myTCCR1A = (unsigned char*) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char*) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char*) 0x82;
volatile unsigned int  *myTCNT1  = (unsigned int*)  0x84;
volatile unsigned char *myTIFR1  = (unsigned char*) 0x36;

//System definitions
#define switchOFF 0
#define switchON 1
#define tempThreshold 24
#define waterThreshold 200
#define stepsPerRevolution 1000
#define motorRPM 10

//Pin definitions
#define waterSensorPin A0
#define potPin A15
#define buttonPin 7      //PH4
#define resetButtonPin 8 //PH5
#define disabledPinPower A1  //PF1
#define idlePinPower A2      //PF2
#define runningPinPower A3   //PF3
#define errorPinPower A4     //PF4

//Objects
DHT11 dht11(2);
LiquidCrystal lcd(4, 3, 44, 45, 46, 47);
Stepper myStepper(stepsPerRevolution, 14, 15, 16, 17);

//Port pointers
volatile unsigned char *portB = (unsigned char *)0x25;
volatile unsigned char *ddrB  = (unsigned char *)0x24;
volatile unsigned char *ddrK  = (unsigned char *)0x107;
volatile unsigned char *portK = (unsigned char *)0x108;
volatile unsigned char *portF = (unsigned char *)0x31;
volatile unsigned char *ddrF  = (unsigned char *)0x30;
volatile unsigned char *portH = (unsigned char *)0x102;
volatile unsigned char *ddrH  = (unsigned char *)0x101;
volatile unsigned char *pinH  = (unsigned char *)0x100;

//ADC registers
volatile unsigned char *my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char *my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char *my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int *my_ADC_DATA = (unsigned int*) 0x78;

//Variables
int temp = 0;
int humid = 0;
int previousPotVal = 0;
int currentPotVal = 0;
int switchState = switchOFF;
int systemEnabled = 0;
bool inErrorState = false;

void timer1Delay(unsigned int ms) {
  *myTCCR1A = 0x00;  //Normal mode
  *myTCCR1B = 0x05;  //Prescaler 1024
  
  while(ms > 0) {
    *myTCNT1 = 64536;  //65536 - 1000 (1ms)
    while((*myTIFR1 & (1 << 0)) == 0);
    *myTIFR1 |= (1 << 0);
    ms--;
  }
  *myTCCR1B = 0x00;  //Stop timer
}

void U0init(int U0baud) {
  unsigned long FCPU = 16000000;
  unsigned int tbaud = (FCPU / 16 / U0baud - 1);
  *myUBRR0 = tbaud;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
}

unsigned int readPotentiometer() {
  *my_ADMUX = 0x47;       //ADC15 with AVCC reference
  *my_ADCSRB |= (1 << 3); //MUX5 for ADC15
  *my_ADCSRA |= (1 << 6); //Start conversion
  while (*my_ADCSRA & (1 << 6));
  return *my_ADC_DATA;
}

void adc_init() {
  *my_ADCSRA = 0x87; //Enable ADC, prescaler 128
  *my_ADCSRB = 0x00;
  *my_ADMUX = 0x40;  //AVCC reference, right adjust
}

void printString(const char *str) {
  while(*str) U0putchar(*str++);
}

void U0putchar(unsigned char c) {
  while(!(*myUCSR0A & TBE));
  *myUDR0 = c;
}

void printInt(int num) {
  char buffer[10];
  itoa(num, buffer, 10);
  printString(buffer);
}

void println() {
  U0putchar('\r');
  U0putchar('\n');
}

void setup() {
  U0init(9600);
  
  //Configure pins
  *ddrH &= ~((1 << 4) | (1 << 5));
  *portH &= ~((1 << 4) | (1 << 5));
  
  *ddrF |= (0x0F << 1);
  *portF &= ~(0x0F << 1);
  
  *ddrB |= (1 << 4);
  *portB &= ~(1 << 4);
  
  *ddrK &= ~(1 << 7);
  *portK &= ~(1 << 7);

  lcd.begin(16, 2);
  timer1Delay(500);
  lcd.clear();

  myStepper.setSpeed(motorRPM);
  previousPotVal = readPotentiometer();
  adc_init();
  timer1Delay(500);
}

void loop() {
  currentPotVal = readPotentiometer();
  bool resetPressed = (*pinH & (1 << 5)) != 0;
  
  //Check on/off button
  switchState = (*pinH & (1 << 4)) != 0;
  if(switchState) {
    timer1Delay(50);
    if((*pinH & (1 << 4)) != 0) {
      systemEnabled = !systemEnabled;
      if(systemEnabled && inErrorState) {
        if(adc_read(0) >= waterThreshold) {
          inErrorState = false;
        }
      }
      while((*pinH & (1 << 4)) != 0);
    }
  }

  //Reset Button
  if(resetPressed && inErrorState) {
    if(adc_read(0) >= waterThreshold) {
      inErrorState = false;
      *portF &= ~(1 << 4);
    }
    timer1Delay(50);
    while((*pinH & (1 << 5)) != 0);
  }

  if(inErrorState) {
    previousPotVal = currentPotVal;
    timer1Delay(1000);
    return;
  }

  //Read temp and humidity
  int temperature, humidity;
  if(dht11.readTemperatureHumidity(temperature, humidity) == 0) {
    temp = temperature;
    humid = humidity;
    printString("Temp:");
    printInt(temp);
    printString(", Humidity:");
    printInt(humid);
    println();
  }

  //Reading water level
  unsigned int waterLevel = adc_read(0);
  printString("Water Level:");
  printInt(waterLevel);
  println();

  //LCD stuff
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temp);
  lcd.print(" C");
  lcd.setCursor(0, 1);
  lcd.print("Humid: ");
  lcd.print(humid);
  lcd.print(" %");

  if(systemEnabled) {
    if(waterLevel < waterThreshold) {
      //Error State
      printString("ERROR: Water Low");
      println();
      *portF = (*portF & ~0x0E) | (1 << 4);
      *portB &= ~(1 << 4); //Fan off
      previousPotVal = currentPotVal;
      inErrorState = true;
      return;
    }
    else if(temp > tempThreshold) {
      //Running State
      *portF = (1 << 3);
      *portB |= (1 << 4);
    }
    else {
      //Idle State
      *portF = (1 << 2);
      *portB &= ~(1 << 4);
    }
    //Vent adjustment
    myStepper.step(currentPotVal - previousPotVal);
    previousPotVal = currentPotVal;
  }
  //System Disabled
  else {
    *portF = (1 << 1); //Disabled LED on, others off
    *portB &= ~(1 << 4); //Fan off
  }

  timer1Delay(500);
}

unsigned int adc_read(unsigned char ch) {
  *my_ADMUX = 0x40 | (ch & 0x07);
  *my_ADCSRB = (ch & 0x08) ? (1 << 3) : 0;
  *my_ADCSRA |= (1 << 6);
  while(*my_ADCSRA & (1 << 6));
  return *my_ADC_DATA;
}