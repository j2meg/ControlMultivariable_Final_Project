// ESP32 Arduino IDE
/*
This program uses an ESP32 DEV module to see the delay an analog input and a DAC output.
The update of the input and output is driven by an arbitrary sample time defined by a hardware timer. 

Materials list. 
ESP32 DEV module
MCP4725 (12 bits I2C DAC)
*** Use an external osciloscope and wave generator to view the delay between ADC input and DAC 
The recommended signal from the wave generator to asses the system is a 20 hertz square signal
with 70% of duty cycle, 1 volt of amplitude and 1 volt of offset. 
Note: ESP32 ADC inputs only support voltages between 0 and 3.3 volts. Don't exceed these thresholds. 

Connections:
ESP32 Pins         | MCP4725 Pins 
    GND            |   GND
    3.3V           |   VCC
    GPIO 21        |   SDA
    GPIO 22        |   SCL
    GND            |   GND
    GPIO34 (ADC0)  |  wave generator | Osciloscope C2
                   |   Out            Osciloscope Ch1
*/
#include <Wire.h>//Include the Wire library to talk I2C
//This is the I2C Address of the MCP4725, by default (A0 pulled to GND).
//Please note that this breakout is for the MCP4725A0. 
#define MCP4725_ADDR 0x60   

// ESP32 PIN DISTRIBUTION
#define ESP_SCL_1_PIN 22 // Pin GPIO 22 of ESP32 as scl
#define ESP_SDA_1_PIN 21 // Pin GPIO 21 of ESP32 as sda
#define ADC0_PIN 34 //Pin GPIO34 is recommended as first adc port

hw_timer_t *timer = NULL;      // timer pointer declaration


// Variable definition
const int analogInPin = ADC0_PIN;
volatile uint adc_0_lecture=0;
volatile bool interruption_flag = false;
volatile uint Out =0;

// ISR del timer
void IRAM_ATTR onTimer()
{
interruption_flag = true;
}
void setup()
{
    //
   // Serial.begin(115200); // For easy debug
    // I2C CONFIGURATION
    Wire.begin(ESP_SDA_1_PIN,ESP_SCL_1_PIN);
    // Output Initialization 
    Wire.beginTransmission(MCP4725_ADDR);
    Wire.write(64);
    Wire.write(Out >> 4);
    Wire.write((Out& 15) << 4);
    Wire.endTransmission();

    // TIMER CONFIGURATION
    // Timer a 1 MHz
    // 1 tick = 1 microsegundo  timerBegin(frecuency) Frecuency of the timer clock, not of the timer interruption
    timer = timerBegin(1000000);
    // Attach timer struct with timmer interruption handle function
    timerAttachInterrupt(timer, &onTimer);
    //Configure alarm value and autoreload of the timer. Alarm is automatically enabled.
    // Alarm each 30000 us = 30 ms
    // autoreload = true
    // reload_count = 0 => infinito
    timerAlarm(timer, 700, true, 0); // Sample time 700 microseconds

}


void loop()
{
  if(interruption_flag){   
   interruption_flag = false;   
   // ADC
   adc_0_lecture= analogRead(analogInPin); 
   // Control law  
   Out = adc_0_lecture;// * 4; ADC is of 12 bits, it doesn't requires *4 factor to output
   Out=constrain(Out,0,4095);
   //Serial.println(Out);// For easy debug
   //Out=2500; // For easy debug
   //Serial.println(Out);// For easy debug

  // DAC
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64);
  Wire.write(Out >> 4);
  Wire.write((Out& 15) << 4);
  Wire.endTransmission();
  }
}
