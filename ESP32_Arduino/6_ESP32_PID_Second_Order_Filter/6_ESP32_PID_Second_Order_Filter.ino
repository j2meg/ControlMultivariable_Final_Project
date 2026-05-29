// ESP32 Arduino IDE
/*

This program uses an ESP32 DEV module to implement a digital Proportional Integral Derivative control system
tunned by the Ziegler-Nichols method.
The update of the input and output is driven by an arbitrary sample time defined by a hardware timer. 

Materials list. 
-ESP32 DEV module
-MCP4725 (12 bits I2C DAC)
The plant attached to this PI control system is a two stage Low Pass Filter
made with 
-two 15kOhms resistors
-2 100nF capacitors.  

*** Use an external osciloscope to view output of the system (Output of the plant) and control effort (DAC Output) 
The signal reference to follow is defined by software
Note: ESP32 ADC inputs only support voltages between 0 and 3.3 volts. Don't exceed these thresholds. 

Connections:
ESP32 Pins         | MCP4725 Pins    |   Plant System   |   Osciloscope
    GND            |   GND           |                  |
    3.3V           |   VCC           |                  |
    GPIO 21        |   SDA           |                  |
    GPIO 22        |   SCL           |                  |
    GND            |   GND           |                  |
    GPIO34 (ADC0)  |                 |     Output       | Osciloscope Ch2
                   |   Out           |     Input        | Osciloscope Ch1
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
volatile float fInput =0;
volatile bool interruption_flag = false;
volatile uint Out =0; // Value to send to MCP4725 (12 bits value: 0,..,4095)
volatile float ek=0,ek1=0,ek2=0; // current error (k) and 1 and 2 sample time before error (k1), (k2)
volatile float Uk=0; // current control effort
volatile float Uk1=0;// one sample time before control effort
volatile float Uk2=0;// two sample time before control effort



// Control Gains
/* //Traditional PID
float Kp=1; // High 15
float Ki=2;
float Kd=0;
*/

// Ziegler-Nichols
float Ku= 8;//3;
float Pu= 0.00525; //5.25ms
//Uncomment just one control scheme
//Osscilations
/*
float Kp=8; 
float Ki=0;
float Kd=0;
*/
//P
/*
float Kp=Ku*0.5;  
float Ki=0;
float Kd=0;
*/
//PI

float Kp=Ku*0.45;  
float Ki=Kp/(Pu/1.2); // Ki=Kp/Ti -- Ti=Pu/1.2
float Kd=0;

//PID
/* 
float Kp=0.6*Ku; // 
float Ki=Kp/(0.5*Pu); //Ki=Kp/Ti -- Ti=0.5*Pu
float Kd=Kp*Pu/8; // Kd=Kp*Td -- Td=Pu/8
*/




//
float SetPoint       = 0; // Signal to Follow by the driver
uint  ts = 700; // 700 microseconds
const float h=(float)ts*(1.0f/1000000.0f);
uint  samples_to_change = 72; // time to change the setpoint 50 ms = ts*72
uint  sample_counter  = 0; 
bool reference_state = false;
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
   sample_counter++;   
     // Change of setpoint every N samples
      if(sample_counter >= samples_to_change)
        {
          sample_counter = 0;
          reference_state = !reference_state;
          SetPoint = reference_state ? 1.5f : 0.0f;
        }

   // ADC
   adc_0_lecture= analogRead(analogInPin); 
   fInput=adc_0_lecture*(3.3/4096); // Data convertion to volts
   ek = SetPoint-fInput; // current error
   
   // Control law  
   //Uk= Uk1+Kp*ek+Ki*(ek-ek1); // Esfuerzo de control Integral
   //Uk= Uk1+(kp+ki*h+kd/h)*ek+(-2*kd/h-kp)*ek1+(kd/h)*ek2; 
   //Uk= Uk1+Kp*ek + ki*h*ek + kd/h*ek+(-2*kd/h-kp)*ek1+(kd/h)*ek2; 
  
   Uk= Uk1+(Kp+Ki*h+Kd/h)*ek+(-2*Kd/h-Kp)*ek1+(Kd/h)*ek2; 
   Uk1=Uk;
   ek2=ek1;
   ek1=ek;
   // System thresholds 
   if (Uk>3.3){
      Uk=3.3;
      }
   if (Uk<0){
      Uk=0;
      }
   Out=Uk*4096/3.3; // Data convertion from volts to DAC values
   Out=constrain(Out,0,4095);// Last output protection
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
  // Falta manejar el cambio de setpoint
}
