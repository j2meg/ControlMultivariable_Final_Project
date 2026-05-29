/*
  AnalogReadSerial

  Reads an analog input on pin p35 and other in the pin P34, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  
*/

// the setup routine runs once when you press reset:

#define ADC0_PIN 34 //Pin GPIO34 is recommended as first adc port
#define ADC1_PIN 35 //Pin GPIO35 is recommended as second adc port
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int adc_0_lecture = analogRead(ADC0_PIN);
  int adc_1_lecture = analogRead(ADC1_PIN);
  // print out the value you read:
  Serial.println(adc_0_lecture);
  Serial.println(adc_1_lecture);
  delay(100);  // delay in between reads for stability

}
