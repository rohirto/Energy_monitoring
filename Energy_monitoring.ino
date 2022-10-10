//#define DEBUG 1

//NRF Includes 
#include <string.h>
#include <stdio.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LowPower.h>
#define CE_PIN   10
#define CSN_PIN 9
const byte slaveAddress[5] = {'R','x','A','A','A'};
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio
char dataToSend[50];
char txNum = '0';
char str_temp[20];

//Emon Includes
#include <EmonLib.h>
#define VOLT_CAL 353.7
#define CURRENT_CAL 150.1 //sensor 1 calibration
#define FILTERSETTLETIME 10000         
boolean settled = false;
const float FACTOR = 100;
const float VMIN = 1.08;
const float VMAX = 3.92;
const float ADCV = 5.02;
//Timer variables
volatile byte ten_sec_counter = 0;
bool one_min_elapsed = false;
unsigned long startMillis;
unsigned long currentMillis;

EnergyMonitor emon1;
//Current Derivate Calculation
//float current_derivative = 0, previous_current = 0;
//unsigned long previousMillis;
float currentRMS = 0, supplyVoltage = 0;

//PIR
volatile byte HC_SR01_State = LOW;
const byte HC_SR01_input_pin = 2;
const byte Relay_output_pin = 3;

void setup() {
  // put your setup code here, to run once:
  #ifdef DEBUG
  Serial.begin(9600);
  #endif
  Relay_setup();
  NRF24_setup();
  emon1.voltage(0, VOLT_CAL, 1.7);
  emon1.current(1, CURRENT_CAL);  
  startMillis = millis();
  //Settling Time
  delay(1000);
}

void loop() {
  //Update Timers  
  timer_function();
  PIR_loop();
  currentRMS = getCurrent();
  printMeasure("Irms", currentRMS, "A");
  delay(1000);
  emon1.calcVI(20,2000); 
  supplyVoltage   = emon1.Vrms;
  #ifdef DEBUG 
  Serial.println(supplyVoltage);
  #endif
  
  if(one_min_elapsed == true)
  {
    one_min_elapsed = false;

    //Signal Conditioning
    if(supplyVoltage < 20)
    {
      supplyVoltage = 0;
      currentRMS = 0;
    }
    //Send Values to Server
    dtostrf(supplyVoltage, 7, 3, str_temp);
    sprintf(dataToSend,"7~%s~", str_temp);
    send();
    delay(1000);
    dtostrf(currentRMS, 7, 3, str_temp);
    sprintf(dataToSend,"8~%s~", str_temp);
    send();
    delay(1000);
  }

}
void HC_SR01_callback(){
  HC_SR01_State = HIGH;
}
void printMeasure( String prefix, float value, String postfix)
{
  #ifdef DEBUG
  Serial.print(prefix);
  Serial.print(value,3);
  Serial.println(postfix);
  #endif
}
float getCurrent()
{
  float voltage;
  float current;
  float sum = 0;
  long timepo = millis();
  int counter = 0;

  while(millis()- timepo < 500)
  {
    //previousMillis = millis(); 
    //previous_current = currentRMS;
    voltage = analogRead(A3) *ADCV/1023.0;
    current = fmap(voltage, VMIN, VMAX, -FACTOR, FACTOR);
    //Calculate Derivative to discard transients
    //interval = millis()-previousMillis;
    //current_derivative = (current - previous_current)/interval;
    //if(current_derivative > 2)
    //{
    //  current = 0;
    //}
    sum += sq(current);
    counter = counter +1;
    
    delay(1);
  }
  //Serial.println(current_derivative);
  //Serial.println(interval);
  current = sqrt(sum/counter);
  return (current);
}
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x-in_min)*(out_max-out_min)/(in_max-in_min) + out_min;
}
void send() {

    bool rslt;
    rslt = radio.write( &dataToSend, sizeof(dataToSend) );
        // Always use sizeof() as it gives the size as the number of bytes.
        // For example if dataToSend was an int sizeof() would correctly return 2
    #ifdef DEBUG
    Serial.print("Data Sent ");
    Serial.print(dataToSend);
    #endif
    if (rslt) {
        #ifdef DEBUG
        Serial.println("  Acknowledge received"); 
        #endif
    }
    else {
      #ifdef DEBUG
        Serial.println("  Tx failed");
        #endif
    }
}
void NRF24_setup()
{
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.setRetries(3,5); // delay, count
  radio.openWritingPipe(slaveAddress);
}
void timer_function()
{
  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if( currentMillis - startMillis >= 10000)
  {
    startMillis = currentMillis;
    ten_sec_counter++;

    if((ten_sec_counter % 6) == 0)  //test whether the period has elapsed
    {
      one_min_elapsed = true;
      HC_SR01_State = LOW;   //clear the flag
      ten_sec_counter = 0;  //IMPORTANT to save the start time of the current LED state.
    }   
  }
 
}
void Relay_setup()
{
  //define relay pins and their state
  pinMode(HC_SR01_input_pin, INPUT);  //Digital Pin 2 is input
  pinMode(Relay_output_pin, OUTPUT);
  digitalWrite(Relay_output_pin, HIGH);
  //attachInterrupt(digitalPinToInterrupt(HC_SR01_input_pin), HC_SR01_callback, RISING);
  
}
void PIR_loop()
{
//  if(HC_SR01_State == HIGH)
//  {
//    digitalWrite(Relay_output_pin, LOW);
//    //Serial.println("Detected");
//    //HC_SR01_State = LOW;   //clear the flag
//  }
//  else
//  {
//    digitalWrite(Relay_output_pin, HIGH);
//  }
    if(digitalRead(HC_SR01_input_pin) == HIGH)
    {
      digitalWrite(Relay_output_pin, LOW);
    }
    else
    {
      digitalWrite(Relay_output_pin, HIGH);
    }
}
