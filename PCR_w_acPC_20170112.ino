#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/io.h> //library for AVR chip timer
#include <avr/interrupt.h> //library for AVR chip interrupts 
#include <SPI.h>
#include <PID_v1.h>
#include "SoftwareSerial.h"
//Some parts of our code are based on Arduino’s 
//AC phase controlling tutorial and 
//we re-used some part of that code as it is including comments.
//Other parts based upon code Written by Russell Durrett, red272@nyu.edu

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 10 //Use this for Temperature probe
#define DETECT 2  //zero cross detect
#define GATE 11    //triac gate
#define PULSE 4   //trigger pulse width (counts)

const int hot = 95; // 95C 
const int tm = 72; // 55C
const int elongtemp = 72; //72

const int initialdenaturetime = 30; //value in sec
const int denaturetime = 10; //value in sec
const int annealtime = 1; //value in sec
const int elongtime = 59; //value in sec
const int finalelongtime = 120; //value in sec
const int cycles = 23; //number of cycles

int phasecount = 0;
const int coolpin = 9;
int i; // in theory, could be up to 520, but lowered to avoid overrunns due to any delay, or fluctuations in power cycle
double currentTemperature;
unsigned long steptime; // long variable to measure length of steps
unsigned long stepstart; // long variable to measure start of steps

//Define Variables we'll be connecting to
double setPoint, Input, Output;
//Specify the links and initial tuning parameters
//double Kp=200, Ki=200, Kd=30; //works pretty well 55 +/- 0.0, 75 deg +/- 0.1; 95 +/- 0.2
double Kp=50, Ki=10, Kd=30;
PID myPID(&Input, &Output, &setPoint, Kp, Ki, Kd, DIRECT);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

int cyclecount = 1;

void setup(){
  pinMode(coolpin, OUTPUT);
  analogReference(EXTERNAL);
  Serial.begin(9600);
  // Start up the library
  sensors.begin();
  
  pinMode(DETECT, INPUT);     //zero cross detect
  digitalWrite(DETECT, HIGH); //enable pull-up resistor
  pinMode(GATE, OUTPUT);      //triac gate control
  setPoint = 25;     // set the default temperature as the setPoint
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 448);// in theory, could be 520, but lowered to avoid overrunns due to any delay, or fluctuations in power cycle

  // set up Timer1 
  //(see ATMEGA 328 data sheet pg 134 for more details)
  OCR1A = 100;      //initialize the comparator
  TIMSK1 = 0x03;    //enable comparator A and overflow interrupts
  TCCR1A = 0x00;    //timer control registers set for
  TCCR1B = 0x00;    //normal operation, timer disabled


  // set up zero crossing interrupt
  attachInterrupt(0,zeroCrossingInterrupt, RISING);    
    //IRQ0 is pin 2. Call zeroCrossingInterrupt 
    //on rising signal
}

//Interrupt Service Routines
//Theses run in the background and will trigger and itnerrupt the main program when triggered by the timer

void zeroCrossingInterrupt(){ //zero cross detect   
  TCCR1B=0x04; //start timer with divide by 256 input
  TCNT1 = 0;   //reset timer - count from zero
  delay(1);
//  delay needed otherwise it get's confused and dimmer doesn't work
}

ISR(TIMER1_COMPA_vect){ //comparator match
  digitalWrite(GATE,HIGH);  //set triac gate to high
  TCNT1 = 65536-PULSE;      //trigger pulse width, counts up from here for "PULSE" until it hits the overflow
}

ISR(TIMER1_OVF_vect){ //timer1 overflow
  digitalWrite(GATE,LOW); //turn off triac gate
  TCCR1B = 0x00;          //disable timer stopd unintended triggers
}


void loop(){
//Serial.println(phasecount);
sensors.requestTemperatures(); // Send the command to get temperatures   
double temp = (sensors.getTempCByIndex(0)); //define variable temp
digitalWrite(coolpin, LOW);
boolean reachedtemp = false; 
stepstart=millis();
steptime=(millis()-stepstart)/1000;
//initial denaturing step
  while (steptime < initialdenaturetime){
    sensors.requestTemperatures(); // Send the command to get temperatures   
    double temp = (sensors.getTempCByIndex(0)); //define variable temp
    if (isnan(temp)) {
      Serial.println("Something wrong with temperature probe!");
      i=450;
      delay(10000);
    } else {
      setPoint = hot;     // set the default temperature as the setPoint
      Input = temp;
      myPID.Compute();
      //Serial.print(Output);
      i = 450-Output;

      if (temp >= hot) {
        reachedtemp = true;
      }
      //Serial.println(reachedtemp);
      if (!reachedtemp) {
        stepstart=millis();
        Serial.println("HEATING");
      }
      steptime= (millis()-stepstart)/1000 ;
      Serial.println("INITIAL DENATURING");
      Serial.print(" Temp = ");
      Serial.print(temp);
      Serial.print(", DNA Denaturing. Target temp = ");
      Serial.print(hot); 
      Serial.print(", Time: ");
      Serial.print(steptime);
      Serial.print("/");
      Serial.println(initialdenaturetime); 
      OCR1A = i; //set the time for the ac phase control to delay
      delay(500);
    }
  }
//main program      
do {
//Denaturing step
  //Begin PCR Reaction by heating to 95 degrees;
  boolean reachedtemp = false;
  stepstart = millis();
  steptime = (millis()-stepstart)/1000;
  while (steptime < denaturetime){
    sensors.requestTemperatures(); // Send the command to get temperatures   
    double temp = (sensors.getTempCByIndex(0)); //define variable temp
    if (isnan(temp)) {
      Serial.println("Something wrong with temperature probe!");
      i=450;
      delay(10000);
    } else {
      setPoint = hot;     // set the default temperature as the setPoint
      Input = temp;
      myPID.Compute();
      //Serial.print(Output);
      i = 450-Output;
      
      if (temp >= hot) { 
        reachedtemp = true; 
        }
      if (!reachedtemp) { //reset the timer if it hasn't reached hot
        stepstart=millis();
        Serial.println("HEATING");
        } 
      steptime=(millis()-stepstart)/1000;
      Serial.print("Cycle #");
      Serial.println(cyclecount);
      Serial.print(" Temp = ");
      Serial.print(temp);
      Serial.print(", DNA Denaturing, Target temp = ");
      Serial.print(hot);
      Serial.print(", Time: ");
      Serial.print(steptime);
      Serial.print("/");
      Serial.println(denaturetime);
      OCR1A = i; //set the time for the ac phase control to delay
      delay(500);
    }
  }
//Anealing step
  digitalWrite(coolpin, HIGH); //cooling to Tm with fan
  reachedtemp = false;
  stepstart = millis();
  steptime = (millis()-stepstart)/1000;
  while (steptime < annealtime){
    sensors.requestTemperatures(); // Send the command to get temperatures   
    double temp = (sensors.getTempCByIndex(0)); //define variable temp
    if (isnan(temp)) {
      Serial.println("Something wrong with temperature probe!");
      i=450;
      delay(10000);
    } else {
      setPoint = tm;     // set the default temperature as the setPoint
      Input = temp;
      myPID.Compute();
      //Serial.print(Output);
      i = 450-Output;
      
      if (temp <= tm) {
        reachedtemp = true;
        digitalWrite(coolpin, LOW);
      }
      //Serial.println(reachedtemp);
      if (!reachedtemp) {
        stepstart=millis();
        Serial.println("COOLING TO ANNEALING TEMPERATURE");
        }
      steptime=(millis()-stepstart)/1000;
      Serial.print("Cycle #");
      Serial.println(cyclecount);
      Serial.print(" Temp = ");
      Serial.print(temp);
      Serial.print(", Annealing. Target temp = ");
      Serial.print(tm); 
      Serial.print(", Time: ");
      Serial.print(steptime);
      Serial.print("/");
      Serial.println(annealtime); 
      OCR1A = i; //set the time for the ac phase control to delay
      delay(500);
      }
  }
  digitalWrite(coolpin, LOW); //this should be redundant, but left in just to make sure
  //elongation step  
  //heating to extension temp
  reachedtemp = false;
  stepstart = millis();
  steptime = (millis()-stepstart)/1000;
  while (steptime < elongtime){
    sensors.requestTemperatures(); // Send the command to get temperatures   
    double temp = (sensors.getTempCByIndex(0)); //define variable temp
    if (isnan(temp)) {
      Serial.println("Something wrong with temperature probe!");
      i=450;
      delay(10000);
    } else {
      setPoint = elongtemp;     // set the default temperature as the setPoint
      Input = temp;
      myPID.Compute();
      //Serial.print(Output);
      i = 450-Output;
      if (temp >= elongtemp) {
        reachedtemp = true;
      }
      if (!reachedtemp) { //reset the timer if it hasn't reached hot
        stepstart=millis();
        Serial.println("HEATING TO EXTENSION TEMPERATURE");
        }
      steptime=(millis()-stepstart)/1000;
      Serial.print("Cycle #");
      Serial.println(cyclecount);
      Serial.print(" Temp = ");
      Serial.print(temp);
      Serial.print(", DNA Extending. Target temp = ");
      Serial.print(elongtemp);
      Serial.print(", Time: ");
      Serial.print(steptime);
      Serial.print("/");
      Serial.println(elongtime);
      OCR1A = i; //set the time for the ac phase control to delay
      delay(500);
    }
  }
  
  cyclecount++;
}while (cyclecount <= cycles);

//final extension step
  stepstart = millis();
  steptime = (millis()-stepstart)/1000;
  while (steptime < finalelongtime){
    sensors.requestTemperatures(); // Send the command to get temperatures   
    double temp = (sensors.getTempCByIndex(0)); //define variable temp
    if (isnan(temp)) {
      Serial.println("Something wrong with temperature probe!");
      i=450;
      delay(10000);
    } else {
      setPoint = elongtemp;     // set the default temperature as the setPoint
      Input = temp;
      myPID.Compute();
      i = 450-Output;
      steptime=(millis()-stepstart)/1000;
      Serial.println("FINAL Extension Step");
      Serial.print(" Temp = ");
      Serial.print(temp);
      Serial.print(", DNA Extending. Target temp = ");
      Serial.print(elongtemp);  
      Serial.print(", Time: ");
      Serial.print(steptime);
      Serial.print("/");
      Serial.println(finalelongtime);
      OCR1A = i; //set the time for the ac phase control to delay
      delay(500);
    }
  }
Serial.print("Completed ");
Serial.print(cycles);
Serial.print("Run completed");
do{
digitalWrite(coolpin, LOW);
i=450;
}while (!(cyclecount <= cycles));
}

