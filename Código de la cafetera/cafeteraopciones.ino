#include <Arduino.h>
#include "HX711.h"

#include <OneWire.h>                
#include <DallasTemperature.h>
#include <PIDController.hpp>

//const int PIN_INPUT = 0;
const int PIN_OUTPUT = 3;

PID::PIDParameters<double> parameters(4.0, 0.2, 1);
PID::PIDController<double> pidController(parameters);
 
OneWire ourWire(8);                //Se establece el pin 2  como bus OneWire
 
DallasTemperature sensors(&ourWire); //Se declara una variable u objeto para nuestro sensor

//Setpoint
double setpoint = 85;


//Pins
//digital
int InitPin = A1; //Satrt pin
int Init2Pin = A2;
int Init3Pin = A3;
int ReturnPin = 12;

const int LOADCELL_DOUT_PIN =11; //Weight sensor pin
const int LOADCELL_SCK_PIN = 10; //weigt sensor pin
int GrindPin = A5; //grind rel pin pin

const int TempPin = 3; //pin rele temp

int PresPin = A0;
const int BombREL = 4; //pin bomba

//PWM
const int motor01 = 9;  // PWM Motor tornillo
//const int motor01A = 3;  // PWM Motor tornillo
//const int motor01B = 4;  // PWM Motor tornillo


const int motor02A = 5;  // PWM Motor tornillo
const int motor02B = 6;  // PWM Motor tornillo
const int motor02 = 7;  // PWM Motor tornillo

// Pesado del café
int coffeeWeight = 9;
float goalW;
int inst_01;
int inst_02;
int inst_03;
int proceso = 0; //para leer el proceso

HX711 scale;



void setup() {
  // Setup code
  delay(1000);
  Serial.begin(9600);
  Serial.println("Setup Start..");

  //Sensor de peso
  Serial.println("Setup Scale");
  //rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  Serial.println("Config Weight");
  scale.set_scale(-209.6765784);
  
  Serial.println("Tare");
  scale.tare();
  
  Serial.println("Setup PWM");

  //PWM Motors
  //Motores
  pinMode(motor01, OUTPUT); // controlado motor pin
  pinMode(motor02, OUTPUT); // controlado motor pin
  pinMode(motor02A, OUTPUT); // dir 01
  pinMode(motor02B, OUTPUT); // dir 02
  digitalWrite(motor02A, LOW); // controlado motor pin
  digitalWrite(motor02B, HIGH); // controlado motor pin
  Serial.println("Return pin low");
  digitalWrite(ReturnPin, LOW);// = 12;

  
  Serial.print("Setup Sensors");

  sensors.begin();   //Se inicia el sensor
  Serial.print("Setup Complete..");
  Serial.print("Please wait");
  delay(5000);
  Serial.print("done");

  Serial.print("Starting PID and temp");
  sensors.requestTemperatures();   //Se envía el comando para leer la temperatura
  float temp= sensors.getTempCByIndex(0); //Se obtiene la temperatura en ºC4
  
  pidController.Input = temp;
  
  pidController.Setpoint = 90.0;
  pidController.TurnOn();

  delay(5000);

}


void fun_tornilloSF(){
  Serial.print("reading:\t");
  Serial.println(scale.get_units(), 1);
  int weight = (scale.get_units());
  Serial.println("Weight");
  Serial.println(weight);
  goalW = weight - coffeeWeight;
  Serial.println("goalW");
  Serial.println(goalW);
  while (weight > goalW){
    weight = (scale.get_units());
    Serial.println("Moving motor");
    Serial.println("Weight cambiante:");
    Serial.println(weight);
    analogWrite(motor01, 250);
    //dutyCycle = dutyCycle + 1;
    delay(10);
    
    }
  Serial.println("Apagando motor");
  analogWrite(motor01, LOW);
}

void mechPosGrind(){
  Serial.println("Moving motor dir1");

  digitalWrite(motor02A, HIGH); // dir sentido horario
  digitalWrite(motor02B, LOW); 
  analogWrite(motor02, 250);
  delay(25000);
  Serial.println("Stopping motor dir1");

  analogWrite(motor02, 0);
  delay(25000);
}

void mechPosCoffee(){
  Serial.println("moving motor dir1");

  digitalWrite(motor02A, LOW); // antihorario
  digitalWrite(motor02B, HIGH); 
  analogWrite(motor02, 250);
  delay(25000);
  Serial.println("Stopping motor dir1");

  analogWrite(motor02, 0);
  delay(5000);
}

int inst_IN(){
  // Devuelve un valor entre 0 y 3 indicando cual seleccion o no seleccion
  int instruct;
  inst_01 = digitalRead(InitPin);   // read the input pin 01
  inst_02 = digitalRead(Init2Pin);   // read the input pin 02
  inst_03 = digitalRead(Init3Pin);   // read the input pin 03
  //num 0-7 
  instruct = inst_01+(inst_02*2)+(inst_03*4);
  return instruct;
}

void loop() {
  // put your main code here, to run repeatedly:
   //Iteration 1
  delay(500);  
  Serial.println("INICIO");
  proceso = 0;
  proceso = inst_IN();
  Serial.println("Proceso a hacer:");
  Serial.println(proceso);

  
  delay (50);
if (proceso == 3){
    delay(5000);  

  //SISTEMA PESO
  fun_tornilloSF();
  Serial.println("fin tornillo");

  //ENCENDER MOLIENDA
  
  Serial.println("encender molienda 35s");
  Serial.println("moliendo ...");
  digitalWrite(GrindPin,HIGH);
  delay(35000);
  Serial.println("fin molienda");
  digitalWrite(GrindPin,LOW);

  Serial.println("Bomba y temperatura");

  
  //añadir temperatura y boma con PID
  int sensorValue = analogRead(PresPin);
  float pressure = (((sensorValue * (5.0 / 1023.0)))-0.5)*0.3*10; // in Bar 
  Serial.println("Presion: ");   
  Serial.println(pressure); 
  for (int itread = 0; itread < 10; itread++) {
    sensorValue = analogRead(PresPin);
    pressure = (((sensorValue * (5.0 / 1023.0)))-0.5)*0.3*10; // in Bar
    Serial.println("Presion in system: ");   
    Serial.println(pressure); 
    digitalWrite(BombREL,HIGH);
    delay(500);
    if (pressure > 4.0){
      if (itread > 5){
        break;
      }
    }
  }
  digitalWrite(BombREL,LOW);
  //Control PID 
  sensors.requestTemperatures();   //Se envía el comando para leer la temperatura
  float temp= sensors.getTempCByIndex(0); //Se obtiene la temperatura en ºC
  Serial.print("Temperatura= ");
  Serial.print(temp);
  Serial.println(" C");
  float refTemp = temp + 2;
  while (temp < refTemp){
    sensors.requestTemperatures();   //Se envía el comando para leer la temperatura
    temp= sensors.getTempCByIndex(0); //Se obtiene la temperatura en ºC
    Serial.print("Temperatura= ");
    Serial.print(temp);
    Serial.println(" C");
    pidController.Input = temp;
    pidController.Update();
    digitalWrite(PIN_OUTPUT, HIGH);//pidController.Output);
    delay(20);
    if (temp > 35.0){
      break;
    }
    
  }
  digitalWrite(PIN_OUTPUT, LOW);
  
  //digitalWrite(TempPin,HIGH);
  //delay(5000);
  //digitalWrite(TempPin,LOW);


  //Maintain Pressure
  sensorValue = analogRead(PresPin);
  pressure = (((sensorValue * (5.0 / 1023.0)))-0.5)*0.3*10; // in Bar 
  Serial.println("Presion: ");   
  Serial.println(pressure); 
  while (pressure < 3.0) {
        sensorValue = analogRead(PresPin);
        pressure = (((sensorValue * (5.0 / 1023.0)))-0.5)*0.3*10; // in Bar
        Serial.println("Pressurizing after temp to: ");   
        Serial.println(pressure); 
        digitalWrite(BombREL,HIGH);
        delay(50);
      }
  digitalWrite(BombREL,LOW);

  Serial.println("mech pos 25s each");
  
  mechPosCoffee();
  digitalWrite(BombREL,HIGH);
  delay(2500);
  digitalWrite(BombREL,LOW);
  mechPosGrind();
  digitalWrite(ReturnPin,HIGH);
  delay(2500);
  digitalWrite(ReturnPin,LOW);
  


  }
//Doppio
else if (proceso == 2){
  delay(5000);  

  for (int dop1 = 0; dop1 < 2; dop1++){
    
  
  //SISTEMA PESO
  fun_tornilloSF();
  Serial.println("fin tornillo");

  //ENCENDER MOLIENDA
  
  Serial.println("encender molienda 35s");
  Serial.println("moliendo ...");
  digitalWrite(GrindPin,HIGH);
  delay(35000);
  Serial.println("fin molienda");
  digitalWrite(GrindPin,LOW);

  Serial.println("Bomba y temperatura");

  
  //añadir temperatura y boma con PID
  int sensorValue = analogRead(PresPin);
  float pressure = (((sensorValue * (5.0 / 1023.0)))-0.5)*0.3*10; // in Bar 
  Serial.println("Presion: ");   
  Serial.println(pressure); 
  for (int itread = 0; itread < 10; itread++) {
    sensorValue = analogRead(PresPin);
    pressure = (((sensorValue * (5.0 / 1023.0)))-0.5)*0.3*10; // in Bar
    Serial.println("Presion in system: ");   
    Serial.println(pressure); 
    digitalWrite(BombREL,HIGH);
    delay(500);
    if (pressure > 4.0){
      if (itread > 5){
        break;
      }
    }
  }
  digitalWrite(BombREL,LOW);
  //Control PID 
  sensors.requestTemperatures();   //Se envía el comando para leer la temperatura
  float temp= sensors.getTempCByIndex(0); //Se obtiene la temperatura en ºC
  Serial.print("Temperatura= ");
  Serial.print(temp);
  Serial.println(" C");
  float refTemp = temp + 2;
  while (temp < refTemp){
    sensors.requestTemperatures();   //Se envía el comando para leer la temperatura
    temp= sensors.getTempCByIndex(0); //Se obtiene la temperatura en ºC
    Serial.print("Temperatura= ");
    Serial.print(temp);
    Serial.println(" C");
    pidController.Input = temp;
    pidController.Update();
    digitalWrite(PIN_OUTPUT, HIGH);//pidController.Output);
    delay(20);
    if (temp > 35.0){
      break;
    }
    
  }
  digitalWrite(PIN_OUTPUT, LOW);
  
  //digitalWrite(TempPin,HIGH);
  //delay(5000);
  //digitalWrite(TempPin,LOW);


  //Maintain Pressure
  sensorValue = analogRead(PresPin);
  pressure = (((sensorValue * (5.0 / 1023.0)))-0.5)*0.3*10; // in Bar 
  Serial.println("Presion: ");   
  Serial.println(pressure); 
  while (pressure < 3.0) {
        sensorValue = analogRead(PresPin);
        pressure = (((sensorValue * (5.0 / 1023.0)))-0.5)*0.3*10; // in Bar
        Serial.println("Pressurizing after temp to: ");   
        Serial.println(pressure); 
        digitalWrite(BombREL,HIGH);
        delay(50);
      }
  digitalWrite(BombREL,LOW);

  Serial.println("mech pos 25s each");
  
  mechPosCoffee();
  digitalWrite(BombREL,HIGH);
  delay(2500);
  digitalWrite(BombREL,LOW);
  mechPosGrind();
  delay(3000);

  }
  
  digitalWrite(ReturnPin,HIGH);
  delay(2500);
  digitalWrite(ReturnPin,LOW);
}

  // Americano
  else if (proceso == 1){
   delay(5000);  

  //SISTEMA PESO
  fun_tornilloSF();
  Serial.println("fin tornillo");

  //ENCENDER MOLIENDA
  
  Serial.println("encender molienda 35s");
  Serial.println("moliendo ...");
  digitalWrite(GrindPin,HIGH);
  delay(35000);
  Serial.println("fin molienda");
  digitalWrite(GrindPin,LOW);

  Serial.println("Bomba y temperatura");

  
  //añadir temperatura y boma con PID
  int sensorValue = analogRead(PresPin);
  float pressure = (((sensorValue * (5.0 / 1023.0)))-0.5)*0.3*10; // in Bar 
  Serial.println("Presion: ");   
  Serial.println(pressure); 
  for (int itread = 0; itread < 10; itread++) {
    sensorValue = analogRead(PresPin);
    pressure = (((sensorValue * (5.0 / 1023.0)))-0.5)*0.3*10; // in Bar
    Serial.println("Presion in system: ");   
    Serial.println(pressure); 
    digitalWrite(BombREL,HIGH);
    delay(500);
    if (pressure > 4.0){
      if (itread > 5){
        break;
      }
    }
  }
  digitalWrite(BombREL,LOW);
  //Control PID 
  sensors.requestTemperatures();   //Se envía el comando para leer la temperatura
  float temp= sensors.getTempCByIndex(0); //Se obtiene la temperatura en ºC
  Serial.print("Temperatura= ");
  Serial.print(temp);
  Serial.println(" C");
  float refTemp = temp + 2;
  while (temp < refTemp){
    sensors.requestTemperatures();   //Se envía el comando para leer la temperatura
    temp= sensors.getTempCByIndex(0); //Se obtiene la temperatura en ºC
    Serial.print("Temperatura= ");
    Serial.print(temp);
    Serial.println(" C");
    pidController.Input = temp;
    pidController.Update();
    digitalWrite(PIN_OUTPUT, HIGH);//pidController.Output);
    delay(20);
    if (temp > 35.0){
      break;
    }
    
  }
  digitalWrite(PIN_OUTPUT, LOW);
  
  //digitalWrite(TempPin,HIGH);
  //delay(5000);
  //digitalWrite(TempPin,LOW);


  //Maintain Pressure
  sensorValue = analogRead(PresPin);
  pressure = (((sensorValue * (5.0 / 1023.0)))-0.5)*0.3*10; // in Bar 
  Serial.println("Presion: ");   
  Serial.println(pressure); 
  while (pressure < 3.0) {
        sensorValue = analogRead(PresPin);
        pressure = (((sensorValue * (5.0 / 1023.0)))-0.5)*0.3*10; // in Bar
        Serial.println("Pressurizing after temp to: ");   
        Serial.println(pressure); 
        digitalWrite(BombREL,HIGH);
        delay(50);
      }
  digitalWrite(BombREL,LOW);

  Serial.println("mech pos 25s each");
  
  mechPosCoffee();
  digitalWrite(BombREL,HIGH);
  delay(6000);
  digitalWrite(BombREL,LOW);
  mechPosGrind();
  digitalWrite(ReturnPin,HIGH);
  delay(2500);
  digitalWrite(ReturnPin,LOW);

  }
  
delay(2000);
Serial.println("loop");
sensors.requestTemperatures();   //Se envía el comando para leer la temperatura
float temp= sensors.getTempCByIndex(0); //Se obtiene la temperatura en ºC
Serial.print("Temperatura= ");
Serial.print(temp);
Serial.println(" C");
delay(2000);


}
