//=================================================
// Shield L293D pour Arduino
//=================================================
// Sorties pont en H -> moteurs DC, relais, éclairage
// Pas de moteur pas à pasavec de programme
// Les 2 servos utilisent la librairie Servo.h
// 4 moteurs DC au maximum (ou 8 sorties)
//==================================================
#include <Servo.h>   //Librairie pour les 2 servomoteurs
#include "Ultrasonic.h"

Ultrasonic ultrasonic(22);

#include <IRremote.h>

int RECV_PIN = 23;
IRrecv irrecv(RECV_PIN);
decode_results results;



// Pins Arduino pour le registre à décalage 4 7 8 12
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

// Bus 8-bit en sortie du registre à décalage 74HC595 
// Utilisés pour fixer la direction des ponts de commande
#define MOTOR1_A 2  //ce ne sont apsdes pins Arduino
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5
#define MOTOR3_B 7
#define MOTOR4_A 0
#define MOTOR4_B 6

// Pins Arduino pour les signaux PWM (moteurs et servos) 3 5 6 9 10 11
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3

// Codes pour les fonctions de moteur
#define FORWARD 1     //4 modes de commande 
#define BACKWARD 2    //avant arrière frein stop
#define BRAKE 3
#define RELEASE 4


float vald;
float valg ;
int stop;

int pin_captddd = A8 ;
int pin_captdd = A9 ;
int pin_captd = A10 ;
int pin_captg = A11 ;
int pin_captgg = A12 ;
int pin_captggg = A13 ;
float seuil = 1,80;

int vitessemax = 100;
int virage1 = 90;
int virage2 = 80;
int virage3 = 70;

void affichage(){

    Serial.print("captg : ");
  Serial.println( analogRead(pin_captg)*(5.0 / 1023.0));

  Serial.print("captgg : ");
  Serial.println( analogRead(pin_captgg)*(5.0 / 1023.0));
  Serial.print("captggg : ");
  Serial.println( analogRead(pin_captggg)*(5.0 / 1023.0));
  Serial.print("captd : ");
  Serial.println( analogRead(pin_captd)*(5.0 / 1023.0));
  Serial.print("captdd : ");
  Serial.println( analogRead(pin_captdd)*(5.0 / 1023.0));
  Serial.print("captddd : ");
  Serial.println( analogRead(pin_captddd)*(5.0 / 1023.0));
  Serial.println("");
  Serial.println("");
  Serial.println("");
  // stop the program for for <sensorValue> milliseconds:
  //delay(500);
}

//Initialisations
void setup()
{
  Serial.begin (9600);
irrecv.enableIRIn(); // Initialise le recepteur
  Serial.println("Motor Shield L293D");
 

}

//Programme principal
void loop()
{/**
  valg = analogRead(A15)*(5.0 / 1023.0);
  vald =0.53+ analogRead(A14)* (5.0 / 1023.0);
   if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    irrecv.resume(); // Recoit la valeur suivante
   }
 // Serial.print("capteur droit = ");
 // Serial.println(vald);
  //Serial.print("capteur gauche = ");
  //Serial.println(valg);
  delay(500);

  if(vald>valg){
    motor(1, FORWARD, 90);  //Tourner à fond en avant
    motor(2, FORWARD, 50);
  }
  else if(vald<valg){
    motor(1, FORWARD, 50);  //Tourner à fond en avant
    motor(2, FORWARD, 90);

  }

  else{
    motor(1, FORWARD, 90);  //Tourner à fond en avant
    motor(2, FORWARD, 90);  //Tourner à fond en avant

  }
  delay(1000);
  
  **/
  affichage();
  

  
// long RangeInCentimeters;
   // RangeInCentimeters = ultrasonic.MeasureInCentimeters();
    if(analogRead(pin_captg)*(5.0 / 1023.0) > seuil){
       motor(2, BACKWARD, vitessemax);  //Tourner à fond en avant
        motor(1, BACKWARD, vitessemax-40);  //Tourner à fond en avant
        delay(100);
      
    }
   else  if(analogRead(pin_captgg)*(5.0 / 1023.0) > seuil){
        motor(2, BACKWARD, vitessemax);  //Tourner à fond en avant
  motor(1, BACKWARD, vitessemax-50);  //Tourner à fond en avant
        delay(200);
      
    }
      else  if(analogRead(pin_captggg)*(5.0 / 1023.0) > seuil){
       motor(2, BACKWARD, vitessemax);  //Tourner à fond en avant
  motor(1, BACKWARD, 0);  //Tourner à fond en avant
        delay(300);
      
    }
     else  if(analogRead(pin_captd)*(5.0 / 1023.0) > seuil){
        motor(2, BACKWARD, vitessemax-40);  //Tourner à fond en avant
         motor(1, BACKWARD, vitessemax);  //Tourner à fond en avant
        delay(100);
      
    }
      else  if(analogRead(pin_captdd)*(5.0 / 1023.0) > seuil){
        motor(2, BACKWARD, vitessemax -50);  //Tourner à fond en avant
         motor(1, BACKWARD, vitessemax);  //Tourner à fond en avant
        delay(200);
      
    }
      else  if(analogRead(pin_captddd)*(5.0 / 1023.0) > seuil){
     motor(2, BACKWARD, 0);  //Tourner à fond en avant
     motor(1, BACKWARD, vitessemax);  //Tourner à fond en avant
        delay(300);
      
    } 
    else{
  motor(1, BACKWARD, vitessemax);  //Tourner à fond en avant
  motor(2, BACKWARD, vitessemax);  //Tourner à fond en avant
    
    }
}




  //Moteur DC connectés entre M1_A(+) and M1_B(-) et entre M2_A(+) and M2_B(-)



//=== Fonction motor
// Choisir le moteur (1-4), la commande et la vitesse (0-255).
// Les commandes sont : FORWARD, BACKWARD, BRAKE, RELEASE.
void motor(int nMotor, int command, int speed)
{
  int motorA, motorB;

  if (nMotor >= 1 && nMotor <= 4)
  {  
    switch (nMotor)
    {
    case 1:
      motorA   = MOTOR1_A;
      motorB   = MOTOR1_B;
      break;
    case 2:
      motorA   = MOTOR2_A;
      motorB   = MOTOR2_B;
      break;
    default:
      break;
    }

    switch (command)
    {
    case FORWARD:   //Tourner en avant
      motor_output (motorA, HIGH, speed);
      motor_output (motorB, LOW, -1);     // -1: no PWM set
      break;
    case BACKWARD:  //Tourner en arrière
      motor_output (motorA, LOW, speed);
      motor_output (motorB, HIGH, -1);    // -1: no PWM set
      break;
    case BRAKE:   //Freiner
      motor_output (motorA, LOW, 255); // 255: fully on.
      motor_output (motorB, LOW, -1);  // -1: no PWM set
      break;
    case RELEASE:   //Stop
      motor_output (motorA, LOW, 0);  // 0: output floating.
      motor_output (motorB, LOW, -1); // -1: no PWM set
      break;
    default:
      break;
    }
  }
}


//=== Fonction motor_output
// Utilise le driver pour piloter des sorties
// Mettre la  variable high_low sur HIGH / LOW pour des lampes
// On une speed = 0
// speed varie de 0-255 pour les 2 pins (0 = arrêt, 255 = maxi)
// à mettre sur -1 pour ne pas régler de PWM du tout
void motor_output (int output, int high_low, int speed)
{
  int motorPWM;

  switch (output)
  {
  case MOTOR1_A:
  case MOTOR1_B:
    motorPWM = MOTOR1_PWM;
    break;
  case MOTOR2_A:
  case MOTOR2_B:
    motorPWM = MOTOR2_PWM;
    break;

  default:
    // Utilise speed comme flag d'erreur, -3333 = invalid output.
    speed = -3333;
    break;
  }

  if (speed != -3333)   //La valeur speed est valide
  {
    // Set the direction with the shift register 
    // on the MotorShield, even if the speed = -1.
    // In that case the direction will be set, but not the PWM.
    shiftWrite(output, high_low);

    // Ajuster le PWM seulemernt s il est valide
    if (speed >= 0 && speed <= 255)    
    {
      analogWrite(motorPWM, speed);
    }
  }
}


// Fonction shiftWrite
// The parameters are just like digitalWrite().
// The output is the pin 0...7 (the pin behind the shift register).
// The second parameter is HIGH or LOW.
void shiftWrite(int output, int high_low)
{
  static int latch_copy;
  static int shift_register_initialized = false;

  // Do the initialization on the fly, 
  // at the first time it is used.
  if (!shift_register_initialized)
  {
    // Set pins for shift register to output
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);

    // Set pins for shift register to default value (low);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);
    // Enable the shift register, set Enable pin Low.
    digitalWrite(MOTORENABLE, LOW);

    // start with all outputs (of the shift register) low
    latch_copy = 0;

    shift_register_initialized = true;
  }

  // The defines HIGH and LOW are 1 and 0.
  // So this is valid.
  bitWrite(latch_copy, output, high_low);

  // Use the default Arduino 'shiftOut()' function to
  // shift the bits with the MOTORCLK as clock pulse.
  // The 74HC595 shiftregister wants the MSB first.
  // After that, generate a latch pulse with MOTORLATCH.
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, LOW);
}
