#include <AccelStepper.h>
#include <MultiStepper.h>
#include <PinChangeInt.h>
#include <PID_v1.h>
int encodPinA1=2;                       
int encodPinB1=3;                          
int min1 = 13; //1 driver 
int min2 = 12;//2 driver
int men1=7 ; //3 driver 
int men2=4; //4driver 
int pwminput=6; //6driver
String elevmasuk;String azmasuk;
int azimuth; 

//---------------deklarasi stepper
#define STEPPER1_DIR_PIN 5
#define STEPPER1_STEP_PIN 8
AccelStepper motor_Stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);

//-------------- deklarasi PID motor DC---------------
double kp = 2, ki = 1.5,  kd = 0.0075;            
double input = 0, output = 0, setpoint = 0;
long temp,a,b,c;
volatile long encoderPos = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'

//-------------------------------------------------

void setup() {
  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  pinMode (min2, OUTPUT);
  pinMode (min1, OUTPUT);
  pinMode (men2, OUTPUT);
  pinMode (men1, OUTPUT);
  pinMode (pwminput, OUTPUT);
  attachInterrupt(0, encoder, FALLING);               // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
  Serial.begin (115200);                              // for debugging
    motor_Stepper1.setSpeed(50);
  motor_Stepper1.setAcceleration(850);
  motor_Stepper1.moveTo(0);
}

void loop() {
   int elevasi;
   
  if (Serial.available() > 0) {
       azmasuk = Serial.readStringUntil(',');
       elevmasuk = Serial.readStringUntil('#');
           
  }
  //convert string azimuth----------------------- 
       azimuth = azmasuk.toInt ();
       a= map(azimuth, 0,180, 400,800)*-7 ; //a b dipalao jika angin ke arah utara
       b= map(azimuth, 181,359, 0,399)*-7;
       
       c=map(azimuth, 0,360, 0,800)*-7; // c dipakai jika angin ke arah selatan
  //---------------------------------------------

  //convert string elevasi-----------------------     
       elevasi =  elevmasuk.toDouble() * 3.2;
       if (elevasi >=1 && elevasi <= 96)
       {
        setpoint = 96;
       }
       else
       {
        setpoint = elevasi;
       }
       
       
       input = encoderPos ;                                // data from encoder
       myPID.Compute();   //PID running

  //elevasi run-----------------------                                            
       pwmOut(output);
  //----------------------------------

   //azimuth run----------------------

        motor_Stepper1.runToNewPosition(c); //dipakai jika angin utara jika angin selatan di comand
          motor_Stepper1.run();
          
          //dipakai jika angin selain jika angin utara di comand
          /*
          //dipakai jika angin utara jika angin selatan di comand
       if ((azimuth >= 0) && (azimuth <= 180))
      {
          motor_Stepper1.runToNewPosition(a);
          motor_Stepper1.run();
          //Serial.print(a);Serial.println(" ");
      }
       else if ((azimuth >= 181) &&  (azimuth <=360))
      {
          motor_Stepper1.runToNewPosition(b);
          motor_Stepper1.run();
       //   Serial.print(b);Serial.println(" ");
      }
       else
      {
  
      } 
      */
      //-----------------------------------
       Serial.print("encoder   :");Serial.print(encoderPos);Serial.print("  ");
       Serial.print("setpoint :");Serial.print(" ");Serial.print(setpoint);   
       Serial.print("pwminput :");Serial.print(" ");Serial.print(pwminput);   
       Serial.print("output :");Serial.print(" ");Serial.println(output);   
 //


}


void pwmOut(int out) {    

   if (out > 0) {
    digitalWrite(min1,HIGH);   
    digitalWrite(min2, LOW);
    digitalWrite(men1,HIGH);   
    digitalWrite(men2, HIGH);
    analogWrite(pwminput, out);
  }
  else {
    digitalWrite(min1,LOW);  
    digitalWrite(min2, HIGH);
    digitalWrite(men1,HIGH);   
    digitalWrite(men2, HIGH);
    analogWrite(pwminput, out);
  }
}

void encoder()  {                                     // pulse and direction, direct port reading to save cycles
  if (digitalRead(encodPinB1)==HIGH)    encoderPos++;             // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      encoderPos--;             // if(digitalRead(encodPinB1)==LOW)   count --;
}



