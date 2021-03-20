#include <Arduino.h>
#include <QTRSensors.h>
#include <SparkFun_TB6612.h>
#include <SimpleKalmanFilter.h>
//#include <SharpIR.h>


//********************************************************************
//SENSORS
//--------------------------------------------------------------------
#define NUM_SENSORS             5  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  1  // average 4 analog samples per sensor reading
#define EMITTER_PIN             13  // arduino nano indicator LED is at 13
//#define sarp                    A7// sarp je na A7 pinu
//#define model 430

//A4, A2, A0, A1, A3, A5, A7, A6 za najbrze motore sa 8 senzora
//A4, A2, A1, A3, A5, A7, A6 za 7 senzora srednji motori
// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsAnalog qtra((unsigned char[]) {A0, A1, A2,A3, A4},NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
unsigned int center=3000;
unsigned int position=center;
//********************************************************************
//MOTORS
//--------------------------------------------------------------------
#define AIN1 6
#define BIN1 8
#define AIN2 7
#define BIN2 9
#define PWMA 5
#define PWMB 10
#define STBY 12

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = -1;
const int offsetB = -1;

// Initializing motors. 
Motor motorLeft = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motorRight = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

//********************************************************************
//PID FOR DIFFERENTIAL SPEED
//--------------------------------------------------------------------
float Kp=0.090; // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
float Kd=1.600; // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
float Ki=0.000;
//********************************************************************
//PID FOR BASE SPEED
//--------------------------------------------------------------------
float Kp1=0.030;
float Kd1=0.000;
float Ki1=0.000;

float gornja=20.0;
float donja=15.0;

int maxSpeed=100;  // max speed of the robot
int minSpeed=-100; // min speed of the robot
int baseSpeed=1000; // this is the speed at which the motors should spin when the robot is perfectly on the line

int rightMotorSpeed = baseSpeed;
int leftMotorSpeed = baseSpeed;
int diffSpeed=0;
int correctBaseSpeed=0;

int error = 0;
int last_error = 0;
int i_error = 0;
int d_error = 0;

int abs_error = 0;
int last_abs_error = 0;
int i_abs_error = 0;
int d_abs_error = 0;

//sarp senzor
//********************************************************************

//********************************************************************
//KALMAN FILTER
//--------------------------------------------------------------------
SimpleKalmanFilter positionKalmanFilter(15, 15, 0.04);
//SimpleKalmanFilter distanceKalmanFilter(15, 15, 0.04);
int estimated_position=3000;

//********************************************************************
//VARIABLES
//--------------------------------------------------------------------
String command, c;
unsigned char run=0, read=0, calibrate=0, get=1;


void processCommand(String c)
{
  Serial.print("GETDATA : ");
  Serial.println(c);
  if(c.startsWith("RUN"))
  {
         run=1;
         Serial.println("OK");
  }
  if(c.startsWith("READ"))
  {
         read=~read;
         Serial.println("OK");
  }    
  if(c.startsWith("CAL"))
  {
         calibrate=1;
         Serial.println("OK");
  } 
    
  if(c.startsWith("GET"))
  {
         Serial.println("POSITION PID:");
         Serial.print("Kp=");
         Serial.print(Kp,3);
         Serial.print(" Kd=");
         Serial.print(Kd,3);
         Serial.print(" Ki=");
         Serial.println(Ki,3);
         Serial.println("SPEED PID:");
         Serial.print("Kp1=");
         Serial.print(Kp1,3);
         Serial.print(" Kd1=");
         Serial.print(Kd1,3);
         Serial.print(" Ki1=");
         Serial.println(Ki1,3);
         Serial.println("SPEEDs:");
         Serial.print("H=");
         Serial.print(maxSpeed);
         Serial.print(" L=");
         Serial.print(minSpeed);
         Serial.print(" B=");
         Serial.println(baseSpeed);   
         Serial.print("center=");
         Serial.println(center);                         
  }               
  if(c.startsWith("P"))
  {
      Kp=c.substring(1).toFloat(); 
      Serial.print("Kp=");
      Serial.println(Kp,3);
  }
  if(c.startsWith("I"))
  {
      Ki=c.substring(1).toFloat(); 
      Serial.print("Ki=");
      Serial.println(Ki,3);
  }

  if(c.startsWith("D"))
  {
      Kd=c.substring(1).toFloat(); 
      Serial.print("Kd=");
      Serial.println(Kd,3);
  }
  if(c.startsWith("p"))
  {
      Kp1=c.substring(1).toFloat(); 
      Serial.print("Kp1=");
      Serial.println(Kp1,3);
  }
  if(c.startsWith("i"))
  {
      Ki1=c.substring(1).toFloat(); 
      Serial.print("Ki1=");
      Serial.println(Ki1,3);
  }
  if(c.startsWith("d"))
  {
      Kd1=c.substring(1).toFloat(); 
      Serial.print("Kd1=");
      Serial.println(Kd1,3);
  }
  if(c.startsWith("H"))
  {
      maxSpeed=c.substring(1).toFloat(); 
      Serial.print("maxSpeed=");
      Serial.println(maxSpeed);
  }

  if(c.startsWith("L"))
  {
       minSpeed=c.substring(1).toFloat(); 
       Serial.print("minSpeed=");
       Serial.println(minSpeed);
  }

  if(c.startsWith("B"))
  {
      baseSpeed=c.substring(1).toFloat();         
      Serial.print("baseSpeed=");
      Serial.println(baseSpeed);
  } 
  if(c.startsWith("X"))
  {
      center=c.substring(1).toInt();         
      Serial.print("center=");
      Serial.println(center);
  }
  if(c.startsWith("sx"))
  {
      donja=c.substring(2).toFloat(); 
      Serial.print("donja=");
      Serial.println(donja);
  }
  if(c.startsWith("sy"))
  {
      gornja=c.substring(2).toFloat(); 
      Serial.print("gornja=");
      Serial.println(gornja);
  }        
  Serial.print("ENDDATA : ");
  Serial.println(c);
}


int readline(){
  // //A4, A2, A1, A3, A5
  // int count = 0;
  // // for(int i = 0 ; i < NUM_SENSORS ; i ++)
  // // {
  // //     Serial.println(sensor_values[i]);
  // // } 
  // // Serial.println(A4);
  // // Serial.println(A2);
  // // Serial.println(A1);
  // // Serial.println(A3);
  // // Serial.println(A5);

  // count = count + (digitalRead(sensor_values[0])*1000*-1);
  // count = count + (digitalRead(sensor_values[1])*1000*-1);
  // count = count + (digitalRead(sensor_values[2])*3000);
  // count = count + (digitalRead(sensor_values[3])*1000*1);
  // count = count + (digitalRead(sensor_values[4])*1000*1);
  // return count; 
  int a[5];
  a[0] = digitalRead(A4);
  a[1] = digitalRead(A3);
  a[2] = digitalRead(A2);
  a[3] = digitalRead(A1);
  a[4] = digitalRead(A0); 
  int v;
  v = (5000*a[0] + 4000*a[1] + 3000*a[2] + 2000*a[3] + 1000*a[4])/
      (a[0] + a[1] + a[2] + a[3] + a[4]);
  return v; 
}
 
//********************************************************************
//SETUP
//--------------------------------------------------------------------
void setup()
{
  delay(1000);
  Serial.begin(9600); 
}

//********************************************************************
//LOOP
//--------------------------------------------------------------------
void loop()
{
 
  if(Serial.available())
  {
    command=Serial.readString();
    processCommand(command);
    Serial.println(command);
    command="";
  }
  
  if(calibrate)
  {
    Serial.println("CALIBRATING...");
    delay(1000);
    for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
    {
      qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
    }
    Serial.println("DONE");
    calibrate=0;
    delay(1000);
  }
  

  if(read)
  {
    //test
    // read line sensor [0-6000]
    position = readline();
    //position = 3000;
    Serial.println(position);
    estimated_position = positionKalmanFilter.updateEstimate(position);
    error = estimated_position - center; 
    delay(500);
  }
  
  while(run)
  {
    //get position
    position = readline();
    
    Serial.println(position);
    //position = 6000; 
    estimated_position = positionKalmanFilter.updateEstimate(position);
    
    //get error
    error = estimated_position - center;
    d_error=error-last_error;
    i_error=i_error+error;
    //remember last error
    last_error = error; 

    //get abs error
    abs_error=abs(error);
    d_abs_error=abs_error-last_abs_error;
    i_abs_error=i_abs_error+abs_error;
    //remember last error
    last_abs_error = abs_error;
    
    //get motor correction speed from PIDs
    diffSpeed = Kp*error + Kd*d_error + Ki*i_error;
    correctBaseSpeed=Kp1*abs_error + Kd1*d_abs_error + Ki1*i_abs_error;
    
    //adjust left and right motor speed
    rightMotorSpeed = baseSpeed - correctBaseSpeed + diffSpeed;
    leftMotorSpeed = baseSpeed - correctBaseSpeed - diffSpeed;
  
    //check motor limits
    if (rightMotorSpeed > maxSpeed ) rightMotorSpeed = maxSpeed; // prevent the motor from going beyond max speed
    if (leftMotorSpeed > maxSpeed ) leftMotorSpeed = maxSpeed; // prevent the motor from going beyond max speed
    if (rightMotorSpeed < minSpeed) rightMotorSpeed = minSpeed; // keep the motor speed positive
    if (leftMotorSpeed < minSpeed) leftMotorSpeed = minSpeed; // keep the motor speed positive

    //drive
     motorLeft.drive(leftMotorSpeed);
     //Serial.print("motorLeft.drive(leftMotorSpeed) : ");
    // Serial.println(leftMotorSpeed);
     
     motorRight.drive(rightMotorSpeed);
     //Serial.print("motorLeft.drive(rightMotorSpeed) : ");
     //Serial.println(rightMotorSpeed);

    //delay(2000);
    // STOP COMMAND
    if(Serial.available())
    {
        motorLeft.brake();
        motorRight.brake();
        delay(20);
        motorLeft.drive(0);
        motorRight.drive(0);      
        command=Serial.readString();
        
      if(command.startsWith("STOP"))
      {
        run=0;
        Serial.println("OK");
      }else
      {
       motorLeft.drive(leftMotorSpeed);
       motorRight.drive(rightMotorSpeed);
      } 
      command="";
    }
  }
}

