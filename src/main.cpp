#include <QTRSensors.h>
#include <SparkFun_TB6612.h> 
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
float Kp=120.00; // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
float Kd=150.00; // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
float Ki=0.000;
//********************************************************************
//PID FOR BASE SPEED
//--------------------------------------------------------------------
float Kp1=0.000;
float Kd1=0.000;
float Ki1=0.000;

float gornja=20.0;
float donja=15.0;

int maxSpeed=255;  // max speed of the robot
int minSpeed=-255; // min speed of the robot
int baseSpeed=200; // this is the speed at which the motors should spin when the robot is perfectly on the line

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



float P=0, I=0, D=0, PID_value=0;
float previous_error=0, previous_I=0;







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
  } 
    
  if(c.startsWith("GET"))
  {
                                 
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

//********************************************************************
//SETUP
//--------------------------------------------------------------------
// float calib[5];
// void calribrate(){
//   calib[0] = (float)analogRead(A0)*5/1024;
//     calib[1] = (float)analogRead(A1)*5/1024;
//     calib[2] = (float)analogRead(A2)*5/1024;
//     calib[3] = (float)analogRead(A3)*5/1024;
//     calib[4] = (float)analogRead(A4)*5/1024;

//     int sum = 0 ;
//     for(int i = 0; i < 5 ; i ++){

//     }
// }

void getError()
{
  
    int sensor[5];
    float sensorValue[5];
    sensorValue[0] = (float)analogRead(A0)*5/1024;
    sensorValue[1] = (float)analogRead(A1)*5/1024;
    sensorValue[2] = (float)analogRead(A2)*5/1024;
    sensorValue[3] = (float)analogRead(A3)*5/1024;
    sensorValue[4] = (float)analogRead(A4)*5/1024;
    if(read){
      Serial.println();
      for(int i = 0 ; i < 5 ; i++)
      {
        Serial.print("A");
        Serial.print(i);
        Serial.print(" : ");
        Serial.println(sensorValue[i]);
      }  
    }
    if(sensorValue[0] < 4)
    {
      sensor[0] = 1;
    }
    else
    {
    sensor[0] = 0;
    }
    if(sensorValue[1] < 4)
    {
      sensor[1] = 1;
    } 
    else
    {
      sensor[1] = 0;
    }
    if(sensorValue[2] < 4)
    {
      sensor[2] = 1;
    } 
    else
    {
      sensor[2] = 0;
    }
    if(sensorValue[3] < 4)
    {
      sensor[3] = 1;
    } 
    else
    {
      sensor[3] = 0;
    }
    if(sensorValue[4] < 4.5)
    {
      sensor[4] = 1;
    } 
    else
    {
       sensor[4] = 0;
    }   
      
    Serial.print(sensor[0]);
    Serial.print(sensor[1]);  
    Serial.print(sensor[2]);  
    Serial.print(sensor[3]);  
    Serial.print(sensor[4]); 

   int pidCount[] = {-3,-1,0,1,3};
   int tempCount =0;
    for(int i = 0 ; i < 5 ; i++)
    {
        tempCount =tempCount+ sensor[i] * pidCount[i];
        
    }
    if(tempCount != 0)
    {
      error = tempCount;
    }
    else
    {
      if(sensor[2]  == 1)
      {
        error = 0;
      }
      else
      {
          error = error;
      }
      
    }
    
 
  
}
void setup()
{ 
  Serial.begin(9600);
}
 
void loop()
{
 
  if(Serial.available())
  {
    command=Serial.readString();
    processCommand(command);
    command="";
  } 
  

  if(read)
  { 
      getError(); 
      Serial.println();
      Serial.print("ERROR : ");
      Serial.print(error);   

  }
  
  while(run)
  {
    
    getError();
    Serial.print("ERROR : ");
    Serial.print(error);



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
    

    // Kp=10.000; // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
    // Kd=0.000; // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
    // Ki=0.000;

    // Kp1=0.000;
    // Kd1=0.000;
    // Ki1=0.000;


    P = error;
    I = I + error;
    D = error - previous_error;
    
    PID_value = (Kp*P) + (Ki*I) + (Kd*D);
    previous_error=error;

    //get motor correction speed from PIDs
    diffSpeed = Kp*error + Kd*d_error + Ki*i_error;
    correctBaseSpeed=Kp1*abs_error + Kd1*d_abs_error + Ki1*i_abs_error ; 
    
    //adjust left and right motor speed
    rightMotorSpeed = baseSpeed - correctBaseSpeed + PID_value;
    leftMotorSpeed = baseSpeed - correctBaseSpeed - PID_value;
  
    //check motor limits
    if (rightMotorSpeed > maxSpeed ) rightMotorSpeed = maxSpeed; // prevent the motor from going beyond max speed
    if (leftMotorSpeed > maxSpeed ) leftMotorSpeed = maxSpeed; // prevent the motor from going beyond max speed
    if (rightMotorSpeed < minSpeed) rightMotorSpeed = minSpeed; // keep the motor speed positive
    if (leftMotorSpeed < minSpeed) leftMotorSpeed = minSpeed; // keep the motor speed positive

    //drive

    Serial.print(" : lMS->");
    Serial.print(leftMotorSpeed);
    Serial.print(" : rMS->");
    Serial.print(rightMotorSpeed);
    motorLeft.drive(leftMotorSpeed);
    motorRight.drive(rightMotorSpeed);


    Serial.println();
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
       //motorLeft.drive(leftMotorSpeed);
       //motorRight.drive(rightMotorSpeed);
      } 
      command="";
    }

  }
}
