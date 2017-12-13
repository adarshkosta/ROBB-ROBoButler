#include <Servo.h>

#define DUTY 100
#define DUTY1 255
#define DUTY2 190
#define DELAY1 400
#define DELAY2 100
#define DELAY3 200

const int lMotor = 8, rMotor = 9, inA = 53, inB = 51, inC = 49, inD = 47;
const int gripper = 10, ginA = 12, ginB = 11;
const int IRpin = A15;
Servo baseServo, armServo;

int gripCommand = false, gripStatus = false;

String BlueData;
boolean stringComplete = false;
int mode = 0;

//BASIC FUNCTIONS
void _delay()
{
   delay(40);
}

void setPin(int pin)
{
  digitalWrite(pin, HIGH); 
}

void resetPin(int pin)
{
  digitalWrite(pin, LOW); 
}

boolean checkIR()
{
  if(digitalRead(IRpin) == HIGH)
  {
    return HIGH;
  }
  else if(digitalRead(IRpin) ==LOW)
  {
    return LOW;
  }
}

int driveMotors(int torque, int dir)  
{
  if (dir == 0)  
  {                                        // drive motors forward
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    digitalWrite(inC, HIGH);
    digitalWrite(inD, LOW);
  }  
  else if(dir == 1) 
  {                                     // drive motors backward
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    digitalWrite(inC, LOW);
    digitalWrite(inD, HIGH);
  }
  else if (dir == 2)  
  {                                        // drive motors left
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    digitalWrite(inC, LOW);
    digitalWrite(inD, HIGH);
  }  
  else if(dir == 3) 
  {                                     // drive motors right
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    digitalWrite(inC, HIGH);
    digitalWrite(inD, LOW);
  }
  else if(dir == 4) 
  {                                     // stall motors 
    digitalWrite(inA, HIGH);
    digitalWrite(inB, HIGH);
    digitalWrite(inC, HIGH);
    digitalWrite(inD, HIGH);
  }  
  analogWrite(rMotor,max(torque, 0)); 
  analogWrite(lMotor, max(torque,0)); 
}

int grip(int torque, int dir)  
{
  if (dir == 0)  
  {                                        // drive motors forward
    digitalWrite(ginA, HIGH);
    digitalWrite(ginB, LOW);
  }  
  else if(dir == 1) 
  {                                     // drive motors backward
    digitalWrite(ginA, LOW);
    digitalWrite(ginB, HIGH);
  } 
  else 
  {
    digitalWrite(ginA, HIGH);            // Stall
    digitalWrite(ginB, HIGH);
  }
  analogWrite(gripper,max(torque, 0)); 
}

void pinsInit()
{
  
    pinMode(inA, OUTPUT);
    pinMode(inB, OUTPUT);
    pinMode(inC, OUTPUT);
    pinMode(inD, OUTPUT);
    pinMode(lMotor, OUTPUT);
    pinMode(rMotor, OUTPUT);
    pinMode(ginA, OUTPUT);
    pinMode(ginB, OUTPUT);
    pinMode(gripper, OUTPUT);
    
    pinMode(IRpin, INPUT);
}

void processMotionData()
{        
    if(BlueData == "forward")
    {
      Serial.println("Moving Forward");
      driveMotors(DUTY, 0);
      delay(DELAY1);
      driveMotors(DUTY, 4);
      Serial.println("Stopped");
    }
  
    else if(BlueData == "back")
    {
      Serial.println("Moving Back");
      driveMotors(DUTY, 1);
      delay(DELAY1);
      driveMotors(DUTY, 4);
      Serial.println("Stopped");
    }
  
    else if (BlueData == "right")
    {
      Serial.println("Moving right");
      driveMotors(DUTY2, 3);
      delay(DELAY2);
      driveMotors(0, 3);
      //driveMotors(DUTY2, 4);
      Serial.println("Stopped");
    }
  
    else if ( BlueData == "left")
    {
      Serial.println("Moving left");
      driveMotors(DUTY2, 2);
      delay(DELAY2);
      driveMotors(0, 2);
      //driveMotors(DUTY2, 4);
      Serial.println("Stopped");
    }
  
    else if (BlueData == "brake")
    {
      driveMotors(DUTY, 4);
      delay(1000);
    }
    else
    {
      Serial.println("Invalid Motion Command");
    }
}

void armReset()
{
   armServo.write(0);
   baseServo.write(180);  
}

void handleGrip(int dir)
{
  if(dir == 0)
  {
    grip(DUTY1, dir);
    delay(DELAY3);
    grip(DUTY1, 3); 
  }
  else if(dir == 1)
  {
    grip(DUTY1, dir);
    delay(DELAY3);
    grip(DUTY1, 3); 
  }
  
}

void processRotationData()
{      
    String rotation_angle="";
    int s_len = BlueData.length();
    int angle =0;
    
   // boolean IRvalue = checkIR();
    
    if (BlueData[0] == 's' || BlueData[0] == 'S')
    {
        for(int i=2;i<=s_len;i++)
        {
            rotation_angle += BlueData[i];
        }
        angle = (int)(rotation_angle.toInt());
        if(angle >= 90 && angle <= 180)  
        {  
           Serial.print("Current base Angle : "); Serial.println(angle);
           baseServo.write(angle);      
        }
        else
        {
           Serial.println("Invalid Rotation Command");
        }
    }  
    else if (BlueData[0] == 'h' || BlueData[0] == 'H')
    {
        for(int i=2;i<=s_len;i++)
        {
            rotation_angle += BlueData[i];
        }
        angle = (int)(rotation_angle.toInt());
        if(angle <=65 && angle >= 0)  
        {  
           Serial.print("Current arm Angle : "); Serial.println(angle);
           armServo.write(angle);      
        }
        else
        {
           Serial.println("Invalid Rotation Command");
        }
    } 
    else if(BlueData == "catch")
    {
      Serial.println("Gripping Object");
      gripStatus = true;
      handleGrip(0);
    }
    else if(BlueData == "release")
    {
      Serial.println("Releasing Object");
      gripStatus = false;
      gripCommand = false;
      handleGrip(1);
      delay(2000);
    }
    else
    {
         armReset();
    }
}

void gotoGrippingPostion()
{
  baseServo.write(110);
  armServo.write(0);
}

void setup() 
{
  pinsInit(); 
  baseServo.attach(5);
  armServo.attach(6);
  
  Serial1.begin(9600);
  Serial.begin(115200);
  
  armReset();
}

void handleBlueData()
{
  
  if(checkIR() == true)
    {
      //Serial.println("Object within Grip Range");
      
       if(gripStatus == true)
         gripCommand = false;
       else if(gripStatus == false)
         gripCommand = true; 
    }
    
    if(gripCommand == true)
    {
        Serial.println("Gripping Object");
        gripStatus = true;
        handleGrip(0); 
    }
  
  if(stringComplete == true)
  {
     processMotionData();
     processRotationData();
     stringComplete = false;
  }
  BlueData = "";
}
//-----------------------------------------------------------------------// 
void loop() 
{ 
   handleBlueData();
//  if(checkIR() == true)
//  {
//    Serial.println("Object detected..!!");
//  } 
}


void serialEvent1() {
  while (Serial1.available()) {
    // get the new byte:
    delay(5); // Delay to get complete packet as a string
    char inChar = (char)Serial1.read(); 
    // add it to the inputString:
    BlueData += inChar;
    
    //exits if no new data is lined up
  }
  
  stringComplete = true;
}
