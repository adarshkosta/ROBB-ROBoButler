#include <Servo.h>

#define DUTY 100  // For moving forward and back
#define DUTY1 255 // For Gripper 
#define DUTY2 190 // For turning left and right
#define DELAY1 400
#define DELAY2 100
#define DELAY3 400

const int lMotor = 8, rMotor = 9, inA = 53, inB = 51, inC = 49, inD = 47;
const int gripper = 10, ginA = 12, ginB = 11;
const int IRpin = A15;
Servo baseServo, armServo;

const int buzzPin = 2, battPin = A14;
int battValue = 0;

int gripCommand = false, gripStatus = false, armDownStatus = false;

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
    pinMode(buzzPin, OUTPUT);
    pinMode(battPin, INPUT);
    
    pinMode(IRpin, INPUT);
}

void buzzer(int num)
{
  if(num == 1)
  {
    digitalWrite(buzzPin, HIGH);
    delay(180);
    digitalWrite(buzzPin, LOW);
  }
  else if(num == 2)
  {
    digitalWrite(buzzPin, HIGH);
    delay(100);
    digitalWrite(buzzPin, LOW);
    delay(80);
    digitalWrite(buzzPin, HIGH);
    delay(100);
   digitalWrite(buzzPin, LOW); 
  }
}

void processMotionData()
{     
    String str1 = "", str2 = "";
    int i, s_len = 0, DELAY;
    
    if(BlueData[0] == 'f' || BlueData[0] == 'b' || BlueData[0] == 'r' || BlueData[0] == 'l')
    {  
      s_len = BlueData.length();
      
      for(i=0; BlueData[i] != ' '; i++)
      {
        str1 += BlueData[i];
      }
      i++;
      
      for(; i <= s_len; i++)
      {
         str2 += BlueData[i]; 
      }
      
      DELAY = str2.toInt();
    
      if(str1 == "forward")
      {
        //Serial.println("Moving Forward");
        driveMotors(DUTY, 0);
        delay(DELAY);
        driveMotors(DUTY, 4);
       // Serial.println("Stopped");
      }
    
      else if(str1 == "back")
      {
       // Serial.println("Moving Back");
        driveMotors(DUTY, 1);
        delay(DELAY);
        driveMotors(DUTY, 4);
       // Serial.println("Stopped");
      }
    
      else if(str1 == "right")
      {
        //Serial.println("Moving right");
        driveMotors(DUTY2, 3);
        delay(DELAY);
        //driveMotors(0, 3);
        driveMotors(DUTY2, 4);
       // Serial.println("Stopped");
      }
    
      else if(str1 == "left")
      {
       // Serial.println("Moving left");
        driveMotors(DUTY2, 2);
        delay(DELAY);
        //driveMotors(0, 2);
        driveMotors(DUTY2, 4);
       // Serial.println("Stopped");
      }
    
      else if (str1 == "brake")
      {
        driveMotors(DUTY, 4);
        delay(DELAY);
      }
      else if (str1 == "beep")
      { 
         buzzer(DELAY); 
      }
      else if (str1 == "batt")
      { 
         Serial.print("b");
         Serial.println(map(battValue, 0, 1023, 0, 255)); 
      }
      else
      {
        //Serial.println("Invalid Motion Command");
      }
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

void gotoGrippingPosition()
{
  baseServo.write(70);
  armServo.write(60);
}

void gotoReleasingPosition()
{
  baseServo.write(90);
  armServo.write(60);
}

void goto90Pose()
{
  baseServo.write(135);
  armServo.write(30);
}

void processRotationData()
{      
    String rotation_angle="";
    int s_len = BlueData.length();
    int angle =0;
    
    //Serial.println(BlueData);
    
    if (BlueData[0] == 's' || BlueData[0] == 'S')
    {
        for(int i=2;i<=s_len;i++)
        {
            rotation_angle += BlueData[i];
        }
        angle = (int)(rotation_angle.toInt());
        if(angle >= 90 && angle <= 180)  
        {  
           //Serial.print("Current base Angle : "); Serial.println(angle);
           baseServo.write(angle);      
        }
        else
        {
           //Serial.println("Invalid Rotation Command");
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
           //Serial.print("Current arm Angle : "); Serial.println(angle);
           armServo.write(angle);      
        }
        else
        {
           //Serial.println("Invalid Rotation Command");
        }
    } 
    else if(BlueData == "catch")  // Can add && gripStatus == false, over here in condition
    {
      handleGrip(0);
      gripStatus = true;
      Serial.println("gripped");
    }
    else if(BlueData == "drop")
    {
      handleGrip(1);
      gripStatus = false;
      gripCommand = false;
      Serial.println("dropped");
      delay(1000);
    }
    else if(BlueData == "arm down")
    {
      gotoGrippingPosition();
      armDownStatus = true;
      //delay(100); 
    }
    else if(BlueData == "arm up")
    {
      goto90Pose();
      armDownStatus = false; 
      //delay(200);
    }
    else if(BlueData == "arm reset")
    {
      armReset();
      armDownStatus = false; 
      //delay(1000);
    }
    else if(BlueData == "arm lower")
    {
      gotoReleasingPosition();
      armDownStatus = false; 
      //delay(1000);
    }
    else
    {
      //Serial.println("Invalid Rotation Command");
    }
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
       else if(gripStatus == false && armDownStatus == true)
         gripCommand = true;
    }
    
    if(gripCommand == true)
    {
        Serial.println("gripped");
        gripStatus = true;
        handleGrip(0); 
    }
  
  if(stringComplete == true)
  {
     processRotationData();
     processMotionData();
     stringComplete = false;
  }
  BlueData = "";
}

void checkBatt()
{
 battValue = analogRead(A14); 
}

//-----------------------------------------------------------------------// 
void loop() 
{ 
   handleBlueData();
   checkBatt();
}


void serialEvent() 
{
  while (Serial.available()) 
  {
    // get the new byte:
    delay(5); // Delay to get complete packet as a string
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    BlueData += inChar;
    
    //exits if no new data is lined up
  } 
  stringComplete = true;
}
