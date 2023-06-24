#include <Servo.h>

Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;

double Servo1Pos;
double Servo2Pos;
double Servo3Pos;
double Servo4Pos;

int PotVR1 = A0; //Maxpos Servo1
int PotVR2 = A1; //Maxpos Servo2
int PotVR3 = A2; //Maxpos Servo3
int PotVR4 = A3; //Maxpos Servo4
int PotVR5 = A4; //Minpos Servo1
int PotVR6 = A5; //Minpos Servo2
int PotVR7 = A6; //Minpos Servo3
int PotVR8 = A7; //Minpos Servo4

double PotVal1;
double PotVal2;
double PotVal3;
double PotVal4;
double PotVal5;
double PotVal6;
double PotVal7;
double PotVal8;

double Increment1;
double Increment2;
double Increment3;
double Increment4;

double Interval1 = 1.5; // Controls the speed at which the landing gear goes up
double Interval2 = 4; // Controls the speed at which the landing gear goes down

int RX = 7;
int RXValue;

bool Init; //initialisation


void setup() 
{
  Servo1.attach(2);
  Servo2.attach(3);
  Servo3.attach(4);
  Servo4.attach(5);

  Init = HIGH;
  
  Serial.begin(9600);
}

void loop() 
{

  PotVal1 = analogRead(PotVR1);
  PotVal1 = map(PotVal1, 0, 1023, 90, 180);

  
  PotVal2 = analogRead(PotVR2);
  PotVal2 = map(PotVal2, 0, 1023, 90, 180);

  
  PotVal3 = analogRead(PotVR3);
  PotVal3 = map(PotVal3, 0, 1023, 90, 180);

  
  PotVal4 = analogRead(PotVR4);
  PotVal4 = map(PotVal4, 0, 1023, 90, 180);

  
  PotVal5 = analogRead(PotVR5);
  PotVal5 = map(PotVal5, 0, 1023, 0, 90);

  
  PotVal6 = analogRead(PotVR6);
  PotVal6 = map(PotVal6, 0, 1023, 0, 90);

  
  PotVal7 = analogRead(PotVR7);
  PotVal7 = map(PotVal7, 0, 1023, 0, 90);

  
  PotVal8 = analogRead(PotVR8);
  PotVal8 = map(PotVal8, 0, 1023, 0, 90);

  if (Init == HIGH) //This 'if' function runs only for the first cycle and moves the legs to the extended min possition set by the potentiometers
  {
    Servo1.write(PotVal5);
    Servo2.write(PotVal6);
    Servo3.write(PotVal7);
    Servo4.write(PotVal8);
    Init = LOW;
  }

  RXValue = pulseIn (RX, HIGH, 50000);
  
  if (RXValue < 1500) //gear channel of RX = '1'
  {
    if (Servo1Pos < PotVal1) //if the servo possition is smaller than the max possition -> add 1 to servo possition
    {
      Servo1Pos = Servo1Pos + Interval1;
      Servo1.write(Servo1Pos);
    }
    if (Servo1Pos > PotVal1) //if the servo possition is bigger than the max possition -> subtract 1 from servo possition
    {
      Servo1Pos = Servo1Pos - Interval1;
      Servo1.write(Servo1Pos);
    }


    if (Servo2Pos < PotVal2) //if the servo possition is smaller than the max possition -> add 1 to servo possition
    {
      Servo2Pos = Servo2Pos + Interval1;
      Servo2.write(Servo2Pos);
    }
    if (Servo2Pos > PotVal2) //if the servo possition is bigger than the max possition -> subtract 1 from servo possition
    {
      Servo2Pos = Servo2Pos - Interval1;
      Servo2.write(Servo2Pos);
    }


    if (Servo3Pos < PotVal3) //if the servo possition is smaller than the max possition -> add 1 to servo possition
    {
      Servo3Pos = Servo3Pos + Interval1;
      Servo3.write(Servo3Pos);
    }
    if (Servo3Pos > PotVal3) //if the servo possition is bigger than the max possition -> subtract 1 from servo possition
    {
      Servo3Pos = Servo3Pos - Interval1;
      Servo3.write(Servo3Pos);
    }


    if (Servo4Pos < PotVal4) //if the servo possition is smaller than the max possition -> add 1 to servo possition
    {
      Servo4Pos = Servo4Pos + Interval1;
      Servo4.write(Servo4Pos);
    }
    if (Servo4Pos > PotVal4) //if the servo possition is bigger than the max possition -> subtract 1 from servo possition
    {
      Servo4Pos = Servo4Pos - Interval1;
      Servo4.write(Servo4Pos);
    }
  }
  
  
  else if (RXValue > 1500) //gear channel of RX = '0'
  {
    if (Servo1Pos < PotVal5) //if the servo possition is smaller than the min possition -> add 1 to servo possition
    {
      Servo1Pos = Servo1Pos + Interval2; 
      Servo1.write(Servo1Pos);
    }
    if (Servo1Pos > PotVal5) //if the servo possition is bigger than the min possition -> subtract 1 to servo possition
    {
      Servo1Pos = Servo1Pos - Interval2;
      Servo1.write(Servo1Pos);
    }

    
    if (Servo2Pos < PotVal6) //if the servo possition is smaller than the min possition -> add 1 to servo possition
    {
      Servo2Pos = Servo2Pos + Interval2; 
      Servo2.write(Servo2Pos);
    }
    if (Servo2Pos > PotVal6) //if the servo possition is bigger than the min possition -> subtract 1 to servo possition
    {
      Servo2Pos = Servo2Pos - Interval2;
      Servo2.write(Servo2Pos);
    }


    if (Servo3Pos < PotVal7) //if the servo possition is smaller than the min possition -> add 1 to servo possition
    {
      Servo3Pos = Servo3Pos + Interval2; 
      Servo3.write(Servo3Pos);
    }
    if (Servo3Pos > PotVal7) //if the servo possition is bigger than the min possition -> subtract 1 to servo possition
    {
      Servo3Pos = Servo3Pos - Interval2;
      Servo3.write(Servo3Pos);
    }


    if (Servo4Pos < PotVal8) //if the servo possition is smaller than the min possition -> add 1 to servo possition
    {
      Servo4Pos = Servo4Pos + Interval2; 
      Servo4.write(Servo4Pos);
    }
    if (Servo4Pos > PotVal8) //if the servo possition is bigger than the min possition -> subtract 1 to servo possition
    {
      Servo4Pos = Servo4Pos - Interval2;
      Servo4.write(Servo4Pos);
    }
  }
  

  
  /*
  Serial.print(PotVal1);
  Serial.print(", ");
  Serial.print(PotVal2);
  Serial.print(", ");
  Serial.print(PotVal3);
  Serial.print(", ");
  Serial.print(PotVal4);
  Serial.print(", ");
  Serial.print(PotVal5);
  Serial.print(", ");
  Serial.print(PotVal6);
  Serial.print(", ");
  Serial.print(PotVal7);
  Serial.print(", ");
  Serial.print(PotVal8);
  Serial.print(", ");
  Serial.print(Increment1);
  Serial.print(", ");
  Serial.print(Increment2);
  Serial.print(", ");
  Serial.print(Increment3);
  Serial.print(", ");
  Serial.print(Increment4);
  Serial.print(", ");
  Serial.println(RXValue);
  */
  
  
}