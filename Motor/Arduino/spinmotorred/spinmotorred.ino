#include <DualMC33926MotorShield.h>

#include <Encoder.h>


#define nD2 4 //turn on the motor
#define M1DIR 7 // direction of motor 1
#define nsF 12 // fault indication
#define M1FB A0 // current sensor for motor 1
#define m1Speed 9 //speed of motor 1 is defined as pin 9
#define PID_USE 0 //if using pid, sel=1
float IntThresh = 12;  
float Integral = 0;
float Last = 0;
float Drive;
unsigned long previousMillis = 0;        // will store last time LED was updated
// constants won't change :
const long interval = 5;           // interval at which to blink (milliseconds)
//Define Variables we'll be connecting to
double Setpoint = 6; //motorspeed


//Define the aggressive and conservative Tuning Parameters

double kP = 17.89637, kI = 18.51852, kD = 0; //test
//double kP =10, kI=1, kD=0;


Encoder myEnc(2, 3);
long oldPosition  = -999;

void setup() {
  pinMode(nD2, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(nsF, INPUT);

  digitalWrite(nD2, HIGH);
  digitalWrite(M1DIR, HIGH);
  delay(1000);
  
  //analogWrite(m1Speed, (Setpoint - 0.0501) / 0.0791);
  //analogWrite(m1Speed, Setpoint);
  //PID INITIALIZATION
  
  

  //turn the PID on
  

  Serial.begin(115200);
  // put your setup code here, to run once:
}

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {

    long newPosition = myEnc.read();
    long pulse = newPosition - oldPosition;
    long timepassed = currentMillis - previousMillis;
    Serial.print(currentMillis);
    Serial.print(", ");
   //Serial.print(timepassed);
   // Serial.print(", ");
   // Serial.print(pulse);
    
    float angularVel = (((float)pulse / (((float)timepassed) / 1000)) / 3200) * 2 * 3.14;
    
    //Serial.print(", "); 

    previousMillis = currentMillis;
    if (newPosition != oldPosition) {
      oldPosition = newPosition;
      // Serial.println(newPosition);
    }
    
    Serial.print(", RADIANS/SEC, ");
    Serial.print(angularVel);
    Serial.print(", ANALOG (RAD/S), ");
    Serial.print(Convert255(angularVel));
    Serial.print(", ANALOG (SETPOINT),  ");
    Serial.print(Convert255(Setpoint));
   if (PID_USE == 1) calcPID(timepassed, angularVel);
   else voltControl(); 

  }

}
void voltControl()
{
  analogWrite(m1Speed, Convert255(Setpoint));
  Serial.println(""); 
}

double Convert255(float value)
{
  value = value + .482;
  value = value / .04532;
  return value;
}

void calcPID(long timepassed, long Actual)
{
  //Actual = angularVel;
  float Error = Setpoint - Actual;

  if (abs(Error) < IntThresh) { // prevent integral 'windup'
    Integral = Integral + Error*5/1000; // accumulate the error integral
    Serial.print(" integral: ");
    Serial.print(Integral);
  }
  else {
    Integral = 0; // zero it if out of bounds
  }
  float P = Error * kP; // calc proportional term
  float I = Integral * kI ; // integral term
  Drive = P + I; // Total drive = P+I+D
  Drive = Convert255(Drive); // scale Drive to be in the range 0-255
  if (Drive < 0) { // Check which direction to go.
    digitalWrite (M1DIR, HIGH); // change direction as needed
  }
  else { // depending on the sign of Error
    digitalWrite (M1DIR, LOW);
  }
  if (abs(Drive) > 255) {
    Drive = 255;
  }
  analogWrite (m1Speed, Drive); // send PWM command to motor board
   Serial.print(" DRIVE: ");
    Serial.print(Drive);
   Serial.print(" ERROR: ");
    Serial.println(Error); 
  Last = Actual; // save current value for next time
}


