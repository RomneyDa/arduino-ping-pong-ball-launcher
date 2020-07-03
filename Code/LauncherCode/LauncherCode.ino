/****************************************************************
Author: Dallin Romney
Sketch Description: Preliminary Competition Code
*******************************************************************/

/****************************
 ** #defines and #includes **
 ****************************/ 
#include <Servo.h>

/***********************
 ** Global Variables ***
 ***********************/
//Pins
  int irPIN = A5; 
  int motorDirectionPin = 4;
  int motorPowerPin = 5;
  int solenoidPowerPin = 6;
  int solenoidDirectionPin = 7;
  int launchServoPin = 9;
  int reloaderPin = 10;
  int rightSwitch = 11;
  int leftSwitch = 12;
  int IRLED = 13;

//Switch Variables
  int values = 0;
  int buttonPressed = 0;
  int leftSwitchReading = 0;
  int rightSwitchReading = 0;

//CountStripes Variables
  int irSensorReading = 0;
  int threshold = 150; //ms
  int counts = 100;
  unsigned long stripeTime;
  unsigned long currentTime;
  boolean stripeBoolean = 0;
  boolean currentBoolean = 0;
  boolean previousBoolean = 0;

//Other Program Variables
  boolean motorOn = 0;
  boolean motorLeft = 1;
  int maxStripe = 38;
  int motorPower = 255;
  int solenoidPower = 255;
  int solenoidActivationTime = 400; //ms
  int servoSmallIncrement = 1;      //deg
  int servoLargeIncrement = 5;      //deg
  int launcherServoAngle = 2;      //deg
  int launcherReloadAngle = 60;    //deg
  int reloaderHoldAngle = 25;       //deg
  int reloaderDispenseAngle = 60;   //deg

//Target Variables
  int target = 0;
  int desiredPosition = 0;
  int writeToServo[6];
  byte userInput;
  byte driveTo[6];
  double xTarget[6];

//Servos
  Servo launchServo; //creates launch servo object
  Servo reloaderServo;

/********************
 **      Setup     **
 ********************/  
void setup() {

//Configure Pins, Attach Servos
  pinMode(irPIN,INPUT);
  pinMode(leftSwitch,INPUT_PULLUP);
  pinMode(rightSwitch,INPUT_PULLUP);
  pinMode(solenoidPowerPin,OUTPUT);
  pinMode(solenoidDirectionPin,OUTPUT);
  pinMode(IRLED,OUTPUT);
  pinMode(reloaderPin, OUTPUT);
  
  launchServo.attach(launchServoPin);
  reloaderServo.attach(reloaderPin);

//Initialize serial communication
  Serial.begin(9600);

//Initial Hardware Movements
  reloaderServo.write(reloaderHoldAngle);
  launchServo.write(launcherServoAngle);
  
//Send matlab ready signal
  Serial.write(1);

//Read incoming targeting data from matlab, convert to useful numbers, and return it
  int k;
  for(k = 0; k < 6; k++) {
      while (Serial.available() < 3){}
  
      byte encoderPos = Serial.read();
      byte xTarget_HB = Serial.read();
      byte xTarget_LB = Serial.read();

      double xTarget_mm = 256*xTarget_HB + xTarget_LB;
      double xTarget_m = xTarget_mm / 1000.0;

      driveTo[k] = encoderPos;
      xTarget[k] = xTarget_m;
  } 

  //xTarget[1] = xTarget[5];
//Display stripe destination and distance data for each target
  for(k = 0; k < 6; k++) {
      Serial.print("For target ");
      Serial.print(k + 1);
      Serial.print(", drive to stripe ");
      Serial.print(driveTo[k]);
      Serial.print(" and aim at ");
      Serial.print(xTarget[k], 3);
      Serial.println(" m.");
  }
  
//Check initial launcher position
  leftSwitchReading = digitalRead(leftSwitch);

  if (leftSwitchReading == 1){
      Serial.println("Launcher is at home position moving to setup position");
      counts = 0;
      MoveLauncher(15);
      delay(5000);
  }
  else {
      Serial.println("Launcher is moving setup position");
      MoveLauncher(81);
      delay(5000);
  }
  
  
  if (leftSwitchReading == 1){
      Serial.println("Launcher is already at home position");
      counts = 0;
  }
  else {
      Serial.println("Launcher is moving home");
      MoveLauncher(-10);
  }
    
//Initial Readings
  stripeBoolean = GetEncoderBoolean();
  stripeTime = millis();

//Initialize cannon parameters
  double d1 = 4.1/100.;  
  double d2 = 19.0/100.; 
  double d3 = 6.7/100.;  
  double d[3] = {d1, d2, d3};
  
  double v0 = 3.34; 
  
  double H1 = 0.1313; 
  double H2 = 0.128;  
  double H3 = 0.096;  
  double H4 = 0.0315; 
  double H[4] = {H1, H2, H3, H4};
  
  double launchAngleOffset = 11.91;
  double servoAngleOffset = 31.67;
  
//Blink the Infrared LED to start the timer
  digitalWrite(IRLED, HIGH); //ON
  delay(1000);               //DELAY
  digitalWrite(IRLED, LOW);  //OFF

  Serial.println("Timer is being started.");
  
//Calculate the servo angle for each target and display it
  TargetServoAngles(d, v0, H, servoAngleOffset, launchAngleOffset, xTarget);
  for(k = 0; k < 6; k++) {
      Serial.print("Target distance = ");
      Serial.print(xTarget[k]);
      Serial.print(" --> Servo angle = ");
      Serial.print(writeToServo[k]);
      Serial.println(" deg");
  }
  
  //Serial.println(); //Send final message (MATLAB cut off by empty message)
}

/*******************
 ** Loop Function **
 *******************/
void loop() {
  if(target == 0){
      Serial.println("Moving Launcher");
      MoveLauncher(driveTo[target]);
      
      Serial.println("Adjusting Servo.");
      launchServo.write(writeToServo[target]);
      
      Serial.println("Firing Solenoid!");
      FireSolenoid();
      
      Reload();
      
      target++;
  }
  else if(target == 1){
      Serial.println("Moving Launcher");
      MoveLauncher(driveTo[target]);
      
      Serial.println("Adjusting Servo.");
      launchServo.write(writeToServo[target]);
      
      Serial.println("Firing Solenoid!");
      FireSolenoid();
      
      Reload();
      
      target++;
  }
  else if(target < 5){
      Serial.println("Moving Launcher");
      MoveLauncher(driveTo[target]);
      
      Serial.println("Adjusting Servo.");
      launchServo.write(writeToServo[target]);
      
      Serial.println("Firing Solenoid!");
      FireSolenoid();
      
      Reload();
      
      target++;
  }
  else {
      Serial.println("Moving launcher to final target");
      MoveLauncher(driveTo[target]);
      
      Serial.println("Adjusting Servo.");
      launchServo.write(writeToServo[target]);
      
      Serial.println("Firing Solenoid!");
      FireSolenoid();
      
      MoveLauncher(-10);
      
      if (counts == 0);{
          digitalWrite(IRLED, HIGH); //ON
          delay(1000);               //DELAY
          digitalWrite(IRLED, LOW);  //OFF
          Serial.println("");
          while(1){}                 //ALL DONE!
      }
  }
  return;
}

/****************************
 ** User-Defined Functions **
 ****************************/

void TurnMotorOn(){
    digitalWrite(motorDirectionPin,motorLeft); //Set direction
    analogWrite(motorPowerPin,motorPower);     //Set speed
    motorOn = 1;                               //ON
}

void BrakeMotor(){
    analogWrite(motorPowerPin, LOW);           //Power Off
    motorOn = 0;                               //OFF
    delay(10);                                 //ms
  
    motorLeft = !motorLeft;                    //Change Directions
    digitalWrite(motorDirectionPin,motorLeft); //Reset Direction
    
    analogWrite(motorPowerPin,motorPower);     //Power on
    motorOn = 1;                               //ON
    delay(50);                                 //ms
  
    analogWrite(motorPowerPin, LOW);           //Power Off
    motorOn = 0;                               //OFF
}

boolean GetEncoderBoolean(){
    irSensorReading = analogRead(irPIN);       //Read IR pin
    return (irSensorReading >= threshold);     //Is the value greater than the threshold?
}

void CountStripes(){
  delay(5);
  unsigned long elapsedTime;
  //currentBoolean = GetEncoderBoolean();
  previousBoolean = currentBoolean;
  currentBoolean = GetEncoderBoolean();
  currentTime = millis();
  elapsedTime = currentTime - stripeTime;
  if ((elapsedTime > 50) && (currentBoolean != stripeBoolean) && (currentBoolean == previousBoolean)){
    if (motorLeft){
      counts--;
    }
    else {
      counts++;
    }
    stripeBoolean = currentBoolean;
    stripeTime = currentTime;
    Serial.print("Counts: ");
    Serial.print(counts);
    Serial.print("    ElapsedTime: ");
    Serial.println(elapsedTime);
  }
}

void FireSolenoid(){
    digitalWrite(solenoidDirectionPin, HIGH); //sets the direction of the solenoid 
    delay(solenoidActivationTime); 
    analogWrite(solenoidPowerPin, solenoidPower);  //fires cannon
    delay(300);
    analogWrite(solenoidPowerPin, LOW); //turns cannon off
}

void MoveLauncher(int desiredPosition){
    if (desiredPosition == counts){
        Serial.print("Launcher already at ");
        Serial.println(desiredPosition);
        return;
    }
    else if (desiredPosition > counts){
        motorLeft = 0;
    }
    else {
        motorLeft = 1;
    }
  
    TurnMotorOn();
    while (desiredPosition != counts){
        leftSwitchReading = digitalRead(leftSwitch);
        rightSwitchReading = digitalRead(rightSwitch);
        if((motorLeft == 0) && (rightSwitchReading == 1)){
          break;
        }
        if((motorLeft == 1) && (leftSwitchReading == 1)){
          break;
        }
        CountStripes();    
    }
    BrakeMotor();
    if (leftSwitchReading == 1){
        counts = 0;
        Serial.println("Launcher is home, counts reset to 0");
    }
    if (rightSwitchReading == 1){
        counts = maxStripe;
        Serial.println("Launcher is at reloader, counts reset to maxStripe");
    }
    return;
}

void Reload(){
   launchServo.write(launcherReloadAngle);          //Move to the end
   MoveLauncher(maxStripe+10);
   
   reloaderServo.write(reloaderDispenseAngle);      //Open the gate
   Serial.println("Servo is in dispense position.");
   delay(310);
   
   reloaderServo.write(reloaderHoldAngle);          //Close the gate
   Serial.println("Servo is in hold position.");
   
   return;
}
