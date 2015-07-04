/*****************************************************************************************
 * Senior Design Project: Electronic "Soft Differential"
 * Credit: UNCA-NCSU Senior Class 2014
 * Purpose: The purpose of this program is to read in various inputs from two wheel speed
 *          encoders, one steering potentiometer, and one throttle potentiometer.  The 
 *          code will determine if the car is straight or turning, and by how much it is
 *          turning.  An algorithm will process this and send outputs to two electric
 *          motors which will run at different speeds in a turn to simulate what is 
 *          commonly seen in a regular mechanical differential.
 *  
 * Versions & Version Notes:
 *   v0.1.0- Code written for analog input of potentiometer inputs for steering and throttle
 *      Credit: Dakota Lazenby
 *   v0.1.1- Added support for motor encoders and modified structure
 *      Credit: Dakota Lazenby
 *   v1.0.0- Ported code over for Arduino Due and added throttle response functionality
 *      Credit: Dakota Lazenby
 *   v1.0.1- Imported simplified working motor encoder code
 *      Credit: Brandon Zschokke, Jennifer Cory, and Hallie Sheaffer
 *   v2.0.0- Combined and created confirmed working throttle code and encoder code
 *      Credit: Brandon Zschokke
 *   v2.1.0- Re-Located the encoder code to a function, cleaned up some variables and added conditional prints
 *      Credit: Dakota Lazenby
 *   v2.2.0- Added in a throttleFilter function.
 *      Credit: Dakota Lazenby
 *   v2.3.0- Added functionality for power LED, checking pack voltage from LiPo battery, new encoder Z pin reads
 *            at high RPMs, 
 *      Credit: Dakota Lazenby, Hallie Shaeffer
 *    
 * Future Versions / Roadmap:
 *   v2.x.x- Implement Hallie's SD Card code for data logging. Implement Steering Code
 *   v3.0.0- Implement controller code
 *   v3.x.x- Debug and refine control code
 *   v4.0.0- Remove excess global variables and modularize code into various functions   
 *****************************************************************************************/

#define DEBUG      1
#define USER_INPUT 1
#define ENCODERS   1
#define SPEED      1
#define ADC_DELAY 10
// Global Declaration Section

int controlScheme = 1;
int loopCount = 0;

// Throttle Declarations
int throttlePin = A11; // Throttle Pot Signal
int leftMotor = DAC0;  // To Protoboard (5)
int rightMotor = DAC1;  // To Protoboard (6)
int minThrottle = 7;
int maxThrottle = 235; // This value needs to correspond to the maximum voltage reading off of the potentiometer 
                       // plugged into the formula : maxThrottle = measuredVoltage * (3.3/(2^Res)) 
int throttle_in_left = 0;
int throttle_in_right = 0;

int Res = 10;  // Resolution
int noiseDelay = 10; //Delay to ensure that analogReads are performed correctly

// Steering Declarations
int steeringPin = A10; // Steering Pot Signal
int steeringMax = 600;   //THESE VALUES NEED TO BE MEASURED AND ENTERED CORRECTLY
int steeringMidpt = 400;  //IDEALLY WE WRITE CALIBRATION CODE FOR THESE VALUES
int steeringMin = 200;
int Lr = 35;                    //35 inches from center to center on rear wheels
int minTurnRadiusRight = 148;  //Minimum turn radius in inches
int minTurnRadiusLeft = 244;
int leftSteerBuffer = -75;
int rightSteerBuffer = 75;

// Encoder Declarations 
 //CHECK THESE PINOUTS!!!!!
 int encoder0PinA = 25; // Encoder A2
 int encoder0PinB = 29; // Encoder A1
 int encoder0PinZ = 33;
 int encoder1PinA = 23; // Encoder B2
 int encoder1PinB = 27; // Encoder B1
 int encoder1PinZ = 31;
 int maxRPM = 20;
 int RPM_0_Last = 0;
 int RPM_1_Last = 0;
 
 //PID control Variables
 int Kp = 10;
 int Ki = 1;
 int Kd = 0;
 long loopStart = 0;
 long loopEnd = 5000; //Need to tune this value for first run
 long dT = 0;
 
 double control = 0;
 double error = 0;
 double controlPrev = 0;
 double errorPrev = 0;

 //Encoder read variables
 int encoderPos = 0;
 int encoderPinALast = LOW;
 int encoderSampleTime = 125;  //Sample time for the encoders (in milliSeconds)
 int RPM_0 = 0;
 int RPM_1 = 0;
 
 double steerLast = 0;
 
 //Pack voltage variables
 int minPackVoltage = 720;  //Integer value corresponding to roughly 9.65 volts (safe operating LiPo voltage)
 
 // Extra Pins
 int PwrIn = A9;  //Pin to read the battery pack nominal voltage
 int LEDPwr = 11; //Pin to control the "power to arduino" LED
 int SDPin = 10;
 int PullDownPin = 9;          // Press Sensor pin
 int LEDPin = 8;          //Red LED in digital pin 8 with resistor in series.
//------------------------------------------------------------------------------------------



/****************************************************************************************
 ****************************************************************************************
 * Functions are defined below:
 ****************************************************************************************
 ****************************************************************************************/
 
int readEncoderAB(int encoderPinA, int encoderPinB){
  int n = 0;
  int RPM = 0;
    uint32_t time = millis();
   while(millis() < (time + encoderSampleTime)){
     
   n = digitalRead(encoderPinA);
   if ((encoderPinALast == LOW) && (n == HIGH)) {
     if (digitalRead(encoderPinB) == LOW) {
       encoderPos--;
     } else {
       encoderPos++;
     }
   } 
   encoderPinALast = n;
 }
  encoderPos = 0;
  RPM = (((encoderPos*(1000/encoderSampleTime)*60)/400)/5);  //In terms of wheel RPM
  return(RPM);
}

int readEncoderZ(int encoderPinZ){
  int RPM = 0;
  int Z_Rev = 0;
    long time = micros();
    long no_time = 0;
     //Serial.print("Begin Read");
     while((digitalRead(encoderPinZ) == LOW)&&((no_time = ((micros() - time)/1000)) < 125)) {} 
     while((digitalRead(encoderPinZ) == HIGH)&&((no_time = ((micros() - time)/1000)) < 125)) {}
     long time1 = micros();     //Take a timestamp
      while ((Z_Rev >= 0)&&(Z_Rev < 20)&&((no_time = ((micros() - time)/1000)) < 125)) {     // Turn 10 Revolutions
          //Serial.print("Reading");
          if ((digitalRead(encoderPinZ)) == LOW) {  //Test for high pulse
            while (((digitalRead(encoderPinZ)) == LOW)&&((no_time = ((micros() - time)/1000)) < 125)) {}  //Wait for a pulse
               Z_Rev++;
           } 
       }
       long time2 = micros() - time1;
       RPM = ((Z_Rev + 1)*1000*1000*60/time2)/5; 
       if(RPM > 750){
        RPM = 0; 
       }
       if(no_time > 150){
         Serial.println("NO_TIME_NO_TIME_NO_TIME_NO_TIME_NO_TIME_NO_TIME_NO_TIME_NO_TIME_NO_TIME_NO_TIME");
         RPM = 0;
       }
 return(RPM);
}

//*****************************************************************************************************
//*****************************************************************************************************

int packVoltage(){
    int Voltage = analogRead(PwrIn);
    delay(ADC_DELAY);
    if(Voltage < minPackVoltage) {
       while(1){  
         //Infinite Loop Halting all other processes other than indicating low voltage (LED Blinks)
         digitalWrite(LEDPwr, HIGH);
         delay(500);
         digitalWrite(LEDPwr, LOW);
         delay(500);
       }
    }
}
  
//*****************************************************************************************************  
//*****************************************************************************************************

void differential(int Steering, int throttle){
  //UNFINISHED, NEED TO WORK OUT EQUATIONS FOR Ri AND THEN WORK OUT GOOD CODE FOR DIFFERENTIAL
  //THESE EQUATIONS ARE EXTREMELY RUDIMENTARY AND NEED TUNING BUT THEY SHOULD "WORK"...
    double IV, OV, Ri;
    double idealDiff;
    double steerDifference = Steering - steeringMidpt;
    double controlApplied;
    double rpmDifference;
    
    if(((steerDifference >= 0) && (steerLast <= 0)) || ((steerDifference <= 0) && (steerLast >= 0))){
       control = 0;  //This is the "passed through zero" check and resets control to accomodate that
    }
    
    if(steerDifference < leftSteerBuffer){
      Ri = minTurnRadiusLeft*((steeringMidpt - steeringMin)/(steeringMidpt - Steering));
      if((RPM_1 == 0)){
        rpmDifference = 0;
      } else {
        rpmDifference = ((RPM_1) - (RPM_0))/(RPM_1);  //Outer wheel minus inner wheel
      }
    } else if(steerDifference > rightSteerBuffer){
      Ri = minTurnRadiusRight*((steeringMax - steeringMidpt)/(steeringMax - Steering));
      rpmDifference = ((RPM_0) - (RPM_1))/(RPM_0);  //Outer wheel minus inner wheel
      if((RPM_0 == 0)){
        rpmDifference = 0;
      } else {
        rpmDifference = ((RPM_0) - (RPM_1))/(RPM_0);  //Outer wheel minus inner wheel
      }
    } else {
      Ri = 35000; // This value is "infinity" because the radius of curvature for a straight line is infinity
    }
    
    #if DEBUG
        Serial.print("rpmDifference = ");
        Serial.println(rpmDifference);
        Serial.print("Ri = ");
        Serial.println(Ri);
    #endif
      
      OV = throttle;      //currently throttle referenced, need to get in speed referenced??
      IV = (Ri/(Ri+Lr))*OV;
      
      #if DEBUG
        Serial.print("OV = ");
        Serial.print(OV);
        Serial.print(".....IV = ");
        Serial.println(IV);
      #endif
      
      idealDiff = ((OV - IV)/OV);
      
      #if DEBUG
        Serial.print("Weighted difference of velocities = ");
        Serial.println(idealDiff);
      #endif
      
      // INSERT PI CONTROL
      error = idealDiff - rpmDifference;
      
      #if DEBUG
        Serial.print("Error (calculated)");
        Serial.println(error);
      #endif  

      dT = loopEnd - loopStart;
      dT = dT/(1000*1000);
      //dT = 1;
      control = (Kp*error)+(Ki*(((error+errorPrev)/2)*dT+controlPrev))+(Kd*((error-errorPrev)/dT));
      
      #if DEBUG
        Serial.print("deltaTime = ");
        Serial.println(dT);
        Serial.print("Control (calculated) = ");
        Serial.println(control);
      #endif  
      
      controlApplied = control;
      
      if(steerDifference < leftSteerBuffer){
        throttle_in_right = throttle;
        throttle_in_left = throttle - control; // plus or minus control???  
      } else if(steerDifference > rightSteerBuffer){  
        throttle_in_right = throttle - control; // plus or minus control???
        throttle_in_left = throttle;
      } else {
        throttle_in_right = throttle;
        throttle_in_left = throttle;
      }
      
      #if DEBUG
        Serial.print("Throttle (right calculated) = ");
        Serial.println(throttle_in_right);
        Serial.print("Throttle (left calculated) = ");
        Serial.println(throttle_in_left);
      #endif  
      
      errorPrev = error;
      controlPrev = control;
      steerLast = steerDifference;
    
}

//*****************************************************************************************************  
//*****************************************************************************************************

void calibrateThrottle () {
   //This function establishes min and max throttle values______________________________
  pinMode (PullDownPin, INPUT);
  digitalWrite(PullDownPin, HIGH);
  pinMode(LEDPin, OUTPUT);
  while (digitalRead(PullDownPin) == HIGH)  {
   digitalWrite(LEDPin, LOW);
   delay(500);
   digitalWrite(LEDPin, HIGH);
   delay(500); 
  }
  analogReadResolution(Res);
  minThrottle = analogRead(throttlePin);
  delay(ADC_DELAY);
  minThrottle = analogRead(throttlePin);
  delay(ADC_DELAY);
  #if DEBUG
    Serial.print("minThrottle = ");
    Serial.print(minThrottle);
  #endif  
  digitalWrite(LEDPin, LOW);
  delay(1000);
  while (digitalRead(PullDownPin) == LOW)  {
   digitalWrite(LEDPin, LOW);
   delay(500);
   digitalWrite(LEDPin, HIGH);
   delay(500); 
  }
  analogReadResolution(Res);
  maxThrottle = analogRead(throttlePin);
  delay(ADC_DELAY);
  maxThrottle = analogRead(throttlePin);  
  delay(ADC_DELAY);
  maxThrottle = (maxThrottle + 10);
  #if DEBUG
    Serial.print(".......maxThrottle = ");
    Serial.println(maxThrottle);
  #endif  
  digitalWrite(LEDPin, LOW);
  delay(500);
}

//*****************************************************************************************************  
//*****************************************************************************************************

void calibrateSteering () {
   //This function establishes min and max throttle values______________________________
  pinMode (PullDownPin, INPUT);
  digitalWrite(PullDownPin, HIGH);
  pinMode(LEDPin, OUTPUT);
  while (digitalRead(PullDownPin) == HIGH)  {
   digitalWrite(LEDPin, LOW);
   delay(200);
   digitalWrite(LEDPin, HIGH);
   delay(200); 
  }
  analogReadResolution(Res);
  steeringMin = analogRead(steeringPin);  //Pull steering wheel full left
  delay(ADC_DELAY);
  steeringMin = analogRead(steeringPin);
  delay(ADC_DELAY);
  #if DEBUG
    Serial.print("minSteering = ");
    Serial.print(steeringMin);
  #endif  
  digitalWrite(LEDPin, LOW);
  delay(1000);
  while (digitalRead(PullDownPin) == LOW)  {
   digitalWrite(LEDPin, LOW);
   delay(200);
   digitalWrite(LEDPin, HIGH);
   delay(200); 
  }
  analogReadResolution(Res);
  steeringMax = analogRead(steeringPin);  //Pull steering wheel full right
  delay(ADC_DELAY);
  steeringMax = analogRead(steeringPin);  
  delay(ADC_DELAY);
  #if DEBUG
    Serial.print(".......maxSteering = ");
    Serial.print(steeringMax);
  #endif  
  while (digitalRead(PullDownPin) == HIGH)  {
   digitalWrite(LEDPin, LOW);
   delay(75);
   digitalWrite(LEDPin, HIGH);
   delay(75); 
  }
  while (digitalRead(PullDownPin) == LOW)  {
   digitalWrite(LEDPin, HIGH);
   delay(250); 
  }
  analogReadResolution(Res);
  steeringMidpt = analogRead(steeringPin);
  delay(ADC_DELAY);
  steeringMidpt = analogRead(steeringPin);  
  delay(ADC_DELAY);
  digitalWrite(LEDPin, LOW);
  delay(500);
  steeringMidpt = (steeringMidpt + (steeringMax/2))/2;
  #if DEBUG
    Serial.print(".......midSteering = ");
    Serial.println(steeringMidpt);
  #endif  
}


//*****************************************************************************************************
//*****************************************************************************************************


void setup() {
  // Setup encoder inputs
  pinMode (encoder0PinA,INPUT);
  pinMode (encoder0PinB,INPUT);
  pinMode (encoder0PinZ,INPUT);
  pinMode (encoder1PinA,INPUT);
  pinMode (encoder1PinB,INPUT);
  pinMode (encoder1PinZ,INPUT);
  pinMode (LEDPwr, OUTPUT);
  digitalWrite(LEDPwr, HIGH);
  Serial.begin (9600); // Start serial output for debugging
  loopCount = 0;
  calibrateThrottle();
  calibrateSteering();
}

/***************************************************************************************************** 
 * \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
 * /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
 *!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! MAIN LOOP STARTS HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
 * \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
 *****************************************************************************************************/ 


void loop() {
  
  loopStart = millis();
  //packVoltage();  //Check the battery pack to see if it is within safe operating voltage
 // This section of the code uses the throttle code by Dakota to drive the cart from the throttle potentiometer

  analogReadResolution(Res);
  int throttle_in = analogRead(throttlePin);
  delay(ADC_DELAY);   // Delays after analogRead to allow capacitors to charge and cancels cross talk between reads 
  throttle_in = analogRead(throttlePin);                    

  analogReadResolution(Res);
  int Steering = analogRead(steeringPin);
  delay(ADC_DELAY);
  Steering = analogRead(steeringPin);
  
      #if DEBUG
         #if USER_INPUT  
        Serial.print("Gas: ");                            //Debug Statements ~ visual data
        Serial.println(throttle_in);
        Serial.print("Steering");
        Serial.println(Steering);
        #endif
      #endif  
  
  /*if ((RPM_0_Last < maxRPM) || (RPM_1_Last < maxRPM)){
    RPM_0 = readEncoderAB(encoder0PinA, encoder0PinB);  //This function has no output because it changes global variables within the function.
                              // Read Encoder pins A and B if RPM is in "low range" (Reads for both encoders)
    RPM_1 = readEncoderAB(encoder1PinA, encoder1PinB);  //This function has no output because it changes global variables within the function.
                              // Read Encoder pins A and B if RPM is in "low range" (Reads for both encoders)
  } else {
    RPM_0 = readEncoderZ(encoder0PinZ);   // Read Encoder pin Z if RPM is in "high range" (Reads for both encoders)
    RPM_1 = readEncoderZ(encoder1PinZ);   // Read Encoder pin Z if RPM is in "high range" (Reads for both encoders)
  }*/
  
  RPM_0 = readEncoderZ(encoder0PinZ);   // Read Encoder pin Z if RPM is in "high range" (Reads for both encoders)
  RPM_1 = readEncoderZ(encoder1PinZ);   // Read Encoder pin Z if RPM is in "high range" (Reads for both encoders)
  #if DEBUG 
    Serial.print("RPM 0 =");
    Serial.print(RPM_0);
    Serial.print("......RPM 1 =");
    Serial.println(RPM_1);
  #endif  
  //RPM_0_Last = RPM_0;
  //RPM_1_Last = RPM_1;
  
  //(insert diff code here)
  
  throttle_in_left = throttle_in;
  throttle_in_right = throttle_in;
  
  if((controlScheme == 1) && (loopCount > 0)){
      differential(Steering, throttle_in);
  }
  
  int leftSpeed  = map(throttle_in_left, minThrottle, maxThrottle, 0, 1023);
  int rightSpeed = map(throttle_in_right, minThrottle, maxThrottle, 0, 1023);
  
  if( leftSpeed < 5 ){  // Error checking code to ensure we do not have throttle spikes at the extremities
    leftSpeed = 0;
  }
  if ( leftSpeed > 1023 ){
    leftSpeed = 1023;
  }
  if ( rightSpeed < 5 ){
    rightSpeed = 0;
  }
  if( rightSpeed > 1023 ){
    rightSpeed = 1023;
  } 
    
  analogWriteResolution(Res);
  analogWrite(leftMotor, leftSpeed);
  delay(ADC_DELAY/5);
  analogWriteResolution(Res);
  analogWrite(rightMotor, rightSpeed);
  delay(ADC_DELAY/5);

      #if DEBUG
        #if SPEED
        Serial.print("Left Motor Voltage ");                //Debug Statements ~ visual data
        Serial.println(leftSpeed);  
        Serial.print("Right Motor Voltage");
        Serial.println(rightSpeed);
        #endif
      #endif

  loopEnd = millis();
  loopCount = loopCount + 1;
  delayMicroseconds(5);    
}


