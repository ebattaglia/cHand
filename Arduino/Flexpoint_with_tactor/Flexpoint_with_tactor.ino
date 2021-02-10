int motorPins[] = {6}; // to handle more than one motor just add more pins e.g. {6,7,8}
int nmotors = 1; // update if needed
unsigned long timers[] = {0};

int PulseWidth = 450; // duration of vibrations

// activation values for motors
int off = 0;
int high = 190;

const int FLEX_PIN_POS = A1; // Pin connected to voltage divider output (bend +)
const int FLEX_PIN_NEG = A2; // Pin connected to voltage divider output (bend -)

const float VCC = 5.07; // voltage of Ardunio 5V line
const float R_DIV = 8100.0; //  resistance in the voltage divider

void setup() 
{
  Serial.begin(115200);
  //-pinMode(FLEX_PIN_POS, INPUT);
  //pinMode(FLEX_PIN_NEG, INPUT);
  
  for (int motorID = 0; motorID < nmotors; motorID++) {
    pinMode( motorPins[motorID], OUTPUT );
  }
}

void processIncomingByte (const byte inByte)
{
  // this expects a number between 1 and nmotor, and will activate motor number inByte if timers[inByte] is zero
  for (int motorID = 0; motorID < nmotors; motorID++) {
    if (inByte == byte(motorID + int('1'))) {
      timers[motorID] = millis();
      // turn motor on
      analogWrite(motorPins[motorID], high);
    }
  }
  
} 

void updateMotors()
{// for each motor check if enough time has passed, if yes deactivate it
  for (int motorID = 0; motorID < nmotors; motorID++) {
    if (timers[motorID] > 0) {
      if ( (millis() - timers[motorID]) > PulseWidth){
          analogWrite(motorPins[motorID], off);
          timers[motorID] = 0;
      }
    }  
  }
}

float readBendSensor_movingAverage(int pinN, float fixedR, int windowSize)
{// read bend sensor and filter out some noise
  float flexRSum = 0;
  float flexRMax = 0;
  for (int i = 0; i < windowSize; i++)
  {
    for (int j = 0; j < windowSize; j++)
    {
      int flexADC = analogRead(pinN);
  
      float flexV = flexADC * VCC / 1023.0;
      float flexR = fixedR * (VCC / flexV - 1.0);

    
      //flexRSum += flexR;
      if (flexR > flexRMax) {flexRMax = flexR;}
       
    }
    flexRSum += flexRMax;
  }
  
  return(flexRSum/windowSize);
}


void loop() 
{
  
  // Handle incoming data from the C++ program (used to control the motor)
  while (Serial.available () > 0)
    processIncomingByte (Serial.read ());

  /////////////////////////////////////////////
  // Do some other things (e.g., here I am reading from some bend sensors)  
  // Read the ADC, and calculate voltage and resistance from it
  int flexADC_POS = analogRead(FLEX_PIN_POS);

  float flexR_POS = readBendSensor_movingAverage(FLEX_PIN_POS,R_DIV,16);
  Serial.println(flexR_POS);

  // Call function to deactivate vibration motor after a certain amount of time has passed
  updateMotors();
}
