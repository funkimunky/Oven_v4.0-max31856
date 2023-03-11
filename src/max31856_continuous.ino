// This example demonstrates continuous conversion mode using the
// DRDY pin to check for conversion completion.
#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Adafruit_MAX31856.h>

#define DRDY_PIN 5

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(53, 50, 51, 52);
// use hardware SPI, just pass in the CS pin
// Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(10);


//timers
unsigned long startMillis;
unsigned long currentMillis;
unsigned long controlsStartMillis;
unsigned long ovenDoorStartMillis;
unsigned long ovenSafetyStartMillis;
unsigned long lcdClearStartMillis;
unsigned long windowStartTime;
unsigned long controlsDelay = 300;
unsigned long backlightStartMillis;
unsigned long backlightCountDown = 120000;
unsigned long lcdClearDelay = 2000;

//States
bool isError = false;
bool DOOR_OPEN = false;
bool lightsOn = false;
bool backlightIsOn = false;
bool backlightAuto = true;
bool timerStarted = false;

//LCD VARIABLES
LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
char Line1Col1Buf[8];
char Line1Col2Buf[8];
char Line2Col1Buf[8];
char Line2Col2Buf[8];

int currentTemperature = 0;

//PID Tuning
//              RISE TIME	    OVERSHOOTS	    SETTLING TIME   STEADY STATE ERROR
//Kp	        DECREASE	    INCREASE	    SMALL CHANGE	DECREASE
//Ki	        DECREASE	    INCREASE	    INCREASE	    ELIMANATE
//Kd	        INCREASE	    DECREASE	    DECREASE	    NO CHANGE
// PID VARIABLES
double sampleTime = 50;
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp = 250, Ki = 250, Kd = 0.07;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);

int WindowSize = 200;



//OVEN VARIABLES
int ovenSafeTemp = 80;

// OVEN CONTROL VARIABLES
int maxTemp = 280;
int maxPotValue = 1000;
int ovenProg = 0;
int ovenTempSet = 0;

//Oven programs
#define OVEN_OFF  0
#define FAN_OVEN  1


//PINS

//analog potentiometer pins
int TEMPCONTROL_PIN = A15;
int OVENPROG_PIN = A14;

//relay pins
int RELAYSAFETY_PIN = 33;
int ELEMENTFAN_PIN = 32;
int TOPFAN_PIN = 35;
int LIGHTS_PIN = 34;


//Door detect pin
int DOOROPEN_PIN = 37;

//SCR Pin
int SCR_PIN = 36;



void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("MAX31856 thermocouple test");

  pinMode(DRDY_PIN, INPUT);

  if (!maxthermo.begin()) {
    Serial.println("Could not initialize thermocouple.");
    while (1) delay(10);
  }

  maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);

  Serial.print("Thermocouple type: ");
  switch (maxthermo.getThermocoupleType() ) {
    case MAX31856_TCTYPE_B: Serial.println("B Type"); break;
    case MAX31856_TCTYPE_E: Serial.println("E Type"); break;
    case MAX31856_TCTYPE_J: Serial.println("J Type"); break;
    case MAX31856_TCTYPE_K: Serial.println("K Type"); break;
    case MAX31856_TCTYPE_N: Serial.println("N Type"); break;
    case MAX31856_TCTYPE_R: Serial.println("R Type"); break;
    case MAX31856_TCTYPE_S: Serial.println("S Type"); break;
    case MAX31856_TCTYPE_T: Serial.println("T Type"); break;
    case MAX31856_VMODE_G8: Serial.println("Voltage x8 Gain mode"); break;
    case MAX31856_VMODE_G32: Serial.println("Voltage x8 Gain mode"); break;
    default: Serial.println("Unknown"); break;
  }

  maxthermo.setConversionMode(MAX31856_CONTINUOUS);

   //set sample sample time to match temperature check
    myPID.SetSampleTime(sampleTime);
    //tell the PID to range between 0 and the full window size
    myPID.SetOutputLimits(0, WindowSize);

    //turn the PID on
    myPID.SetMode(AUTOMATIC);

    //set up digital pins

    //digital inputs
    pinMode(DOOROPEN_PIN, INPUT_PULLUP);

    //digital outputs
    pinMode(SCR_PIN, OUTPUT);
    digitalWrite(SCR_PIN, LOW);


    //digital outputs
    pinMode(RELAYSAFETY_PIN, OUTPUT);
    digitalWrite(RELAYSAFETY_PIN, HIGH);//relay is LOW ENABLED so high sets this off    

    pinMode(LIGHTS_PIN, OUTPUT);
    digitalWrite(LIGHTS_PIN, HIGH);//relay is LOW ENABLED so high sets this off

    pinMode(ELEMENTFAN_PIN, OUTPUT);
    digitalWrite(ELEMENTFAN_PIN, HIGH);//relay is LOW ENABLED so high sets this off

    pinMode(TOPFAN_PIN, OUTPUT);
    digitalWrite(TOPFAN_PIN, HIGH);//relay is LOW ENABLED so high sets this off

    lcd.init(); // initialize the lcd 
    backlightAuto = true;
    backLightON();

    lcd.setCursor(0, 0);
    lcd.print("DAVEOVEN V4");
    delay(2000);
    lcd.clear();

    sprintf(Line1Col1Buf, "1:%s", "TEST1");
    sprintf(Line1Col2Buf, "2:%s", "TEST2");
    sprintf(Line2Col1Buf, "3:%s", "TEST3");
    sprintf(Line2Col2Buf, "4:%s", "TEST4");

    lcd.setCursor(0, 0);
    lcd.print(Line1Col1Buf);
    lcd.setCursor(9, 0);
    lcd.print(Line1Col2Buf);
    lcd.setCursor(0, 1);
    lcd.print(Line2Col1Buf);
    lcd.setCursor(9, 1);
    lcd.print(Line2Col2Buf);

    delay(1000);
    lcd.clear();

    //startTimers
    startMillis = millis();
    controlsStartMillis = millis();
    ovenDoorStartMillis = millis();
    ovenSafetyStartMillis = millis();
    backlightStartMillis = millis();
    lcdClearStartMillis = millis();
}

void loop() {
  // The DRDY output goes low when a new conversion result is available
  int count = 0;
  currentTemperature = maxthermo.readThermocoupleTemperature()

  // Serial.println(maxthermo.readThermocoupleTemperature());
  

  printTemp();

  // Serial.println(digitalRead(DRDY_PIN));
  
  while (digitalRead(DRDY_PIN)) {
    if (count++ > 200) {
      count = 0;
      
      Serial.print(".");
    }

    // need to refactor to use temperature previously read when DRDY pin is true
    
    checkSafetyTemp();
    backlightCheck();

    if (isError) {
        digitalWrite(SCR_PIN, LOW);
        Serial.println("ERROR");
    }
    else {
        ReadControls();
        if (ovenProg == FAN_OVEN) {
			//Serial.println("OvenProg = FAN_OVEN");
            backlightAuto = false;
            backLightON();
			      startWindowTimer();
            if (DOOR_OPEN) {
                switchLightOn();
                digitalWrite(SCR_PIN, LOW);
                relayOff(RELAYSAFETY_PIN);
                relayOff(ELEMENTFAN_PIN);
                Input = 0;
                Output = 0;
				timerStarted = false;
            }
            else {
                switchLightOn();
                relayOn(RELAYSAFETY_PIN);
                relayOn(ELEMENTFAN_PIN);
                processPID();
            }

        }
        else {
            backlightOFF();
            backlightAuto = true;
            digitalWrite(SCR_PIN, LOW);
            relayOff(RELAYSAFETY_PIN);
            relayOff(ELEMENTFAN_PIN);
            switchLightOff();
			timerStarted = false;
        }
    }
    
  }
  // Serial.println("low");
  // Serial.println(maxthermo.readThermocoupleTemperature());
}



void startWindowTimer(){
	if(!timerStarted){
		windowStartTime = millis();
		timerStarted = true;
	}	
}

void backlightCheck() {
    currentMillis = millis();
    if (backlightAuto) {
        if (currentMillis - backlightStartMillis >= backlightCountDown)  //test whether the period has elapsed
        {
            backlightOFF();

            backlightStartMillis = currentMillis;
        }
    }

}

void backLightON() {

    if (!backlightIsOn) {
        lcd.backlight();
        backlightIsOn = true;
    }
}

void backlightOFF() {
    if (backlightIsOn) {
        lcd.noBacklight();
        backlightIsOn = false;
    }
}

void switchLightOn() {
    if (lightsOn) {
        return;
    }
    else {
        lightsOn = true;
        relayOn(LIGHTS_PIN);
    }
}

void switchLightOff() {
    if (lightsOn && ovenProg != FAN_OVEN) {
        lightsOn = false;
        relayOff(LIGHTS_PIN);
        return;
    }
    else {
        return;
    }
}


void checkSafetyTemp() {

    currentMillis = millis();
    if (currentMillis - ovenSafetyStartMillis >= controlsDelay)  //test whether the period has elapsed
    {
        if (currentTemperature > ovenSafeTemp)
        {

            relayOn(TOPFAN_PIN);
        }
        else
        {

            relayOff(TOPFAN_PIN);
        }
        ovenSafetyStartMillis = currentMillis;
    }
}

/*
Read all the control knobs to evaluate state.
*/
void ReadControls() {
    currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
    // float temp = 0;

    if (currentMillis - controlsStartMillis >= controlsDelay)  //test whether the period has elapsed
    {

		//if (currentMillis - lcdClearStartMillis >= lcdClearDelay)  //test whether the period has elapsed
		//{
			//lcd.clear();
			//lcdClearStartMillis = currentMillis;
		//}
		
        readPotentionmeterTemp(analogRead(TEMPCONTROL_PIN));

        readPotentiometerProg(analogRead(OVENPROG_PIN));

        readOvenDoor();

        controlsStartMillis = currentMillis;
    }
}

//pass control readings from potentiometer and determine what program to set
void readPotentiometerProg(int potentiometerValue)
{
    if (potentiometerValue <= 512)
    {
        sprintf(Line1Col1Buf, "p:%s", "OFF");

        lcd.setCursor(0, 1);
        lcd.print(Line1Col1Buf);

        ovenProg = OVEN_OFF;

        Output = 0;

        return;

    }
    else if (potentiometerValue > 512 && potentiometerValue <= 1040)
    {
        sprintf(Line1Col1Buf, "p:%s", "ON ");
        lcd.setCursor(0, 1);
        lcd.print(Line1Col1Buf);

        ovenProg = FAN_OVEN;
        return;
    }

}

int roundToNearestMultiple(int numberToRound, int multiple) {

    int result;
    result = numberToRound + multiple / 2;
    result = result - result % multiple;
    return result;
}


void readPotentionmeterTemp(int potentiometerValue)
{
    int temp = ceil(((float)potentiometerValue / (float)maxPotValue) * maxTemp);
    //DEBUG
    Serial.print("TARGTEMP ");
    Serial.println(temp);

    int result = roundToNearestMultiple(temp, 5);

    ovenTempSet = result;

    sprintf(Line1Col1Buf, "t:%03d", result);
    lcd.setCursor(0, 0);
    lcd.print(Line1Col1Buf);
}

void processPID()
{
    //Serial.println("ProcessPID called");
	Input = currentTemperature;//use current tempertature as input
    Setpoint = ovenTempSet;
    myPID.Compute();

    /************************************************
    * turn the output pin on/off based on pid output
    ************************************************/
    if (millis() - windowStartTime > WindowSize)
    { //time to shift the Relay Window
        windowStartTime += WindowSize;
    }

	// Serial.print("Output");
	// Serial.print("\t");
	// Serial.print(Output);
	// Serial.print("\t");
	
	// Serial.print("windowStartTime");
	// Serial.print("\t");
	// Serial.print(windowStartTime);
	// Serial.print("\t");	
	
	unsigned long timeTest = millis() - windowStartTime;
	
	// Serial.print("millis - windowStartTime");
	// Serial.print("\t");
	// Serial.print(timeTest);
	// Serial.print("\t");
	
    if (Output <= timeTest)
    {
		// Serial.print("ProcessPID LOW");
		// Serial.print("\n");
		
        digitalWrite(SCR_PIN, LOW);
    }
    else
    {
		// Serial.print("ProcessPID HIGH");
		// Serial.print("\n");
	    digitalWrite(SCR_PIN, HIGH);
    }
}

void printTemp() {
   
        sprintf(Line1Col2Buf, "a:%03d", currentTemperature);
        lcd.setCursor(11, 0);
        lcd.print(Line1Col2Buf);

    //DEBUG     
    // sprintf (buf, "RTDTEMP is %3d \r\n", thermo.readRTD());
    // Serial.print (buf);

    // currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
	
    // if (currentMillis - startMillis >= sampleTime)  //test whether the period has elapsed
    // {
    //     //Serial.println("temp checked");
		// startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
    //     // Check and print any faults
    //     uint8_t fault = thermo.readFault();
    //     if (fault)
    //     {
    //         printFault(fault, thermo);
    //         return;
    //     }
    //     currentTemperature = thermo.temperature(RNOMINAL, RREF);
    //     sprintf(Line1Col2Buf, "a:%03d", currentTemperature);
    //     lcd.setCursor(11, 0);
    //     lcd.print(Line1Col2Buf);
    // }
}

void printFault(uint8_t  fault, Adafruit_MAX31865 thermo) {
    Serial.print("Fault 0x");
    Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
        Serial.println("RTD High Threshold");
        isError = true;
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
        Serial.println("RTD Low Threshold");
        isError = true;
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
        Serial.println("REFIN- > 0.85 x Bias");
        isError = true;
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
        Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
        isError = true;
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
        Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
        isError = true;
    }
    if (fault & MAX31865_FAULT_OVUV) {
        Serial.println("Under/Over voltage");
    }
    thermo.clearFault();
    Serial.println();
}


void relayOn(int pin) {
    digitalWrite(pin, LOW);
}

void relayOff(int pin) {
    digitalWrite(pin, HIGH);
}

void readOvenDoor() {
    currentMillis = millis();

    if (currentMillis - ovenDoorStartMillis >= controlsDelay) {
        int sensorVal = digitalRead(DOOROPEN_PIN);
        if (sensorVal == HIGH) {
            switchLightOn();
            DOOR_OPEN = true;
        }
        else {
            switchLightOff();
            DOOR_OPEN = false;
        }

        ovenDoorStartMillis = currentMillis;
    }

}
