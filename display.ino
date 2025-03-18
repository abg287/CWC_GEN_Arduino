// Libraries
#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>


// Constants

  // LCD screen


    #define USING_I2C true


    // WITHOUT I2C
      // Pins
      #define RS 13 // Register Select
      #define E 12  // Enable
      #define D4 11  // Decibel 4
      #define D5 10  // Decibel 5
      #define D6 9  // Decibel 6
      #define D7 8  // Decibel 7

    // WITH I2C
      // Address
      #define ADDRESS 0x27

    // Screen Size
    #define COLUMNS 20
    #define ROWS 4

	// Units
	#define VOLTAGE_UNIT "V"
	#define CURRENT_UNIT "A"
	#define RPM_UNIT "RPM"
	#define TORQUE_UNIT "N-m"



  // IR sensor

    // Pin
    #define IR_PIN 2

    // Attribute
    #define PPR 1


  // Voltage measuring

    // Pin
    #define VOLTAGE_ANALOG_PIN A0

    // Resistors
    #define R1 9.92
    #define R2 0.995
    
    // Attribute
    #define REFERENCE_VOLTAGE ( float ) 4.62 // V (ACTUALLY CHECK FOR REFERENCE)




  // Current Sensor

    // Pin
    #define CURRENT_ANALOG_PIN A1

    // Attribute
    #define SENSITIVITY 0.066



  // Load amp

    // Pins
    #define SCK 7
    #define DT 6


  // Other
    #define DELAY_TIME 750 // ms
    #define ADC_RESOLUTION 1023
	  #define BUTTON_PIN 5
    #define DECIMAL_PLACES 2 // Arduino only allows 6 decimal places before it's followed by zeros



#define VOLTAGE_PER_COUNT REFERENCE_VOLTAGE / ADC_RESOLUTION


// class LCDScreen {

//   public:
//     virtual void printMeasurement( float value, String unit ) = 0;
//     virtual void clearLCDLine( int line ) = 0;
// };


// class LCDScreenWithI2C : public LCDScreen {
//   public:
//     LCDScreenWithI2C() : lcdScreen( ADDRESS, COLUMNS, ROWS ) {
//       lcdScreen.init();
//       lcdScreen.backlight(); // Turn on the backlight
//       lcdScreen.clear();

//       lcdScreen.setCursor( 0, 0 );
//       lcdScreen.print( "25' CWC GEN" );
//     }

//     void printMeasurement( float value, String unit ) {
//       int stringLength = unit.length();
//       int cursorPosition = COLUMNS - stringLength;
    
//       lcdScreen.setCursor( 0, 1 );
//       lcdScreen.print( value, DECIMAL_PLACES );
//       lcdScreen.setCursor( cursorPosition, 1 );
//       lcdScreen.print( unit );
//     }

//     void clearLCDLine( int line ) {               
//       lcdScreen.setCursor( 0,line );
//       for( int n = 0; n < COLUMNS; n++ )
//       {
//         lcdScreen.print(" ");
//       }
//     }

//   private:
//     LiquidCrystal_I2C lcdScreen;
// };

// class LCDScreenWithoutI2C : public LCDScreen {
//   public:

//     LCDScreenWithoutI2C() : lcdScreen( RS, E, D4, D5, D6, D7 ) {
//       lcdScreen.begin( COLUMNS, ROWS );
//       lcdScreen.setCursor( 0, 0 );
//       lcdScreen.print( "25' CWC GEN" );
//     }

//     void printMeasurement( float value, String unit ) {
//       int stringLength = unit.length();
//       int cursorPosition = COLUMNS - stringLength;
    
//       lcdScreen.setCursor( 0, 1 );
//       lcdScreen.print( value, DECIMAL_PLACES );
//       lcdScreen.setCursor( cursorPosition, 1 );
//       lcdScreen.print( unit );
//     }

//     void clearLCDLine(int line) {
//       lcdScreen.setCursor( 0,line );
//       for( int n = 0; n < COLUMNS; n++ )
//       {
//         lcdScreen.print(" ");
//       }
//     }

//   private:
//     LiquidCrystal lcdScreen;
// };

// Initialize variables and functions for

  // LCD Screen
  // LCDScreen *lcdScreen;

  // if ( USING_I2C ) {
    // lcdScreen = new LCDScreenWithI2C();
    LiquidCrystal_I2C LCD_Screen( ADDRESS, COLUMNS, ROWS );
  // }

  // else {
  //   LiquidCrystal LCD_Screen( RS, E, D4, D5, D6, D7 );
  //   // lcdScreen = new LCDScreenWithoutI2C();
  // }
  

  void initializeLCDScreen();
  void printMeasurement(float value, String unit);
  void clearLCDLine(int line);


  // Voltage Divider

  // class VoltageDivider {
  //   public:
  //     VoltageDivider() : multiplier( R1 + R2 ) {
  //       pinMode(VOLTAGE_ANALOG_PIN, INPUT);
  //     }

  //     void calculateVoltage() {

  //       float voltageIn = getVoltageInput( VOLTAGE_ANALOG_PIN );
  
  //       float totalVoltage = voltageIn * multiplier;
        
  //       lcdScreen->printMeasurement( totalVoltage, VOLTAGE_UNIT );
  //     }

  //   private:
  //     float multiplier;
  // }

  float resistor1 = 9.92;
  float resistor2 = 0.995;

  float multiplier = resistor1 + resistor2;

  void initializeVoltageInput();
  void calculateVoltage();
  float getVoltageInput( int pinNumber ); // Current also uses this
  float digitalToAnalogConverter( int count ); // Current also uses this


  // Current Sensor
  // class CurrentSensor {

  //   public:
  //     CurrentSensor() {
  //       pinMode(BUTTON_PIN, INPUT);
  //     }

  //     void calculateCurrent() {
  //       float voltageIn = getVoltageInput( CURRENT_ANALOG_PIN );
        
  //       float current = ( voltageIn - ( float ) ( REFERENCE_VOLTAGE / 2 ) ) / SENSITIVITY;
        
  //       lcdScreen->printMeasurement( current, CURRENT_UNIT );
  //     }

  // };

  void initializeCurrentInput();
  void calculateCurrent();


  // IR Sensor
  // class IRSensor {
  //   public:
  //     IRSensor() : rpm( 0 ), ppr( PPR ), counter( 0 ), previousMillis( millis() ) {
  //       pinMode( IR_PIN, INPUT_PULLUP );
  //       attachInterrupt( digitalPinToInterrupt( IR_PIN ), IRInterrupt, FALLING );
  //     }

  //     void calculateRPM() {
  //       unsigned long currentMillis = millis();
  
  //       lcdScreen->printMeasurement( rpm, RPM_UNIT );
        
  //       if ( currentMillis - previousMillis >= 1000 ) {

  //         detachInterrupt( digitalPinToInterrupt( IR_PIN ) );
  //         rpm = ( counter / PPR ) * 60;  // Calculate RPM
  //         counter = 0;
  //         attachInterrupt( digitalPinToInterrupt( IR_PIN ), IRInterrupt, FALLING );
  //         previousMillis = currentMillis;
  //       }
  //     }

  //     void IRInterrupt() {
  //       counter++;
  //     }

  //   private:
  //     unsigned int rpm;
  //     int ppr;
  //     volatile unsigned int counter;
  //     unsigned long previousMillis;
  // };

  unsigned int rpm = 0;
  volatile unsigned int counter = 0;  // Counter variable for revolutions
  unsigned long previousMillis = millis();

  void initializeRPMInput();
  void calculateRPM();
  void IRinterrupt();
  


  // Load Amp
  void calculateTorque();
  
  // Button
  int previousButtonState = 0;
  unsigned long debounceDuration = 50; // ms
  unsigned long previousButtonPressMillis = millis();

  bool buttonWasPressed();
  void initializeButtonInput();
  void checkForButtonPress();

  // Other
  int displayNum = 0;

  void calculateMeasurement();

bool buttonWasPressed() {
 
  int buttonState;
  
  if ( millis() - previousButtonPressMillis >= debounceDuration ) {
   
    buttonState = digitalRead( BUTTON_PIN );
    
    if ( buttonState != previousButtonState ) {
    
      	previousButtonPressMillis = millis();
      	previousButtonState = buttonState;
      
    	return buttonState == HIGH;
    }
  }
  
  return false;
}

void checkForButtonPress() {
  if ( buttonWasPressed() ) {
    displayNum++;
    clearLCDLine( 1 );
  }
}

void calculateMeasurement() {
  switch ( displayNum % 3 ) {
    
    case 0:
      calculateVoltage();
   	  break;

    case 1:
      calculateCurrent();
      break;

    case 2:
      calculateRPM();
      break;

    case 3:
      calculateTorque();
      break;
  }
}

void setup() {
  
  Serial.begin(9600);
  initializeLCDScreen();
  initializeButtonInput();
  initializeVoltageInput();
  initializeCurrentInput();
  initializeRPMInput();
}

void loop() {
  
  checkForButtonPress();
  
  calculateMeasurement();
  delay(DELAY_TIME);
}

void initializeLCDScreen() {
  
  if ( USING_I2C ) {
    LCD_Screen.init();
    LCD_Screen.backlight(); // Turn on the backlight
    LCD_Screen.clear();
  }

  else {
    LCD_Screen.begin( COLUMNS, ROWS );
  }

  LCD_Screen.setCursor( 0, 0 );
  LCD_Screen.print( "25' CWC GEN" );
}

void initializeVoltageInput() {
  pinMode(VOLTAGE_ANALOG_PIN, INPUT);
}

void initializeCurrentInput() {
  pinMode(CURRENT_ANALOG_PIN, INPUT);
}

void initializeButtonInput() {
  pinMode(BUTTON_PIN, INPUT);
}

void initializeRPMInput() {
  pinMode( IR_PIN, INPUT_PULLUP );
  attachInterrupt(digitalPinToInterrupt(IR_PIN), IRinterrupt, FALLING);
}

void calculateVoltage() {
  
  float voltageIn = getVoltageInput( VOLTAGE_ANALOG_PIN );
  
  Serial.println( voltageIn );

  float totalVoltage = voltageIn * multiplier;
  
  printMeasurement( totalVoltage, VOLTAGE_UNIT );
}

void calculateCurrent() {
  
  float voltageIn = getVoltageInput( CURRENT_ANALOG_PIN );

  Serial.println(voltageIn);
  
  float current = ( voltageIn - ( float ) ( REFERENCE_VOLTAGE / 2 ) ) / SENSITIVITY;
  Serial.println( current );
  
  printMeasurement( current, CURRENT_UNIT );
}

void calculateRPM() {
 
  unsigned long currentMillis = millis();
  
  printMeasurement( rpm, RPM_UNIT );
  
  if (currentMillis - previousMillis >= 1000) {
    detachInterrupt(digitalPinToInterrupt(IR_PIN));
    rpm = (counter / PPR) * 60;  // Calculate RPM
    counter = 0;
    attachInterrupt(digitalPinToInterrupt(IR_PIN), IRinterrupt, FALLING);
    previousMillis = currentMillis;
  }
}

float getVoltageInput( int pinNumber ) {
  
  int count = analogRead( pinNumber );


  return digitalToAnalogConverter( count );
  
}

float digitalToAnalogConverter( int count ) {
  
 return count * VOLTAGE_PER_COUNT;
  
}

void printMeasurement( float value, String unit ) {
  
  int stringLength = unit.length();
  int cursorPosition = COLUMNS - stringLength;
  
  LCD_Screen.setCursor( 0, 1 );
  LCD_Screen.print( value, DECIMAL_PLACES );
  LCD_Screen.setCursor( cursorPosition, 1 );
  LCD_Screen.print( unit );
  
}

void IRinterrupt() {
    counter++;
}

void clearLCDLine(int line)
{               
        LCD_Screen.setCursor( 0,line );
        for( int n = 0; n < COLUMNS; n++ )
        {
          LCD_Screen.print(" ");
        }
}
