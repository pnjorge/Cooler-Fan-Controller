



/*
 * This sketch is for COOLER FAN CONTROLLER.
 * 
 * A configurable dual fan controller.
 *   
 *   
 * See  https://github.com/pnjorge
 *      https://www.hackster.io/pnjorge
 *      paulonjorge@yahoo.com
 */





//************************************************************************************************************

#define DEBUG 0       // Set here for Enable/Disable of serial debugging

//************************************************************************************************************



#if DEBUG == 1

  #define debug(x) Serial.print(x)
  #define debugln(x) Serial.println(x)

#else

  #define debug(x)
  #define debugln(x)

#endif





//THIRD-PARTY LIBRARIES
//these must be added to your Arduino IDE installation


//TimedAction
//allows us to set actions to perform on separate timed intervals

#include <TimedAction.h>
//NOTE: This library has an issue on newer versions of Arduino. After
//      downloading the library you MUST go into the library directory and
//      edit TimedAction.h. Within, overwrite WProgram.h with Arduino.h



// Include the libraries we need for the temperature sensor DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>






#define ONE_WIRE_BUS 7

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);




// OLED DISPLAY

#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>

#include "FAN_fixed_bold10x15.h"

#define I2C_ADDRESS 0x3C       // OLED display address 0X3C+SA0 - 0x3C or 0x3D
#define RST_PIN -1             // Define proper RST_PIN if required.

SSD1306AsciiWire oled;


const char *oledConfig[]  = { "   FAN MIN  ", "   FAN MAX  ", "  TEMP MIN  ", "  TEMP MAX  ", "    MODE    ", " SWEEP RATE ", "  TRIGGER   ", "TEMP OFFSET ", "SAVE & EXIT " };

const char *oledMode[]  = { "0           ", "1 Sweeping  ","2 Triggered " };

const char *oledTrigger[]  = { "0           ", "1 Temp/Zero ", "2 Temp/Fmin ", "3 Pin/Zero  ", "4 Pin/Fmin  " };

const char *oledConfigExit[]  = { "     No     ","    Yes     " };



// Fixed definitions cannot change

#define VENT_PWM_PIN1A 9
#define VENT_PWM_PIN1B 10


#define PIN_DEGREES 5               // Jumper selection pin degC/degF  (LOW for degF)

#define PIN_PINPUT 8                // Optocoupler input (Positive voltage to enable)
#define PIN_POUTPUT 11              // Output Pin 

#define PIN_SYMM  A2                // Symmetric input (Positive voltage to enable)
#define PIN_ASYMM A3                // Asymmetric input (Positive voltage to enable)



#define PIN_TACH 2                  




#define FILLED_BLOCK 128
#define EMPTY_BLOCK 129

#define ICON_OUT_OF_RANGE 130

#define TEMP_C 131
#define TEMP_F 132

#define ICON_TEMP 133   


#define ICON_PIN_TRIG 150   
#define ICON_PIN_NOTRIG 151   



#define RPM_ARROW 134
#define RPM_JAMMD 135



#define ICON_TEMP_ZERO 136
#define ICON_TEMP_FMIN 137
#define ICON_PIN_ZERO  138
#define ICON_PIN_FMIN  139



#define ICON_SWEEP_1  140
#define ICON_SWEEP_2  141
#define ICON_SWEEP_3  142
#define ICON_SWEEP_4  143
#define ICON_SWEEP_5  144
#define ICON_SWEEP_6  145
#define ICON_SWEEP_7  146
#define ICON_SWEEP_8  147
#define ICON_SWEEP_9  148
#define ICON_SWEEP_10 149



bool  OORflag = false;      // Out of range flag

bool pInputFlag = false;



// ..ROTARY ENCODER THREAD
//*********************************************************
// Used for generating interrupts using DT signal
const int pinEncoderDT = 3;

// Used for reading CLK signal
const int pinEncoderCLK = 4;

// Used for the push button switch
const int PinEncoderSW = 6;

// Keep track of last rotary value
int lastCount = 0;

// Updated by the encoderISR (Interrupt Service Routine)
volatile int virtualPosition = 0;


int menuItem = -1;

//*********************************************************


                                                                                     //10 is Fast 
int savedValue[] = {   0         ,  0         ,  0         ,  0         ,  0         ,  0         ,  0         ,  0         ,  0         };         
                  
                  // [0]          [1]          [2]          [3]          [4]          [5]          [6]          [7]          [8]
                  // FAN MIN      FAN MAX      TEMP MIN     TEMP MAX     MODE         SWEEP RATE   TRIGGER      TEMP OFFSET  SAVE & EXIT 
                  //Initial values and then stores the settings you change            10 is fast
                  //                                                                   1 is slow


bool inConfigMenu = false;


uint8_t modeIcon = 0;

uint8_t pwmBars = 0;

int fanPercentValueA = 0;
int fanPercentValueB = 0;


uint16_t  spinDownDelay = 7000;

int fanStopLimit = 0;

uint32_t startCountTach = 0;

uint16_t volatile countTach = 0;
uint16_t rpmA = 0;



bool  triggerFlag = false;


                              //  User friendly variables declared (will be same as savedValue array)
int fanMin      = 0;          //  savedValue[0]
int fanMax      = 0;          //  savedValue[1]
int tempMin     = 0;          //  savedValue[2]
int tempMax     = 0;          //  savedValue[3]
int opMode      = 0;          //  savedValue[4]
int sweepDelay  = 0;          //  savedValue[5]
int fanTrigger  = 0;          //  savedValue[6]
int tempOffset  = 0;          //  savedValue[7]


int temperature = 0;

bool tempdegC = true;         // Default temperature units in degC 


uint8_t tempChar = TEMP_C; 


int key1 = 73;              // key1 and key2 values chosen arbitrarily
int key2 = 42;              // ...highly unlikely these values will be found on a new or used ATmega328P


uint8_t fanBmode = 0;


bool fanAjammed = false;          // Flags when main fan A is jammed/stpped



void  encoderRead();              //define prototypes before timedActions
void  tachCounter();              
void  getTemperature();



TimedAction encoderReadThread = TimedAction(50,encoderRead);          // Fires every 50 mSec

TimedAction tachCounterThread = TimedAction(1000,tachCounter);        // Fires every 1 second

TimedAction temperatureThread = TimedAction(3000,getTemperature);     // Fires every 3 seconds







// S E T U P -------------------------------------------------------------
// -----------------------------------------------------------------------


void setup() 

{

  Serial.begin (115200);
  delay(500);
  
  debugln(F("\n> Serial debug ok"));

  debug("\nArduino running sketch: ");
  debug(__FILE__);
  debugln();
  debug("Compiled on: \t");
  debug(__DATE__);
  debug(" at ");
  debug(__TIME__);
  debugln("\n\n");


    
  sensors.begin();      // Start up the temperature sensor library


  DDRC = B00000011;       // A0 and A1 (PC0 and PC1) as outputs for tone
           
  
  pinMode(VENT_PWM_PIN1A, OUTPUT);
  pinMode(VENT_PWM_PIN1B, OUTPUT);

  pinMode(PIN_POUTPUT,OUTPUT);        // Pin Output  

  

  pinMode(PIN_TACH,INPUT_PULLUP);


  pinMode(PIN_DEGREES,INPUT_PULLUP);        // High degC, Low degF

  pinMode(PIN_PINPUT,INPUT_PULLUP);        // Optocoupler input



  pinMode(PIN_SYMM, INPUT_PULLUP);            //  Low to enable symmetric Fan B
  pinMode(PIN_ASYMM,INPUT_PULLUP);            //  Low to enable asymmetric Fan B
                                              //  ...either but not both simultaneously low (If both simultaneously High, then HotStandby)

  digitalWrite(PIN_POUTPUT, LOW);



// ...ROTARY ENCODER

  // Rotary pulses are INPUTs
  pinMode(pinEncoderDT, INPUT_PULLUP);
  pinMode(pinEncoderCLK, INPUT_PULLUP);

  // Switch is floating so use the in-built PULLUP so we don't need a resistor
  pinMode(PinEncoderSW, INPUT_PULLUP);

  // Attach the routine to service the interrupts
  attachInterrupt(digitalPinToInterrupt(pinEncoderDT), encoderISR, LOW);



// ...READ TEMP UNITS

  
  if (digitalRead(PIN_DEGREES) == LOW)                // High degC, Low degF
  {
    tempChar = TEMP_F;            // character 132
    tempdegC = false;             // temperature set now to degF

    savedValue[2] = 100;      // Temp Min degF  (about 40degC)
    savedValue[3] = 160;      // Temp Max degF  (about 70degC)
    tempMin     = 100;          
    tempMax     = 160;         
    
    debugln(F("> temperature units = degF"));
  }
  else
  {
    debugln(F("> temperature units = degC"));
  }


  

// ...READ FAN B MODE OF OPERATION

  fanBsetup();



// ...READ EEPROM

  readEeprom();

  firstRunCheck();

  readConfig();

                              //  User friendly variables from savedValue array
  fanMin      = savedValue[0];          
  fanMax      = savedValue[1];         
  tempMin     = savedValue[2];         
  tempMax     = savedValue[3];         
  opMode      = savedValue[4];         
  sweepDelay  = savedValue[5];         
  fanTrigger  = savedValue[6];         
  tempOffset  = savedValue[7];          



  getTemperature();
  oneDit();    



// ...OLED DISPLAY

  Wire.begin();

  #if RST_PIN >= 0
    oled.begin(&Adafruit128x32, I2C_ADDRESS, RST_PIN);    // Type of oled display
  #else 
    oled.begin(&Adafruit128x32, I2C_ADDRESS);
  #endif 

  oled.setFont(FAN_fixed_bold10x15);                          // Font choice
  //oled.setFont(System5x7);
  oled.clear();
  oled.home();


  oled.println("COOLER-FAN ");
  oled.println("CONTROLLER ");



  delay(spinDownDelay);         // Wait for fan to stop/stabilize after bootup.




  
  pwm25kHzSet();          // Set PWM frequency at 25KHz for use with four wire fan.



  
  fanPercentValueA = 0;        // initial PWM percentages  
  fanPercentValueB = 0;

  
  percentToPWM();
  

  attachInterrupt(digitalPinToInterrupt(PIN_TACH), tachISR, FALLING);

  
  countTach = 0;              // Zero the tach counter
    

  
  oled.clear();
  oled.home();


  
  fanStopLimit = findFanStopLimit();

  fanMin = savedValue[0];
  
  oled.home();
  oled.println(" Fan starts ");
  oled.print(" at PWM:");  oled.print(fanStopLimit);  oled.print("%  ");
  delay(2000);  




  debug(F("> fanStopLimit = "));  debugln(fanStopLimit); 
  debug(F("> fanMin = "));  debug(fanMin); debug(F("   fanMax = "));  debugln(fanMax);

  
}












//************************************************************************************************************
// L O O P ---------------------------------------------------------------
// -----------------------------------------------------------------------
//************************************************************************************************************




void loop() 
{


    encoderReadThread.check();
    tachCounterThread.check();
    temperatureThread.check();



    if ( inConfigMenu == true )
    {


    }  

    else

    {
    
      operationalMode();

      //delay(500);
      //debug(F("> Pinput = "));  debugln(digitalRead(PIN_PINPUT));

    }



}













//************************************************************************************************************
// F U N C T I O N S -----------------------------------------------------
// -----------------------------------------------------------------------
//************************************************************************************************************






void  writeConfig()
{


    int8_t signedValue = 0;

    int addr = 0;
    
           
    EEPROM.update(0, key1 );          //  Key values 73 and 42 to later confirm not first time run of this sketch
    EEPROM.update(1, key2 );


    debugln(F("Write relevant config to eeprom:"));
    

    if ( tempdegC == true )               // degC     ..writes Temp Min and Temp Max to addresses 4 and 5 ( skips 6 and 7)
    {
        debugln(F("tempdegC = true"));

        
        for (int i = 0; i <= 6; i++) 
        {

        debug(F("savedValue["));  debug(i); debug(F("]"));

        addr = i + 2;
        
          if ( i >= 4 )
          {
            addr = addr + 2;
          }


        
        EEPROM.update(addr, savedValue[i]);

        debug(F("  writing to addr = "));  debug(addr);

        debug(F(" is "));  debugln(savedValue[i]);
          
        }

    }   
  
    else
                                          // degF     ..writes Temp Min and Temp Max to addresses 6 and 7 ( skips 4 and 5)
    {

        debugln(F("tempdegC = false"));

        for (int i = 0; i <= 6; i++) 
        {

        debug(F("savedValue["));  debug(i); debug(F("]"));  

        addr = i + 2;
        
          if ( i >= 2 )
          {
            addr = addr + 2;
          }
        


        EEPROM.update(addr, savedValue[i]);

        debug(F("  writing to addr = "));  debug(addr);        

        debug(F(" is "));  debugln(savedValue[i]);
          
        }
         
    }



        signedValue = savedValue[7];          // Read and cast tempOffset to accomadate negative numbers in eeprom byte
                                              //... int8_t signedValue ( -127 to +127 )
        EEPROM.update(11, signedValue );
        debug(F("savedValue[7]  writing to addr = 11 is "));  debugln(savedValue[7]);




        EEPROM.update(12, 0 );
        debug(F("savedValue[8]  writing to addr = 12 is "));  debugln(savedValue[8]); 
  
}





//************************************************************************************************************



void  readConfig()
{

    int8_t signedValue = 0;

    int addr = 0;

    debugln(F("Reading relevant config from eeprom:"));

    if ( tempdegC == true )               // degC     ..reads Temp Min and Temp Max from addresses 4 and 5 ( skips 6 and 7)
    {
        debugln(F("tempdegC = true"));

        
        for (int i = 0; i <= 6; i++) 
        {

        debug(F("savedValue["));  debug(i); debug(F("]"));

        addr = i + 2;
        
          if ( i >= 4 )
          {
            addr = addr + 2;
          }
          

        savedValue[i] = EEPROM.read( addr );

        debug(F("  read from addr = "));  debug(addr);

        debug(F(" is "));  debugln(savedValue[i]);
          
        }

    }   
  
    else
                                          // degF     ..reads Temp Min and Temp Max from addresses 6 and 7 ( skips 4 and 5)
    {

        debugln(F("tempdegC = false"));

        for (int i = 0; i <= 6; i++) 
        {

        debug(F("savedValue["));  debug(i); debug(F("]"));  

        addr = i + 2;
        
          if ( i >= 2 )
          {
            addr = addr + 2;
          }


        savedValue[i] = EEPROM.read( addr );  
        
        debug(F("  read from addr = "));  debug(addr);

        debug(F(" is "));  debugln(savedValue[i]);
          
        }

         
    }



        signedValue = EEPROM.read(11);        // Read and cast tempOffset to accomadate negative numbers in eeprom byte
                                              //... int8_t signedValue ( -127 to +127 )
        savedValue[7] = signedValue;
        debug(F("savedValue[7]  read from addr = 11 is "));  debugln(savedValue[7]);




        savedValue[8] = EEPROM.read(12);      
        debug(F("savedValue[8]  read from addr = 12 is "));  debugln(savedValue[8]);
           


  
}




//************************************************************************************************************



void  firstRunCheck()
{
  
  debugln(F("Checking if first run of sketch:"));

  int key1tmp = EEPROM.read(0);                             // Looking for key1 and key2 values in eeprom
  int key2tmp = EEPROM.read(1);          

  
  if (  ( key1tmp == key1 ) && ( key2tmp == key2 )  )
  {
    debugln(F("Keys found"));
  }  
  else
  {
    debugln(F("Keys NOT found...")); 
    firstLoadOfEeprom();
  }
  
}




//************************************************************************************************************



void  readEeprom()            // For debugging
{
    
  debugln(F("Reading EEPROM:"));

  for (int eepromAddress = 0; eepromAddress <= 12; eepromAddress++) 
  {
    
    int eepromValue = EEPROM.read(eepromAddress);
    debug(F("address: ")); debug(eepromAddress); debug(F(" has ")); debug(eepromValue);
    
    if ( eepromAddress == 11 )
    {
      int8_t eepromValueSigned = eepromValue;
      debug(F(" which is ")); debug(eepromValueSigned);
    }

    debugln();
    
  }


}





//************************************************************************************************************



void  firstLoadOfEeprom()
{

    debugln(F("Loading default EEPROM values")); 

    EEPROM.update(0, 73);     //  Key values 73 and 42 ...to indicate later that it is not first run of sketch
    EEPROM.update(1, 42);       

  
                              // Then update eeprom with some initial values:
    EEPROM.update(2, 10);     // fanMin
    EEPROM.update(3, 90);     // fanMax


    EEPROM.update(4, 40);     // tempMin degC
    EEPROM.update(5, 70);     // tempMax degC
  
    EEPROM.update(6, 100);    // tempMin degF
    EEPROM.update(7, 160);    // tempMax degF

    
    EEPROM.update(8, 1);      // Mode
    EEPROM.update(9, 5);      // Sweep Rate
    EEPROM.update(10, 1);     // Trigger
    EEPROM.update(11, 0);     // tempOffset
    EEPROM.update(12, 0);     // 'Save & Exit'

    readEeprom();

  
}





//************************************************************************************************************



void  operationalMode()
{

  fanMin      = savedValue[0];
  fanMax      = savedValue[1];

  switch ( savedValue[4] ) 
  {
    
    case 1:
      steppedIncDec();    // Sweeping
    break;
    
    case 2:
      triggerMode();      // Triggered    
    break;
    
    default:
      // Never here
    break;
  }


  
}





//************************************************************************************************************




void  triggerMode()
{



    int tempToMap = 0;

    tempToMap = constrain(temperature, tempMin, tempMax);    // limits range of sensor value to between tempMin and tempMax




    switch ( savedValue[6] ) 
    {

  
      case 1:                         // Temp/Zero
        
        if ( triggerFlag == false )
        {

          fanPercentValueA = 0;  
          fanPercentValueB = 0;      
          percentToPWM();

        }

        else

        {    

          fanPercentValueA = map(tempToMap, tempMin, tempMax, fanMin, fanMax);
          fanPercentValueB = map(tempToMap, tempMin, tempMax, fanMin, fanMax);
          
          debug(F("*** fanPercentValueA = "));  debug(fanPercentValueA); 
          debug(F("  *** fanPercentValueB = "));  debug(fanPercentValueB);
          debug(F("  temperature = "));  debug(temperature); 
          debug(F("  tempMin = "));  debug(tempMin);
          debug(F("  tempMax = "));  debug(tempMax);
          debug(F("  fanMin = "));  debug(fanMin);
          debug(F("  fanMax = "));  debugln(fanMax);
          
          percentToPWM();
          
        }

      break;


  
      case 2:                       // Temp/Fmin
      
        if ( triggerFlag == false )
        {

          fanPercentValueA = fanMin;
          fanPercentValueB = fanMin;         
          percentToPWM();

        }

        else

        {
           
          fanPercentValueA = map(tempToMap, tempMin, tempMax, fanMin, fanMax);
          fanPercentValueB = map(tempToMap, tempMin, tempMax, fanMin, fanMax);
          
          debug(F("*** fanPercentValueA = "));  debug(fanPercentValueA); 
          debug(F("  *** fanPercentValueB = "));  debug(fanPercentValueB);
          debug(F("  temperature = "));  debug(temperature); 
          debug(F("  tempMin = "));  debug(tempMin);
          debug(F("  tempMax = "));  debug(tempMax);
          debug(F("  fanMin = "));  debug(fanMin);
          debug(F("  fanMax = "));  debugln(fanMax);
          
          percentToPWM();
          
        }
      
      break;



      case 3:                       // Pin/Zero
        
        if ( triggerFlag == false )
        {

          fanPercentValueA = 0;  
          fanPercentValueB = 0;        
          percentToPWM();

        }

        else

        {    

          fanPercentValueA = map(tempToMap, tempMin, tempMax, fanMin, fanMax);
          fanPercentValueB = map(tempToMap, tempMin, tempMax, fanMin, fanMax);
          
          debug(F("*** fanPercentValueA = "));  debug(fanPercentValueA); 
          debug(F("  *** fanPercentValueB = "));  debug(fanPercentValueB); 
          debug(F("  temperature = "));  debug(temperature); 
          debug(F("  tempMin = "));  debug(tempMin);
          debug(F("  tempMax = "));  debug(tempMax);
          debug(F("  fanMin = "));  debug(fanMin);
          debug(F("  fanMax = "));  debugln(fanMax);
          
          percentToPWM();
          
        }

      break;                      
        

  
      case 4:                       // Pin/Fmin
        
        if ( triggerFlag == false )
        {

          fanPercentValueA = fanMin;
          fanPercentValueB = fanMin;       
          percentToPWM();

        }

        else

        {
           
          fanPercentValueA = map(tempToMap, tempMin, tempMax, fanMin, fanMax);
          fanPercentValueB = map(tempToMap, tempMin, tempMax, fanMin, fanMax);
          
          debug(F("*** fanPercentValueA = "));  debug(fanPercentValueA); 
          debug(F("  *** fanPercentValueB = "));  debug(fanPercentValueB); 
          debug(F("  temperature = "));  debug(temperature); 
          debug(F("  tempMin = "));  debug(tempMin);
          debug(F("  tempMax = "));  debug(tempMax);
          debug(F("  fanMin = "));  debug(fanMin);
          debug(F("  fanMax = "));  debugln(fanMax);
          
          percentToPWM();
          
        }
      
      break; 


  
      default:
        // never here
      break;
    
    }



}






//************************************************************************************************************




int findFanStopLimit()                // Find PWM value where primary fan A starts to turn
{

  int minLimit = 0;


      encoderReadThread.disable();
      tachCounterThread.disable();
      temperatureThread.disable();


  for (int i = 0; i <= 20; ++i) 
  {

      fanPercentValueA = i;
      percentToPWM();

      oled.home();
      oled.println("  fan test  ");



       if ( fanPercentValueA <= 9 )                                                                           
       {
         oled.print(" PWM  "); oled.print(fanPercentValueA); oled.println(":100 "); 
       }
          
       else 
       {  
          oled.print(" PWM "); oled.print(fanPercentValueA); oled.println(":100 "); 
       }   



      oneDit();
      
      debug(F(">Trying fan at: "));  debug(i); debugln(F( "%")); 
      delaySeconds(1);


      isFanSpinning();   

      if ( rpmA != 0 )
      {        
        
        minLimit = i;
        
        oneRoger();
        
        break;
      }

      
  }

      encoderReadThread.enable();
      tachCounterThread.enable();
      temperatureThread.enable();

      
  
  return minLimit;

  

}





//************************************************************************************************************

void  isFanSpinning()             
{                               


    debug(F("countTach : "));  debug(countTach);
  
    rpmA = countTach * 30;        // ( tacho pulses/2 x 60 ) ...since fan sends to pulses per revolution.

    debug(F("        rpmA : "));  debugln(rpmA);



    countTach = 0;

    


  
}




//************************************************************************************************************



void  oneDit()
{

  for ( int toneT = 0; toneT <= 140; toneT = toneT + 1 )
  {
  
    PORTC |= B00000001;   // PC0 High
    PORTC &= B11111101;   // PC1 Low

    delayMicroseconds(250);
  
    PORTC &= B11111110;   // PC0 Low
    PORTC |= B00000010;   // PC1 High
  
    delayMicroseconds(250);

  }

  
}



//************************************************************************************************************



void  oneDah()
{

  for ( int toneT = 0; toneT <= 420; toneT = toneT + 1 )
  {
  
    PORTC |= B00000001;   // PC0 High
    PORTC &= B11111101;   // PC1 Low

    delayMicroseconds(250);
  
    PORTC &= B11111110;   // PC0 Low
    PORTC |= B00000010;   // PC1 High
  
    delayMicroseconds(250);

  }

  
}



//************************************************************************************************************



void  oneRoger()
{

    oneDit();
    delay(70);
    oneDah();
    delay(70);
    oneDit();


}



//************************************************************************************************************
void encoderRead() 
{


  // Is someone pressing the rotary switch?
  if ((!digitalRead(PinEncoderSW))) 
  {
    
    while (!digitalRead(PinEncoderSW))
      delay(10); 

    menuClicked();
    
  }



  if (virtualPosition != lastCount)           // If the current rotary switch position has changed then update everything
  {
    
    debug(F(">vir post value = "));  debugln(virtualPosition); 

    
    valueChanged();    
    
    lastCount = virtualPosition ;             // Keep track of this new value
  }



}





//************************************************************************************************************
void menuClicked() 
{

  if (  savedValue[8] == 1  )                     // if clicked on Save & Exit with YES choice
  {
                
                inConfigMenu = false;                     // Flags out of config menu
                debug(F("Outside ConfigMenu = "));  debugln(inConfigMenu);
                menuItem = -1;            // Resets menuItem

                savedValue[8] = 0;      // Save & Exit always defaults at NO


                writeConfig();         // Write all the config into eeprom

                
                oled.home();
                oled.println("  configs   ");
                oled.println("   saved    "); 
                oneRoger();
                delay(1500);
                
                tachCounterThread.enable();
                temperatureThread.enable();

                debug(F("Saved.. inConfigMenu = "));  debugln(inConfigMenu);

    }

    else
    
    {
      
      menuItem = menuItem + 1;

        debugln(F("CLICK..menuItem =    \t0\t1\t2\t3\t4\t5\t6\t7\t8"));  
        debug(F("\t\t\t")); debug(savedValue[0]); debug(F("\t")); debug(savedValue[1]); debug(F("\t")); debug(savedValue[2]); debug(F("\t")); debug(savedValue[3]); debug(F("\t")); debug(savedValue[4]); debug(F("\t")); debug(savedValue[5]); debug(F("\t")); debug(savedValue[6]);  debug(F("\t")); debug(savedValue[7]);    debug(F("\t")); debugln(savedValue[8]);   
 

      inConfigMenu = true;      
      
      tachCounterThread.disable();
      temperatureThread.disable();

      fanPercentValueA = 0;           // Stop both fans whilst in config mode
      fanPercentValueB = 0;
      percentToPWM();
      
      
      debug(F("Inside ConfigMenu = "));  debugln(inConfigMenu);

      
      if ( menuItem >= 9 )
      {
        
        menuItem = 0;       // Will wrap back to first menu

      }  




      if ( (menuItem == 5)  &&  (savedValue[4] == 2) )
      {

        menuItem = 6;
        
      }


      if ( (menuItem == 6)  &&  (savedValue[4] == 1) )
      {

        menuItem = 7;
        
      }




    
    oled.home();
    oled.println(oledConfig[menuItem]);

    virtualPosition = savedValue[menuItem];
                 
    switch (menuItem)
    {
      case 0:

          if ( savedValue[menuItem] <= 9 )                                                                            // Fan Min %   
          {
            oled.print("      "); oled.print(savedValue[menuItem]);  oled.print("%    ");       
          }
          
          else if ( savedValue[menuItem] <= 99 )   
          {  
            oled.print("     "); oled.print(savedValue[menuItem]);  oled.print("%    ");         
          }  
          
          else   
          {  
            oled.print("    "); oled.print(savedValue[menuItem]);  oled.print("%    ");    
          }  
                
      break;                                                      
                                                                
      case 1:  
          
          if ( savedValue[menuItem] <= 9 )                                                                            // Fan Max %   
          {
            oled.print("      "); oled.print(savedValue[menuItem]);  oled.print("%    ");       
          }
          
          else if ( savedValue[menuItem] <= 99 )   
          {  
            oled.print("     "); oled.print(savedValue[menuItem]);  oled.print("%    ");         
          }  
          
          else   
          {  
            oled.print("    "); oled.print(savedValue[menuItem]);  oled.print("%    ");    
          }  
                 
      break;                                                      

      case 2:
      
          if ( savedValue[menuItem] <= 9 )                                                                            // Temperature Min   
          {
            oled.print("      "); oled.print(savedValue[menuItem]);  oled.print(char(tempChar));   oled.println("    ");      
          }
          
          else if ( savedValue[menuItem] <= 99 )   
          {  
            oled.print("     "); oled.print(savedValue[menuItem]);  oled.print(char(tempChar));   oled.println("    ");      
          }  
          
          else   
          {  
            oled.print("    "); oled.print(savedValue[menuItem]);  oled.print(char(tempChar));   oled.println("    ");      
          }  
         
      break;  
        
      case 3:
      
          if ( savedValue[menuItem] <= 9 )                                                                            // Temperature Max   
          {
            oled.print("      "); oled.print(savedValue[menuItem]);  oled.print(char(tempChar));   oled.println("    ");      
          }
          
          else if ( savedValue[menuItem] <= 99 )   
          {  
            oled.print("     "); oled.print(savedValue[menuItem]);  oled.print(char(tempChar));   oled.println("    ");      
          }  
          
          else   
          {  
            oled.print("    "); oled.print(savedValue[menuItem]);  oled.print(char(tempChar));   oled.println("    ");      
          }  
         
      break;  



      case 4:
        oled.println(oledMode[savedValue[menuItem]]);                                                             // Mode
      break;    


      case 5:

          if ( savedValue[menuItem] >= 10 )                                                       // Sweep rate  
          {
            oled.print("     "); oled.print(savedValue[menuItem]);  oled.println("fast ");      
          }
          else if ( savedValue[menuItem] <= 9  &&  savedValue[menuItem] >= 2 ) 
          {
            oled.print("      "); oled.print(savedValue[menuItem]);  oled.println("     ");  
          }          
          else 
          {  
            oled.print("      "); oled.print(savedValue[menuItem]);  oled.println("slow ");      
          }  
   
      break;  


      case 6:   
      
        //oled.print("     "); oled.print(savedValue[menuItem]);  oled.println("      ");      // Trigger  
        oled.println(oledTrigger[savedValue[menuItem]]);          
        
      break;  


      case 7:

          if ( savedValue[menuItem] < 0 )                                                                               // Temerature sensor correction offset   
          {
            oled.print("    "); oled.print(savedValue[menuItem]);  oled.print(char(tempChar));   oled.println("     ");      
          }
          else if ( savedValue[menuItem] > 0 )
          {  
            oled.print("    +"); oled.print(savedValue[menuItem]);  oled.print(char(tempChar));   oled.println("     ");      
          }       
          else 
          {  
            oled.print("     "); oled.print(savedValue[menuItem]);  oled.print(char(tempChar));   oled.println("     ");      
          } 

      break;    


      case 8:
        oled.println(oledConfigExit[savedValue[menuItem]]);                                                             // Save & Exit    
      break;    

      
      default:    
      // Not here.    
      break;
    }


  }
               


}












//************************************************************************************************************
void valueChanged()
{


    delimitValue(); 


    savedValue[menuItem] = virtualPosition; 
    
    debugln(F("VALUE..menuItem =    \t0\t1\t2\t3\t4\t5\t6\t7\t8"));  
    debug(F("\t\t\t")); debug(savedValue[0]); debug(F("\t")); debug(savedValue[1]); debug(F("\t")); debug(savedValue[2]); debug(F("\t")); debug(savedValue[3]); debug(F("\t")); debug(savedValue[4]); debug(F("\t")); debug(savedValue[5]); debug(F("\t")); debug(savedValue[6]);  debug(F("\t")); debug(savedValue[7]);    debug(F("\t")); debugln(savedValue[8]);  
    

    oled.home();


    oled.println(oledConfig[menuItem]);       


    switch (menuItem)
    {
      case 0:

          fanMin = virtualPosition;

          if ( savedValue[menuItem] <= 9 )                                                                            // Fan Min %   
          {
            oled.print("      "); oled.print(savedValue[menuItem]);  oled.print("%    ");       
          }
          
          else if ( savedValue[menuItem] <= 99 )   
          {  
            oled.print("     "); oled.print(savedValue[menuItem]);  oled.print("%    ");         
          }  
          
          else   
          {  
            oled.print("    "); oled.print(savedValue[menuItem]);  oled.print("%    ");    
          }  
                
      break;                                                      
                                                                
      case 1:

          fanMax = virtualPosition;     
          
          if ( savedValue[menuItem] <= 9 )                                                                            // Fan Max %   
          {
            oled.print("      "); oled.print(savedValue[menuItem]);  oled.print("%    ");       
          }
          
          else if ( savedValue[menuItem] <= 99 )   
          {  
            oled.print("     "); oled.print(savedValue[menuItem]);  oled.print("%    ");         
          }  
          
          else   
          {  
            oled.print("    "); oled.print(savedValue[menuItem]);  oled.print("%    ");    
          }  
                 
      break;                                                      

      case 2:

          tempMin = virtualPosition;
      
          if ( savedValue[menuItem] <= 9 )                                                                            // Temperature Min   
          {
            oled.print("      "); oled.print(savedValue[menuItem]);  oled.print(char(tempChar));   oled.println("    ");      
          }
          
          else if ( savedValue[menuItem] <= 99 )   
          {  
            oled.print("     "); oled.print(savedValue[menuItem]);  oled.print(char(tempChar));   oled.println("    ");      
          }  
          
          else   
          {  
            oled.print("    "); oled.print(savedValue[menuItem]);  oled.print(char(tempChar));   oled.println("    ");      
          }  
         
      break;  
        
      case 3:

          tempMax = virtualPosition;
      
          if ( savedValue[menuItem] <= 9 )                                                                            // Temperature Max   
          {
            oled.print("      "); oled.print(savedValue[menuItem]);  oled.print(char(tempChar));   oled.println("    ");      
          }
          
          else if ( savedValue[menuItem] <= 99 )   
          {  
            oled.print("     "); oled.print(savedValue[menuItem]);  oled.print(char(tempChar));   oled.println("    ");      
          }  
          
          else   
          {  
            oled.print("    "); oled.print(savedValue[menuItem]);  oled.print(char(tempChar));   oled.println("    ");      
          }  
         
      break;  


      case 4:
        oled.println(oledMode[savedValue[menuItem]]);                                                             // Mode
      break;    


      case 5:
      
          sweepDelay = virtualPosition;

          if ( savedValue[menuItem] >= 10 )                                                       // Sweep rate  
          {
            oled.print("     "); oled.print(savedValue[menuItem]);  oled.println("fast ");      
          }
          else if ( savedValue[menuItem] <= 9  &&  savedValue[menuItem] >= 2 ) 
          {
            oled.print("      "); oled.print(savedValue[menuItem]);  oled.println("     ");  
          }          
          else 
          {  
            oled.print("      "); oled.print(savedValue[menuItem]);  oled.println("slow ");      
          }  
   
      break;  


      case 6:  
       
        //oled.print("     "); oled.print(savedValue[menuItem]);  oled.println("      ");      // Trigger 
        oled.println(oledTrigger[savedValue[menuItem]]); 
          
      break;  


      case 7:
                                                                                    
          tempOffset = virtualPosition;
          
          if ( savedValue[menuItem] < 0 )                                                                               // Temerature sensor correction offset   
          {
            oled.print("    "); oled.print(savedValue[menuItem]);  oled.print(char(tempChar));   oled.println("     ");      
          }
          else if ( savedValue[menuItem] > 0 )
          {  
            oled.print("    +"); oled.print(savedValue[menuItem]);  oled.print(char(tempChar));   oled.println("     ");      
          }       
          else 
          {  
            oled.print("     "); oled.print(savedValue[menuItem]);  oled.print(char(tempChar));   oled.println("     ");      
          } 

      break;    


      case 8:
        oled.println(oledConfigExit[savedValue[menuItem]]);                                                             // Save & Exit    
      break;    

      
      default:    
      // Not here.    
      break;
    }

  
}










//************************************************************************************************************
void delimitValue() 
{


  int fanMinAllowed   = (savedValue[1] - 5);  // Current Fan Max, minus 5%.
  int fanMaxAllowed   = (savedValue[0] + 5);  // Current Fan Min, plus 5%.
  int tempMinAllowed  = (savedValue[3] - 1);  // Current Temp Min, minus 1deg.
  int tempMaxAllowed  = (savedValue[2] + 1);  // Current Temp Min, plus 1deg.

 
  
  switch (menuItem)
  {
    
    case 0:
    virtualPosition = min(fanMinAllowed, max(0, virtualPosition));                // Fan Min %.   
    break;                                                      

                                                                
    case 1:
    virtualPosition = min(100, max(fanMaxAllowed, virtualPosition));              // Fan Max  %.      
    break;                                                      


    case 2:

    if (tempdegC == false)    
      {
        virtualPosition = min(tempMinAllowed, max(32, virtualPosition));              // Temperature Min 32 degF
      }
      else
      {
        virtualPosition = min(tempMinAllowed, max(0, virtualPosition));              // Temperature Min 0 degC
      }
    
    break;  

        
    case 3:

    if (tempdegC == false)    
      {
        virtualPosition = min(248, max(tempMaxAllowed, virtualPosition));              // Temperature Max degF
      }
      else
      {
        virtualPosition = min(120, max(tempMaxAllowed, virtualPosition));              // Temperature Max degC
      }
            
    break;  


    case 4:
    virtualPosition = min(2, max(1, virtualPosition));                // Mode           
    break;


    case 5:
    virtualPosition = min(10, max(1, virtualPosition));               // Sweep rate         
    break;  


    case 6:
    virtualPosition = min(4, max(1, virtualPosition));                // Trigger           
    break;  


    case 7:
    virtualPosition = min(9, max(-9, virtualPosition));             // Temerature sensor correction offset    
    break;    


    case 8:
    virtualPosition = min(1, max(0, virtualPosition));                // Save & Exit         
    break;    

      
    default:    
     // Not here.    
    break;
    
  }



}
  



//************************************************************************************************************
void oledDisplay()
{

      oled.home();



      if ( savedValue[4] == 1 )                           // If in Sweep mode, print oled info
      {


      modeIcon = savedValue[5];


          switch (modeIcon) 
          {

            case 1:
              oled.print(char(ICON_SWEEP_1));  
            break;

            case 2:
              oled.print(char(ICON_SWEEP_2)); 
            break;
  
            case 3:
              oled.print(char(ICON_SWEEP_3)); 
            break;

            case 4:
              oled.print(char(ICON_SWEEP_4)); 
            break;

            case 5:
              oled.print(char(ICON_SWEEP_5));  
            break;

            case 6:
              oled.print(char(ICON_SWEEP_6)); 
            break;
  
            case 7:
              oled.print(char(ICON_SWEEP_7)); 
            break;

            case 8:
              oled.print(char(ICON_SWEEP_8)); 
            break;

            case 9:
              oled.print(char(ICON_SWEEP_9)); 
            break;

            case 10:
              oled.print(char(ICON_SWEEP_10)); 
            break;
  

            default:
            // never here
            break;

          }


        }

        else                                                //  Otherwise we are in Trigger mode; print oled info

        {


        modeIcon = savedValue[6];


          switch (modeIcon) 
          {

            case 1:
              oled.print(char(ICON_TEMP_ZERO));  
            break;

            case 2:
              oled.print(char(ICON_TEMP_FMIN)); 
            break;
  
            case 3:
              oled.print(char(ICON_PIN_ZERO)); 
            break;

            case 4:
              oled.print(char(ICON_PIN_FMIN)); 
            break;
  
            default:
              // never here
            break;
  
          }


        }




      switch (pwmBars) 
      {
        
        case 0:
          oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK));
          oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.println(" "); 
        break;
        
        case 1:
          oled.print(char(FILLED_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK));
          oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.println(" ");   
        break;

        case 2:
          oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK));
          oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.println(" "); 
        break;

        case 3:
          oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK));
          oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.println(" ");  
        break;

        case 4:
          oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(EMPTY_BLOCK));
          oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.println(" "); 
        break;                

        case 5:
          oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK));
          oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.println(" "); 
        break;
        
        case 6:
          oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK));
          oled.print(char(FILLED_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.println(" ");   
        break;

        case 7:
          oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK));
          oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.println(" "); 
        break;

        case 8:
          oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK));
          oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.println(" "); 
        break;       

        case 9:
          oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK));
          oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(EMPTY_BLOCK)); oled.println(" "); 
        break;

        case 10:
          oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK));
          oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.print(char(FILLED_BLOCK)); oled.println(" ");   
        break;   

    
        default:
            // never here
        break;
    
      }



      
      oled.println(""); 



      showTempTrigIcon(); 


       
      if ( temperature <= 9 )
      {
        oled.print("  "); 
      }
      
      else if ( temperature <= 99 )
      {
        oled.print(" "); 
      }
     
      else  
      {
        oled.print(""); 
      }
        



      oled.print(temperature);  oled.print(char(tempChar));   // Displays corrected temperature 
 

      

      if (OORflag == true)
      {
        oled.print(char(ICON_OUT_OF_RANGE));      // Shows '!' (temp from sensor is OOR)
      }
      else
      {
        oled.print(" "); 
      }

       
      if ( rpmA <= 9 )
      {
        oled.print("   "); 
      }
      
      else if ( rpmA <= 99 )
      {
        oled.print("  "); 
      }
     
      else if ( rpmA <= 999 )  
      {
        oled.print(" "); 
      }   
     
      else  
      {
        oled.print(""); 
      }
        

      oled.print(rpmA);  oled.print(char(RPM_ARROW));  oled.println(" ");  
     
 
}





//************************************************************************************************************



void  showTempTrigIcon() 
{



      if ( savedValue[4] == 1 )           // We are in Sweep Mode, so don't display any icon.
      {
        oled.print(" ");
      }

      else

      {                                   // We are in Trigger Mode


              if (   (savedValue[6] == 3) || (savedValue[6] == 4)   )    // We are in a Trigger Mode w/ Pinput 
              {


                  if ( pInputFlag == true ) 
                  {
          
                      if ( triggerFlag == false ) 
                      {
                      oled.print(char(ICON_PIN_NOTRIG)); 
                      }
                      else
                      {
                      oled.print(char(ICON_PIN_TRIG));
                      }


          
                  }

                  else

                  {

                    oled.print(" ");
          
                  }


              }


              else

      
              {

                  if ( triggerFlag == true )
                  {
      
                    oled.print(char(ICON_TEMP)); 

                  }

                  else

                  {
                    oled.print(" ");
                  }
        
        
              }



      }




}






//************************************************************************************************************


void  tachCounter()             // timedAction fires every 1 second
{                               //---------------------------------

    debugln("fanPercentValueA = " + String(fanPercentValueA)  + "%" );
    debug(F("countTach : "));  debug(countTach);
  
    rpmA = countTach * 30;        // ( tacho pulses/2 x 60 ) ...since fan sends two pulses per revolution.

    debug(F("     rpmA : "));  debugln(rpmA);

    rpmA = constrain(rpmA, 0, 9999);  // Limits to only four oled characters (so, rpm max of 9999)
    
    debug(F("     rpmA : "));  debug(rpmA);     debugln(F(" constrained"));

    
    fanJamCheck();
    

    oledDisplay();

    countTach = 0;

    startCountTach = millis();


  
}



//************************************************************************************************************






void getTemperature()             // timedAction ... should be short otherwise adds to sweepDelay time
{ 
    // call sensors.requestTemperatures() to issue a global temperature 
    // request to all devices on the bus

    float tempC = 0;
    float tempF = 0;
    

    
    sensors.requestTemperatures(); 



    // Use the function ByIndex, and get the temperature from the first sensor only.
    //Then round the value for displaying
    
    tempC = sensors.getTempCByIndex(0);

    debug(F("..sensor temp = "));  debug(tempC);   debugln("degC tempC float f/sensor.");


    if ( tempC == -127 )        // Value from sensor is above the datasheet max (about 125degC)
    {                           // ...so keep at 120degC (ie. 248degF which keeps it <255) and display '!'
      tempC = 120;
    }

    

    if ( (tempC >= 120)  ||  (tempC < 0)  )       // Flags if temperature is Out Of Range (0-120degC, 32-248degF)
    {
      OORflag = true;
    }
    else
    {
      OORflag = false;
    }

    debug(F("OORflag = "));  debugln(OORflag);



    debug(F("..sensor temp = "));  debug(tempC);   debugln("degC tempC float filter -127.");


    
    tempC = constrain(tempC, 0, 120);      // constrain the value from sensor to between 0degC and 120degC
    debug(F(">>sensor temp = "));  debug(tempC);   debugln("degC tempC float constrained.");  




    if ( tempdegC == false )
    {
      tempF = ( (tempC * 1.8) + 32 );           // Do the conversion
      debug(F("tempF = "));  debug(tempF);  debugln("degF tempF float *converted");
      
      temperature = round(tempF);
      debug(F("tempF = "));  debug(round(tempF));  debugln("degF float rounded");
      debug(F("temperature = "));  debug(temperature);  debugln("degF int rounded");
      temperature = (temperature + savedValue[7]);       // Correct temperature with saved tempOffset value 
      debug(F("temperature = "));  debug(temperature); debugln("degF int corrected");
    }

    else

    {
      temperature = round(tempC);                       // round float tempC 
      debug(F("tempC = "));  debug(round(tempC));  debugln("degC float rounded");
      debug(F("temperature = "));  debug(temperature);  debugln("degC int rounded");
      temperature = (temperature + savedValue[7]);       // Correct temperature with saved tempOffset value 
      debug(F("temperature = "));  debug(temperature); debugln("degC int corrected");
    }
  

    triggerCheck();
    

}







//************************************************************************************************************



void  triggerCheck()
{


        if (   (savedValue[4] == 2)   &&  ( (savedValue[6] == 3) || (savedValue[6] == 4) )   )    // Are we in Trigger Mode w/ Pinput ?
        {

            if ( digitalRead(PIN_PINPUT) == LOW )
            {

              pInputFlag = true;

                if ( temperature >= tempMin )
                {
                  triggerFlag = true;
                }

                else

                {
                  triggerFlag = false;
                }

            }

            else

            {
              
              pInputFlag = false;
              triggerFlag = false;
            
            }


        }


        else


        {


            if ( temperature >= tempMin )
            {
              triggerFlag = true;
            }

            else

            {
              triggerFlag = false;
            }

            
        }



debug(F("triggerFlag = "));  debugln(triggerFlag);
debug(F("pInputFlag = "));  debugln(pInputFlag);
  
}







//************************************************************************************************************



void fanBsetup()
{

  uint8_t binary1 = digitalRead(PIN_SYMM);
  uint8_t binary2 = digitalRead(PIN_ASYMM);

  
      fanBmode = 1*binary1 + 2*binary2;

      debug(F("**** fanBmode = "));  debugln(fanBmode);



      //    Active low, with internal pullup

      //    PIN_ASYMM   PIN_ASYMM
      //    A3          A2
      //    
      //    0           0    Never here     0d
      //    0           1    Asymmetric     1d    A3 Low
      //    1           0    Symmetric      2d    A2 Low
      //    1           1    Hot Standby    3d    Neither low (No jumper)


  
}






//************************************************************************************************************



void fanBconversion()
{

      debug(F("--- fanPercentValue B = "));  debug(fanPercentValueB);
      debug(F("       --- fanMin = "));  debug(fanMin);  debug(F("  --- fanMax = "));  debugln(fanMax);




    if ( inConfigMenu == false )            // Do all this only if not inConfigMode (If inConfigMode, don't invert fanB ie:0% stops also)
    {



        switch (fanBmode) 
        {

        
          case 1:
            // Asymmetric

          if ( savedValue[4] == 2 )   // Do this, when in trigger mode
          {

              if (  ( savedValue[6] == 1 )   ||   ( savedValue[6] == 3 )  )   // Temp/Zero  OR  Pin/Zero
              {                                                               // savedValue[6] == 1 or 3

                if ( triggerFlag == true )          //  Trigger, invert fan B. No trigger, fan B stays zero
                { 
                  fanPercentValueB = constrain(fanPercentValueB, fanMin, fanMax);                   // Limits range between fanMin and fanMax
                  fanPercentValueB = map(fanPercentValueB, fanMin, fanMax, fanMax, fanMin);         // Invert to opposite value for secondary fan B
                }

              }

              else                                                            // Temp/Fmin  OR  Pin/Fmin
                                                                              // savedValue[6] == 2 or 4
              {

                if ( triggerFlag == true )          //  Trigger, invert fan B. No trigger, fan B stays Fmin
                { 
                  fanPercentValueB = constrain(fanPercentValueB, fanMin, fanMax);                   // Limits range between fanMin and fanMax
                  fanPercentValueB = map(fanPercentValueB, fanMin, fanMax, fanMax, fanMin);         // Invert to opposite value for secondary fan B
                }
            
              }

          }

          else                        // Do this, when in sweep mode (savedValue[4] == 1)

          {

  debug(F("fanMin = "));  debugln(fanMin);   
  debug(F("fanMax = "));  debugln(fanMax);    
            
              debug(F("Before const fanPercentValue B = "));  debugln(fanPercentValueB);
            fanPercentValueB = constrain(fanPercentValueB, fanMin, fanMax);                   // Limits range between fanMin and fanMax
              debug(F("After const fanPercentValue B = "));  debugln(fanPercentValueB);
              debug(F("fanMin = "));  debugln(fanMin);   
              debug(F("fanMax = "));  debugln(fanMax);   
            fanPercentValueB = map(fanPercentValueB, fanMin, fanMax, fanMax, fanMin);         // Invert to opposite value for secondary fan B
              debug(F("After map fanPercentValue B = "));  debugln(fanPercentValueB);
          }

          
          break;




          
          case 2:
            // Symmetric (no inversion performed on value for fan B)
            // fanPercentValueB will be the same as fanPercentValueA
          break;



          case 3:
            // Hot Standby 

                                                      // Fan B only substitutes Fan A, if fan A is jammed
            if ( fanAjammed == false )                // ...if main fan A is ok, fan B stays stopped
            {
              fanPercentValueB = 0;
            }
            

            
          break;


          
          default:
            // Never here
          break;


          
        }




      
    }


      debug(F("+++ fanPercentValue B = "));  debugln(fanPercentValueB);   




}







//************************************************************************************************************



void percentToPWM()
{


    float pwmValue = 0;
    uint16_t pwmNew = 0;



 

    pwmValue = ( (float)fanPercentValueA/100.0 ) * 320; 

    pwmNew = round(pwmValue); 

    OCR1A = pwmNew;    // PWM set on pin 9



    fanBconversion();

    debug(F(">>> fanPercentValue A = "));  debug(fanPercentValueA);     debug(F("  >>> fanPercentValue B = "));  debugln(fanPercentValueB);     

    pwmValue = ( (float)fanPercentValueB/100.0 ) * 320; 

    pwmNew = round(pwmValue); 

    OCR1B = pwmNew;    // PWM set on pin 10



    percentToBars();



}







//************************************************************************************************************


void  percentToBars()
{
    
    float pwmBarsValue = ( (float)fanPercentValueA/10.0 );
    
    pwmBars = round(pwmBarsValue); 
         
}






//************************************************************************************************************

void  fanJamCheck()
{

    static uint8_t newJam = 0;

    
    debug(F("newJam: "));  debug(newJam);  debugln(F(" <- in jam check"));

    if ( (rpmA == 0) && (fanPercentValueA >= fanStopLimit) )      
    { 

        newJam = newJam + 1;
        
        debugln(); 
        debug(F("newJam: "));  debug(newJam);  debugln(F("**FAN JAM"));

          if ( newJam >= 4 )            // Maaximum number of violations here is 4
          { 
            debug(F("newJam: "));  debug(newJam);  debugln(F("  FAN JAMMED!!!!!!!!! > 3secs"));

            fanAjammed = true;
            debug(F("fanAjammed: "));  debug(fanAjammed);  debugln(F(" !!!!! "));
            
            oled.home();
            oled.println(""); 

            if ( triggerFlag == true )
            {
              //oled.print(char(ICON_TEMP)); 
              showTempTrigIcon();
            }
            else
            {
              oled.print(" ");
            }

             
            if ( temperature <= 9 )
            {
              oled.print("  "); 
            }     
            else if ( temperature <= 99 )
            {
              oled.print(" "); 
            }
            else  
            {
              oled.print(""); 
            }
        
            oled.print(temperature);  oled.print(char(tempChar));   // Displays corrected temperature 

            
            oled.print("    0");
            oled.print(char(RPM_JAMMD));  oled.println(" ");  

          digitalWrite(PIN_POUTPUT, HIGH);
          oneDah();  
          digitalWrite(PIN_POUTPUT, LOW);
          
          newJam = 0;
     
          }  

    
    debug(F("fanAjammed: "));  debug(fanAjammed);  debugln(F(" !!!!! "));
  
    }

    else

    {
      newJam = 0;
      fanAjammed = false;
      debug(F("fanAjammed: "));  debug(fanAjammed);  debugln(F(" !!!!! "));
    }

    
}




//************************************************************************************************************

void  steppedDecrement()
{
debug(F("\\ fanMin = "));  debug(fanMin); debug(F("   fanMax = "));  debugln(fanMax);

    for ( int i = fanMax; i >= fanMin; i = i - 5 )              // Decrease in steps of 5% 
    {


      if (inConfigMenu == true) 
        { 
          fanPercentValueA = 0;
          fanPercentValueB = 0;
          percentToPWM();
          break;
        }


      debugln(); debugln("Dec");
      
      fanPercentValueA = i;
      fanPercentValueB = i;
      
      debugln(""); debug(F("i: "));  debugln(i); 
      debug(F("fanPercentValueA: "));  debugln(fanPercentValueA); 
      
      percentToPWM();

      
      delaySweepSeconds(map(sweepDelay, 1, 10, 10, 1));     // Reverse maps value so 1 is Slow and 10 is Fast

     
            
    }   

    
 
}





//************************************************************************************************************


void  steppedIncrement()
{
debug(F("// fanMin = "));  debug(fanMin); debug(F("   fanMax = "));  debugln(fanMax);

    for ( int i = fanMin; i <= fanMax; i = i + 5 )            // Increase in steps of 5%     
    {
      

      if (inConfigMenu == true) 
        {  
          fanPercentValueA = 0;
          fanPercentValueB = 0;
          percentToPWM();
          break;
        }

         
      debugln(); debugln("Inc");
      
      fanPercentValueA = i;
      fanPercentValueB = i;
      
      debugln(""); debug(F("i: "));  debugln(i); 
      debug(F("fanPercentValueA: "));  debugln(fanPercentValueA); 
      
      percentToPWM();


      delaySweepSeconds(map(sweepDelay, 1, 10, 10, 1));     // Reverse maps value so 1 is Slow and 10 is Fast


   
    }   
  
}



//************************************************************************************************************


void  steppedIncDec()
{



      steppedIncrement();
      steppedDecrement();



}



//************************************************************************************************************


void pwm25kHzSet() 
{
  
    //Set PWM frequency to about 25khz on pins 9,10 (Timer1 mode 10, no prescale, count to 320)
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << CS10) | (1 << WGM13);
    ICR1 = 320;
    OCR1A = 0;
    OCR1B = 0;

}



//************************************************************************************************************


void delaySweepSeconds(int seconds)           //for long delays feed wdt
{                                             // call using   delaySeconds(2);  for two second delay   
                                                
  unsigned long currentMillis = 0;
  int millisPeriod = 100;             //100mS period


                                               
  for (int i = 0; i <= seconds*10; i++)
  {
    currentMillis = millis();
    
    while(millis() < currentMillis + millisPeriod)
    {  

    if (inConfigMenu == true) 
    {
      break;
    }


      
      encoderReadThread.check();  
      tachCounterThread.check();
      temperatureThread.check();
      
    }        
  }

  
}




//************************************************************************************************************
                                                //Example for short delays
void delayTseconds(int tseconds)                //for short delays feed wdt
{                                               //call using   delayTseconds(2);  // two tenths of seconds, 0.2secs
  
  unsigned long currentMillis = 0;
  int millisPeriod = 100;
  
  for (int i = 0; i <= tseconds; i++)
  {
    currentMillis = millis();
    
    while(millis() < currentMillis + millisPeriod)
    {

    if (inConfigMenu == true) 
    {
      break;
    }
      
      encoderReadThread.check();  
      tachCounterThread.check();
      temperatureThread.check();
      
    }     
      
  }    

}




//************************************************************************************************************


void delaySeconds(int seconds)                //for long delays feed wdt
{                                             // call using   delaySeconds(2);  for two second delay   
                                                
  unsigned long currentMillis = 0;
  int millisPeriod = 100;             //100mS period


                                               
  for (int i = 0; i <= seconds*10; i++)
  {
    currentMillis = millis();
    while(millis() < currentMillis + millisPeriod)
    {  
      encoderReadThread.check();  
      tachCounterThread.check();
      temperatureThread.check();
    }        
  }

  
}






// ----------------------------------------------------------------------
// INTERRUPTS     INTERRUPTS     INTERRUPTS     INTERRUPTS     INTERRUPTS
// ----------------------------------------------------------------------



void encoderISR ()  
{
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();


  if ( inConfigMenu == true )     // Only updates virtualPosition if in configMenu
  {


      if (interruptTime - lastInterruptTime > 5)        // If interrupts come faster than 5ms, assume it's a bounce and ignore
      {

        if  ( (menuItem == 0) || (menuItem == 1) )      
        {
                                                          // fanMin and fanMax in steps of 5%
              if (digitalRead(pinEncoderCLK) == LOW)
              {
                virtualPosition = virtualPosition - 5 ; 
              }
              else 
              {
                virtualPosition = virtualPosition + 5 ; 
              }
                   
        }
        else
        {
     
              if (digitalRead(pinEncoderCLK) == LOW)
              {
                virtualPosition = virtualPosition - 1 ; 
              }
              else 
              {
                virtualPosition = virtualPosition + 1 ; 
              }
                
        }  
       
      }

    lastInterruptTime = interruptTime;        // Keep track of when we were here last (no more than every 5ms)
      
   }
          

}





// ----------------------------------------------------------------------


void  tachISR()
{
  countTach++;
}
