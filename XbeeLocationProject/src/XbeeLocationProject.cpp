//============================================================================
// Name        : XbeeLocationProject.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

// Use this code for the project. It uses interupts and therefore does not slow down code so that I can get
// good RSSI values.

//Make sure to install the adafruit GPS library from https://github.com/adafruit/Adafruit-GPS-Library

  #include <XBee.h>
  #include <Average.h>
  #include <Encoder.h>

  #include <Wire.h>
  #include <avr/wdt.h> // this library helps control the watchdog timer which resets the arduino after a certain amount of time.
                       //more on the watchog can be found online in an arduino forum with a pdf attachment. watchdog20110611.pdf


  #include <LiquidCrystal_I2C.h>
  #include <Adafruit_GPS.h> //Load the GPS Library. Make sure you have installed the library form the adafruit site above
  #include <SoftwareSerial.h> //Load the Software Serial Library. This library in effect gives the arduino additional serial ports
  #include <HMC5883L.h>   // for the compass
   #include <Adafruit_NeoPixel.h> for the neopixel lights
//**** Compass CODE Variables
   HMC5883L compass;

   #define NUM_LEDS 12  // Number of LEDs on Ring
    #define DATA_PIN_RING 3 // Pin 3 connected to RGB Ring

    Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, DATA_PIN_RING, NEO_GRB + NEO_KHZ800);
    //CRGB leds_RING[NUM_LEDS];
    int fixedHeadingDegrees; // Used to store Heading value

 float headingDegrees = 0.0;  // measurement of the heading





//**** GPS CODE Variables


    SoftwareSerial mySerial(53, 52); //Arduino UnoInitialize SoftwareSerial, and tell it you will be connecting through pins 7 and 8
    Adafruit_GPS GPS(&mySerial); //Create GPS object

    //#define mySerial Serial1 //Necessary for Arduino Mega
    //Adafruit_GPS GPS(&mySerial); //Necessary for Arduino Mega


    // Like Parse Gps example
    // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
    // Set to 'true' if you want to debug and listen to the raw GPS sentences.
    #define GPSECHO  true

    // this keeps track of whether we're using the interrupt
    // off by default!
    boolean usingInterrupt = false;
    void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
    //

    String NMEA1;  //We will use this variable to hold our first NMEA sentence
    String NMEA2;  //We will use this variable to hold our second NMEA sentence
    char cc;       //Used to read the characters spewing from the GPS module


    float timer = 0.0;
    //Store Gps Locations during calculations
    float GpsLat1 = 0.0; // gives Gps locations.
    float GpsLon1 = 0.0;
    float GpsLat2 = 0.0;
    float GpsLon2 = 0.0;
    float GpsLat3 = 0.0;// Gps 3's are for the new approx. xbee location.
    float GpsLon3 = 0.0;


    //To calculate the distance between GPS locations
    float setGPS1 = 1; //this helps us set the first GPS location and keep those values
    float distanceMeters = 0.0;
    float CurLat = 0.0;
    float CurLon =  0.0;

    //Calculate the bearing of the boat.
    float yCoord = 0.0;
    float xCoord = 0.0;
    float boatBearing = 0.0;

    //Calculate Gps loc of a point knowing distance and bearing from another Gps location

    float r_CurLon;
    float r_CurLat;
    float Bearing = 45; // Bearing of travel //**** IS this used?
    float r_Bearing;
    float Distance = 0.0; // km per update
    int Eradius = 6371; // mean radius of the earth


    //Variables from the larger code
    //float maxArrayValue = 21;
    //float maxArrayValueAngle = 0.0;






     //approx calculations of xbee location and bearing
     float firstAngle = 0.0;
     float secondAngle = 0.0;
     float firstInnerAngle = 0.0;
     float secondInnerAngle = 0.0;
     float currentApproxBearing = 0.0;
     float approxDistanceXbee = 0.0;
     float currentApproxDistance = 0.0;

//MODES VARIABLES.
    //Mode 2 variables
    const int Mode2Switch = 4; //*** Need to hook this up with a switch
    //void convertAngle();
    void Mode2(); // Creates Prototype at the beginning of code

//SCAN ARRAY CODE

    #define ssRX 2
    #define ssTX 3
    SoftwareSerial nss(ssRX, ssTX);

    //RSSI SCAN CODE VARIABLES
    //Rssi Command
    uint8_t d0Cmd[]={'D','B'};
    uint8_t* retVal; // variables for Xbee Rssi command
    uint16_t retSize;
    int rssi = 0;

    XBee xbee = XBee();//******* need to include all the Xbee Code
    XBeeResponse response = XBeeResponse();
    XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x403b0afd);//Address of the Router Xbee;
    RemoteAtCommandResponse racr = RemoteAtCommandResponse();
    RemoteAtCommandRequest racrq = RemoteAtCommandRequest(addr64, d0Cmd);


    //Scan and ShortScan function Arrays and Variables for functions
    int ScanArrayRssi[5];    // maybe make these global values
    int ScanAngleArray[5];//makes an array of angles that correspond to Rssi array
    int setValue = 0;     // These three variables keep track of how many times the system has changed directions.
    int PreValue = 0;
    int PreValue1 = 0;
    int NowRssi = 0;         //make this global as well
    int NowAngle = 0;
    int EqualRssi = 0;
    int  shortScanArrayRssi[5] = {0,0,0,0,0};    // maybe make these global values
    int shortScanAngleArray[5] = {0,0,0,0,0};   //makes an array of angles that correspond to Rssi array
    int shortScanNowRssi = 0;     //make this global as well
    int shortScanNowAngle = 0;
    int ShortScanEqualRssi = 0;
    int shortScanEqualRssi =0;
    int maxArrayValueAngle = 0;
    int maxArrayValue = 0;
    int LenArray =5;
    int AveragedAngle = 0;
    int AddedAngles = 0;

    //motor control variables
    const int enA = 12;
    const int in1 = 11;
    const int in2 = 9;


    //Second motor control Variables
    //const int in4 = 11;
    //const int in3 = 12;
    //const int enB = 13;



    //Set Point pin
    //const int setPointPin=7;
    //const int setPointAnalogPin = A0;

    //Time Variables
    unsigned long lastTimePid;

    //Encoder Object
    Encoder myEnc(18, 19);

    //Current Position
    double currentPos = 0;
    //int i = 0;
    //int maxat;

    //Encoder position variable
    long encoderPosPrev  = -999;

    //Control Variables
    double kp = 10;
    double ki = 0;
    double kd = 40;
    double lastErr = 0;
    double errSum = 0;
    double pidControlSignal = 0;
    double zero = 0;        // helps with tracking setpoint value

    //int digitalPin = 3;   // rssi signal strength data
    unsigned long rfData = 30;
    unsigned long rfSignal = 0;
    double rfAngle = 0;     // current postion rf data was taken.
    int k = 0;
    int x = 0;


    bool currentDir;
    bool before = true;

    //Target Set Point Variable
    const double firstValue = 160;
    double setPointTarget = firstValue;
    double setPointTargetValue = firstValue;
    double setPointTargetValue1 = firstValue;
    double motorAngle = 0;
    double interval = 4000;                   // amount of time millis is waiting to then turn off motors in moveWheels() function
    double interval1 = 4000;
    long previousMillis = 0;

    //Environment Variables
    double trackLength = 1.1176;

    //Feed Forward Variables
    //float cartPrev[2] = {0,0};
    float cartCurrent[3] = {0,0,0};
    //// RF signal Strength
    //float SignalStrength[6] = {0,0,0,0,0,0};


    // Variables to be created for CalibrateMotor180
    int CalZeroPosition = 0; // variable for the zero calibration encoder reading
    int CalOneEightyPosition = 0;//Varibale to hold 180 encoder reading
    const int ZeroCalPin = 4; // pin to read when the zero cal limit switch goes off
    const int OneEightyCalPin = 5;//Pin to read when 180 limit switch goes off
    bool calibFinished = true;
    int TotalEncoderCalDegrees  = 0; //keeps the total number of encodervalues
    int ActivatedOneEighty = 0; //tells us that the 180 degree limit has been switched.
    bool zerosystem = true;// true,false for the while loop in setup

    void watchdogSetup(void){
    cli();   // disable all interrupts
    wdt_reset(); // reset the WDT timer
    /*
     WDTCSR configuration:
     WDIE = 1: Interrupt Enable
     WDE = 1 :Reset Enable
     WDP3 = 0 :For 2000ms Time-out
     WDP2 = 1 :For 2000ms Time-out
     WDP1 = 1 :For 2000ms Time-out
     WDP0 = 1 :For 2000ms Time-out
    */
    // Enter Watchdog Configuration mode:
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    // Set Watchdog settings:
     WDTCSR = (1<<WDIE) | (1<<WDE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
    sei();
    }

    //**** mode 1
    /*
     if(digitalRead(Mode1Switch)== HIGH)) {
        digitalWrite(GreenLedMode1,HIGH); // LED turn on to say that mode is on.
        digitalWrite(RedLedMode1,LOW);    // LED turn off to say mode is on.

        Mode1(); // turn on mode1() function that will run

        //LCD.print("Mode1");
           //LCD.print("Current RSSI:"); LCD.pirnt(RSSI);
           //LCD.print("Heading"); LCD.print(heading);

     }

     void Mode1(){
               readRSSI();

          readEncoders();

          scan();          // tells us which rssi value is the highest and sends information to LCD screen about compass

          AngleAverage();

          setPointValue(); // sets value for motors to turn around

          controlpid();    // sets value of PID pwm to put for motors

          moveWheels();

     }

      - this will activate in the loop function
      - have a button or switch that activates the mode 1 a
      - make antenna move in 180 degrees scans and give the direction(heading) of the highest rssi signal.
      - show the results using the LCD screen and the lights.
      - idicate that the system is in Mode 1 and to turn on an LED green light that indicates this
      - when mode 1 a is off use a red LED to indicate this.
      -


    */

    //**** mode 2
    /*
     *
        if(digitalRead(Mode2Switch == HIGH){
              digitalWrite(GreenLedMode2,HIGH); // LED turn on to say that mode is on.
              digitalWrite(RedLedMode2,LOW);    // LED turn off to say mode is on.

              Mode2();
              //GPS
              //Heading
              //LCD
              //Neopixel
        }

      void Mode2(){

          Mode2();//(recalled GPSCalc

          readRSSI();

          readEncoders();

          scan();          // tells us which rssi value is the highest and sends information to LCD screen about compass

          AngleAverage();

          setPointValue(); // sets value for motors to turn around

          controlpid();    // sets value of PID pwm to put for motors

          moveWheels();
      }
      - this will activate in the loop function
      - have a button or switch that activates the mode 2
      - make antenna move in 180 degrees scans and give the direction(heading) of the highest rssi signal.
      - Activate gps readings
      - show the results using the LCD screen and the lights.
      - idicate that the system is in Mode 2 and to turn on an LED green light that indicates this
      - when mode 2 is off use a red LED to indicate this.
      - show gps locations of current location and estimated XBee location.
      - show direction of calculation and direction of highest read RSSI signal
      - Show idicator on the Neopixel sheet.
      -


    */


//**** mode 3 Manual mode
    /*
     *
     *
        if(digitalRead(Mode3Switch == HIGH){
              digitalWrite(GreenLedMode3,HIGH); // LED turn on to say that mode is on.
              digitalWrite(RedLedMode3,LOW);    // LED turn off to say mode is on.

              Mode3();
              //ManualMove
              //GPS
              //Heading
              //
              //Neopixel
        }

      - this will activate in the loop function
      - have a button or switch that activates the mode 3
      - have another button that increments the location to the right.
      - have a button that increments the location to the left.
      - continously be reporting the RSSI Signal being read and the highest signal read in the last 1 minute.
      - **** Try to use a joystick to move the antenna. better for customer interaction. could also help to get it to move more naturally.
      - show the results using the LCD screen and the lights.
      - idicate that the system is in Mode 3 and to turn on an LED green light that indicates this
      - when mode 3 is off use a red LED to indicate this.
      -
      - show direction of calculation and direction of highest read RSSI signal
      - Show idicator on the Neopixel sheet.
      -


    */

 //****   Mode 4 to edit range of system.
 //make it so you can edit the range over which the system moves back and forth between.
    /*
     *
     *
        if(digitalRead(Mode4Switch == HIGH){
              digitalWrite(GreenLedMode4,HIGH); // LED turn on to say that mode is on.
              digitalWrite(RedLedMode4,LOW);    // LED turn off to say mode is on.

              Mode4();
              //turn off motors
              //acivate potentiometers
              //change variables that control range.
              ///****maybe make a test button that only acitvate during setting the range

        }

      - set up two pots to do this.
      - make sure they cant be set up incorrectly.
      - Use Neopixel led to indicate range.
      -
      - show the results using the LCD screen and the lights.
      - idicate that the system is in Mode 4 and to turn on an LED green light that indicates this
      - when mode 4 is off use a red LED to indicate this.
      -



    */


void setup()
{
      Serial.begin(9600);  //Turn on the Serial Monitor

      GPS.begin(9600);       //Turn GPS on at baud rate of 9600
      GPS.sendCommand("$PGCMD,33,0*6D"); // Turn Off GPS Antenna Update
      GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Tell GPS we want only $GPRMC and $GPGGA NMEA sentences
      GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
      //useInterrupt(true);
      delay(1000);  //Pause

      Serial3.begin(9600);
      // now that they are started, hook the XBee into
      //the Serial3 pins
      xbee.setSerial(Serial3);
      Serial.println("Initialization all done!");
      //set the pins for the calibration mode
      pinMode(ZeroCalPin,4);
      pinMode(OneEightyCalPin,5);

      //Setup Code for the Compass and Neopixels
       Serial.println("Initialize HMC5883L");
      while (!compass.begin())
        {
          Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
          delay(500);

        }

                  // Set measurement range
            compass.setRange(HMC5883L_RANGE_1_3GA);

            // Set measurement mode
            compass.setMeasurementMode(HMC5883L_CONTINOUS);

            // Set data rate
            compass.setDataRate(HMC5883L_DATARATE_30HZ);

            // Set number of samples averaged
            compass.setSamples(HMC5883L_SAMPLES_8);

            // Set calibration offset. See HMC5883L_calibration.ino
            compass.setOffset(-5, -113);

            strip.begin();
            strip.show();



     //Watchdog setup *** This code helps to reset the arduino mega using a software reset in order to help zero out the encoders
       watchdogSetup();



    if(digitalRead(ZeroCalPin) != HIGH){
            while(zerosystem){
                //*** tell the system watchdog to reset
                if(digitalRead(ZeroCalPin) == HIGH){
                  analogWrite(enA, 0);//Stop motors
                  zerosystem = false;
                  delay(3000); //this gets whole arduino system to reset by not resting the watchdog timer before 2 seconds.
                  }
                else{
                    wdt_reset();
                    Serial.println("Waiting for zero Position before reseting arduino");
                    digitalWrite(in1, LOW);
                    digitalWrite(in2, !LOW);
                    analogWrite(enA, abs(100));//move motor at const.speed.
                    }
                }
       }


}
//Code from GPS parsing example
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
  SIGNAL(TIMER0_COMPA_vect) {
     //Serial.println("Signal Activated");
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
  #ifdef UDR0
    if (GPSECHO)
      if (c) UDR0 = c;
      //Serial.println(c);
      // writing direct to UDR0 is much much faster than Serial.print
      // but only one character can be written at a time.
  #endif
  }

void useInterrupt(boolean v) {
          Serial.println("UseInterupt Activated");
      if (v) {
          // Timer0 is already used for millis() - we'll just interrupt somewhere
          // in the middle and call the "Compare A" function above
          OCR0A = 0xAF;
          TIMSK0 |= _BV(OCIE0A);
          usingInterrupt = true;
          }
      else {
          // do not call the interrupt function COMPA anymore
          TIMSK0 &= ~_BV(OCIE0A);
          usingInterrupt = false;
        }
    }



void loop()                     // run over and over again
{
   wdt_reset();

   // run compass code
      CompassAntennaBearing();

  //From GPs Example Parsing code
       if (! usingInterrupt) {
            // read data from the GPS in the 'main loop'
            char c = GPS.read();
            // if you want to debug, this is a good time to do it!
            if (GPSECHO){
                if (c) {Serial.print(c);}
             }
        }
       if (GPS.newNMEAreceived()) {
              // a tricky thing here is if we print the NMEA sentence, or data
              // we end up not listening and catching other sentences!
              // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
            //  Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
            if (!GPS.parse(GPS.lastNMEA())) {  // this also sets the newNMEAreceived() flag to false
                 return;  // we can fail to parse a sentence in which case we should just wait for another

             }
         }

    useInterrupt(true);

    GPS_Calc();
/*

       if(digitalRead(Mode1Switch)== HIGH)) {
            digitalWrite(GreenLedMode1,HIGH); // LED turn on to say that mode is on.
            digitalWrite(RedLedMode1,LOW);    // LED turn off to say mode is on.

            Mode1(); // turn on mode1() function that will run

        //LCD.print("Mode1");
           //LCD.print("Current RSSI:"); LCD.pirnt(RSSI);
           //LCD.print("Heading"); LCD.print(heading);

         }


     if(digitalRead(Mode2Switch == HIGH){
              digitalWrite(GreenLedMode2,HIGH); // LED turn on to say that mode is on.
              digitalWrite(RedLedMode2,LOW);    // LED turn off to say mode is on.

              Mode2();

        }


      if(digitalRead(Mode3Switch == HIGH){
              digitalWrite(GreenLedMode3,HIGH); // LED turn on to say that mode is on.
              digitalWrite(RedLedMode3,LOW);    // LED turn off to say mode is on.

              Mode3();
              //ManualMove
              //GPS
              //Heading
              //
              //Neopixel
        }


   */



///////////////////////////////////////////////////
 if(setValue < 4){
  //  when testing this needs to be  large
  //    for(int i = 0; i< 5 ; i++){
  //      shortScanArrayRssi[i] = 0;
  //      shortScanAngleArray[i] = 0;
  //    }
  //



    readRSSI();
    //delay(25);
    readEncoders();

    scan();          // tells us which rssi value is the highest and sends information to LCD screen about compass
    AngleAverage();
    setPointValue(); // sets value for motors to turn around
    controlpid();    // sets value of PID pwm to put for motors
    moveWheels();    // moves motor to setpoint location
    Serial.println("setValue");
    Serial.println(setValue);
   }




    //Protect system from going to far past limit switches

          if (cartCurrent[0] <= -5 || cartCurrent[0] >= 200 ){
                    analogWrite(enA, 0);//Stop motors
                  //tell motors to stop and to give the error

                  while(true){ //this will also make the arduino reset because of the watchdog reset method being used
                    Serial.println("Got to close to limit. Reset arduino. Stuck in while loop");
                  }


                  //Serial.println("readRssi"); //debuging



 }
   else if (setValue == 4){
    Serial.println("FIRST else if was accessed");
     Serial.println("setValue");
    Serial.println(setValue);
            setPointTargetValue  = AveragedAngle;  //*********    Figure out how to use this with the new compass anlges    **************//////

            for(int i = 0; i< 5 ; i++){ // make the two arrays have zero's in them.
              shortScanArrayRssi[i] = 0;
              shortScanAngleArray[i] = 0;

              }
               setValue = setValue +1;
   }
   else if (4 < setValue && setValue <= 10){ //*** make sure there is a statement in here that sends it back to the first if statement when there is no high Rssi value
    Serial.println("second else if was accessed");
     Serial.println("setValue");
    Serial.println(setValue);
    readRSSI();
    readEncoders();   // read encoders
    AngleAverage();
    setPointValue2(); // sets value for motors to turn around that centers with Rssi value
    controlpid();     // sets value of PID pwm to put for motors
    moveWheels();     // moves motor to setpoint location
    shortScan();      // runs a scan that is much shorter

   }


   else if (10 < setValue && setValue <= 16){ //*** make sure there is a statement in here that sends it back to the first if statement when there is no high Rssi value

    setPointTargetValue  = maxArrayValueAngle;
    readRSSI();
    readEncoders(); // read encoders
    setPointValue2(); //sets value for motors to turn around that centers with Rssi value
    controlpid();   //sets value of PID pwm to put for motors
    moveWheels();  // moves motor to setpoint location
    shortScan();  // runs a scan that is much shorter
   }
     else if (16 < setValue){ // brings the loop back to the first if statement.
      setValue = 0;
     }



/////////////////////////////////////////////////
   /*  for( int i = 0; i < 6; i++){ // this should help get more readings and thus better calculations for our tests.
        readRSSI();

        readEncoders();

        scan();          // tells us which rssi value is the highest and sends information to LCD screen about compass

        AngleAverage();

        setPointValue(); // sets value for motors to turn around

        controlpid();    // sets value of PID pwm to put for motors

        moveWheels();    // moves motor to setpoint location
        Serial.println("setValue");
        Serial.println(setValue);
        wdt_reset();
     }
 */
}


void CalibHeadToPosition(){
   readEncoders();
   currentPos = cartCurrent[0];
 NowAngle = headingDegrees;

float Num = NowAngle-currentPos;
if(Num >=0 ){                               ////////// *****************not finished here!!!!! still need to complete
     int Total = Num;
     int TotalPlus180 = 180+total;

     if(TotalPlus180 > 360){
           int NewTotal = 360 - TotalPlus180;

        }
     else{
          NewTotal = TotalPlus180
         }
         if(AverageAngle>=180){
           int Degrees = map(AverageAngle, Total, 360, 0, abs(Num));
         }

           else {
         int Degrees = map(AveragedAngle,0,NewTotal,0,NewTotal);
     }



     }

  }
  if(Num < 0){
    Total = 360+Num;
    TotalPlus180 = Total+180;
      if(TotalPlus180 > 360){
        int NewTotal = 360 - TotalPlus180;
         if(headingDegrees>Total){
                int Degrees = map(headingDegrees, Total, 360, 0, abs(Num));  ////////// *****************not finished here!!!!! still need to complete
             }
         else  {
                int Degrees = map(headingDegrees,0,NewTotal,0,180);
           }
     }
      Total->TotalPlus180
     TotalPlus180 -> NewTotal
     TotalPlus180


  }
}




     void Mode1(){
               readRSSI();

          readEncoders();

          scan();          // tells us which rssi value is the highest and sends information to LCD screen about compass

          AngleAverage();

          setPointValue(); // sets value for motors to turn around

          controlpid();    // sets value of PID pwm to put for motors

          moveWheels();

     }


  void Mode2(){

        //GPS
              //Heading
              //LCD
              //Neopixel

          GPS_Calc();//(recalled GPSCalc

          readRSSI();

          readEncoders();

          scan();          // tells us which rssi value is the highest and sends information to LCD screen about compass

          AngleAverage();

          setPointValue(); // sets value for motors to turn around

          controlpid();    // sets value of PID pwm to put for motors

          moveWheels();
      }


void GPS_Calc(){
  wdt_reset(); //reset the watchdog timer.
   //Serial.println("Mode2 running");
   // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

   //   readGps(); is a function we define below which reads two NMEA sentences from GPS
      if (millis() - timer > 2000) {
     if (GPS.fix) {
         Serial.println("GPS has a fix");

         //**** Bug seams to happen when no other Xbees are found and signal strength becomes 100   *******///

    if(maxArrayValue >= 20 && setGPS1 == 1){//Pick a value that the Rssi must be larger than before we begin to calculate the approx distance of the xbee.
        // maxArrayValueAngle = 45; //for testing the code. using a made up maxArray Value Angle
        // firstAngle = convertAngle(AveragedAngle); //convertAngle converts the max array angle into a bearing with respect to the compass.

         //readGPS();
         GpsLat1 = GPS.latitudeDegrees;
         GpsLon1 = GPS.longitudeDegrees;
         Serial.println("GpsLat1");
         Serial.print(GpsLat1,6);
         Serial.print(",");
         Serial.print(GpsLon1,6);
          Serial.println("GpsLon1");
         setGPS1 = 10; //**FOR TESTING: this will stop this if statement from being used after the first time.
    }
    if (millis() -timer> 45000 && setGPS1 == 10){
        firstAngle = convertAngle(AveragedAngle);
        setGPS1 = 11;
    }
    //readGPS();
        Serial.println("continually measuring");
        CurLat = GPS.latitudeDegrees;
        CurLon = GPS.longitudeDegrees;
        Serial.println("GpsLat1");
         Serial.print(GpsLat1,6);
          Serial.print(",");
          Serial.print(GpsLon1,6);
          Serial.println("GpsLon1");
        Serial.print("CurLat");
          Serial.print(CurLat,6);
          Serial.print(",");
          Serial.print(CurLon,6);
          Serial.println("CurLon");
        distanceMeters = calc_dist(GpsLat1,GpsLon1,CurLat,CurLon);
        Serial.println("distanceMeters");
        Serial.println(distanceMeters);


     if (distanceMeters >= 15 && setGPS1 == 11){  // this resets arrays so that we can get new data before reaching 20meters.
         for(int i = 0; i< 5 ; i++){ // make the two arrays have zero's in them.
              shortScanArrayRssi[i] = 0;
              shortScanAngleArray[i] = 0;

              }
                 setGPS1 = 12;
          }


    //if(maxArrayValue >= 20){///********need to Create something that gets data taken a new for a certain amount of time. that way data when we hit 20meters is accurate.
    if (distanceMeters >= 20){ //****need to make something that lets us know if there is no max Rssi signal and angle any more. then restart.
//        Serial.println("distanceMeters");
//        Serial.println(distanceMeters);
        GpsLat2 = CurLat;
        GpsLon2 = CurLon;
        //maxArrayValueAngle = 90;
         Serial.println("firstAngle");
         Serial.println(firstAngle);
        secondAngle = convertSecondAngle(AveragedAngle); //convert second inner angle
        Serial.println("secondAngle");
        Serial.println(secondAngle);
        boatBearing =  findMyBearing(GpsLat1,GpsLon1, GpsLat2,GpsLon2);// calculates the bearing of the boat.
        firstInnerAngle = calcFirstInnerAngle(firstAngle, boatBearing); //function that calculates the inner angle with respect to the bearing and line between two gps locations
        Serial.println("firstInnerAngle");
        Serial.println(firstInnerAngle);

        secondInnerAngle = calcSecondInnerAngle(secondAngle, boatBearing);//function that calculates the Second inner angle with respect to the bearing and line between two gps locations
        Serial.println("secondInnerAngle");
        Serial.println(secondInnerAngle);
        approxDistanceXbee = lawofSinesApproxDistCalc(firstInnerAngle,secondInnerAngle,distanceMeters); // calculates approx. distance in meters Xbee is from second gps location.
        Serial.println("approxDistanceXbee");
        Serial.println(approxDistanceXbee);
        Distance = approxDistanceXbee*1/1000; // converts from meters to km
         Serial.println("Distance");
        Serial.println(Distance);
        GpslatLonofDesiredLocation(GpsLat2, GpsLon2, secondAngle, Distance); //**** could have issues with lat lon not being the lat lon of the boat. that should be okay. we want gps of xbee
         Serial.print("GpsLat3");
         Serial.print(GpsLat3,6);
         Serial.print(",");
         Serial.print(GpsLon3,6);
         Serial.println("GpsLon3");
        currentApproxBearing = findMyBearing(CurLat,CurLon,GpsLat3,GpsLon3);//approxBearingofBoatToXbee(CurLat,CurLon,GpsLat3,GpsLon3); // this function also prints value to the LCD screen.
        Serial.println("currentApproxBearing");
        Serial.println(currentApproxBearing);
        currentApproxDistance = calc_dist(CurLat,CurLon,GpsLat3,GpsLon3); //this gives the approx distance to the xbee
        Serial.println("currentApproxDistance");
        Serial.println(currentApproxDistance);
       }
      }

     }
    }


    //convertAngle converts the max array angle into a bearing with respect to the compass.
    float convertAngle(float Angle){
      return Angle;//*****ratio of angles or might have to use mapping function.
      //***** might need two compasses in order to coordinate bearing of rssi direction and bearing of boat direction.
    }
    float convertSecondAngle(float Angle2){
      return Angle2; //******ratio of angles or might have to use mapping function.
      //***** might need two compasses in order to coordinate bearing of rssi direction and bearing of boat direction.
    }

    float calcFirstInnerAngle(float angleFirst, float bearing){
            Serial.println("Inside function calcFirstinnerAngle");
            Serial.println("firstangle,SecondAngle,boatbearing");
            Serial.println(angleFirst);
            Serial.println(secondAngle);
            Serial.println(bearing);
       if ( firstAngle < 180 && secondAngle < 180 && bearing < 180){
              Serial.println("InfirstInnerAngle If Statement");
             firstInnerAngle = abs(bearing - angleFirst);

             }
        else if ( firstAngle < 90 && secondAngle < 270 && bearing < 90){
             firstInnerAngle =  abs(bearing - angleFirst);
          }
        else if ( firstAngle > 270 && secondAngle > 270 && bearing < 90){
             firstInnerAngle = (bearing +(360 - angleFirst));
          }
        else if ( firstAngle < 90 && secondAngle < 95 && bearing > 270){
             firstInnerAngle = (angleFirst +(360 - bearing ));
          }
        else if ( firstAngle > 270 && secondAngle < 95 && bearing > 270){
             firstInnerAngle =  abs(bearing - angleFirst);
          }
        else if ( firstAngle > 270 && secondAngle > 270 && bearing > 270){
             firstInnerAngle =  abs(bearing - angleFirst);
          }
        else{
           firstInnerAngle =  abs(bearing - angleFirst);
          }
        return firstInnerAngle;
      }


    float calcSecondInnerAngle(float angleSecond,float bearing){
            Serial.println("firstangle,SecondAngle,boatbearing");
            Serial.println(angleSecond);
            Serial.println(secondAngle);
            Serial.println(bearing);
        if ( firstAngle < 95){
          if(secondAngle < 180){
              if(bearing < 180){
                 Serial.println("InSecond InnerAngle If Statement");
                 secondInnerAngle = 180.0 - 10.0*abs(bearing - angleSecond)/10.0;
                 Serial.println("abs(bearing - angleSecond)");
                 Serial.println(abs(bearing - angleSecond));
              }
          }
             }
        else if ( firstAngle > 180 && secondAngle < 270 && bearing < 90){ // what happens here when you have 1xt angle 140 second angle 140 and bearing 40.
            secondInnerAngle = 180 - (bearing +(360-angleSecond));
          }
        else if ( firstAngle > 270 && secondAngle > 270 && bearing < 90){
            secondInnerAngle = 180 - (bearing +(360-angleSecond));
          }
        else if ( firstAngle < 90 && secondAngle < 95 && bearing > 270){
            secondInnerAngle = 180 - (angleSecond + (360-bearing));
          }
        else if ( firstAngle > 270 && secondAngle < 95 && bearing > 270){
            secondInnerAngle = 180 - (angleSecond + (360-bearing));
          }
        else  if ( firstAngle > 270 && secondAngle > 270 && bearing > 270){
            secondInnerAngle = 180 - abs(bearing- angleSecond);
          }
        else {
             secondInnerAngle = 180-abs(bearing- angleSecond);
          }
        return secondInnerAngle;
      }


    float lawofSinesApproxDistCalc(float firstInAngle,float secondInAngle,float distMeters){// use law of sines to calculate distance from second Gps to appox. Xbee location.
          float thirdInAngle = 180 - (firstInAngle+secondInAngle);
                Serial.println("thirdInAngle");
                Serial.println(thirdInAngle);
          float ratioDist = distMeters/sin(radians(thirdInAngle));
                Serial.println("ratioDist");
                Serial.println(ratioDist);
         float distSecondGps = ratioDist*sin(radians(firstInAngle));

               Serial.println("distSecondGps");
               Serial.println(distSecondGps);
         return distSecondGps;
    }



//Function to tell us the bearing of the approximate xbee Gps location
float approxBearingofBoatToXbee(float GpsLa1,float GpsLo1,float GpsLa2,float GpsLo2){
      float approxBearing = findMyBearing(GpsLa1,GpsLo1, GpsLa2, GpsLo2); //calculates the approx. bearing
//        lcd.setCursor(0,1);
//
//        lcd.print("curApproxBearing: ");  lcd.print(approxBearing);
        return approxBearing;
}





//**********************************************************************************




//XBEE SCAN CODE FUNCTIONS
void readRSSI(){

  xbee.send(racrq);
  //delay(50);
  xbee.readPacket();
     //Serial.println("Hi You"); //Debug
  if (xbee.getResponse().isAvailable()) {
            //Debugging
        //               Serial.println("");
        //               Serial.println("AVAILABLE");
        //               Serial.println(xbee.getResponse().getApiId());
        //               Serial.println("REMOTE_AT_COMMAND_RESPONSE");
        //               Serial.print(REMOTE_AT_COMMAND_RESPONSE);
        //               Serial.println("");

        if (xbee.getResponse().getApiId() == REMOTE_AT_COMMAND_RESPONSE) {
         // xbee.getResponse().getZBRxResponse(rx);
          xbee.getResponse().getRemoteAtCommandResponse(racr);
          retVal = xbee.getResponse().getFrameData();
          retSize = xbee.getResponse().getPacketLength();


      // debugging

        //      for (int i = 0; i < retSize; i++) {
        //            Serial.print(" ");
        //            Serial.print(retVal[i],HEX);
        //
        //
        //         }
          rssi = retVal[14];
          rssi = map(rssi, 0, 100, 100, 0);

          //Debugging
                        Serial.println("RSSI");
                        Serial.println(rssi);
                        //Serial.println(millis());
          }


    }



    else if (xbee.getResponse().isError()) {
           //nss.print("Error reading packet.  Error code: ");
           //nss.println(xbee.getResponse().getErrorCode());
          }

  //back to code
}


void setPointValue(){  //*********** zero has to be changed the the calibrMotor location that is found.
  //setPointTarget = 10;
  // create a setpoint that changes once the setpoint is reached.
  if( cartCurrent[0] >= setPointTarget-10){ //gets within 2 degrees of Setpoint
      PreValue = 1; // increments setValue every time the motor changes direction

      setPointTarget = zero;
      //Serial.println("into the if statement");
//    if (before == true){
//      before = false;
//      setPointTarget = 10;
//
//      //break // hopefully gets me out of if statement so I dont trigger next if statement.
    }
  if (cartCurrent[0] <= zero+10 ){
     PreValue1 = 1; // increments setValue every time the motor changes direction // this helps to change from scan to shortscan //This is incrementing to rapidly.

      setPointTarget = setPointTargetValue1;//*********** need to create this variable

    }

   if (PreValue == 1 & PreValue1 == 1){
    Serial.println("setPointValue");
    Serial.println("setValue");
    Serial.println(setValue);

       setValue = setValue+1;
       PreValue = 0;
       PreValue1 = 0;
      }
  }

void setPointValue2(){
  Serial.println("SetPointValue2 has been accessed");
  //This is used for the shortScan Function in order to scan around the rssi degree angle.
  if( cartCurrent[0] >= setPointTarget-10){ //gets within 2 degrees of Setpoint
    PreValue = 1; // increments setValue every time the motor changes direction // this helps to change from scan to shortscan

      setPointTarget = setPointTargetValue - 40; //**** need to switch the rssi value with the highest value from the scan()
    }
  if (cartCurrent[0] <= setPointTarget+10 ){
      PreValue1 = 1; // increments setValue every time the motor changes direction // this helps to change from scan to shortscan

      setPointTarget = setPointTargetValue + 40; //**** need to switch the rssi value with the highest value from the scan()

    }
    if (PreValue == 1 & PreValue1 == 1){
       setValue = setValue+1;
       PreValue = 0;
       PreValue1 = 0;
      }
}




  void readEncoders(){

  //Read Encoder and calculate time
  //gain* encoder where gain = 360/ 1 revolution of encoder counts
  //cartCurrent[0] = (.147*myEnc.read()); // gives value in degrees of where system is located for 437rpm motor
 // cartCurrent[0] = (0.002667*myEnc.read()); // gives value in degrees of where system is located for 12rpm motor using OD(outerdiameters)
 cartCurrent[0] = (0.002476*myEnc.read()); // gives value in degrees of where system is located for 12rpm motor using PD(Pitch diameters)//Best***
  currentPos = cartCurrent[0];
 Serial.println("Encoders");
 Serial.println(currentPos);
 Serial.println(myEnc.read());
// NowAngle = currentPos;
// shortScanNowAngle = currentPos;
 //currentPos = cartCurrent[0];
 NowAngle = headingDegrees;
 shortScanNowAngle = headingDegrees;
  return;

}

void controlpid(){

   //How long since we last calculated
   unsigned long timeNow = millis();
   double deltaTime = (double)(timeNow - lastTimePid);

   //Compute all the working error variables
   double error = setPointTarget - currentPos;
   errSum += (error * deltaTime);

   double dErr = (error - lastErr) / deltaTime;

   /*Compute PID Output*/
   pidControlSignal = kp * error + ki * errSum + kd * dErr;

   //Remember some variables for next time
   lastErr = error;
   lastTimePid = timeNow;

  /*

  */
//   Serial.print(" Kp:");
//  Serial.print(kp*error);
//  Serial.print(" Ki:");
//  Serial.print(ki*errSum);
//  Serial.print(" Kd:");
//  Serial.println(kd*dErr);
  return;
}

void moveWheels(){

  if(pidControlSignal>=0){
    currentDir = LOW;
  }
  if(pidControlSignal<0){
    currentDir = HIGH;
  }
  digitalWrite(in1, !currentDir);
  digitalWrite(in2, currentDir);
//  digitalWrite(in1, currentDir);       //move counterclockwise
//  digitalWrite(in2, !currentDir);

  if(abs(pidControlSignal) > 120){
    pidControlSignal = 120;
  }
  //Serial.println(pidControlSignal);
  analogWrite(enA, abs(pidControlSignal));
//  analogWrite(enB, abs(pidControlSignal));

//  unsigned long currentMillis = millis();
//
//
//  if(currentMillis - previousMillis > interval) {
//    // save the last time you blinked the LED
//    previousMillis = currentMillis;
//     analogWrite(enA, 0);
//     analogWrite(enB, 0);
//     delay(100);
//  }

  return;
}


  void  scan(){



  //****Scan from 130-180 degrees
  //tells motor to move the correct amount
  //Gathers the data into five different locations in the array.
//check to see if the data gathered is higher than the values in the array

//think about using Gps clock to help gather data when out on the water.
//this will help with trouble shooting.


  //readRSSI(); //reads the RSSI value while scanning  //Might have trouble reading both data sets
  NowRssi = rssi; // sets NowRssi to rssi value
  NowAngle = NowAngle; // gets NowAngle from Encoder reading of angle
for (int i = 0; i < 5; i++){  //for loop to compare all past values of Rssi to the new value //***** Chris saw that the values will always start with the hightest number.
                              ///***** For loop is getting each value to the same number because when the first number is the same it goes to the next number in the array.
  if (NowRssi > ScanArrayRssi[i]){ //Makes sure that the new Rssi is bigger than a scan arrayRssi before switching locations
        ScanArrayRssi[i]  = NowRssi; //switches higher value Rssi with for the older value
        ScanAngleArray[i] = NowAngle;
        break;
      }

//      else if (NowRssi == ScanArrayRssi[i]){
//        EqualRssi = EqualRssi + 1;
//
//      }

}

//debugging print out ScanArrayRssi
            Serial.println("Rssi then EncoderAngle");
            Serial.println(NowRssi);
            Serial.println(NowAngle);
          Serial.println("ScanArray");
          for (int i = 0; i < 5; i++){
                Serial.println(ScanArrayRssi[i]);

             }
//             //debugging print out ScanArrayRssi
            Serial.println("ScanAngleArray");
            Serial.println(ScanAngleArray[0]);
          for (int i = 0; i < 5; i++){
                Serial.println(ScanAngleArray[i]);

             }

//back to code
 // maxArrayValue  = getIndexOfMaximumValue(ScanArrayRssi,5); //the function to get highest value isnt working.
            maxArrayValue  = ScanArrayRssi[0];
            maxArrayValueAngle = ScanAngleArray[0];
//Debugging maxArrayValue

            Serial.println("maxArrayValue");
            Serial.println(maxArrayValue);

//back to code
  //if anything has gotten higher values then the array will change.
  //might want to use this array to see if it is the same and we are getting only low readings

//  for (int i = 0; i <LenArray ; i++){//Get the max Rssi value and the angle for that value
//        if (maxArrayValue == ScanArrayRssi[i]){
//            maxArrayValueAngle = ScanAngleArray[i];
//                                                  //Serial.print("Found Max array Value")
//           }
//     }

                  //maxArrayValue = maximum(ScanArrayRssi,5); //get the maximum Rssi value

 // Debugging
//            Serial.println("maxArrayValue"); //Find out what are the largest Rssi and Angle values
//            Serial.println(maxArrayValue);
//            Serial.println("maxArrayValueAngle");
//            Serial.println(maxArrayValueAngle);


//Back to code
// Now we have the highest RSSI value and the corresponding angle
//*** Send this data to compass and have an indicator on the LCD screen
//*** compare this data with Compass to help guide vehicle
  if (maxArrayValue < 20 || maxArrayValue > 90 ){// values are percentages out of 100 and will show that there is too much or too little signal to use effectively

          maxArrayValue = 0; // This will make shortScan not work if it is accessed
     }

}



void shortScan(){

  if (maxArrayValue < 20 || maxArrayValue > 90 ){// values are percentages out of 100 and will show that there is too much or too little signal to use effectively
    Serial.println("RESET IF STATEMENT INITIATED");
    Serial.println("maxArrayValue");
    Serial.println(maxArrayValue);

      setValue = 0; // this resets to the loop to the scan sequence
                // if maxArrayValue is == 0 then the value is not arround and we need to do the Scan function
      maxArrayValue = 0;
}
else if(20<maxArrayValue<90){
    // sets the location of the motor around the correct angle


  // run else, if max array value is in between the values above.

 //***move motor 30 degrees on either side of the MaxArrayValue

  //***scan Rssi Values as antennea moves

     //going to start again with an empty arrays.

  //need to account for when we dont see any data.


// setting up variables with values to be used to fill arrays.
shortScanNowRssi = rssi;
shortScanNowAngle = NowAngle;

//think about using Gps clock to help gather data when out on the water.
//this will help with trouble shooting.
 for (int i = 0;i < LenArray; i++){//for loop to compare all past values of Rssi to the new value

   if (shortScanNowRssi > shortScanArrayRssi[i]){ //Makes sure that the new Rssi is bigger than a scan arrayRssi before switching locations
       shortScanArrayRssi[i] = shortScanNowRssi; //switches higher value Rssi with for the older value
       shortScanAngleArray[i] = shortScanNowAngle;

       //Debugging
       Serial.println("Entered Shortscan if statement");

       break;
      }
//   else if (shortScanNowRssi == shortScanArrayRssi[i]){
//          shortScanEqualRssi = shortScanEqualRssi + 1;
//
//      }

  }

 //Debugging
   Serial.println("shortScanArrayRssi");
          for (int i = 0; i < 5; i++){
                Serial.println(shortScanArrayRssi[i]);

             }
//             //debugging print out ScanArrayRssi
                Serial.println("shortScanAngleArray");

          for (int i = 0; i < 5; i++){
                Serial.println(shortScanAngleArray[i]);

             }

   if (setValue > 5){
     maxArrayValue  = shortScanArrayRssi[0];//getIndexOfMaximumValue(shortScanArrayRssi,5);
   }

  //if anything has gotten higher values then the array will change.
  //might want to use this array to see if it is the same and we are getting only low readings
 for (int i = 0 ;i < LenArray; i++){//Get the max Rssi value and the angle for that value
     if (maxArrayValue == shortScanArrayRssi[i]){
         maxArrayValueAngle = shortScanAngleArray[i]; //needs to be the same variable so the motor starts on the same position
                                               //Serial.print("Found Max array Value")
       }
   }
     // maxArrayValue = maximum(shortScanArrayRssi,5); //get the maximum Rssi value and keep same variable as scan funtion


     // Now we have the highest RSSI value and the corresponding angle
     // *** Send this data to compass and have an indicator on the LCD screen
     // *** compare this data with Compass to help guide vehicle
   }
}

void AngleAverage(){
              AddedAngles = 0;
         for (int i = 0 ;i < LenArray; i++){//Get the max Rssi value and the angle for that value
                AddedAngles = ScanAngleArray[i] + AddedAngles;
            }
         AveragedAngle = AddedAngles/5;

         Serial.println("AveragedAngle");
         Serial.println(AveragedAngle);

}


void CompassRun(){
  //*** read compass direction from the IMU
  // Serial.print(headingDegrees);

  //*** read the angle on the highest Rssi signal
  // maxArrayValueAngle
  //*** calculate equivalent direction of source
  // map(beginValueMotorCal,FinalValueMotorCal,+100degreesdirectionofboat,-100degreeDirectionofBoat)
  //*** compare the angle to the compass direction


  //*** PrintOut compass direction of boat and of source on the LCD screen
}

void calibMotor(){


  //*** make motor turn until it hits the boundary/marker
  // int calValue = analogRead(A0);
  //if (calValue >= ***);{
  //   int MotorZero = }
  //*** then set this as the zero position(degree zero)

  //***

}
