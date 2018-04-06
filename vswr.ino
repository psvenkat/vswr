// VSWR Bridge using SI5351. This is an extension of the work done by
// several people. Sincere thanks to all those people.
// Antenna VSWR analyser by dr_phil
// finishing the Arduino job started by K6BEZ (see below)
// Arduinos are now (in 2016) much cheaper than when he published his work
//
// My thanks to all authors credited below for making this work possible
// I've either hacked their code or used it verbatim
// UPDATE Feb 27, 2018
// Use Si5251 - barely fit in the 32K space of Nano - just 4 bytes free!
// Added a arbitrary freq selection
// Added bar display for forward and revers voltage
// Moved lot of static data to PGM space to free up RAM

// UPDATE 24/8/2017
// Added support for 160 m and 60 m bands

// UPDATE 28/4/2017
// Graphing - display a simple freq versus VSWR plot for single-band sweeps, with autoscaling and minimum VSWR/frequency indication

// UPDATE 21/4/2017
// Discovered that this unit was under-estimating VSWR owing to the low DDS output level
// relative to the detector diode forward voltages - this is widely reported by others who have
// built similar devices
// Fix is to use uncorrected sketch on device to show VSWR at midpoint of each band for resistive loads
// of 50ohm, 100 ohm, 150 ohm and 200 ohm. Then plot recorded values as function of frequency and
// 'actual' VSWR (i.e. 1,2,3 & 4 for the loads listed). Use logarithmic curve fit to determine correction function
// required at each frequency (it's different for each, as DDS output rolls off with increasing frequency). Then
// apply this correction function in the measurement subroutine
// Also, a minimum measurement time has been built in, as the detector circuits need time to charge
// capacitances - from LTSPICE 50 ms per measurement seems to do it.


/***************************************************************************\
*  Name    : DDS_Sweeper.BAS                                                *
*  Author  : Beric Dunn (K6BEZ)                                             *
*  Notice  : Copyright (c) 2013  CC-BY-SA                                   *
*          : Creative Commons Attribution-ShareAlike 3.0 Unported License   *
*  Date    : 9/26/2013                                                      *
*  Version : 1.0                                                            *
*  Notes   : Written using for the Arduino Micro                            *
*          :   Pins:                                                        *
*          :    A0 - Reverse Detector Analog in                             *
*          :    A1 - Forward Detector Analog in                             *
\***************************************************************************/

/*
  Universal 8bit Graphics Library, https://github.com/olikraus/u8glib/
  
  Copyright (c) 2012, olikraus@gmail.com
  All rights reserved.
*/

/* ******Interrupt-based Rotary Encoder Menu Sketch*******
 * by Simon Merrett, based on insight from Oleg Mazurov, Nick Gammon, rt and Steve Spence, and code from Nick Gammon
 * 3,638 bytes with debugging on UNO, 1,604 bytes without debugging
 */
// NOTE by dr_phil: you don't need the cli & sei statements in the ISRs - in this sketch they cause problems
// Also seems to need an encoder with two detents per pulse - doesn't work well with ky-040

// NOTE 2 by dr_phil: having successfully used interrupts on a DC motor encoder, 
// I reckon the encoder handling could be improved (with single click cheap encoders);
// unfortunately it's not a priority at the moment as the existing system works - watch this space...

// note that the following screen library is no longer supported, but it still works and I
// couldn't get its successor to work in a sufficiently short length of time on the Nano

#include "U8glib.h"

// need maths library for logarithmic calibration curve

#include "math.h"

#include <Wire.h>
#include <si5351.h>
si5351 si = si5351();

// setup u8g object for cheapo 128x64 OLED screen with i2c from Ebay
// choose the right line for your display from Oli's demo code

U8GLIB_SH1106_128X64 u8g(13, 11, 10, 9);  // SW SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9

// Rotary encoder declarations
static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
volatile byte dir = 0;    //encoder direction
// Button reading, including debounce without delay function declarations
const byte buttonPin = 8; // this is the Arduino pin we are connecting the push button to
byte oldButtonState = HIGH;  // assume switch open because of pull-up resistor
const unsigned long debounceTime = 10;  // milliseconds
unsigned long buttonPressTime;  // when the switch last changed state
boolean buttonPressed = 0; // a flag variable
boolean sweepAbort = LOW;

// Menu and submenu/setting declarations
byte Mode = 0;   // This is which menu mode we are in at any given time (top level or one of the submenus)
const byte modeMax = 3; // This is the number of submenus/settings you want
byte setting1 = 0;  // a variable which holds the value we set 
byte setting2 = 0;  // a variable which holds the value we set 
byte setting3 = 0;  // a variable which holds the value we set 
/* Note: you may wish to change settingN etc to int, float or boolean to suit your application. 
 Remember to change "void setAdmin(byte name,*BYTE* setting)" to match and probably add some 
 "modeMax"-type overflow code in the "if(Mode == N && buttonPressed)" section*/

double    Fstart_MHz = 1;       // Start Frequency for sweep
double    Fstop_MHz = 10;       // Stop Frequency for sweep
double    current_freq_MHz;     // Temp variable used during sweep
int       band_choice = 0;      // doesn't need to be zeroed as this happens during initial sweep - used for single band sweeps and graph plotting
double    freq_MHz_min_VSWR;    // stores the frequency of min swr during a sweep
double    min_VSWR;             // stores minimum VSWR found during sweep
double    FWD=0;
double    REV=0;
long      serial_input_number;  // Used to build number from serial stream
int       num_steps = 100;      // Number of steps to use in the sweep
char      incoming_char;        // Character read from serial stream
char      band[6];              // Character holding current band in metres
char      minfreq[8];
char      maxfreq[8];

boolean   use_calibration=HIGH; // apply logarithmic correction coefficients from resistor measurements

int       menuChoice=0;

const byte MULT = 36;       //Max PLL= 900Mhz
const long MAXFREQ = 225000000;
const int NDIGITS = 9; 
const int NBANDS = 10;
const int CUSTOM = 10; 

const long hfbands [10][4]PROGMEM ={
  {160,1810000,2000000,64},
  {80,3500000,3800000,100}, // wavelength,Fstart,Fstop,nsteps (nsteps chosen for ~3 kHz intervals)
  {60,5258500,5406500,50},
  {40,7000000,7200000,67},
  {30,10100000,10150000,17},
  {20,14000000,14350000,117},
  {17,18068000,18168000,33},
  {15,21000000,21450000,150},
  {12,24890000,24990000,33},
  {10,28000000,29700000,150}  // nsteps for this one should really be 567, but that would take ages!
};

int swr_results [11][4]={
  
  //SWRmin,Freq(SWRmin)in MHz,correction gradient (*ln(X)), correction intercept - all numberx x1000
  // separate array from hfbands
  // because this stores floats
  // changed all numbers to int and x1000 to save 88 bytes
  
  {0,0,3500,660},  // correction needs to be measured for 160 m
  {0,0,3548,623},
  {0,0,3575,590},  // ditto for 60 m 
  {0,0,3609,570},                        
  {0,0,3696,500},                        
  {0,0,3758,413},
  {0,0,3767,257},
  {0,0,3804,187},
  {0,0,3818,118},
  {0,0,3754,50},
  {0,0,0,0}        // this last slot is for arbitrary sweeps
};
  
unsigned int moving_avg_swr [9];        // nine-point moving average for VSWR (x1000) to save space

int graph_array [150];          // storage array for single graph - try storing vswr values as ints (after multiplying by 100) to save memory

char minvswr[6];
char minfreqdisp[16];
  
// the setup routine runs once on reset:
void setup() {

  /* Initialise the sensor */
  Wire.begin();
  si.begin();
  Serial.println(F("OK"));
  si.set_xtal(25000000, 0);                   // XTAL = 27.000 MHz with 0ppm correction
  si.pll_config(PLL_A, MULT, 0, 1);           // PLL_A in Int mode
  //si.pll_integer_config(PLL_A, MULT);       // PLL_A = MULT * XTAL = 900 MHz
  si.clk_config(CLK0, (SRC_PLL << CLKx_SRC)   // CLK0 powerd up, fractional mode, MultiSynth0, 
    | (IDRV_8 << CLKx_IDRV));                 //  PLL A, not inverted, 8mA drive
  si.set_phase(CLK0, 0);                      // no phase offset for CLK0
  si.clk_enable(CLK0);                        // enable CLK0 output
  
  // Configure LED pin for digital output
  pinMode(13,OUTPUT);

  //Rotary encoder section of setup
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0,PinA,RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1,PinB,RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  
  // Rotary encoder button section of setup
  pinMode (buttonPin, INPUT_PULLUP); // setup the button pin

  // Set up analog inputs on A0 and A1, internal reference voltage
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  analogReference(INTERNAL);
  
  // initialize serial communication at 57600 baud
  Serial.begin(9600);

  //Initialise the incoming serial number to zero
  serial_input_number=0;

  // startup screen
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_9x15B);
      u8g.drawStr(5,15,F(" HF VSWR 2.0"));
      //u8g.drawStr(0,30,F(" analyser v7"));
      //u8g.setFont(u8g_font_7x13);
      //u8g.drawStr(0,50,"    by dr_phil");
      // Sorry no space for the above displays!
      if(use_calibration==LOW) u8g.drawStr(0,63,F("Uncalibrated VSWR!"));
    } while(u8g.nextPage() );

    Serial.println(F("HF VSWR analyser"));

      SetDDSFreq(7000000);  // start the DDS working at 7 MHz

      delay(1000);  // let the DDS settle and allow time to read splash screen

      // then do a couple of dummy reads on the ADC to flush out the known problem with the 
      //  first reading after changing the reference
      analogRead(A0);
      delay(100);
      analogRead(A1);
      delay(100);
      
      // On power-up, sweep Amateur HF bands 80m to 10m
      // as it makes device useful without controls
      // just press reset on Nano to restart measurement and clear minimum values
      
      Sweep_bands();
      Sweep_report();

      do {  // wait for button press before clearing report and resuming script
      } while(buttonCycled() == LOW);

}

void loop() {
  
  cli();  // disable interrupts to prevent volatile variables being changed mid-operation
  if(oldEncPos!=encoderPos){
    if(encoderPos>200){
      encoderPos=0;
    }
    if(encoderPos>2){
      encoderPos=2;
    }

    oldEncPos=encoderPos;
   }
   sei();  // switch them back on again

      // display menu with a 'picture loop'
      u8g.firstPage();
      do {
        u8g.setFont(u8g_font_7x13);
        u8g.drawStr(4,15,F("*** Main Menu ***"));
        u8g.drawStr(0,30,F("  Sweep all bands"));
        u8g.drawStr(0,45,F("  Sweep one band"));
        u8g.drawStr(0,60,F("  Single frequency"));
        u8g.drawStr(0,30+15*encoderPos,">");
        } while(u8g.nextPage() );
    
   if(buttonCycled() == HIGH){
     switch (encoderPos){
       case 0:  // sweep all HF bands and display summary
       
       Sweep_bands();
       Sweep_report();
       
       do {  // wait for button press before clearing report and return to menu
       } while(buttonCycled() == LOW);
       
       encoderPos=0;
       break;
       
       case 1:  // sweep single HF band
       
       band_choice=Choose_band();
       Serial.print(hfbands[band_choice][0]);
       Serial.print(pgm_read_dword(&(hfbands[band_choice][0])));
       Serial.println(F("m"));

       Fstart_MHz = ((double)pgm_read_dword(&(hfbands[band_choice][1])))/1000000;
       Fstop_MHz =  ((double)pgm_read_dword(&(hfbands[band_choice][2])))/1000000;
       num_steps = pgm_read_dword(&(hfbands[band_choice][3]));
       Perform_sweep(band_choice);

       do {  // wait for button press before clearing report and return to menu
       } while(buttonCycled() == LOW);

       // now need to generate graph
       plot_graph(band_choice);
       
       do {  // wait for button press before clearing report and return to menu
       } while(buttonCycled() == LOW);
       
       encoderPos=1;
       break;
       
       case 2:  // live VSWR at adjustable spot frequency

       band_choice=Choose_band();
       current_freq_MHz = double(pgm_read_dword(&(hfbands[band_choice][1])) 
          + pgm_read_dword(&(hfbands[band_choice][2])))/2000000;

       Spot_freq();
       
       encoderPos=2;
       break;
       

       }
   }

}

int Choose_band(){
  
  encoderPos=band_choice;  // remember last band choice, rather than having to twiddle to correct band every time
  
  do {  // loop which repeats until button is pressed then released
       
           cli();  // disable interrupts to prevent volatile variables being changed mid-operation
           if(oldEncPos!=encoderPos){
            if(encoderPos>200){
              encoderPos=0;
            }
            if(encoderPos>9){  // was 7 before additional two bands added
              encoderPos=9;
            }
    
            oldEncPos=encoderPos;
           }
           sei();  // switch them back on again
       
           
           itoa(pgm_read_dword(&(hfbands [encoderPos][0])),band,10);
           dtostrf(double(pgm_read_dword(&(hfbands[encoderPos][1])))/1000000,6,3,minfreq);
           dtostrf(double(pgm_read_dword(&(hfbands[encoderPos][2])))/1000000,6,3,maxfreq);
              
           u8g.firstPage();
           do {
              u8g.setFont(u8g_font_7x13);
              u8g.drawStr(4,15,F("** Choose Band **"));
              u8g.drawStr(0,30,F("   >    m band"));
              u8g.drawStr(35,30,band);
              u8g.drawStr(0,45,F("  Min:       MHz"));
              u8g.drawStr(0,60,F("  Max:       MHz"));
              u8g.drawStr(45,45,minfreq);
              u8g.drawStr(45,60,maxfreq);

           } while(u8g.nextPage() );
           
        } while(buttonCycled() == LOW);
  
  return(encoderPos);
}

void Sweep_bands(){

  for(int i=0;i<NBANDS;i++){  
        Serial.print(pgm_read_dword(&(hfbands[i][0])));
        Serial.println(F("m"));
        Fstart_MHz = ((double)pgm_read_dword(&(hfbands[i][1])))/1000000;
        Fstop_MHz = ((double)pgm_read_dword(&(hfbands[i][2])))/1000000;
        num_steps = pgm_read_dword(&(hfbands[i][3]));
        
        band_choice=i;  // needed to ensure correct calibration factors are applied for band
        
        // need to pass current minimum VSWR & freq to sweep routine
        // easiest to use array subscript
        Perform_sweep(i);
        
        // checkButton();  // test for abort (i.e. button pressed)
        if(sweepAbort==HIGH){
          sweepAbort=LOW;
          return;
        }
        
        }
}

void Perform_sweep(int j){
  char freqdisp[16];
  char vswrdisp[16];
  char vswrchar[6];
  char minvswr[6];
  char minfreqdisp[16];
  int avg_ctr=0;  // counter for moving average (over frequency) array

  double AVG_VSWR;
  double Fstep_MHz = (Fstop_MHz-Fstart_MHz)/num_steps;
  
  swr_results[j][0]=0;  // clear stored minimum for this band choice
  swr_results[j][1]=0;
 
  memset(graph_array,0,sizeof(graph_array));  // zero graphng array

  // Start loop 
  for(int i=0;i<=(num_steps+8);i++){  // pad loop out with additional eight steps for 9 pt moving average

  if(buttonCycled()==HIGH){
    sweepAbort=HIGH;
    return;
  }
        
    // Calculate current frequency
    current_freq_MHz = Fstart_MHz + (i-4)*Fstep_MHz;  // four additional steps prior to Fstart, owing to 9 pt moving average
    // Set DDS to current frequency
    SetDDSFreq(current_freq_MHz*1000000);
    // Wait a little for settling - needs to be at least 50 ms (see update info at start)
    delay(50+100*(i<1));

    moving_avg_swr[avg_ctr]=Get_VSWR()*1000; // load value into next available slot in averaging array
    
    // calculate average of array - crude but effective...
    AVG_VSWR=(moving_avg_swr[0]+moving_avg_swr[1]+moving_avg_swr[2]
      +moving_avg_swr[3]+moving_avg_swr[4]+moving_avg_swr[5]+moving_avg_swr[6]
      +moving_avg_swr[7]+moving_avg_swr[8])/9000;
 
    // Send current line back to PC over serial bus - this may cause problems for interrupts but 
    //    there's no reason to be using these to read encoder during a sweep
    
    if(i>=8){    // VSWR values only valid after 8 measurements have been processed
 
    // load value into graph array if it doesn't overflow
    if(i<158){
      graph_array[i-8]=int(AVG_VSWR*100);
    }
    
    Serial.print(long((current_freq_MHz-4*Fstep_MHz)*1000000)); // remember that the frequency corresponding to current avg_VSWR was four iterations ago
    Serial.print(F(","));
    Serial.println(AVG_VSWR);
      
      // test for minimum VSWR - store and notify if new minimum found
      if((AVG_VSWR<(double)swr_results[j][0]/1000)||(swr_results[j][0]==0)){
         swr_results[j][0]=AVG_VSWR*1000;
         swr_results[j][1]=(current_freq_MHz-4*Fstep_MHz)*1000;  // remember to subtract 4 bin offset for 9 pt moving average
        }
      // test for minimum VSWR over any sweep - note sure if row [10] (was 8 before adding bands) of this array is actually used any more - legacy code?
      if((AVG_VSWR<(double)swr_results[10][0]/1000)||(swr_results[10][0]==0)){
        swr_results[10][0]=AVG_VSWR*1000;
        swr_results[10][1]=(current_freq_MHz-4*Fstep_MHz)*1000;  // remember to subtract 4 bin offset for 9 pt moving average
        }
    
    
      // now put it on the OLED screen
      // convert the numbers to strings...
      dtostrf(current_freq_MHz-4*Fstep_MHz,6,3,freqdisp);
      dtostrf(AVG_VSWR,5,2,vswrdisp);
      // min values over entire range / any bands
      dtostrf((float)swr_results[j][1]/1000,6,3,minfreqdisp);
      dtostrf((float)swr_results[j][0]/1000,5,2,minvswr);
      
      // these last two lines used to display [8] instead of [j] - not useful on a single band sweep
    
      // then display them with a 'picture loop'
      u8g.firstPage();
      do {
        u8g.setFont(u8g_font_7x13);
        u8g.drawStr(0,15,F("Sweeping frequency"));
        u8g.drawStr(0,30,F("Freq:       MHz"));
        u8g.drawStr(0,45,F("VSWR: ")); 
        u8g.drawStr(40,30,freqdisp);
        u8g.drawStr(40,45,vswrdisp);

        u8g.setFont(u8g_font_6x13);
        u8g.drawStr(0,60,F("Min       @       MHz"));
        u8g.drawStr(24,60,minvswr);
        u8g.drawStr(70,60,minfreqdisp);
      
        //u8g.drawStr(90,30,F("MHz"));
        } while(u8g.nextPage() );
    
      }
    
    avg_ctr++;                    // increment moving average array counter to fill next slot
    if(avg_ctr>8) avg_ctr=0;      // wrap back to zero if slot number exceeds 8 (=ninth slot of array)
      
  }
    
}

void Spot_freq(){
  int level=0;  // control 'level' of encoder w.r.t. frequency: 0=select digit; 1=select value of digit
  int digitsPos=0;
  int digitsVal;
  long fHz=(current_freq_MHz*1000000);
  long fHzOld=0;
  char freqdisp [NDIGITS+1];   //Was 9, now we can go to 200MHz
  char vswrdisp [6];
  uint32_t a, b, c, p1, p2;
  uint8_t frac;
  encoderPos=0;

  do{  // repeat this block of code until user exits subroutine
    
    cli();  // read encoder - disable interrupts to prevent volatile variables being changed mid-operation
    if(oldEncPos!=encoderPos){
      if(encoderPos>200){
        encoderPos=0;
      }
      if(encoderPos>9){  // sets maximum permitted encoder value according to 
        encoderPos=9;    // which level of adjustment you're in 
      }
      oldEncPos=encoderPos;
    }
    sei();  // switch them back on again
  
    if(level==0) digitsPos=encoderPos;
    
    if(level==1){
      
      //if(digitsVal<encoderPos && encoderPos<=9){
      if (dir == 2){
        dir = 0;
        fHz=fHz+shift(digitsPos);
        if(fHz>MAXFREQ) fHz=fHz-shift(digitsPos);  // test for out of range and undo change if it will cause this
        digitsVal=long(fHz/shift(digitsPos)) % 10;  // get new value of digitsVal from fHz - use modulo maths %
      }
      
      //if(digitsVal>encoderPos && encoderPos>=0){
      if (dir == 1){
        dir = 0;
        fHz=fHz-shift(digitsPos);
        if(fHz<1) fHz=fHz+shift(digitsPos);  // test for out of range and undo change if it will cause this
        digitsVal=long(fHz/shift(digitsPos)) % 10;  // use modulo maths %
      }
    }

    // Set DDS to current frequency
    if (fHzOld != fHz){

      SetDDSFreq(fHz);
      si.get_params(a, b, c, frac);

      Serial.print(F("X "));
      Serial.print(MULT);
      Serial.print(F(" a "));
      Serial.print(a);
      Serial.print(F(" b "));
      Serial.print(b);
      Serial.print(F(" c "));
      Serial.print(c);
      Serial.print(F(" F "));
      Serial.print(FWD);
      Serial.print(F(" R "));
      Serial.print(REV);
      Serial.print(F(" f ")); Serial.println(frac);
      fHzOld=fHz;
    }

    //dtostrf(fHz,NDIGITS,0,freqdisp);
    ltoa(fHz, freqdisp, 10);
    dtostrf(Get_VSWR(),5,2,vswrdisp);

    if(buttonCycled()==HIGH){
      if(digitsPos==NDIGITS){  // test for EXIT selected
        return;
      }
      if(level==0 && digitsPos<NDIGITS){
        level=1;

        digitsVal=long(fHz/shift(digitsPos)) % 10;  // use modulo maths %
        encoderPos=digitsVal;

      } else {
        level=0;
        encoderPos=digitsPos;
        
      };  // jump into digits adjustment or out again
    }

      // display current freq and cursor with a 'picture loop'
      u8g.firstPage();
      do {
        u8g.setFont(u8g_font_7x13);
        u8g.drawStr(0,15,F(" Select frequency"));
        u8g.drawStr(0,60,F("VSWR:         exit"));
        u8g.setFont(u8g_font_9x15B);
        u8g.drawStr(95,38,F("Hz"));
        //u8g.drawStr(10,38,freqdisp);
        u8g.drawStr(10+9*(NDIGITS-strlen(freqdisp)),38,freqdisp);
        u8g.drawStr(40,60,vswrdisp);
        u8g.drawLine(0,45,FWD/25, 45);
        u8g.drawLine(125-REV/25, 45, 125, 45);     
        if(level==0){
          if(digitsPos<NDIGITS){
          u8g.drawLine(9+9*digitsPos,40,18+9*digitsPos,40);
          } else {
          u8g.drawLine(97,62,125,62);
          }
        } else {
          u8g.drawBox(9+9*digitsPos,40,9,3);
          }
       } while(u8g.nextPage() );
    
       
  }while(HIGH);
}

long shift(int dpos){
  
  long shift=1;
  for(int i=1;i<=(NDIGITS -1 -dpos);i++){
    shift=shift*10;
  }
  Serial.print(F("shift: "));
  Serial.println(shift);
  return shift;
}

double Get_VSWR(){
  double FWD5=0;  // summing array for 5 point averaging at each frequency
  double REV5=0;
  double temp_VSWR;

  // Read the forward and reverse voltages - always returns nonsense on first iteration after reset 
  //    (even if I do multiple reads)
    FWD5=0;
    REV5=0;
    for(int k=0;k<5;k++){
      analogRead(A0);
      delay(10);
      REV5 = REV5 + analogRead(A0);
      analogRead(A1);
      delay(10);
      FWD5 = FWD5 + analogRead(A1);
    }
    FWD=FWD5/5;  //   carry out averaging calc
    REV=REV5/5;
    
    if(REV>=FWD){
      // To avoid a divide by zero or negative VSWR then set to 0
      temp_VSWR = 0;
    }else{
      // Calculate VSWR
      temp_VSWR = (FWD+REV)/(FWD-REV);
    }
    
    // apply correction if requested
    if(temp_VSWR>0 && use_calibration==HIGH){
      temp_VSWR=((double)swr_results[band_choice][2]/1000)*log(temp_VSWR)
        +((double)swr_results[band_choice][3]/1000);  // look up correction factors for band
      if(temp_VSWR<1)temp_VSWR=1;
    }
    
    return temp_VSWR;

}

void Sweep_report(){
  
  char minvswr[6];
  char minfreqdisp[16];
  
      // now put it on the OLED screen
      // convert the numbers to strings...
      // dtostrf(current_freq_MHz-4*Fstep_MHz,6,3,freqdisp);
      // dtostrf(AVG_VSWR,5,2,vswrdisp);
      // min values over entire range / any bands
      // dtostrf(swr_results[8][1],6,3,minfreqdisp);
      // dtostrf(swr_results[8][0],5,2,minvswr);
      // then display them with a 'picture loop'

      u8g.firstPage();
      do {
        u8g.setFont(u8g_font_04b_03br);
        u8g.drawStr(1,6,"160m:          @           MHz");
        u8g.drawStr(4,12,"80m:          @           MHz");
        u8g.drawStr(4,18,"60m:          @           MHz");
        u8g.drawStr(4,24,"40m:          @           MHz"); 
        u8g.drawStr(4,30,"30m:          @           MHz");
        u8g.drawStr(4,36,"20m:          @           MHz");
        u8g.drawStr(6,42,"17m:          @           MHz");
        u8g.drawStr(6,48,"15m:          @           MHz");
        u8g.drawStr(6,54,"12m:          @           MHz");
        u8g.drawStr(6,60,"10m:          @           MHz");
        
        // populate table with min VSWR values for each band
        
        for(int i=0;i<NBANDS;i++){
          dtostrf((float)swr_results[i][1]/1000,6,3,minfreqdisp);
          dtostrf((float)swr_results[i][0]/1000,5,2,minvswr); 
          u8g.drawStr(25,(6*(i+1)),minvswr);  // removed (i+1) and replaced with (i)
          u8g.drawStr(60,(6*(i+1)),minfreqdisp);  // ditto
        }
        
        } while(u8g.nextPage() );
        
}

void SetDDSFreq(long Freq_Hz){
  Serial.print(F("Setting freq - "));
  Serial.print(Freq_Hz);
  si.set_frequency(CLK0, Freq_Hz);
  Serial.println(F("  Done."));
}

void checkButton() {
  
  // Button reading with non-delay() debounce - thank you Nick Gammon!
  byte buttonState = digitalRead (buttonPin); 
  if (buttonState != oldButtonState){
    if (millis () - buttonPressTime >= debounceTime){ // debounce
      buttonPressTime = millis ();  // when we closed the switch 
      oldButtonState =  buttonState;  // remember for next time 
      if (buttonState == LOW){
        buttonPressed = 1;
      }
      else {
        buttonPressed = 0;  
      }  
    }  // end if debounce time up
  } // end of state change

}

boolean buttonCycled() {    // subroutine to check for a button press and release - if called when button is pressed, won't return till released   
   checkButton();
   if(buttonPressed == HIGH){
     do {     // wait for button to be released before starting
         checkButton();
       } while(buttonPressed == HIGH);
       return HIGH;
   } else
   { return LOW;
   }
}
   


//Rotary encoder interrupt service routine for one encoder pin
void PinA(){

  // cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
    dir = 1;
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
 // sei(); //restart interrupts

}

//Rotary encoder interrupt service routine for the other encoder pin
void PinB(){

  //  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
    dir = 2;
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
 // sei(); //restart interrupts

}

// graphing routine
void plot_graph(int bc){
  
  int x;  // points to plot on graph
  int y;  
  int xold;  // last plotted point - used for line drawing
  int yold;
  int maxVSWR=0;
  int ceilmaxVSWR=0;
  int meanVSWR=0;
  char MHZstart[7];
  char MHZstop[7];
  char ylabel[3];
  
  Fstart_MHz = ((double)pgm_read_dword(&(hfbands[bc][1])))/1000000;
  Fstop_MHz = ((double)pgm_read_dword(&(hfbands[bc][2])))/1000000;
  num_steps = pgm_read_dword(&(hfbands[bc][3]));
  dtostrf(Fstart_MHz,6,3,MHZstart);
  dtostrf(Fstop_MHz,6,3,MHZstop);  

  // find max and mean VSWR in dataset for band
  for(int i=0;i<num_steps;i++){
    if(maxVSWR<graph_array[i]) maxVSWR=graph_array[i];
    meanVSWR=meanVSWR+graph_array[i];
  }
  meanVSWR=meanVSWR/num_steps;

  ceilmaxVSWR=ceil(float(maxVSWR)/100);
  Serial.println(float(maxVSWR));
  Serial.println(float(maxVSWR)/100);
  Serial.println(ceilmaxVSWR);
  

  // display with a 'picture loop'
  u8g.firstPage();
  do {
    // generate axes and labels
    u8g.setFont(u8g_font_5x7);
    u8g.drawStr(14-5*(Fstart_MHz<10),63,MHZstart);    // this line not working - doesn't like variable. mem problem?
    u8g.drawStr(61,62,"F/MHz");
    u8g.drawStr(96,63,MHZstop);
    u8g.drawStr(6,57,"1");
    u8g.drawStr(0,20,"V");
    u8g.drawStr(0,28,"S");
    u8g.drawStr(0,36,"W");
    u8g.drawStr(0,44,"R");

    itoa(ceilmaxVSWR,ylabel,10);
    u8g.drawStr(6,7,ylabel);
        
    u8g.drawHLine(12,54,115);
    u8g.drawVLine(13,0,56);
    /*
    for(int i=1;i<6;i++){
      u8g.drawPixel(6,(10*i)-6);
    }  */

    
    // plot points
    
    for(int i=0;i<num_steps;i++){
      // map array values onto 100x54 grid available on OLED
      x=14+int(113*float(i)/float(num_steps));  // autoscale x
      if(graph_array[i]>=100){
        y=int(54*(1-((float(graph_array[i])/100)-1)/(ceilmaxVSWR-1)));  // autoscale y

      } else {
        y=54;  // effectively don't plot this point (hides it under axis)
      }
      
      //check for y overflow (corresponds to trying to plot off the top of the OLED), which shouldn't now happen with autoscaling
      if(y<0) y=0;
      
      if(i==0){  // first point has no meaningful 'old' values to draw a line from so set old values to current values in this case
        xold=x;
        yold=y;
      }
      
      u8g.drawLine(xold,yold,x,y);
      
      xold=x;  // store values for next iteration
      yold=y;
    }
    
    // display minimum VSWR and frequency at which it occurs - try to keep it away from the VSWR curve
    
    y=8+40*((meanVSWR-100)>50*ceil(float(maxVSWR/100)));  // estimate where bulk of curve is by comparing mean VSWR with y-axis half-way point
    
    dtostrf((float)swr_results[bc][1]/1000,6,3,minfreqdisp);
    dtostrf((float)swr_results[bc][0]/1000,5,2,minvswr); 
        u8g.drawStr(22,y,"Min      @        MHz");
        u8g.drawStr(40,y,minvswr);
        u8g.drawStr(74,y,minfreqdisp);

    } while(u8g.nextPage() );
}
