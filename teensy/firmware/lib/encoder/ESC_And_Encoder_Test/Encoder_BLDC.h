/* 
 * lawrence.iviani@gmail.com
 *  
 * Adapted from  BLDC Hall Sensor read and calculation program for Teensy 3.5 in the Arduino IDE (Ver.1). Digi-Key Electronics
 * https://www.digikey.com/en/blog/using-bldc-hall-sensors-as-position-encoders-part-3
 * 
 * Inspiration for interruptHandling with a class fucntion from https://forum.arduino.cc/index.php?topic=365383.msg2518340#msg2518340
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#ifndef Encoder_BLDC_h_
#define Encoder_BLDC_h_


#include "Arduino.h"

// Rotation Table
// SENS_U | SENS_V | SENS_W
//   1    |   1    |   0
//   0    |   1    |   0
//   0    |   1    |   1
//   0    |   0    |   1
//   1    |   0    |   1
//   1    |   0    |   0


#include "digitalWriteFast.h"

typedef struct {
  uint8_t pinU;
  uint8_t pinV;
  uint8_t pinW;
  
  bool HSU_Val;   // Set the U sensor value as boolean and read initial state
  bool HSV_Val;   // Set the V sensor value as boolean and read initial state 
  bool HSW_Val;   // Set the W sensor value as boolean and read initial state 
  
  bool HSU_Read;   // Reading the U sensor value, before being deployed to HSU_VAL is READ and if it is a valid status is checked
  bool HSV_Read;   // Reading the V sensor value, before being deployed to HSV_VAL is READ and if it is a valid status is checked
  bool HSW_Read;   // Reading the W sensor value, before being deployed to HSW_VAL is READ and if it is a valid status is checked
  
  int direct;       // Integer variable to store BLDC rotation direction
  int32_t pulseCount;       // Integer variable to store the pulse count
  int32_t falsePulseCount;       // Integer variable to store the pulse count that are ignored
#if defined(BLDC_ENC_DEBUG) && BLDC_ENC_DEBUG==1
  long debugInterruptProcessed; // the number of time the interrupt calls are executed since last print
#endif
  float startTime;      // Float variable to store the start time of the current interrupt 
  float prevTime;       // Float variable to store the start time of the previous interrupt 
  float pulseTimeW;       // Float variable to store the elapsed time between interrupts for hall sensor W 
  float pulseTimeU;       // Float variable to store the elapsed time between interrupts for hall sensor U 
  float pulseTimeV;       // Float variable to store the elapsed time between interrupts for hall sensor V 
  float AvPulseTime;      // Float variable to store the average elapsed time between all interrupts 
  
  float PPM;        // Float variable to store calculated pulses per minute
  float RPM;        // Float variable to store calculated revolutions per minute
  
  int countsPerRev;

#if defined(BLDC_ENC_DEBUG) && BLDC_ENC_DEBUG==1
  char debug_interrupt_msg[40]; // for debugging!
#endif
} Encoder_BLDC_datastruct;

#include "digitalWriteFast.h"

#define CW             1      // Assign a value to represent clock wise rotation
#define CCW           -1      // Assign a value to represent counter-clock wise rotation
    
class Encoder_BLDC
{
  public:
    Encoder_BLDC(uint8_t pinU, uint8_t pinV, uint8_t pinW, int counts_per_rev);
    void setupInterruptHandler(uint8_t irq_pin, void (*ISR)(void), int value);
    void  handleInterrupt_U(void);
    void  handleInterrupt_V(void);
    void  handleInterrupt_W(void);

    int getRPM(){
      if ((millis() - this->data.prevTime) > 600) this->data.RPM = 0; // we're not moving anymore. sending a 0)
#if defined(BLDC_ENC_DEBUG) && BLDC_ENC_DEBUG==1
      this->print_debug();
#endif
      return this->data.RPM * this->data.direct; // RPM and rotation sense
    }

  inline int32_t read() {
    return this->data.pulseCount;
  }

 
  private:
    Encoder_BLDC_datastruct data;
    float _pstartTime=0;      // Float variable to store the start time of the current interrupt  (I Guess i have used only for debug...)

    void readAll();
    bool checkIsValidStatus();
    void setAllReadValues();
    void update_rpm();
#if defined(BLDC_ENC_DEBUG) && BLDC_ENC_DEBUG==1
    void print_debug();
#endif
};


Encoder_BLDC::Encoder_BLDC(uint8_t pinU, uint8_t pinV, uint8_t pinW, int counts_per_rev) {

// TBH, I am not sure how the pin should be set. In the design the signal sensors is powered via ESC. pinMode(pinXXX, INPUT);  seems to work
// Set digital pins HallSensorU/V/W as inputs
//#ifdef INPUT_PULLUP
//  #warning Encoder_BLDC configured with PULLUP
//      pinMode(pinU, INPUT_PULLUP);     
//      pinMode(pinV, INPUT_PULLUP);     
//      pinMode(pinW, INPUT_PULLUP);
//#else
  #warning Encoder_BLDC configured WITHOUT PULLUP
      pinMode(pinU, INPUT);     
      pinMode(pinV, INPUT);     
      pinMode(pinW, INPUT);
      // Need to set to high? No, they are connected to an external ESC
      //digitalWrite(pinU, HIGH);
      //digitalWrite(pinV, HIGH);
      //digitalWrite(pinW, HIGH);
//#endif //INPUT_PULLUP

      this->data.HSU_Val = digitalReadFast(pinU);   // Set the U sensor value as boolean and read initial state
      this->data.HSV_Val = digitalReadFast(pinV);   // Set the V sensor value as boolean and read initial state 
      this->data.HSW_Val = digitalReadFast(pinW);   // Set the W sensor value as boolean and read initial state 

      this->data.pinU = pinU;
      this->data.pinV = pinV;
      this->data.pinW = pinW;

      this->data.pulseTimeW = 0;       
      this->data.pulseTimeU = 0;       
      this->data.pulseTimeV = 0;       
      this->data.AvPulseTime = 0;
      
      this->data.PPM = 0;
      this->data.RPM = 0;
      this->data.direct = CW;
      this->data.falsePulseCount = 0;

      this->data.countsPerRev = counts_per_rev;
    }

inline void Encoder_BLDC::readAll() {
  this->data.HSU_Read = digitalReadFast(this->data.pinU);          // Read also U actual value
  this->data.HSV_Read = digitalReadFast(this->data.pinV);          // Read the actual V  hall sensor value 
  this->data.HSW_Read = digitalReadFast(this->data.pinW);          // Read the actual W hall sensor value
}

inline bool Encoder_BLDC::checkIsValidStatus() {
//  if (this->data.HSU_Read==this->data.HSV_Read==this->data.HSW_Read){
//    // If all are equal, this is not a valid status
//    return false;
//  } else {
//    return true;  
//  }
 return !((this->data.HSW_Read==this->data.HSU_Read) && (this->data.HSW_Read==this->data.HSV_Read)); // If all are equal, this is not a valid status
}

inline void Encoder_BLDC::setAllReadValues() {
  this->data.HSU_Val = this->data.HSU_Read;
  this->data.HSV_Val = this->data.HSV_Read;
  this->data.HSW_Val = this->data.HSW_Read;
}

inline void Encoder_BLDC::update_rpm() {
  this->data.pulseCount = this->data.pulseCount + (1 * data.direct);         // Add 1 to the pulse count in the direction of moving (+ FWD, - BCK)
  this->data.AvPulseTime = ((this->data.pulseTimeW + this->data.pulseTimeU + this->data.pulseTimeV)/3); // Calculate the average time time between pulses
  this->data.PPM = (1000 / this->data.AvPulseTime) * 60;              // Calculate the pulses per min (1000 millis in 1 second)
  this->data.RPM = this->data.PPM /  this->data.countsPerRev;                       // Calculate revs per minute based on 90 pulses per rev
  this->data.prevTime = this->data.startTime;                   // Remember the start time for the next interrupt
}

#if defined(BLDC_ENC_DEBUG) && BLDC_ENC_DEBUG==1   
void Encoder_BLDC::print_debug() {
  if ( this->_pstartTime!=this->data.startTime) { //If start time has changed this mean an interrupt function was called
    Serial.println("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"); 
    Serial.print("Computed "); Serial.print(this->data.debugInterruptProcessed); Serial.print(" intterrupt calls since last print, LAST computed is "); Serial.println(this->data.debug_interrupt_msg); 
    Serial.print("Moving ");Serial.print(this->data.direct==CW ? "FWD" : "RWD");Serial.print(" - RPM=");Serial.print(this->data.RPM); Serial.print(" PPM="); Serial.print(this->data.PPM); Serial.print(" - UVW="); Serial.print(this->data.HSU_Val); Serial.print(this->data.HSV_Val); Serial.println(this->data.HSW_Val);   
    Serial.print("Pulse Count= ");Serial.print(this->data.pulseCount);Serial.print(" False Count= ");Serial.print(this->data.falsePulseCount);Serial.print(" - AvgPulseTime=");Serial.print(this->data.AvPulseTime); Serial.print(" - PulseTIme(U,V,W)="); Serial.print(this->data.pulseTimeU);Serial.print(","); Serial.print(this->data.pulseTimeV);Serial.print(","); Serial.println(this->data.pulseTimeW); 
    Serial.println("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-");  
    this->_pstartTime   = this->data.startTime;        // Store when the latest 
    this->data.debugInterruptProcessed = 0;           // set to 0 the for next print
  }
}
#endif

/*---------------------------- INTERRUPT HANDLERS ----------------------------*/
void Encoder_BLDC::setupInterruptHandler(uint8_t irq_pin, void (*ISR)(void), int value)
{
  attachInterrupt(digitalPinToInterrupt(irq_pin), ISR, value);
}

inline void Encoder_BLDC::handleInterrupt_W(void)
{
#if defined(BLDC_ENC_DEBUG) && BLDC_ENC_DEBUG==1
  this->data.debugInterruptProcessed++; // Increment the counter
#endif
  // 1. Take the time and Read Sensor
  this->data.startTime = millis();                   // Set startTime to current microcontroller elapsed time value
  this->readAll();                              // Update all the readings
#if defined(BLDC_ENC_DEBUG) && BLDC_ENC_DEBUG==1
  sprintf(this->data.debug_interrupt_msg , "HallSensW (%d,%d,%d)", this->data.HSU_Read, this->data.HSV_Read, this->data.HSW_Read);
#endif 

  // 2. Continue if the status of sensor is valid, and continue updating the values
  if (!this->checkIsValidStatus()) {  // Check if this is a valid status
    this->data.falsePulseCount++;
    return;     
  }
  this->setAllReadValues();
#if defined(BLDC_ENC_DEBUG) && BLDC_ENC_DEBUG==1
  sprintf(this->data.debug_interrupt_msg , "HallSensW-COMPUTE (%d,%d,%d)", this->data.HSU_Val, this->data.HSV_Val, this->data.HSW_Val);
#endif

  // 3. Compute the direction and the time for sensor(both sensor specific) 
  this->data.direct = (this->data.HSW_Val == this->data.HSV_Val) ? CW : CCW;     // Determine rotation direction (ternary if statement)
  this->data.pulseTimeW = this->data.startTime - this->data.prevTime;            // Calculate the current time between pulses
  this->update_rpm();
}

inline void Encoder_BLDC::handleInterrupt_U(void)
{
#if defined(BLDC_ENC_DEBUG) && BLDC_ENC_DEBUG==1
  this->data.debugInterruptProcessed++; // Increment the counter
#endif
   // 1. Take the time and Read Sensor
  this->data.startTime = millis();                   // Set startTime to current microcontroller elapsed time value
  this->readAll();                              // Update all the readings
#if defined(BLDC_ENC_DEBUG) && BLDC_ENC_DEBUG==1
  sprintf(this->data.debug_interrupt_msg , "HallSensU (%d,%d,%d)", this->data.HSU_Read, this->data.HSV_Read, this->data.HSW_Read);
#endif

  // 2. Continue if the status of sensor is valid, and continue updating the values
  if (!this->checkIsValidStatus()) {  // Check if this is a valid status
    this->data.falsePulseCount++;
    return;     
  }
  this->setAllReadValues();
#if defined(BLDC_ENC_DEBUG) && BLDC_ENC_DEBUG==1 
  sprintf(this->data.debug_interrupt_msg , "HallSensU-COMPUTE (%d,%d,%d)", this->data.HSU_Val, this->data.HSV_Val, this->data.HSW_Val);
#endif

  // 3. Compute the direction and the time for sensor(both sensor specific) 
  this->data.direct = (this->data.HSU_Val == this->data.HSW_Val) ? CW : CCW;
  this->data.pulseTimeU = this->data.startTime - this->data.prevTime;        
  this->update_rpm();
}

inline void Encoder_BLDC::handleInterrupt_V(void)
{
#if defined(BLDC_ENC_DEBUG) && BLDC_ENC_DEBUG==1
  this->data.debugInterruptProcessed++; // Increment the counter
#endif  
  // 1. Take the time and Read Sensor
  this->data.startTime = millis();                   // Set startTime to current microcontroller elapsed time value
  this->readAll();                              // Update all the readings
#if defined(BLDC_ENC_DEBUG) && BLDC_ENC_DEBUG==1
  sprintf(this->data.debug_interrupt_msg , "HallSensV (%d,%d,%d)", this->data.HSU_Read, this->data.HSV_Read, this->data.HSW_Read);
#endif

  // 2. Continue if the status of sensor is valid, and continue updating the values
  if (!this->checkIsValidStatus())  {  // Check if this is a valid status
    this->data.falsePulseCount++;
    return;     
  }
  this->setAllReadValues();
#if defined(BLDC_ENC_DEBUG) && BLDC_ENC_DEBUG==1
  sprintf(this->data.debug_interrupt_msg , "HallSensV-COMPUTE (%d,%d,%d)", this->data.HSU_Val, this->data.HSV_Val, this->data.HSW_Val);
#endif

  // 3. Compute the direction and the time for sensor(both sensor specific) 
  this->data.direct = (this->data.HSV_Val == this->data.HSU_Val) ? CW : CCW;
  this->data.pulseTimeV = this->data.startTime - this->data.prevTime;        
  this->update_rpm();
}

#endif
