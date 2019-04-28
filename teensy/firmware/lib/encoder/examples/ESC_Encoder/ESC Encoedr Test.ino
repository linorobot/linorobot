
#include "Encoder_BLDC.h"

//
// PROGRAM START
//

//#define DEBUG 1

//Pins for the encoders
#define ENC1_PIN_SENS_A  15   //Hall sensors pin encoder 1
#define ENC1_PIN_SENS_B  14
#define ENC1_PIN_SENS_C  20

#define ENC2_PIN_SENS_A 11   //Hall sensors pin encoder 2
#define ENC2_PIN_SENS_B 10
#define ENC2_PIN_SENS_C 6

//Setup a pointer for the encoders needed, this has to be init with proper pins in the setup
Encoder_BLDC* encoder_1;
Encoder_BLDC* encoder_2;

//Encapsulate a call for any interrupts to the relative encoder class, this has to be asssociated in the setup
void enc1InterruptHandler_A(void) { encoder_1->handleInterrupt_U(); }
void enc1InterruptHandler_B(void) { encoder_1->handleInterrupt_V(); }
void enc1InterruptHandler_C(void) { encoder_1->handleInterrupt_W(); }

void enc2InterruptHandler_A(void) { encoder_2->handleInterrupt_U(); }
void enc2InterruptHandler_B(void) { encoder_2->handleInterrupt_V(); }
void enc2InterruptHandler_C(void) { encoder_2->handleInterrupt_W(); }


void setup()
{
  Serial.begin(9600);

  // For every encoder instantiate the class and associate the call defined above
  encoder_1 = new Encoder_BLDC(ENC1_PIN_SENS_A,ENC1_PIN_SENS_B,ENC1_PIN_SENS_C);
  encoder_1->setupInterruptHandler(ENC1_PIN_SENS_A, enc1InterruptHandler_A, CHANGE);
  encoder_1->setupInterruptHandler(ENC1_PIN_SENS_B, enc1InterruptHandler_B, CHANGE);
  encoder_1->setupInterruptHandler(ENC1_PIN_SENS_C, enc1InterruptHandler_C, CHANGE);
  
  encoder_2 = new Encoder_BLDC(ENC2_PIN_SENS_A,ENC2_PIN_SENS_B,ENC2_PIN_SENS_C);
  encoder_2->setupInterruptHandler(ENC2_PIN_SENS_A, enc2InterruptHandler_A, CHANGE);
  encoder_2->setupInterruptHandler(ENC2_PIN_SENS_B, enc2InterruptHandler_B, CHANGE);
  encoder_2->setupInterruptHandler(ENC2_PIN_SENS_C, enc2InterruptHandler_C, CHANGE);
}

void loop()
{
  static uint32_t lastMillis = 0;
  static float rpm1, rpm2;
  if (millis() - lastMillis > 1000UL)
  {
    rpm1 = encoder_1->getRPM();
    rpm2 = encoder_2->getRPM();
    lastMillis = millis();

    Serial.print("RPM_ENC1: ");Serial.println(rpm1);
    Serial.print("RPM_ENC2: ");Serial.println(rpm2);

  }
}
