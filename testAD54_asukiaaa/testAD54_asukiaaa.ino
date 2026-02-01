#include <AD5254_asukiaaa.h>

// Throttle
AD5254_asukiaaa potentio(AD5254_ASUKIAAA_ADDR_A0_VDD_A1_VDD);

uint8_t digitalPotChannelThrottle = 0;

// Steering

void writeThrottle(float thVal_0_1)
{
  /*
  2% pressed   resistor should be 540 ohms
  25% pressed  resistor should be 440 ohms
  75% pressed  resistor should be 220 ohms
  100% pressed resistor should be 10 ohms
  */
  // First lerp to use a sub interval of 0..1 since we want resistance in range 10 .. 600
  // instead of 0 .. 1000. Also flip the direction
  float a = 600.0 / 1000.0;
  float b = 10.0 / 1000.0;
  float subrange_0_1 = a + (b - a) * thVal_0_1;  // lerp

  // lerp again to map subrange_0_1 to a wiper position on the digital potentiometer.
  a = 0;
  b = 255; 
  int tmp = a + (b - a) *  subrange_0_1;
  uint8_t dpStep_0_255 = (uint8_t) tmp;
  /*
  Serial.print("Writing resistor value: ");
  Serial.print(thVal_0_1);
  Serial.print("    "); 
  Serial.print(subrange_0_1);
  Serial.print("    ");   
  Serial.print(tmp);
  Serial.print("    ");
  Serial.println(dpStep_0_255);
  delay(1);
  */
  uint8_t errorCode;

  Serial.print("writing throttle step: ");
  Serial.println(dpStep_0_255);

  for (int i = 0; i < 10; i++)
  {
    errorCode = potentio.writeRDAC(digitalPotChannelThrottle, dpStep_0_255);
    if (errorCode == (uint8_t) 0) 
    {
      return;
    }

    delay(15);
  }
  
  int ec = errorCode;
  Serial.print("Write throttle failed ");
  Serial.println(ec);
}

/*
void setSteering(float steer_neg1_1)
{
  float s_01 = (steer_neg1_1 + 1) * 0.5;
  float sVolts_0_3 = s_01 * 1.1 + 0.4;
  int steerVal_0_4096 = 4095 * sVolts_0_3 / 5.5;
  Serial.print("Setting steer val:  ");
  Serial.println(steerVal_0_4096);
  steeringDAC.setVoltage(steerVal_0_4096, false);
}
*/

void setup() 
{
  Serial.begin(9600);
  potentio.begin();
  // steeringDAC.begin(0x61);  // 0x61
  Serial.println("Done  Setup .");
}

void loop() 
{  
  Serial.println("A");
  writeThrottle(0);
  // setSteering(-1);
  Serial.println("aaa");
  delay(3000);

  Serial.println("B");
  writeThrottle(0.5);
  // setSteering(0);
  Serial.println("bbbb");
  delay(3000);  

  Serial.println("C");
  writeThrottle(1.0);
  // setSteering(1.0);
  Serial.println("ccc");
  delay(3000);   
}