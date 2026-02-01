#define __ASSERT_USE_STDERR

#include <assert.h>

#include <AD5254_asukiaaa.h>
#include <Adafruit_MCP4725.h>

// Throttle stuff ===============================
// The arduino controls a variable resistor  AD5254 using i2c
AD5254_asukiaaa throttleResistor(AD5254_ASUKIAAA_ADDR_A0_GND_A1_GND);


uint8_t throttleResistorDigitalPotChan = 0;

int THROTTLE_CRANK_SIG_PIN = A3;

long frameCount;
float db_throttle01;
bool isCrankSigOn;
int crankSigNumTicsOn;

// Each time the pedal crank makes a revolution we record the time in
// ms in a circular buffer. These get converted to pedal frequency in hz which is mapped
// to a normalized throttle value between 0 and 1 
int CircBuff_numInBuffer = 0;
int CircBuff_bufferLastIndex = -1;
const int CircBuff_bufferSize = 3;
unsigned long CircBuff_sampleTimes_ms[CircBuff_bufferSize];

// Steering stuff ==================================
// The arduino controls a DAC using i2c

Adafruit_MCP4725 steeringDAC;
int STEER_SIG_PIN = A2;

// Throttle Circular buffer functions =======================
void CircBuff_InitCircularBuffer()
{
    CircBuff_numInBuffer = 0;
    CircBuff_bufferLastIndex = -1;
    // CircBuff_sampleTimes_ms = new unsigned long[3];
}

void CircBuff_AddSample(unsigned long timeSamp_ms)
{
    // timeSamp_ms can overflow and wrap around to 0. Check that it is greater than the last sample
    if (CircBuff_numInBuffer > 0)
    {
        unsigned long lastSamp = CircBuff_GetSample(0);
        if (timeSamp_ms <  lastSamp)
        {
            // Debug.Log("TIME WRAPPED RESETTING -----");
            CircBuff_InitCircularBuffer();
        }
    }

    // Now add the sample
    CircBuff_bufferLastIndex++;
    if (CircBuff_bufferLastIndex >= CircBuff_bufferSize) { CircBuff_bufferLastIndex = 0; }
    CircBuff_numInBuffer++;
    if (CircBuff_numInBuffer > CircBuff_bufferSize) { CircBuff_numInBuffer = CircBuff_bufferSize; }
    CircBuff_sampleTimes_ms[CircBuff_bufferLastIndex] = timeSamp_ms;
}

unsigned long CircBuff_GetSample(int offsetFromLast)
{
    assert(offsetFromLast >= 0);
    assert(offsetFromLast < CircBuff_numInBuffer);
    assert(CircBuff_bufferLastIndex >= 0 && CircBuff_numInBuffer > 0);
    long idx = (CircBuff_bufferLastIndex - offsetFromLast);
    if (idx < 0) idx = CircBuff_bufferSize + idx;
    assert(idx >= 0 && idx < CircBuff_bufferSize);
    return CircBuff_sampleTimes_ms[ idx ];
}

int CircBuff_RemoveOldSamples(unsigned long currentTime_ms)
{
    unsigned long tm_ms = currentTime_ms;
    int numRemoved = 0;
    for (int i = CircBuff_numInBuffer - 1; i >= 0; i--)
    {
        float delta_ms = tm_ms - CircBuff_GetSample(i);
        // remove any samples older than 1 second
        if (delta_ms > 3000)
        {
            numRemoved++;
            CircBuff_numInBuffer--;
        }
    }

    return numRemoved;
}

// Throttle functions ===========================
float ConvertSamplesToPedalSpeed_hz()
{
    float pedalSpeedHertz = 0.0;
    if (CircBuff_numInBuffer > 1)
    {
        unsigned long curTime = millis();

        float avePeriod_ms = 0.0;
        float numToAverage = 0;
        for (int i = 0; i < CircBuff_numInBuffer - 1; i++)
        {
            unsigned long delta = CircBuff_GetSample(i) - CircBuff_GetSample(i + 1);
            if (delta <= 0) delta = 1.0; // min is one ms
            avePeriod_ms += delta;
            numToAverage ++;
        }

        {
          // We might have stopped pedaling 
          unsigned long curDelta = millis() - CircBuff_GetSample(0);
          unsigned long mostRecentDelta = CircBuff_GetSample(0) - CircBuff_GetSample(1);
          if (curDelta > mostRecentDelta)
          {
              avePeriod_ms = curDelta;
              numToAverage = 1;
          }
        }

        avePeriod_ms /= numToAverage;
        pedalSpeedHertz = 1000.0 / (avePeriod_ms);

        // Debug.Log("  aveHertz: " + pedalSpeedHertz + "    numInBuffer: " + CircBuff_numInBuffer);
    }

    return pedalSpeedHertz;
}

float MapPedalHzToThrottle01(float pedalSpeedHz)
{
    float minPedalSpeedHz = 0.0;
    float maxPedalSpeedHz = 1.9;
    if (pedalSpeedHz < minPedalSpeedHz) pedalSpeedHz = minPedalSpeedHz;
    if (pedalSpeedHz > maxPedalSpeedHz) pedalSpeedHz = maxPedalSpeedHz;
    float throttle01 = (pedalSpeedHz - minPedalSpeedHz) / (maxPedalSpeedHz - minPedalSpeedHz);
    if (throttle01 > 1.0) throttle01 = 1;
    return throttle01;
}

float DoConvertPedalSampsToThrottleAndWriteToController()
{
    float pedalSpeedHz = ConvertSamplesToPedalSpeed_hz();
    float throttle01 = MapPedalHzToThrottle01(pedalSpeedHz);
    writeThrottle(throttle01);
    return throttle01;
}

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
  float a = 500.0 / 1000.0;
  float b = 60.0 / 1000.0;
  float subrange_0_1 = a + (b - a) * thVal_0_1;  // lerp
  subrange_0_1 = 1.0 - subrange_0_1;

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

/*
  Serial.print("writing throttle step: ");
  Serial.print(thVal_0_1);
  Serial.print("   subrange_0_1: ");
  Serial.println(subrange_0_1);
*/

  errorCode = throttleResistor.writeRDAC(throttleResistorDigitalPotChan, dpStep_0_255);
  if (errorCode == (uint8_t) 0) 
  {
    return;
  }
  
  int ec = errorCode;
  Serial.print("Write throttle failed ");
  Serial.println(ec);
}

float SteerMapRawToNeg1To1(float valRaw01)
{
  return (valRaw01 * 2.0) - 1.0;
}

float GetSign(float f)
{
  if (f >= 0) return 1;
  return -1; 
}

// Steering functions
void setSteering(float steer_neg1_1)
{
  {
     // We want steering to be more sensitive close to neutral
     // and less sensetive at the extremes
     float sign = GetSign(steer_neg1_1);
     float exponent = 0.75;
     steer_neg1_1 = sign * pow(abs(steer_neg1_1), exponent);
  }

  float s_01 = (steer_neg1_1 + 1) * 0.5;
  // center is 1v
  float sVolts_0_3 = s_01 * 1.2 + 0.4; 
  int steerVal_0_4096 = 4095 * sVolts_0_3 / 5.5;
  /*
  Serial.print("Setting steer val:  ");
  Serial.print(steer_neg1_1);
  Serial.print("  sVolts_0_3: ");
  Serial.println(sVolts_0_3);
  */
  steeringDAC.setVoltage(steerVal_0_4096, false);
}

void setup() 
{
  frameCount = 0;
  db_throttle01 = 0;
  Serial.begin(9600);
  CircBuff_InitCircularBuffer();
  isCrankSigOn = false;
  crankSigNumTicsOn = 0;
  throttleResistor.begin();
  steeringDAC.begin(0x61);  // 0x61
  Serial.println("Done  Setup .");
}

void loop() 
{ 
  /*
  Serial.println("============="); 
  Serial.println("A");
  writeThrottle(0);
  setSteering(-1);
  Serial.println("aaa");
  delay(5000);

  Serial.println("B");
  writeThrottle(.25f);
  setSteering(-.5);
  Serial.println("aaa");
  delay(5000);

  Serial.println("C");
  writeThrottle(0.5);
  setSteering(0);
  Serial.println("bbbb");
  delay(5000);  

  Serial.println("D");
  writeThrottle(.75f);
  setSteering(.5);
  Serial.println("aaa");
  delay(5000);

  Serial.println("E");
  writeThrottle(1.0);
  setSteering(1.0);
  Serial.println("ccc");
  delay(5000);
  */   


  {
    // =========== Throttle
    // Check the sensor on the pedal crank to see if there was a revolution. If so
    // add a sample to the circular buffer
    bool isThrottleSampThisTick = false;

    float sigPinVoltage01;
    {
      int pinVal = analogRead(THROTTLE_CRANK_SIG_PIN);
      sigPinVoltage01 = (float) pinVal / 1024.0;
    }

    if (isCrankSigOn)
    {
      // signal goes on for several tics we want this
      // set to count as one sample.

      //Serial.print("AAA  ");
      //Serial.print(pinVal);
      //Serial.print("  voltage01  ");
      //Serial.println(sigPinVoltage01);

      if (sigPinVoltage01 > 0.6)
      {
        // signal ended
        isCrankSigOn = false;
        Serial.print(" signal ended  numtics: ");
        Serial.println(crankSigNumTicsOn);
      }
      else
      {
        crankSigNumTicsOn++;
      }
    } else
    {
      if (sigPinVoltage01 < 0.5)
      {
        Serial.print("Throttle starting tick: ");
        isCrankSigOn = true;
        crankSigNumTicsOn = 0;
        isThrottleSampThisTick = true;
      }
    }

    unsigned long newTm = millis();
    if (isThrottleSampThisTick)
    {
      // Debug.Log("Addsamp " + i + "  " + newTm);
      CircBuff_AddSample(newTm);
    }
    
    // If there is a long pause then we should purge the samples in the circular buffer.
    if (CircBuff_RemoveOldSamples(newTm) > 0) Serial.println("Removed some old samples.");
    db_throttle01 = DoConvertPedalSampsToThrottleAndWriteToController();
  }

  /*
  {
    // ============= Steering
    // measure the voltage from the steering potentiometer
    // convert the steering voltage to a -1 .. 1 value
    // convert the steering -1 .. 1 to an input value for the
    // DAC and write it to the GameController
    float steer_neg1_1 = 0.0;
    setSteering(steer_neg1_1);
  }
  */
  
  float val_neg1_1;
  float valRaw01;
  {
    int val_0_1024 = analogRead(STEER_SIG_PIN);
    valRaw01 = (float) val_0_1024 / 1024.0;
    val_neg1_1 = SteerMapRawToNeg1To1(valRaw01);
    setSteering(val_neg1_1);
  }

  {
    frameCount++;
    if (frameCount >  2100000000) frameCount = 0;
    if (frameCount % 100 == 0)
    {
      Serial.print("  throttle: ");
      Serial.print(db_throttle01);
      Serial.print("  steer: ");
      Serial.print(val_neg1_1);
      Serial.print("  valRaw01: ");
      Serial.print(valRaw01);
      Serial.println("  Loop");
    } 
  }

  delay(10); // 100 loops per second.
}