#define __ASSERT_USE_STDERR

#include <assert.h>

#include <Adafruit_MCP4728.h>
#include <Wire.h>

Adafruit_MCP4728 mcp;

// Steering stuff ==================================
// The arduino controls a DAC using i2c

// Adafruit_MCP4725 steeringDAC;
// int STEER_SIG_PIN = A2;
MCP4728_channel_t MCP4728_steerChan = MCP4728_CHANNEL_B;
int STEER_SIG_PIN = A2;

// Throttle stuff ===============================
// The arduino controls a variable resistor  AD5254 using i2c
// AD5254_asukiaaa throttleResistor(AD5254_ASUKIAAA_ADDR_A0_GND_A1_GND);
MCP4728_channel_t MCP4728_throttleChan = MCP4728_CHANNEL_A;


// uint8_t throttleResistorDigitalPotChan = 0;

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
    /*
    if (frameCount % 100 == 0)
    {
      Serial.println("");
      Serial.print("pedal speed  ");
      Serial.print(pedalSpeedHz);
    }
    */
    float throttle01 = MapPedalHzToThrottle01(pedalSpeedHz);
    writeThrottle(throttle01);
    if (frameCount % 100 == 0)
    {
      Serial.print("   throttle01  ");
      Serial.print(throttle01);
    }
    return throttle01;
}

void writeThrottle(float thVal_0_1)
{
  // Clamp input for safety
  thVal_0_1 = constrain(thVal_0_1, 0.0f, 1.0f);

  const float vMin = 1.33f;   // no throttle
  const float vMax = 0.2f;    // full throttle
  const float dacMax = 4095.0f;
  const float vRef = 5.0f;

  // Convert voltage endpoints to DAC steps
  float stepMin = (vMin / vRef) * dacMax;
  float stepMax = (vMax / vRef) * dacMax;

  // Linear interpolation
  float outFloat = stepMin + (stepMax - stepMin) * thVal_0_1;

  uint16_t out = (uint16_t)round(outFloat);

  if (frameCount % 100 == 0)
  {
    float outVolts = (out / dacMax) * vRef;
    Serial.print("Throttle volts: ");
    Serial.print(outVolts);
    Serial.print("  step: ");
    Serial.println(out);
  }

  mcp.setChannelValue(MCP4728_throttleChan, out);
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
  // steeringDAC.setVoltage(steerVal_0_4096, false);
  // mcp.setChannelValue(MCP4728_steerChan, steerVal_0_4096);
  
}

void setup() 
{
  frameCount = 0;
  db_throttle01 = 0;
  Serial.begin(9600);
  CircBuff_InitCircularBuffer();
  isCrankSigOn = false;
  crankSigNumTicsOn = 0;


  // throttleResistor.begin();
  // steeringDAC.begin(0x61);  // 0x61
  if (!mcp.begin()) {
    Serial.println("Failed to find MCP4728 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("Done  Setup .");
}

void loop() 
{ 

   


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
        //Serial.print(" signal ended  numtics: ");
        //Serial.println(crankSigNumTicsOn);
      }
      else
      {
        crankSigNumTicsOn++;
      }
    } else
    {
      if (sigPinVoltage01 < 0.5)
      {
        //Serial.print("Throttle starting tick: ");
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
    //if (CircBuff_RemoveOldSamples(newTm) > 0) Serial.println("Removed some old samples.");
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
    /*
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
    */
  }

  delay(10); // 100 loops per second.
}