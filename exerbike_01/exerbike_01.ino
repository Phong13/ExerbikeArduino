#define __ASSERT_USE_STDERR
#include <assert.h>
#include <Adafruit_MCP4728.h>
#include <Wire.h>

Adafruit_MCP4728 mcp;

// =============================
// PIN DEFINITIONS
// =============================

const int CRANK_A_PIN = 2;
const int CRANK_B_PIN = 3;

int STEER_SIG_PIN = A2;

MCP4728_channel_t MCP4728_throttleChan = MCP4728_CHANNEL_A;
MCP4728_channel_t MCP4728_steerChan    = MCP4728_CHANNEL_B;


// =============================
// EVENT QUEUE
// =============================

const uint8_t EVT_BUF_SZ = 8;

volatile unsigned long evtTimes[EVT_BUF_SZ];
volatile uint8_t evtSensor[EVT_BUF_SZ];

volatile uint8_t evtHead = 0;
volatile uint8_t evtTail = 0;
volatile uint8_t evtCount = 0;


// =============================
// HALF REV CIRCULAR BUFFER
// =============================

const int CircBuff_bufferSize = 6;

unsigned long CircBuff_sampleTimes_us[CircBuff_bufferSize];
uint8_t CircBuff_sensor[CircBuff_bufferSize];

int CircBuff_numInBuffer = 0;
int CircBuff_bufferLastIndex = -1;

void CircBuff_Init()
{
    CircBuff_numInBuffer = 0;
    CircBuff_bufferLastIndex = -1;
}

void CircBuff_AddSample(unsigned long time_us, uint8_t sensor)
{
    CircBuff_bufferLastIndex++;
    if (CircBuff_bufferLastIndex >= CircBuff_bufferSize)
        CircBuff_bufferLastIndex = 0;

    if (CircBuff_numInBuffer < CircBuff_bufferSize)
        CircBuff_numInBuffer++;

    CircBuff_sampleTimes_us[CircBuff_bufferLastIndex] = time_us;
    CircBuff_sensor[CircBuff_bufferLastIndex] = sensor;
}

unsigned long CircBuff_GetSample(int offsetFromLast)
{
    int idx = CircBuff_bufferLastIndex - offsetFromLast;
    if (idx < 0) idx += CircBuff_bufferSize;
    return CircBuff_sampleTimes_us[idx];
}

uint8_t CircBuff_GetSensor(int offsetFromLast)
{
    int idx = CircBuff_bufferLastIndex - offsetFromLast;
    if (idx < 0) idx += CircBuff_bufferSize;
    return CircBuff_sensor[idx];
}


// =============================
// ISR
// =============================

void ISR_crankA()
{
    unsigned long t = micros();

    if (evtCount < EVT_BUF_SZ)
    {
        evtTimes[evtHead] = t;
        evtSensor[evtHead] = 0;   // A
        evtHead = (evtHead + 1) % EVT_BUF_SZ;
        evtCount++;
    }
}

void ISR_crankB()
{
    unsigned long t = micros();

    if (evtCount < EVT_BUF_SZ)
    {
        evtTimes[evtHead] = t;
        evtSensor[evtHead] = 1;   // B
        evtHead = (evtHead + 1) % EVT_BUF_SZ;
        evtCount++;
    }
}


// =============================
// CADENCE CALCULATION
// =============================

float ConvertSamplesToPedalSpeed_hz()
{
    if (CircBuff_numInBuffer == 0)
        return 0.0f;

    unsigned long now = micros();

    float aveHalfPeriod_us = 0.0f;
    int count = 0;

    for (int i = 0; i < CircBuff_numInBuffer - 1; i++)
    {
        unsigned long newer = CircBuff_GetSample(i);
        unsigned long older = CircBuff_GetSample(i + 1);

        unsigned long delta = newer - older;
        if (delta == 0) delta = 1;

        aveHalfPeriod_us += delta;
        count++;
    }

    if (count > 0)
        aveHalfPeriod_us /= count;
    else
        aveHalfPeriod_us = 500000.0f;

    unsigned long timeSinceLast = now - CircBuff_GetSample(0);

    // NEW: hard timeout if pedaling stopped
    const unsigned long STOP_TIMEOUT_US = 2500000UL;  // 2.5 seconds
    if (timeSinceLast > STOP_TIMEOUT_US)
        return 0.0f;

    float effectiveHalfPeriod = max((float)timeSinceLast, aveHalfPeriod_us);

    float hz = 1000000.0f / (2.0f * effectiveHalfPeriod);

    return hz;
}


// =============================
// THROTTLE
// =============================

float MapPedalHzToThrottle01(float pedalHz)
{
    float minHz = 0.0f;
    float maxHz = 1.65f;

    pedalHz = constrain(pedalHz, minHz, maxHz);

    return (pedalHz - minHz) / (maxHz - minHz);
}

void writeThrottle(float thVal_0_1)
{
    thVal_0_1 = constrain(thVal_0_1, 0.0f, 1.0f);

    const float vMin = 1.33f;
    const float vMax = 0.2f;
    const float dacMax = 4095.0f;
    const float vRef = 5.0f;

    float stepMin = (vMin / vRef) * dacMax;
    float stepMax = (vMax / vRef) * dacMax;

    float outFloat = stepMin + (stepMax - stepMin) * thVal_0_1;
    uint16_t out = (uint16_t)round(outFloat);

    mcp.setChannelValue(MCP4728_throttleChan, out);
}


// =============================
// STEERING
// =============================

float SteerMapRawToNeg1To1(float valRaw01)
{
    return (valRaw01 * 2.0f) - 1.0f;
}

float GetSign(float f)
{
    return (f >= 0) ? 1.0f : -1.0f;
}

void setSteering(float steer_neg1_1)
{
    float sign = GetSign(steer_neg1_1);
    float exponent = 0.75f;

    steer_neg1_1 = sign * pow(abs(steer_neg1_1), exponent);

    float s_01 = (steer_neg1_1 + 1.0f) * 0.5f;
    float sVolts = s_01 * 1.2f + 0.4f;

    int steerVal = 4095 * sVolts / 5.5f;

    mcp.setChannelValue(MCP4728_steerChan, steerVal);
}


// =============================
// SETUP
// =============================

void setup()
{
    Serial.begin(9600);

    pinMode(CRANK_A_PIN, INPUT_PULLUP);
    pinMode(CRANK_B_PIN, INPUT_PULLUP);

    // attachInterrupt(digitalPinToInterrupt(CRANK_A_PIN), ISR_crankA, FALLING);
    // attachInterrupt(digitalPinToInterrupt(CRANK_B_PIN), ISR_crankB, FALLING);
    attachInterrupt(digitalPinToInterrupt(CRANK_A_PIN), ISR_crankA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CRANK_B_PIN), ISR_crankB, CHANGE);   

    if (!mcp.begin())
    {
        Serial.println("Failed to find MCP4728 chip");
        while (1) delay(10);
    }

    CircBuff_Init();

    Serial.println("Dual Hall Cadence System Ready.");

Serial.print("INT A = ");
Serial.println(digitalPinToInterrupt(CRANK_A_PIN));

Serial.print("INT B = ");
Serial.println(digitalPinToInterrupt(CRANK_B_PIN));    
}


// =============================
// MAIN LOOP
// =============================

void loop()
{
    while (true)
    {
        unsigned long t;
        uint8_t sensor;

        noInterrupts();

        if (evtCount == 0)
        {
            interrupts();
            break;
        }

        t = evtTimes[evtTail];
        sensor = evtSensor[evtTail];

        evtTail = (evtTail + 1) % EVT_BUF_SZ;
        evtCount--;

        interrupts();

        if (CircBuff_numInBuffer > 0)
        {
            unsigned long lastTime = CircBuff_GetSample(0);
            uint8_t lastSensor = CircBuff_GetSensor(0);

            if (sensor == lastSensor)
                continue;

            // reduced glitch window to 20ms
            if ((t - lastTime) < 20000UL)
                continue;
        }

        CircBuff_AddSample(t, sensor);

        if (sensor == 0)
            Serial.println("crankA");
        else
            Serial.println("crankB");
    }

    float pedalHz = ConvertSamplesToPedalSpeed_hz();
    float throttle01 = MapPedalHzToThrottle01(pedalHz);

    writeThrottle(throttle01);

    static unsigned long lastPrint = 0;
    unsigned long now = millis();

    if (now - lastPrint > 250)
    {
        lastPrint = now;

        Serial.print("Hz: ");
        Serial.print(pedalHz, 2);
        Serial.print("  thr: ");
        Serial.println(throttle01, 2);
    }

    int valRaw = analogRead(STEER_SIG_PIN);
    float val01 = (float)valRaw / 1024.0f;
    float steer = SteerMapRawToNeg1To1(val01);

    setSteering(steer);
}