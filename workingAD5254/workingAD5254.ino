#include <AD5254_asukiaaa.h>

// AD5254_ASUKIAAA_ADDR_A0_VDD_A1_VDD
AD5254_asukiaaa potentio(AD5254_ASUKIAAA_ADDR_A0_GND_A1_GND);

uint8_t targetChannel = 0;
String targetChannelStr = String(targetChannel);

int initializedPot;

void setup() {
  
  initializedPot = false;
  Serial.begin(9600);
  Serial.println("hello world");
  
  // wire.begin();
  // potentio.setWire(&wire);

  initializedPot = potentio.begin();
  // if (initializedPot);
}

void loop() {
  int numVals = 5;
  int myVals[numVals] = {0, 64, 128, 200, 255};
  for (int i = 0; i < numVals; i++)
  {
    uint8_t value;
    if (potentio.readRDAC(targetChannel, &value) == 0) {
      Serial.println("RDAC of channel " + targetChannelStr + " is " + String(value));
    } else {
      Serial.println("Cannot read RDAC of channel " + targetChannelStr + ".");
    }

    uint8_t targetValue = myVals[i]; // (50 * millis() / 1000) % 256;
    if (potentio.writeRDAC(targetChannel, targetValue) == 0) {
      Serial.println("Update RDAC of channel " + targetChannelStr + " to " + String(targetValue));
    } else {
      Serial.println("Cannot update RDAC of channel " + targetChannelStr + ".");
    }

    delay(3000);
  }
  
  Serial.println("loop");
  delay(3000);
}