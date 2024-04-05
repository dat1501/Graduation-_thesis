#include <SimpleTime.h>

#define BLYNK_FIRMWARE_VERSION        "0.1.0"
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL6eF6F6i_R"
#define BLYNK_TEMPLATE_NAME "smart graden"
#define BLYNK_AUTH_TOKEN "knkyDcGQ3Iy6CMPLk-qABPQ5n7fSfAMR"

#include <Blynk.h>
#include <SPI.h>
#include <LoRa.h>
#include <String.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
// #include <SimpleTimer.h>
// #include <SoftwareSerial.h>
#include <Wire.h>
// #include <EEPROM.h>


#define ss 5
#define rst 14
#define dio0 2
#define LED_PIN 13

char auth[] = "knkyDcGQ3Iy6CMPLk-qABPQ5n7fSfAMR";
char ssid[] = "NA__";
char pass[] = "30151998";
SimpleTimer timer;

static byte RxData[20];
static byte TxData[20];
char address;
byte message = 0;
byte LocalAddress = 0x01;               //--> address of this device (Master Address).
byte Destination_ESP32_Slave_1 = 0x02;
unsigned long previousMillis_SendMSG = 0;

void sendMessage(byte Outgoing, byte Destination) {
  LoRa.beginPacket();             //--> start packet
  LoRa.write(LocalAddress);        //--> add destination address
  // LoRa.write(LocalAddress);       //--> add sender address
  LoRa.write(TxData[1]);       //--> add sender address
  // LoRa.print(Outgoing);           //--> add payload
  LoRa.endPacket();               //--> finish packet and send it
  // Serial.println("Transmit successfull!");
}

void onReceive(int packetSize) {
  int i = 0;
  if (packetSize == 0)
  {
    // Serial.print("there's no packet");
    delay(100);
    return;  //--> if there's no packet, return
  }
  for(i = 0; i<6; i++)
  {
    RxData[i] = LoRa.read();
    delay(1);
  }
  //---------------------------------------- if message is for this device, or broadcast, print details:
  Serial.println();
  Serial.println("Received from: 0x" + String(RxData[0], HEX));
  Serial.println("Data: ");
  Serial.println("0x" + String(RxData[1], HEX));
  Serial.println("0x" + String(RxData[2], HEX));
  Serial.println(" 0x" +String(RxData[3], HEX));
  Serial.println("0x" + String(RxData[4], HEX));
  Serial.println(" 0x" +String(RxData[5], HEX));
  // sendMessage(message, Destination_ESP32_Slave_1);
  // sendMessage(message , 0x02);
}

BLYNK_WRITE(V0) { // User option
  int value;
  value = param.asInt();
  digitalWrite(LED_PIN, value);
  // Serial.println(value);
  if(value == 0)
  {
    message = 0;
    TxData[1] = 0;
    Serial.println(TxData[1]);
  }
  else
  {
    message = 1;
    TxData[1] = 1;
    Serial.println(TxData[1]);
  }
}

BLYNK_WRITE(V5) { // Motor control
  int value;
  
  // Serial.println(value);
  if(TxData[1] == 0)
  {
    // do nothing
  }
  else
  {
    value = param.asInt();
    if(value == 0)
    {

    }
    else
    {
    }
  }
  
}

BLYNK_WRITE(V6) { // Led control
  int value;
  // value = param.asInt();
  // digitalWrite(LED_PIN, value);
  // Serial.println(value);
  if(TxData[1] == 0)
  {
    // do nothing
  }
  else
  {
    value = param.asInt();
    if(value == 0)
    {
    }
    else
    {
    }
  }
}

void sendUptime()
{
  Blynk.virtualWrite(V1, RxData[3]); // write Air temperature data to display on web blynk 
  Blynk.virtualWrite(V2, RxData[4]); // write Air humidity data to display on web blynk 
  Blynk.virtualWrite(V3, RxData[2]); // write Earth himidity data to display on web blynk 
  Blynk.virtualWrite(V4, RxData[1]); // write light Data data to display on web blynk
  if(TxData[1] == 0)
  {
    Blynk.virtualWrite(V5, RxData[5]);
    Blynk.virtualWrite(V6, RxData[6]);
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  LoRa.setPins(ss, rst, dio0);
  Blynk.begin(auth, ssid, pass);
  Serial.println("Start LoRa init...");
  if (!LoRa.begin(433E6)) {             // initialize ratio at 915 or 433 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
  Serial.println("LoRa init succeeded.");
  
  timer.setInterval(10, sendUptime);
}

void loop() {
  // put your main code here, to run repeatedly:
  Blynk.run();
  timer.run();
  unsigned long currentMillis_SendMSG = millis();
  if (currentMillis_SendMSG - previousMillis_SendMSG >= 500)
  {
    sendMessage(TxData[1], Destination_ESP32_Slave_1);
    // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
  // delay(10);
  onReceive(LoRa.parsePacket());
}