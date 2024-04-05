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


char address;
byte message = 0;
byte LocalAddress = 0x00;               //--> address of this device (Master Address).
byte Destination_ESP32_Slave_1 = 0x02;
unsigned long previousMillis_SendMSG = 0;
static byte RxData[20];
static byte TxData[20];

void sendMessage(void) {
  LoRa.beginPacket();             //--> start packet
  // LoRa.write(LocalAddress);        //--> add destination address
  // LoRa.write(TxData[1]);       //--> add User option
  // LoRa.write(TxData[2]);       //--> add User control light
  // LoRa.write(TxData[3]);       //--> add User control motor
  // LoRa.write(TxData[4]);       //--> add User control fan
  LoRa.write(TxData, 5);
  LoRa.endPacket();               //--> finish packet and send it
  
  // Serial.println("Transmit successfull!");
}

byte onReceive(int packetSize) {
  int i = 0;
  if (packetSize == 0)
  {
    Serial.print("there's no packet");
    delay(100);
    return 0;  //--> if there's no packet, return
  }
  // RxData[0] = LoRa.read();
  // if(RxData[0] == 0x0)
  // {
  //   return 0;
  // }
  for(i = 0; i<8; i++)
  {
    RxData[i] = LoRa.read();
    delay(1);
  }
  //---------------------------------------- if message is for this device, or broadcast, print details:
  Serial.println();
  Serial.println("Received from: 0x" + String(RxData[0], HEX));
  Serial.print("Data: ");
  Serial.print("0x" + String(RxData[1], HEX));
  Serial.print(" 0x" + String(RxData[2], HEX));
  Serial.print(" 0x" +String(RxData[3], HEX));
  Serial.print(" 0x" + String(RxData[4], HEX));
  Serial.print(" 0x" +String(RxData[5], HEX));
  Serial.print(" 0x" +String(RxData[6], HEX));
  Serial.println(" 0x" +String(RxData[7], HEX));
  // sendMessage();
  return 1;
}

BLYNK_WRITE(V0) { // User option
  int value;
  value = param.asInt();
  digitalWrite(LED_PIN, value);
  // Serial.println(value);
  if(value == 0)
  {
    TxData[1] = 0;
    Serial.println(TxData[1]);
  }
  else
  {
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
      TxData[2] = 0;
    }
    else
    {
      TxData[2] = 1;
    }
  }
  
}

BLYNK_WRITE(V6) { // Light control
  int value;
  if(TxData[1] == 0)
  {
    // do nothing
  }
  else
  {
    value = param.asInt();
    if(value == 0)
    {
      TxData[3] = 0;
    }
    else
    {
      TxData[3] = 1;
    }
  }
}

BLYNK_WRITE(V7) { // Fan control
  int value;
  if(TxData[1] == 0)
  {
    // do nothing
  }
  else
  {
    value = param.asInt();
    if(value == 0)
    {
      TxData[4] = 0;
    }
    else
    {
      TxData[4] = 1;
    }
  }
}

void sendUptime()
{
  Blynk.virtualWrite(V1, RxData[3]); // write Air temperature data to display on web blynk 
  Blynk.virtualWrite(V2, RxData[4]); // write Air humidity data to display on web blynk 
  Blynk.virtualWrite(V3, RxData[2]); // write Earth himidity data to display on web blynk 
  Blynk.virtualWrite(V4, RxData[1]); // write light Data data to display on web blynk
  Blynk.virtualWrite(V8, RxData[5]); // write lightState Data data to display on web blynk
  Blynk.virtualWrite(V9, RxData[6]); // write MotorState Data data to display on web blynk
  Blynk.virtualWrite(V10, RxData[7]); // write FanState Data data to display on web blynk
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
  TxData[0] = LocalAddress;
  timer.setInterval(100, sendUptime);
}

void loop() {
  byte ret;
  // put your main code here, to run repeatedly:
  Blynk.run();
  timer.run();
  unsigned long currentMillis_SendMSG = millis();
  if (currentMillis_SendMSG - previousMillis_SendMSG >= 1000)
  {
    // sendMessage(TxData[1], Destination_ESP32_Slave_1);
    // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
  // if(LoRa.available())
  // {
    // sendMessage();
  // }
  delay(10);
  ret = onReceive(LoRa.parsePacket());
  if(ret == 1)
  {
    sendMessage();
  }
}