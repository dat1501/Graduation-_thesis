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

char data[20] ;
char address;
byte message = 0;
byte LocalAddress = 0x01;               //--> address of this device (Master Address).
byte Destination_ESP32_Slave_1 = 0x02;
unsigned long previousMillis_SendMSG = 0;

void sendMessage(byte Outgoing, byte Destination) {
  LoRa.beginPacket();             //--> start packet
  LoRa.write(LocalAddress);        //--> add destination address
  // LoRa.write(LocalAddress);       //--> add sender address
  LoRa.write(message);       //--> add sender address
  // LoRa.print(Outgoing);           //--> add payload
  LoRa.endPacket();               //--> finish packet and send it
  // Serial.println("Transmit successfull!");
}

void onReceive(int packetSize) {
  int i = 0;
  if (packetSize == 0)
  {
    // Serial.print("there's no packet");
    // delay(100);
    return;  //--> if there's no packet, return
  }
  for(i = 0; i<20; i++)
  {
    data[i] = LoRa.read();
    delay(1);
  }
  //---------------------------------------- if message is for this device, or broadcast, print details:
  Serial.println();
  Serial.println("Received from: 0x" + String(data[0], HEX));
  Serial.println("Data: " + String(data, 20));
  // sendMessage(message, Destination_ESP32_Slave_1);
  // sendMessage(message , 0x02);
}

BLYNK_WRITE(V0) { // Led control
  int value;
  value = param.asInt();
  digitalWrite(LED_PIN, value);
  // Serial.println(value);
  if(value == 0)
  {
    message = 0;
  }
  else
  {
    message = 1;
  }
}

void sendUptime()
{

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
  
  timer.setInterval(200, sendUptime);
}

void loop() {
  // put your main code here, to run repeatedly:
  Blynk.run();
  timer.run();
  unsigned long currentMillis_SendMSG = millis();
  if (currentMillis_SendMSG - previousMillis_SendMSG >= 500)
  {
    sendMessage(message, Destination_ESP32_Slave_1);
  }
  onReceive(LoRa.parsePacket());
  delay(10);
}