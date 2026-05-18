#include <SPI.h>
#include <Lora.h>
#include <string>


/*
We won't be checking actual addresses on the sender and receiver to identify the correct messages, but will just assign a byte as a name

GroundStationEsp32 - 0x7B
CanSatEsp32 - 0x9B


*/

byte localAddress = 0x7B;
byte destinationAddress = 0x9B;

void setup() {
  Serial.begin(115200);

  if (!LoRa.begin(866E6)) {            
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }

  Serial.println("LoRa init succeeded.");
}

void loop() {
  if (Serial.available() > 0) {
    // read() gets the next byte as an integer (ASCII value)
    String incomingChar = Serial.readStringUntil('\n'); 

    if (incomingChar.startsWith("telemetry")){
      sendMessage(incomingChar)
    }else if (incomingChar.startsWith("setgyrozero")){
      sendMessage(incomingChar)
    }

  }

  packetReceive(Lora.parsePacket())
}

void packetReceive(int packet){
  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != destinationAddress) {
    Serial.println("Unintended Message");
    return;                             // skip rest of function
  }

  Serial.println(incoming);
}

void sendMessage(String message){
  LoRa.beginPacket();                   // start packet
  LoRa.write(destinationAddress);              // add destination address
  LoRa.write(localAddress);             // add sender address 
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
}