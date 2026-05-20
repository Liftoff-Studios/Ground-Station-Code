#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>
#include <Preferences.h>
#include <vector>

//Libraries for LoRa
#include <SPI.h>
#include <Lora.h>
#include <string>


/*
We won't be checking actual addresses on the sender and receiver to identify the correct messages, but will just assign a byte as a name

GroundStationEsp32 - 0x7B
CanSatEsp32 - 0x9B

*/
byte localAddress = 0x9B;
byte destinationAddress = 0x7B;


Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;
Preferences preferences;

float telemetry_transfer_rate = 2.0; //in Hz
unsigned long prevTime = millis();
unsigned long prevTimeMissionStateTime = millis();
float currentAltitude = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //I2C init
  Wire.begin(27,14);

  //LoRa init
  if (!LoRa.begin(866E6)) {            
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }

  Serial.println("LoRa init succeeded.");

  if (!bmp.begin()) {
    Serial.println("BMP180 couldn't initialise");
  }

  if(!mpu.begin()){
    Serial.println("MPU6050 couldn't initialise");
  }


  //Basic Setup for the accelerometer
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  


  //Starting flash storage
  preferences.begin("cansat-nv-data");
  preferences.putInt("mission-state",0);
  /*
  Following data will be stored
  mission-time float
  packet-count int 
  mission-state int
  parachute-deployed int 0 ==False 1==True
  ground-level-pressure float
  */

  //Update the altitude on startup
  currentAltitude = getPressureValues(preferences.getFloat("ground-level-pressure",101325.0))[2];
  

  //We can't set it because if reset happens then it should take from previous state
  //preferences.putInt("mission-time",0.0);
  //preferences.putInt("mission-state",0);
  /*
   // Store a value
  preferences.putInt("counter", 10); 

  // Retrieve a value (with a default of 0 if key doesn't exist)
  int val = preferences.getInt("counter", 0);
*/
}


//Main loop for gathering sensor output
void loop() {

    //Code for the mission time
    float current_mission_time =  preferences.getFloat("mission-time", 0.0);

    unsigned long currTime = millis();
    if((currTime-prevTime)>1000){
        preferences.putFloat("mission-time",current_mission_time+1.0);
        prevTime = currTime;
    }
    

    //Handles the radio data sending part and sensor logic handling part
    if((currTime-prevTimeMissionStateTime)>(1000/telemetry_transfer_rate)){
        int current_mission_state = preferences.getInt("mission-state",0);
        if(current_mission_state>1){
          //Get all the sensor data
          std::vector<float> accelerometerValues = getAccelerometerValues();
          std::vector<float> barometerValues = getPressureValues(preferences.getFloat("ground-level-pressure",101325.0));

          int newAltitude = barometerValues[2]; //REPLACE WITH THE ALTIMETER data 

          int currentPacketCount = preferences.getInt("packet-count",0);
          int isParachuteDeployed = preferences.getInt("parachute-deployed",0);
          //Condition for parachute deployment
          if(newAltitude<70.0 && (newAltitude-currentAltitude)<0.0 && isParachuteDeployed==0){
              //Trigger Deployment sequence
              preferences.putInt("mission-state",5);
              preferences.putInt("parachute-deployed",1);
          }

          //Condition for ascending and freefall state
          if((newAltitude-currentAltitude)>0.0){
              preferences.putInt("mission-state",3);
          }else if(newAltitude>70.0){
              preferences.putInt("mission-state",4);
          }

          //Condition for landed followed by controlled-descent
          if((newAltitude-currentAltitude)==0.0 && (newAltitude-currentAltitude)/(currTime-prevTimeMissionStateTime)<0.1){
              preferences.putInt("mission-state",7);
              //Code for lighting up the beacon
          }else if((newAltitude-currentAltitude)<0.0 && (currentAltitude-newAltitude)/(currTime-prevTimeMissionStateTime)<5.0){
              preferences.putInt("mission-state",6);
          }

          //Sending Data Part Radio
          
          //Send data through radio
          String str1 = String("FOO,");
          str1 += current_mission_time;
          str1 += String(",");
          str1 += currentPacketCount;
          str1 += String(",");
          str1 += newAltitude;
          str1 += String(",");
          str1 += barometerValues[0];
          str1 += String(",");
          str1 += barometerValues[1];
          str1 += String(",VOLT_V, GNSS_TIME,GNSS_LAT, GNSS_LON, GNSS_ALT_M, GNSS_SATS,");//Need to implement voltage monitoring and gnss time lat long and all
          str1 += accelerometerValues[0];
          str1 += String(",");
          str1 += accelerometerValues[1];
          str1 += String(",");
          str1 += accelerometerValues[2];
          str1 += String(",spinRate,");
          str1 += preferences.getInt("mission-state",0);

          sendPacket(str1.c_str())
          preferences.getInt("packet-count",currentPacketCount+1);
        }
        //Update the time variables
        prevTimeMissionStateTime = currTime;
        currentAltitude = newAltitude;
    }


    //Radio receive logic, this is where we receive the commands and also change the state of telemetry active 
    //And also update ground-level-pressure when we get the command from ground station
    int answer = packetReceive(Lora.parsePacket())
    switch (answer)
    {
    case 0:
        /* code */
        break;
    case 1:
        preferences.putInt("mission-state",2);
        break;
    case 2:
        std::vector<float> baroVals = getPressureValues(101325.0);
        preferences.putInt("ground-level-pressure",baroVals[0]);
        break;
    case 3:
        preferences.putInt("mission-state",2);
        break;
    default:
        break;
    }

    //Delay for 20ms to give enough time to sensors
    delay(20);
}

//Returns accelerometer values in the format: accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp
std::vector<float> getAccelerometerValues(){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    return {a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z, temp.temperature};
}

//Return temperature sensor values in the following format: pressure, temperature, altitude
std::vector<float> getPressureValues(float groundPressure){
    float altitude = bmp.readAltitude(groundPressure);
    float temperature = bmp.readTemperature();
    int32_t pressure = bmp.readPressure();

    return {static_cast<float>(pressure), temperature, altitude};
}

//Notes
/*
We still need to decide the resistor values for the SDA and SCL line based on the components we're attaching to them
*/


//Functions for the LoRa Module
int packetReceive(int packet){
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
    return -1;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != destinationAddress) {
    Serial.println("Unintended Message");
    return -1;                             // skip rest of function
  }
 
 if(incoming.startsWith("setgyrozero")){
    //Code to set gyro as 0
    return 0;
 }
 if(incoming.startsWith("telemetry")){
    return 1;
 }
 if(incoming.startsWith("set-baro")){
    return 2;
 }
 if(incoming.startsWith("start-systems")){
    return 3;
 }
}

void sendMessage(String message){
  LoRa.beginPacket();                   // start packet
  LoRa.write(destinationAddress);       // add destination address
  LoRa.write(localAddress);             // add sender address 
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
}