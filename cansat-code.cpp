#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_MPU6050.h>
#include <Preferences.h>
#include <vector>
#include <TinyGPS++.h>
#include <Adafruit_ADS1X15.h>
#include <SD.h>
#include <Arducam_Mega.h>
 
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

//Other standard variables
#define SDA 27;
#define SCL 14;

//Variables for the GNSS Module
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600
TinyGPSPlus gps;//Create the GPS Object
HardwareSerial gpsSerial(2);


Adafruit_BMP3XX bmp;
Adafruit_MPU6050 mpu;
Preferences preferences;
Adafruit_ADS1115 ads;

float telemetry_transfer_rate = 2.0; //in Hz
unsigned long prevTime = millis();
unsigned long prevTimeMissionStateTime = millis();
float currentAltitude = 0;


//Camera Initialisation Variables
// Instantiate Arducam Core Target Object
ArducamCamera camera(26);

// Edge Recording Configurations
File videoFile;
String fileName = "";
bool recordingActive = false;
uint8_t bufferBlock; // Local SPI data caching allocation
unsigned long recordingDuration = 1200000; // Cap video sequence at 1200 Seconds
unsigned long startTime = 0;

// Calibration multiplier (1 + R1/R2)
// R1 = 10k, R2 = 3.3k -> 1 + (10/3.3) = 4.03
const float dividerRatio = 4.03; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //I2C init
  Wire.begin(SDA,SCL);

  //SPI Begin
  SPI.begin(18,19,23);


  //Initialising ADS for voltage monitoring
  if (!ads.begin(0x48)) {
    Serial.println("Failed to initialize ADS.");
  }
  // Set PGA (Gain) to 1 (measures +/- 4.096V)
  ads.setGain(GAIN_ONE);

  //Initialise the GNSS Module
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial 2 started at 9600 baud rate");

  //LoRa init
  LoRa.setSPI(SPI);
  if (!LoRa.begin(866E6)) {            
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }
  LoRa.setPins(5);
  Serial.println("LoRa init succeeded.");

  if (!bmp.begin_I2C()) {
    Serial.println("BMP180 couldn't initialise");
  }
  //Setting some configuration values for the BMP sensor
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

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

  //Buzzer Initialisation
  pinMode(12, OUTPUT); 


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
  

  //ArduCam + SD Card Initialisation
  //Set up the SD
  if (!SD.begin(25)) {
    Serial.println("initialization failed!");
  }
  Serial.println(F("Adafruit MicroSD Card detected successfully."));

  // 3. Initialize Arducam Mega Hardware Array (Pin 5)
  camera.begin();

  // 4. Generate next Sequential Filename automatically 
  int fileIndex = 0;
  while (SD.exists("/video_" + String(fileIndex) + ".mjpeg")) {
    fileIndex++;
  }
  fileName = "/video_" + String(fileIndex) + ".mjpeg";
  
  // 5. Build and open the target File Matrix 
  videoFile = SD.open(fileName, FILE_WRITE);
  if (!videoFile) {
    Serial.println(F("CRITICAL ERROR: Failed to instantiate SD stream container."));
    while (1);
  }
  Serial.printf("Target output established: %s\n", fileName.c_str());

  // 6. Initiate Camera Video Buffering Mode (VGA Resolution for speed)
  CamStatus status = camera.startPreview(CAM_VIDEO_MODE_640X480); 
  
  if (status == CAM_ERR_SUCCESS) {
    Serial.println(F("Recording started... Don't remove power."));
    recordingActive = true;
    startTime = millis();
  } else {
    Serial.println(F("Failed to initialize asynchronous capture sequence."));
  }


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

          int newAltitude = barometerValues[2];

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
              digitalWrite(12, HIGH)
          }else if((newAltitude-currentAltitude)<0.0 && (currentAltitude-newAltitude)/(currTime-prevTimeMissionStateTime)<5.0){
              preferences.putInt("mission-state",6);
          }

          //get GNSS data
          std::vector<float> gnssData = getGNSSData(); 

          //Voltage Monitor Value
          float actualVoltageValue = getBatteryVoltageLevel();

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
          str1 += String(",");
          str1 += actualVoltageValue;//Voltage Monitor Value
          str1 += String(",");
          str1 += gnssData[0];
          str1 += String("-");
          str1 += gnssData[1];
          str1 += String("-");
          str1 += gnssData[2];
          str1 += String("-");
          str1 += gnssData[3];
          str1 += String(",");
          str1 += gnssData[4];
          str1 += String(",");
          str1 += gnssData[5];
          str1 += String(",");
          str1 += gnssData[6];
          str1 += String(",");
          str1 += gnssData[7];
          str1 += String(",");
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

    //Camera Recording
    if (recordingActive) {
      uint32_t bytesRead = 0;
      
      // Asynchronously pull MJPEG compressed frame chunks over the shared SPI Bus
      CamStatus readStatus = camera.readStream(&bufferBlock, sizeof(bufferBlock), &bytesRead);

      if (readStatus == CAM_ERR_SUCCESS && bytesRead > 0) {
        // Stream raw package blocks synchronously into the Adafruit MicroSD storage array
        videoFile.write(&bufferBlock, bytesRead);
      }

      // Terminate clip once global execution timeout target is achieved
      if (millis() - startTime >= recordingDuration) {
        recordingActive = false;
        camera.stopPreview();
        videoFile.flush(); // Flush residual storage tracks
        videoFile.close(); // Safeguard data structural integrity 
        Serial.println(F("\n--- Recording Finished! Safe to remove SD card. ---"));
      }
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
    if (! bmp.performReading()) {
      Serial.println("BMP Reading Failed");
      return {0.0,0.0,0.0};
    }
    float altitude = bmp.readAltitude(groundPressure/100.0);//Divided by 100 to convert to hPa
    float temperature = bmp.temperature;
    int32_t pressure = bmp.pressure;

    return {static_cast<float>(pressure), temperature, altitude};
}


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

std::vector<float> getGNSSData(){
  while (gpsSerial.available() > 0){
    gps.encode(gpsSerial.read());
  }

  std::vector<float> valuesToReturn = {};
  valuesToReturn.push_back(gps.date.day());
  valuesToReturn.push_back(gps.time.hour());
  valuesToReturn.push_back(gps.time.minute());
  valuesToReturn.push_back(gps.time.second());
  valuesToReturn.push_back(gps.location.lat());
  valuesToReturn.push_back(gps.location.lng());
  valuesToReturn.push_back(gps.altitude.meters());
  valuesToReturn.push_back(gps.satellites.value());

  return valuesToReturn;
} 

float getBatteryVoltageLevel(){
  int16_t adc0;
  float volts0;

  adc0 = ads.readADC_SingleEnded(0);
  volts0 = ads.computeVolts(adc0);

  // Re-calculate the actual high voltage
  float actualVoltage = volts0 * dividerRatio;

  return actualVoltage;
}