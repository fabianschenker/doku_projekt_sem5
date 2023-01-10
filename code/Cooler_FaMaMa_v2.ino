#define BLYNK_USE_DIRECT_CONNECT
#define BLYNK_PRINT Serial

// Imports
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <BlynkSimpleSerialBLE.h>
#include <TinyGPSPlus.h>
#include "./CoolerDefinitions.h"

// GPS
TinyGPSPlus gps;

// Master Enable
bool enabled = false;

// Serial components
SoftwareSerial bluetoothSerial(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);
SoftwareSerial nss(GPS_TX_PIN, 255);// TXD to digital pin 6

/* Compass */
QMC5883LCompass mag; //Max

GeoLoc checkGPS() {
  Serial.println("Reading onboard GPS: ");
  bool newdata = false;
  unsigned long start = millis();
  while (millis() - start < GPS_UPDATE_INTERVAL) {
    if (feedgps())
      newdata = true;
  }
  if (newdata) {
    return gpsdump(gps);
  }

  GeoLoc coolerLoc;
  coolerLoc.lat = 0.0;
  coolerLoc.lon = 0.0;
  
  return coolerLoc;
}

// Get and process GPS data
GeoLoc gpsdump(TinyGPSPlus &gps) {
  float flat, flon;
  
  GeoLoc coolerLoc;
  coolerLoc.lat = gps.location.lat();
  coolerLoc.lon = gps.location.lng();

  Serial.print(coolerLoc.lat, 7); Serial.print(", "); 
  Serial.println(coolerLoc.lon, 7);

  return coolerLoc;
}

// Feed data as it becomes available 
bool feedgps() {
  while (nss.available()) {
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}

//enabling joystick
bool isButtonPressed;
BLYNK_WRITE(V5) {
  int buttonState = param.asInt();
  if(buttonState == 1){
    isButtonPressed = true;
    Serial.print("Joystick enabled");
  } else {
    isButtonPressed = false;
    Serial.print("Joystick disabled");
  }
}

//Joystick
BLYNK_WRITE(V0) {
  if (!isButtonPressed) {
    return;
  }

    int x = param[0].asInt(); // x-coordinate of joystick
    int y = param[1].asInt(); // y-coordinate of joystick

    // Set speeds to 0 if y < 128
    if (y < 128) {
      analogWrite(MOTOR_A_EN_PIN, 0);
      analogWrite(MOTOR_B_EN_PIN, 0);
      return;
    }

    // Calculate full speed
    int fullSpeed = y - 128;
    // Constrain fullSpeed to the range [0, 128]
    fullSpeed = constrain(fullSpeed, 0, 128); 

    // Calculate autoSteer values
    float autoSteerA = 1.0;
    float autoSteerB = 1.0;

    if (x < 128) {
      autoSteerA = 1-((128 - x) / 128.0);
    } else if (x > 128) {
      autoSteerB = 1-((x - 128) / 128.0);
    }

    // Calculate speeds for motors A and B
    int speedA = fullSpeed * autoSteerA;
    int speedB = fullSpeed * autoSteerB;

    // Constrain speeds to the range [0, 255]
    speedA = constrain(speedA, 0, 128);
    speedB = constrain(speedB, 0, 128);

    // Set motor speeds
    analogWrite(MOTOR_A_EN_PIN, speedA);
    analogWrite(MOTOR_B_EN_PIN, speedB);

    // Output values to serial
    Serial.print("x: "); Serial.println(x);
    Serial.print("y: "); Serial.println(y);
    Serial.print("fullSpeed: "); Serial.println(fullSpeed);
    Serial.print("autoSteerA: "); Serial.println(autoSteerA);
    Serial.print("autoSteerB: "); Serial.println(autoSteerB);
    Serial.print("speedA: "); Serial.println(speedA);
    Serial.print("speedB: "); Serial.println(speedB);
}

// Killswitch Hook
BLYNK_WRITE(V1) {
  
  int buttonState = param[0].asInt();
  if(buttonState == 1){
    enabled = true;
    Serial.print("autodrive enabled");
  } else {
    enabled = false;
    Serial.print("autodrive disabled");
    //Stop the wheels
    stop();  
        
  }
}
 
//displayCompassDetails
BLYNK_WRITE(V4) {
  displayCompassDetails();
}

// GPS Streaming Hook
BLYNK_WRITE(V2) {
  nss.listen();
  GeoLoc coolerLoc = checkGPS();
  bluetoothSerial.listen();
  GeoLoc phoneLoc;
  phoneLoc.lat = param[0].asFloat();
  phoneLoc.lon = param[1].asFloat();
  Serial.print("phoneLoc.lat: "); Serial.print(phoneLoc.lat,7);
  Serial.print(" , "); Serial.print("phoneLoc.lon: "); 
  Serial.println(phoneLoc.lon,7); 
  Serial.print("distancev2: "); 
  Serial.println(geoDistance(coolerLoc,phoneLoc));  
  if(enabled == true && geoDistance(coolerLoc,phoneLoc) > 10){
    driveTo(phoneLoc, GPS_STREAM_TIMEOUT);
  }
}

// Terminal Hook
BLYNK_WRITE(V3) {
  Serial.print("Received Text: ");
  Serial.println(param.asStr());

  String rawInput(param.asStr());
  int colonIndex;
  int commaIndex;
  
  do {
    commaIndex = rawInput.indexOf(',');
    colonIndex = rawInput.indexOf(':');
    
    if (commaIndex != -1) {
      String latStr = rawInput.substring(0, commaIndex);
      String lonStr = rawInput.substring(commaIndex+1);

      if (colonIndex != -1) {
         lonStr = rawInput.substring(commaIndex+1, colonIndex);
      }
    
      float lat = latStr.toFloat();
      float lon = lonStr.toFloat();
    
      if (lat != 0 && lon != 0) {
        GeoLoc waypoint;
        waypoint.lat = lat;
        waypoint.lon = lon;
    
        Serial.print("Waypoint found: "); Serial.print(lat); 
        Serial.println(lon);
        driveTo(waypoint, GPS_WAYPOINT_TIMEOUT);
      }
    }
    
    rawInput = rawInput.substring(colonIndex + 1);
    
  } while (colonIndex != -1);
}

void displayCompassDetails(void)
{

  char myArray[3];
  Serial.println("------------------------------------");
  mag.read();
  Serial.print  ("GetX:    "); Serial.print(mag.getX()); 
  Serial.println(" uT");
  Serial.print  ("GetY:    "); Serial.print(mag.getY()); 
  Serial.println(" uT");
  Serial.print  ("GetZ:    "); Serial.print(mag.getZ()); 
  Serial.println(" uT");
  Serial.print  ("Azimuth:   "); Serial.print(mag.getAzimuth());
  Serial.println(" degrees");
  Serial.print  ("Bearing:   "); 
  Serial.print(mag.getBearing(mag.getAzimuth())); 
  Serial.println(" ");
  mag.getDirection(myArray,mag.getAzimuth());  
  Serial.print  ("Direction:   "); Serial.print(myArray[0]); 
  Serial.print(myArray[1]); Serial.print(myArray[2]);
  Serial.println("------------------------------------");
  Serial.println("");

  delay(500);
  
}

#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif
//coolerloc, loc
float geoBearing(struct GeoLoc &a, struct GeoLoc &b) {
  float y = sin(b.lon-a.lon) * cos(b.lat);
  float x = cos(a.lat)*sin(b.lat) - 
    sin(a.lat)*cos(b.lat)*cos(b.lon-a.lon);
  return atan2(y, x) * RADTODEG;
}
//a = cooler, b = phone
float geoDistance(struct GeoLoc &a, struct GeoLoc &b) {
  const float R = 6371000; // km
  float p1 = a.lat * DEGTORAD;
  float p2 = b.lat * DEGTORAD;
  float dp = (b.lat-a.lat) * DEGTORAD;
  float dl = (b.lon-a.lon) * DEGTORAD;

  float x = sin(dp/2) * sin(dp/2) + 
    cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));

  return R * y;
}

float geoHeading() {
  mag.read();//Max

  float heading = mag.getAzimuth();


  // Offset
  heading -= -COMPASS_OFFSET;
  
  // Correct for when signs are reversed.
  if(heading < 0){
    heading += 360;
  }
    
  // Check for wrap due to addition of declination.
  if(heading > 360){
    heading -= 360;
  }
    
  // Map to -180 - 180
  while (heading < -180) {
      heading += 360;

  }
  while (heading >  180){
      heading -= 360;
  } 

  //return headingDegrees;
  return heading;
}

void setSpeedMotorA(int speed) {
  digitalWrite(MOTOR_A_IN_1_PIN, HIGH); //Max
  digitalWrite(MOTOR_A_IN_2_PIN, HIGH); //Max
  
  // set speed to 200 out of possible range 0~255
  analogWrite(MOTOR_A_EN_PIN, speed + MOTOR_A_OFFSET);
}

void setSpeedMotorB(int speed) {
  digitalWrite(MOTOR_B_IN_1_PIN, HIGH); //Max
  digitalWrite(MOTOR_B_IN_2_PIN, HIGH); //Max
  
  // set speed to 200 out of possible range 0~255
  analogWrite(MOTOR_B_EN_PIN, speed + MOTOR_B_OFFSET);
}

void setSpeed(int speed){
  // this function will run the motors in both directions 
  // at a fixed speed
  // turn on motor A
  setSpeedMotorA(speed);

  // turn on motor B
  setSpeedMotorB(speed);
}

void stop() {
  // now turn off motors
  digitalWrite(MOTOR_A_IN_1_PIN, LOW); //Max
  digitalWrite(MOTOR_A_IN_2_PIN, LOW); //Max
  digitalWrite(MOTOR_B_IN_1_PIN, LOW); //Max
  digitalWrite(MOTOR_B_IN_2_PIN, LOW); //Max
  analogWrite(MOTOR_A_EN_PIN, 0);
  analogWrite(MOTOR_B_EN_PIN, 0);
  Serial.println("Motors stopped!");//Max
}

void drive(int distance, float turn) {

  int fullSpeed = 50;
  int stopSpeed = 0;

  // drive to location
  int s = fullSpeed;
  if ( distance < 15 ) {
    int wouldBeSpeed = s - stopSpeed;
    wouldBeSpeed *= distance / 15.0f;
    s = stopSpeed + wouldBeSpeed;
  }
  
  int autoThrottle = constrain(s, stopSpeed, fullSpeed);
  autoThrottle = 50;

  float t = turn;
  while (t < -180) t += 360;
  while (t >  180) t -= 360;
  
  Serial.print("turn: ");
  Serial.println(t);
  Serial.print("original: ");
  Serial.println(turn);

  float t_modifier = (180.0 - abs(t)) / 180.0;
  float autoSteerA = 1;
  float autoSteerB = 1;

  if (t > 0) {
    autoSteerB = t_modifier;
  } else if (t < 0){
    autoSteerA = t_modifier;
  }

  Serial.print("steerA: "); Serial.println(autoSteerA);
  Serial.print("steerB: "); Serial.println(autoSteerB);

  int speedA = (int) (((float) autoThrottle) * autoSteerA);
  int speedB = (int) (((float) autoThrottle) * autoSteerB);
  
  setSpeedMotorA(speedA);
  setSpeedMotorB(speedB);
}

void driveTo(struct GeoLoc &loc, int timeout) {
  nss.listen();
  GeoLoc coolerLoc = checkGPS();
  bluetoothSerial.listen();


  if (coolerLoc.lat != 0 && coolerLoc.lon != 0 && 
    enabled==true && geoDistance(coolerLoc,loc) > 10.0) {
    float d = 0;
    //Start move loop here
    do{
      nss.listen();
      coolerLoc = checkGPS();
      bluetoothSerial.listen();
      
      d = geoDistance(coolerLoc, loc);
      float t = geoBearing(coolerLoc, loc) - geoHeading();

      Serial.print("Remote gps: "); Serial.print(loc.lat, 7); 
      Serial.print(", "); Serial.println(loc.lon, 7);
      Serial.print("Onboard gps: "); 
      Serial.print(coolerLoc.lat, 7); 
      Serial.print(", "); Serial.println(coolerLoc.lon, 7);
      
      Serial.print("Distance: ");
      Serial.println(geoDistance(coolerLoc, loc));
    
      Serial.print("Bearing: ");
      Serial.println(geoBearing(coolerLoc, loc));

      Serial.print("heading: ");
      Serial.println(geoHeading());
      
      drive(d, t);
      timeout -= 1;
    } while (d > 10.0 && enabled == true && timeout>0);

    stop();
    return;
  }
  return;
}

void setupCompass() {
  mag.init();
  mag.setCalibration(-1401, 803, -1243, 1028, -543, 561);
  displayCompassDetails();
}

void setup()
{
  // Motor pins
  pinMode(MOTOR_A_EN_PIN, OUTPUT);
  pinMode(MOTOR_B_EN_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_1_PIN, OUTPUT); //Max
  pinMode(MOTOR_A_IN_2_PIN, OUTPUT); //Max
  pinMode(MOTOR_B_IN_1_PIN, OUTPUT); //Max
  pinMode(MOTOR_B_IN_2_PIN, OUTPUT); //Max
  
  pinMode(LED_BUILTIN, OUTPUT);

  //Debugging via serial
  Serial.begin(4800);

  Serial.println("This code is doing something in setup");

  //GPS
  nss.begin(9600);
  //Bluetooth
  bluetoothSerial.begin(9600);
  Blynk.begin(bluetoothSerial, auth);
  
  setupCompass();

  Serial.println("This code has gone through setup");
}

// Testing
void testDriveNorth() {
  float heading = geoHeading();
  int testDist = 10;
  Serial.println(heading);
  
  while(!(heading < 5 && heading > -5)) {
    drive(testDist, heading);
    heading = geoHeading();
    Serial.println(heading);
    delay(500);
  }
  
  stop();
}

void loop()
{
  Blynk.run();
  if (enabled == true){
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else digitalWrite(LED_BUILTIN, LOW);
}