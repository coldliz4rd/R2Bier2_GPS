#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>

// #####################################################
// ##### Konfiguration für serielle Schnittstellen #####
// #####################################################

// Software Serial für GPS Moduls
// #define GPS_SS
#ifdef GPS_SS
  #include 
  #define GPS_RXPIN 0
  #define GPS_TXPIN 0
#endif

// Software Serial für Kommunikation
// #define COM_SS
#ifdef COM_SS
  #define COM_RXPIN 0
  #define COM_TXPIN 0
#endif

// Serielle Konfiguration für GPS und Jetson

#define COMBAUD 115200   // Baudrate für Seriellen Monitor
#define GPSBAUD 115200   // Baudrate des GPS Moduls
#define COMSERIAL Serial // Serial Port für die Kommunikation mit dem Jetson
#define GPSSERIAL Serial2 // Serial Port für die Kommunikation mit dem GPS Modul

// I2C Konfiguration für den Kompass

#define KOMPASS_SDA 21
#define KOMPASS_SCL 22
#define KOMPASS_FREQ 100000U
#define KOMPASS_ADDR 0x60

// Auskommentierung entfernen für Debug-Modus
// #define DEBUG

// Kurs auswählen. Immer nur einen Kurs definieren.
#define WAYPOINTS_WHS

#define DIRECTION false // clockwise = false; counter-clockwise = true
#define TOLERANCE_RADIUS 10U

struct wayPoint_t
{
  double lat;
  double lng;
};

#ifdef WAYPOINTS_WHS
static wayPoint_t wayPoints[] = {
  {51.83980, 6.652146},
  {51.83990, 6.652132},
  {51.84003, 6.652134},
  {51.84009, 6.652165},
  {51.84014, 6.652272},
  {51.84020, 6.652412},
  {51.84014, 6.652814},
  {51.84019, 6.653853},
  {51.84015, 6.653983},
  {51.84007, 6.654034},
  {51.83940, 6.653982},
  {51.83939, 6.653805},
  {51.83937, 6.653951},
  {51.83936, 6.653604},
  {51.83937, 6.653399},
  {51.83936, 6.653194},
  {51.83936, 6.653089},
  {51.83939, 6.652949},
  {51.83936, 6.652746},
  {51.83939, 6.652561},
  {51.83937, 6.652397},
  {51.83940, 6.652221},
  {51.83947, 6.652192},
  {51.83980, 6.652146}
};
#endif

uint8_t curWP = 0;
const uint8_t wpCnt = sizeof(wayPoints) / sizeof(wayPoints[0]);

uint8_t findFirstWaypoint(wayPoint_t wayPoints[], TinyGPSLocation curLocation, bool dirCCW = false);
double compassBearing(byte compassAddr);
void checkCurrentWaypoint(uint8_t *currentWaypoint, TinyGPSPlus gpsInstance, wayPoint_t waypointArray[], double distanceTolerance, bool directionCCW = false);

TinyGPSPlus gps;

void setup() {
  // Serielle Schnittstelle für Kommunikation mit Monitor oder Jetson starten
  COMSERIAL.begin(COMBAUD);

  // Serielle Schnittstelle für Kommunikation mit GPS Module starten
  GPSSERIAL.begin(GPSBAUD);

  // I2C Kommunikation mit Kompass starten
  while (!Wire.begin(KOMPASS_SDA, KOMPASS_SCL, KOMPASS_FREQ));

  while (!gps.location.isValid()) {
    while (GPSSERIAL.available() > 0)
      gps.encode(GPSSERIAL.read());
  }

  curWP = findFirstWaypoint(wayPoints, gps.location, DIRECTION);
}

void loop() {
  // Seriellen Datenstrom vom GPS lesen und verarbeiten
  while (GPSSERIAL.available() > 0)
    gps.encode(GPSSERIAL.read());
}

uint8_t findFirstWaypoint(wayPoint_t wayPoints[], TinyGPSLocation curLocation, bool dirCCW) {
  uint8_t i = 0;
  double dist2Cur = 0.0;
  double dist2Next = 0.0;
  double distBetw = 0.0;
  uint8_t wpCnt = sizeof(wayPoints) / sizeof(wayPoints[0]);
  
  while (i <= wpCnt) {
    dist2Cur = gps.distanceBetween(wayPoints[i].lat, wayPoints[i].lng, curLocation.lat(), curLocation.lng());
    dist2Next = gps.distanceBetween(wayPoints[i+1].lat, wayPoints[i+1].lng, curLocation.lat(), curLocation.lng());
    distBetw = gps.distanceBetween(wayPoints[i].lat, wayPoints[i].lng, wayPoints[i+1].lat, wayPoints[i+1].lng);

    if ((dist2Cur < distBetw) and (dist2Next < distBetw)) {
      if (dirCCW) {
        return i;
      }
      else {
        return i+1;
      }
    }

    i += 1;
  }
}

void checkCurrentWaypoint(uint8_t *currentWaypoint, TinyGPSPlus gpsInstance, wayPoint_t waypointArray[], double distanceTolerance, bool directionCCW) {
  double distanceToNext = gpsInstance.distanceBetween(waypointArray[curWP].lat, waypointArray[curWP].lng, gpsInstance.location.lat(), gpsInstance.location.lng());
  uint8_t waypointCount = sizeof(currentWaypoint) / sizeof(currentWaypoint[0]);

  if (distanceToNext <= distanceTolerance) {
    if ((!directionCCW) && (*currentWaypoint < waypointCount) *currentWaypoint += 1;
    else if ((directionCCW) && (*currentWaypoint > 0)) -= 1;
    else if ((!directionCCW) && (*currentWaypoint = waypointCount)) *currentWaypoint = 0;
    else if ((directionCCW) && (*currentWaypoint = waypointCount)) *currentWaypoint = waypointCount;
  }

}

double compassBearing(byte compassAddr) {
  byte highByte;
  byte lowByte;

  Wire.beginTransmission(compassAddr);    //starts communication with cmps03
  Wire.write(2);                          //Sends the register we wish to read
  Wire.endTransmission();

  Wire.requestFrom(compassAddr, (int)2);  //requests high byte
  while(Wire.available() < 2);            //while there is a byte to receive
  highByte = Wire.read();                 //reads the byte as an integer
  lowByte = Wire.read();
  double bearing = ((highByte<<8)+lowByte)/10.0; 

  return bearing;
}