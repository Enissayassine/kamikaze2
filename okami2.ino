//-------------------------------------------------------------------------------
//  TinyCircuits GPS Tracker Tutorial Program
//  Last updated 27 February 2017 (1.02)
//  
//  Using the GPS TinyShield, the Flash Memory TinyShield, and the TinyDuino,
//  this program turns the stack into a miniature GPS tracker and data logger.
//  The code detects which sentence is being read and formats the string accordingly.
//  In order to reduce the number of writes, we write one NMEA sentence per 10 seconds, 
//  which can be modified.
//
//  With the Telit SE868 V2 module with Glonass support, some messages come through
//  as GN** sentences instead of GP**. These are changed back to GP** before logging
//  so that they don't cause problems with programs like Google Earth.
//  Some GPS modules have been shipped with 4800 baud instead of 9600- try this if
//  you see bad data.
//
//  The Software Serial library should be modified for a larger buffer- 256 is enough
//  for GGA and RMC sentences at 1Hz. In SoftwareSerial.cpp, the change looks like:
//  #define _SS_MAX_RX_BUFF 256
//
//  Written by Ben Rose & Lilith Freed for TinyCircuits, http://TinyCircuits.com
//
//-------------------------------------------------------------------------------

#include <stdio.h>
//This may need to be set to 4800 baud
const int GPSBaud = 9600;

#include "SoftwareSerial256.h"

// Include et define pour le compass
#include <Wire.h>
#define Addr 0x1E               // 7-bit address of HMC5883 compass

// The Arduino pins used by the GPS module
const uint8_t GPS_ONOFFPin = A3;
const uint8_t GPS_SYSONPin = A2;
const uint8_t GPS_RXPin = A1;
const uint8_t GPS_TXPin = A0;
const uint8_t chipSelect = 10;

// Stockage des localisations
float locs[1][2] = { 48.9170688, 2.3240448 };
double  angle = 0;
int is_r = 0;

// The GPS connection is attached with a software serial port
SoftwareSerial Gps_Serial(GPS_RXPin, GPS_TXPin);

// Set which sentences should be enabled on the GPS module
// GPGGA - 
char nmea[] = {'1'/*GPGGA*/, '0'/*GNGLL*/, '0'/*GNGSA*/, '0'/*GPGSV/GLGSV*/, '1'/*GNRMC*/, '0'/*GNVTG*/, '0'/*not supported*/, '0'/*GNGNS*/};

void setup()
{
	Gps_Serial.begin(GPSBaud);
	delay(500);
	Serial.begin(115200);

	// Allumage du compass
	Wire.begin();
	Wire.beginTransmission(Addr); 
	Wire.write(byte(0x02));
	Wire.write(byte(0x00));
	Wire.endTransmission();

	Serial.println("Now initiating write mode.");
	Serial.println();

	// Init the GPS Module to wake mode
	pinMode(GPS_SYSONPin, INPUT);
	digitalWrite(GPS_ONOFFPin, LOW);
	pinMode(GPS_ONOFFPin, OUTPUT);
	delay(100);
	Serial.print("Attempting to wake GPS module.. ");
	while (digitalRead( GPS_SYSONPin ) == LOW )
	{
		// Need to wake the module
		digitalWrite( GPS_ONOFFPin, HIGH );
		delay(5);
		digitalWrite( GPS_ONOFFPin, LOW );
		delay(100);
	}
	Serial.println("done.");
	delay(100);

	char command[] = "$PSRF103,00,00,00,01*xx\r\n";
	for (int i = 0; i < 8; i++) {
		command[10] = i + '0';
		command[16] = nmea[i];
		int c = 1;
		byte checksum = command[c++];
		while (command[c] != '*')
			checksum ^= command[c++];
		command[c + 1] = (checksum >> 4) + (((checksum >> 4) < 10) ? '0' : ('A' - 10));
		command[c + 2] = (checksum & 0xF) + (((checksum & 0xF) < 10) ? '0' : ('A' - 10));
		Gps_Serial.print(command);
		delay(20);
	}

	Serial.println();

}

// int necessaires pour le fonctionnement du compass
int x_max=-10000;  // Starting values for hard iron calibration
int y_max=-10000;  // We want these values to be extreme in the 
int x_min=10000;   // opposite direction so it calibrates nicely
int y_min=10000;

void loop() {
  int Degrees = 0;
  float heading = 0;
  int LED = 0;
	unsigned long startTime = millis();

	while (Gps_Serial.read() != '$') {
		//do other stuff here
	}
	while (Gps_Serial.available() < 5);
	Gps_Serial.read(); 
	Gps_Serial.read(); //skip two characters
	char c = Gps_Serial.read();
	Serial.write(c);
	//determine senetence type
	if (c == 'R' || c == 'G') {
		c = Gps_Serial.read();
		if (c == 'M') {
			logNMEA(1);
		} else if (c == 'G') {
			logNMEA(2);
		}
	}

	// Waits 10 seconds before reading next NMEA string
	//while (millis() - startTime < 5000) {
		//Gps_Serial.read(); // clears GPS serial buffer
  	// Communication avec le compass
  	int x, y, z;
  	// Initiate communications with compass
  	Wire.beginTransmission(Addr);
  	Wire.write(byte(0x03));       // Send request to X MSB register
  	Wire.endTransmission();
  
  	Wire.requestFrom(Addr, 6);    // Request 6 bytes; 2 bytes per axis
  	if(Wire.available() <=6) {    // If 6 bytes available
  		x = Wire.read() << 8 | Wire.read();
  		z = Wire.read() << 8 | Wire.read();
  		y = Wire.read() << 8 | Wire.read();
  	}
  	if(x > x_max) //Find values of hard iron distortion
  		x_max = x;  //This will store the max and min values
  	if(y >y_max)	//of the magnetic field around you
  		y_max = y;
  	if(y<y_min)
  		y_min = y;
  	if(x<x_min)
  		x_min = x;
    int xoffset= (x_max+x_min)/2;
   	int yoffset= (y_max+y_min)/2;
  	
  	int x_scale = x-xoffset; // Math to compensate for hard 
  	int y_scale = y-yoffset; // iron distortions
  
  	// Heading in radians
  	heading = atan2(x_scale,y_scale); 
  	//Heading between 0 and 6.3 radians
  	if(heading < 0)
  		heading += 2*PI;
  	if(heading>2*PI)
  		heading -= 2*PI;
  	//Conversion to degrees 
    Serial.println("angle = "); 
    Serial.println(angle, 9);
    Serial.println("heading = ");
    Serial.println(heading, 9);
  	Degrees = fmod(((heading * 180/M_PI) - angle),360); 
    if (Degrees < 0)
    {
      Degrees = 360 - Degrees;
    }
    Serial.println("degrees = ");
    Serial.println(Degrees, 9);
  	LED = Degrees/17; //Led shield has 21 Leds. Dividing 360 by

  	//17 will give us values from 0 to 21
  	if (LED==0)	 		//since there is no Led 0, we will turn	
  		LED=21;				//Led 21 on instead
  	LedOn(LED);
  //}
  //delay(10000);
  //Serial.print("\nHeading (degrees): "); 
  //Serial.print(Degrees);
  //Serial.print("\nHeading: "); 
  //Serial.print(heading * 180/M_PI);
  //Serial.print("\nangle in deg: "); 
  //Serial.print(angle * 180/M_PI);
  //Serial.print("\nangle: "); 
  //Serial.print(angle);
  //Serial.print("\nLED: "); 
  //Serial.println(LED);
	delay(400);
}

void logNMEA(uint8_t type) {
	uint8_t buffer[82];
	// Initializes buffer to null terminators to ensure proper writing to flash memory
	for(int i = 0; i < 82; ++i) {
		buffer[i] = '\0';
	}

	// Writes NMEA string to buffer
	buffer[0] = '$';
	int counter = 1;
	char c = 0;
	while (!Gps_Serial.available());
	c = Gps_Serial.read();
	while (c != '*') {
		buffer[counter++] = c;
		while (!Gps_Serial.available());
		c = Gps_Serial.read();
	}
	buffer[counter++] = c;
	while (!Gps_Serial.available());
	c = Gps_Serial.read();
	buffer[counter++] = c;
	while (!Gps_Serial.available());
	c = Gps_Serial.read();
	buffer[counter++] = c;
	buffer[counter++] = '\r';
	buffer[counter++] = '\n';
 
	buffer[2] = 'P'; // Changes GNRMC to GPRMC

	c = 1;
	byte checksum = buffer[c++];
	while (buffer[c] != '*')
		checksum ^= buffer[c++];
	buffer[c + 1] = (checksum >> 4) + (((checksum >> 4) < 10) ? '0' : ('A' - 10));
	buffer[c + 2] = (checksum & 0xF) + (((checksum & 0xF) < 10) ? '0' : ('A' - 10));
  Serial.print("GPS pos \n");
	Serial.write(buffer, 82);
	Serial.print('\n');

	double  lon = 0;
	double  lati = 0;
	
  //char str[] = "$AP161309.000,4853.7407,N,00211.2042,E,1,06,1.5,30.3,M,47.3,M,,0000*01";
  //char str[] = "$CP175510.000,V,4853.7272,N,00211.2113,E,0.71,143.78,171118,,,E*07";

  //Serial.println("str : ");
  //Serial.println(str);
	is_r = 0;
	lati = latitude((char*)buffer);
	lon = longitude((char*)buffer);
   //lati = latitude(str);
  //lon = longitude(str);
  Serial.println(lati, 8);
  Serial.println(lon, 8);
  Serial.println(locs[0][0], 8);
  Serial.println(locs[0][1], 8);
  angle = get_angle(to_radians(lati), to_radians(lon), to_radians(locs[0][0]), to_radians(locs[0][1]));
  Serial.println(angle);
}


double  conv_coords(float in_coords)
{ 
  //Serial.print("in coords  : ");
  //Serial.println(in_coords);
	//Initialize the location.
	double f = in_coords;
	// Get the first two digits by turning f into an integer, then doing an integer divide by 100;
	// firsttowdigits should be 77 at this point.
	int firsttwodigits = ((int)f)/100; //This assumes that f < 10000.
	double nexttwodigits = f - (double)(firsttwodigits*100);
	double theFinalAnswer = (double)(firsttwodigits + nexttwodigits/60.0);
	return theFinalAnswer;
}

double  get_angle(float lat1, float lon1, float lat2, float lon2)
{
	double  angle2;
	double  dLon;
	double  y;
	double  x;

  
	dLon = (lon2 - lon1);
 /*Serial.println("dLon\n");
  Serial.println(dLon);
  Serial.println();*/
	y = sin(dLon) * cos(lat2);
	x = (cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2)) * cos(dLon);
  /*Serial.println("x\n");
  Serial.println(x);
  Serial.println();
  Serial.println("y\n");
  Serial.println(y);
  Serial.println();*/
	angle2 = atan2(y, x);
 /*Serial.println("angle juste apres atan2\n");
  Serial.println(angle2);
  Serial.println();*/
	angle2 = to_degrees(angle2);
  /*Serial.println("angle juste apres todegrees\n");
  Serial.println(angle2);
  Serial.println();*/
	angle2 = fmod((angle2 + 360), 360);
  //Serial.println("angle juste apres fmod\n");
  //Serial.println(angle2);
  //Serial.println();
	//angle2 = fabs(360 - angle2);
  /*Serial.println("angle juste apres fqbs 180 - angle\n");
  Serial.println(angle2);
  Serial.println();*/
	return (angle2);
}

// fonction pour allumer la LED correspondante
void LedOn(int ledNum)
{
	for(int i=4;i<10;i++)
	{
		pinMode(i, INPUT);
		digitalWrite(i, LOW);
	};
	if(ledNum<1 || ledNum>21) return;
	char highpin[21]={6,7,6,8,5,8,8,7,9,7,9,8,5,9,6,9,9,5,6,5,7};
	char lowpin[21]= {7,6,8,6,8,5,7,8,7,9,8,9,9,5,9,6,4,6,5,7,5};

	ledNum--;
	digitalWrite(highpin[ledNum],HIGH);
	digitalWrite(lowpin[ledNum],LOW);
	pinMode(highpin[ledNum],OUTPUT);
	pinMode(lowpin[ledNum],OUTPUT);
}

double  longitude(char *string)
{
  int i;
  int j;
  int vir;
  char  lon[11];

  i = 0;
  vir = 0;
  while (string[i] && vir < 3 + is_r)
  {
    if (string[i++] == ',')
      vir++;
  }
  j = 0;
  while (string[i] != ',')
    lon[j++] = string[i++];
  while (j < 10)
    lon[j++] = '0';
  lon[j] = '\0';
  return (conv_coords(atof(lon)));
}

double latitude(char *string)
{
  int i;
  int j;
  char  lat[11];

  i = 0;
//  Serial.print("i : ");
  while (string[i] && string[i] != ',')
  {
   // Serial.print(" i : ");
   // Serial.print(string[i]);
    i++;
  }
  i++;
  if (isalpha(string[i]))
  {
  //   Serial.print("IS ALPHA : ");
    // Serial.println(string[i]);
      is_r = 1;
    while (string[i] && string[i] != ',')
    {
     // Serial.print(" i : ");
     // Serial.print(string[i]);
      i++;
    }
    i++;
  }
  //if (!(lat = (char*)malloc(sizeof(char) * 10)))
  //  return (0);
  j = 0;
  //  Serial.println("j :");
  while (string[i] != ',')
  {
    lat[j++] = string[i++];
   // Serial.print(lat[j]);
  }
  while (j < 10)
  {
    lat[j++] = '0';
   // Serial.print(lat[j]);
  }
  lat[j] = '\0';
 // Serial.println("");
 // Serial.println("end j");
  return (conv_coords(atof(lat)));
}

double to_degrees(double radians) {
  return (radians * (180.0 / M_PI));
}

double  to_radians(double degrees) 
{
  return (degrees * M_PI / 180.0);
}

