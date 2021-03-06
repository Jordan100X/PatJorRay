#pragma region Code
/******************************************************************************

GeoCache Hunt Project (GeoCache.cpp)

This is skeleton code provided as a project development guideline only.  You
are not required to follow this coding structure.  You are free to implement
your project however you wish.

List Team Members Here:
	Team #6
1. Jordan Sanderson
2. Patrick Burns
3. Raymond Lewandowski


NOTES:

You only have 32k of program space and 2k of data space.  You must
use your program and data space wisely and sparingly.  You must also be
very conscious to properly configure the digital pin usage of the boards,
else weird things will happen.

The Arduino GCC sprintf() does not support printing floats or doubles.  You should
consider using sprintf(), dtostrf(), strtok() and strtod() for message string
parsing and converting between floats and strings.

The GPS provides latitude and longitude in degrees minutes format (DDDMM.MMMM).
You will need convert it to Decimal Degrees format (DDD.DDDD).  The switch on the
GPS Shield must be set to the "Soft Serial" position, else you will not receive
any GPS messages.

*******************************************************************************

Following is the GPS Shield "GPRMC" Message Structure.  This message is received
once a second.  You must parse the message to obtain the parameters required for
the GeoCache project.  GPS provides coordinates in Degrees Minutes (DDDMM.MMMM).
The coordinates in the following GPRMC sample message, after converting to Decimal
Degrees format(DDD.DDDDDD) is latitude(23.118757) and longitude(120.274060).  By
the way, this coordinate is GlobaTop Technology in Tiawan, who designed and
manufactured the GPS Chip.

"$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C/r/n"

$GPRMC,         // GPRMC Message
064951.000,     // utc time hhmmss.sss
A,              // status A=data valid or V=data not valid
2307.1256,      // Latitude 2307.1256 (degrees minutes format dddmm.mmmm)
N,              // N/S Indicator N=north or S=south
12016.4438,     // Longitude 12016.4438 (degrees minutes format dddmm.mmmm)
E,              // E/W Indicator E=east or W=west
0.03,           // Speed over ground knots
165.48,         // Course over ground (decimal degrees format ddd.dd)
260406,         // date ddmmyy
3.05,           // Magnetic variation (decimal degrees format ddd.dd)
W,              // E=east or W=west
A               // Mode A=Autonomous D=differential E=Estimated
*2C             // checksum
/r/n            // return and newline

******************************************************************************/

// Required
#include <Adafruit_NeoPixel.h>
#include "Arduino.h"

/*
Configuration settings.

These defines makes it easy to enable/disable certain capabilities
during the development and debugging cycle of this project.  There
may not be sufficient room in the PROGRAM or DATA memory to enable
all these libraries at the same time.  You are only permitted to
have NEO_ON, GPS_ON and SDC_ON during the actual GeoCache Treasure
Hunt.
*/
#define NEO_ON 1		// NeoPixelShield
#define TRM_ON 1		// SerialTerminal
#define ONE_ON 0		// 1Sheeld
#define SDC_ON 1		// SecureDigital
#define GPS_ON 1		// GPSShield (off = simulated)

// define pin usage
#define NEO_TX	6		// NEO transmit
#define GPS_TX	7		// GPS transmit
#define GPS_RX	8		// GPS receive

// GPS message buffer
#define GPS_RX_BUFSIZ	128
char cstr[GPS_RX_BUFSIZ];

// variables
uint8_t target = 0;
float distance = 0.0, heading = 0.0;
int sdChip = 10;
int button = 3;

struct TargetLocale
{
	float lon;
	float lat;
};

TargetLocale targets[4];

#if GPS_ON
#include "SoftwareSerial.h"
SoftwareSerial gps(GPS_RX, GPS_TX);
#endif

#if NEO_ON

#include "Adafruit_NeoPixel.h"
Adafruit_NeoPixel strip = Adafruit_NeoPixel(40, NEO_TX, NEO_GRB + NEO_KHZ800);
#endif

#if ONE_ON
#define CUSTOM_SETTINGS
#define INCLUDE_TERMINAL_SHIELD
#include "OneSheeld.h"
#endif

#if SDC_ON
#include <SD.h>
File dataFile;
#endif

/*
Following is a Decimal Degrees formatted waypoint for the large tree
in the parking lot just outside the front entrance of FS3B-116.
*/
#define GEOLAT0 28.594532
#define GEOLON0 -81.304437

#if GPS_ON
/*
These are GPS command messages (only a few are used).
*/
#define PMTK_AWAKE "$PMTK010,002*2D"
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_Q_RELEASE "$PMTK605*31"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"
#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_CMD_HOT_START "$PMTK101*32"
#define PMTK_CMD_WARM_START "$PMTK102*31"
#define PMTK_CMD_COLD_START "$PMTK103*30"
#define PMTK_CMD_FULL_COLD_START "$PMTK104*37"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
#define PMTK_SET_NMEA_OUTPUT_RMC "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_GGA "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

#endif // GPS_ON

float CourseOverGround = 0;
/*************************************************
**** GEO FUNCTIONS - BEGIN ***********************
*************************************************/
#pragma region GEO Functions
/**************************************************
Convert Degrees Minutes (DDMM.MMMM) into Decimal Degrees (DDD.DDDD)

float degMin2DecDeg(char *cind, char *ccor)

Input:
cind = string char pointer containing the GPRMC latitude(N/S) or longitude (E/W) indicator
ccor = string char pointer containing the GPRMC latitude or longitude DDDMM.MMMM coordinate

Return:
Decimal degrees coordinate.

**************************************************/
float degMin2DecDeg(char *cind, char *ccor)
{
	float degrees = 0.0;

	float degMins = atof(ccor);  //Char to float

	int deg = (int)degMins / 100; // Taking out the degrees

	float mins = degMins;
	mins -= deg * 100; // Setting minutes

	float secs;
	secs = mins / 60; // Setting seconds

	degrees = deg + (mins / 60) + (secs / 3600);

	if (cind[0] == 'W' || cind[0] == 'S')
		degrees *= -1;


	return(degrees);
}

/**************************************************
Calculate Great Circle Distance between to coordinates using
Haversine formula.

float calcDistance(float flat1, float flon1, float flat2, float flon2)

EARTH_RADIUS_FEET = 3959.00 radius miles * 5280 feet per mile

Input:
flat1, flon1 = first latitude and longitude coordinate in decimal degrees
flat2, flon2 = second latitude and longitude coordinate in decimal degrees

Return:
distance in feet (3959 earth radius in miles * 5280 feet per mile)
**************************************************/
float calcDistance(float flat1, float flon1, float flat2, float flon2)
{
	float distance = 0.0;

	float lat1 = radians(flat1);
	float lat2 = radians(flat2);
	// add code here
	//Difference
	float latitude = radians(flat2 - flat1);
	float longitude = radians(flon2 - flon1);
	// Haversine Fromula

	float a = (sin(latitude / 2.0f) * sin(latitude / 2.0f)) + (cos(lat1) * cos(lat2) * sin(longitude / 2.0f) * sin(longitude / 2.0f));

	float c = 2.0f * atan2(sqrtf(a), sqrtf((1 - a)));

	distance = (6371.0f * 3280.84f) * c;

	return(distance);
}

/**************************************************
Calculate Great Circle Bearing between two coordinates

float calcBearing(float flat1, float flon1, float flat2, float flon2)

Input:
flat1, flon1 = first latitude and longitude coordinate in decimal degrees
flat2, flon2 = second latitude and longitude coordinate in decimal degrees

Return:
angle in degrees from magnetic north
**************************************************/
float calcBearing(float flat1, float flon1, float flat2, float flon2)
{
	float bearing = 0.0;
	float var1 = 0.0;
	float var2 = 0.0;

	// add code here
#if 1 // Initial Bearing
	{
		var1 = sin(radians(flon2 - flon1)) * cos(radians(flat2));
		var2 = cos(radians(flat1)) * sin(radians(flat2)) - sin(radians(flat1)) * cos(radians(flat2)) * cos(radians(flat2)) * cos(radians(flon2 - flon1));

		bearing = degrees(atan2(var1, var2));

		if (bearing < 0)
			bearing = fmod(bearing + 360.0f, 360);
	}
#else  // Final Bearing
	{
		var1 = sin(radians(flon1 - flon2)) * cos(radians(flat1));
		var2 = cos(radians(flat2)) * sin(radians(flat1)) - sin(radians(flat2)) * cos(radians(flat1)) * cos(radians(flat1)) * cos(radians(flon1 - flon2));

		bearing = degrees(atan2(var1, var2));

		bearing = (fmod(bearing + 180, 360));
	}
#endif

	return(bearing);
}

#pragma endregion
/*************************************************
**** GEO FUNCTIONS - END**************************
*************************************************/
enum Direction
{
	up,
	upRight,
	right,
	behind,
	left,
	upleft
};

#if NEO_ON
/*
Sets target number, heading and distance on NeoPixel Display
*/

void setNeoPixel(uint8_t target, float heading, float distance)
{
	
	strip.clear();
	switch (target)
	{
	case 0:
		strip.setPixelColor(0, strip.Color(229, 83, 0));	//orange
		strip.show();
		break;
	case 1:
		strip.setPixelColor(0, strip.Color(0, 255, 0));		//green
		break;
	case 2:
		strip.setPixelColor(0, strip.Color(0, 0, 255));		//Blue
		break;
	case 3:
		strip.setPixelColor(0, strip.Color(255, 0, 255));	//purple
		break;
	default:
		strip.setPixelColor(0, strip.Color(255, 0, 0));		//error red
		break;

	}

	//heading correct direction

	if (distance >= 25)
	{
		strip.setPixelColor(32, strip.Color(255, 0, 0));
	}
	if (distance >= 50)
	{
		strip.setPixelColor(33, strip.Color(255, 0, 0));
	}
	if (distance >= 100)
	{
		strip.setPixelColor(34, strip.Color(255, 0, 0));
	}
	if (distance >= 200)
	{
		strip.setPixelColor(35, strip.Color(255, 0, 0));
	}
	if (distance >= 400)
	{
		strip.setPixelColor(36, strip.Color(255, 0, 0));
	}
	if (distance >= 800)
	{
		strip.setPixelColor(37, strip.Color(255, 0, 0));
	}
	if (distance >= 1600)
	{
		strip.setPixelColor(38, strip.Color(255, 0, 0));
	}
	if (distance >= 3200)
	{
		strip.setPixelColor(39, strip.Color(255, 0, 0));
	}

	int relativeBearing = 0;
	relativeBearing = heading - CourseOverGround;
	if (relativeBearing < 0)
	{
		relativeBearing = relativeBearing + 360;
	}
	else if (relativeBearing > 360)
	{
		relativeBearing = relativeBearing - 360;
	}

	Direction targetdirection;
	//Serial.println("Relative Bearing: " + String(relativeBearing));

	//do relative bearing calculations
	if (relativeBearing <= 5 || relativeBearing > 355)
		targetdirection = Direction::up;

	else if (relativeBearing > 5 && relativeBearing <= 60)
		targetdirection = Direction::upRight;

	else if (relativeBearing > 60 && relativeBearing <= 120)
		targetdirection = Direction::right;

	else if (relativeBearing > 120 && relativeBearing <= 250)
		targetdirection = Direction::behind;

	else if (relativeBearing > 250 && relativeBearing <= 300)
		targetdirection = Direction::left;

	else if (relativeBearing > 300 && relativeBearing <= 355)
		targetdirection = Direction::upleft;


	switch (targetdirection)
	{
	case 0:	//up
		strip.setPixelColor(3, strip.Color(0, 255, 255));
		strip.setPixelColor(4, strip.Color(0, 255, 255));
		strip.setPixelColor(11, strip.Color(0, 255, 255));
		strip.setPixelColor(12, strip.Color(0, 255, 255));
		strip.setPixelColor(19, strip.Color(0, 255, 255));
		strip.setPixelColor(20, strip.Color(0, 255, 255));
		strip.setPixelColor(27, strip.Color(0, 255, 255));
		strip.setPixelColor(28, strip.Color(0, 255, 255));
		break;
	case 1:	//upright
		strip.setPixelColor(6, strip.Color(0, 255, 255));
		strip.setPixelColor(7, strip.Color(0, 255, 255));
		strip.setPixelColor(14, strip.Color(0, 255, 255));
		strip.setPixelColor(15, strip.Color(0, 255, 255));
		strip.setPixelColor(21, strip.Color(0, 255, 255));
		strip.setPixelColor(28, strip.Color(0, 255, 255));
		break;
	case 2:	//right
		strip.setPixelColor(28, strip.Color(0, 255, 255));
		strip.setPixelColor(29, strip.Color(0, 255, 255));
		strip.setPixelColor(30, strip.Color(0, 255, 255));
		strip.setPixelColor(31, strip.Color(0, 255, 255));
		break;
	case 3:	//behind
		strip.setPixelColor(24, strip.Color(0, 255, 255));
		strip.setPixelColor(25, strip.Color(0, 255, 255));
		strip.setPixelColor(26, strip.Color(0, 255, 255));
		strip.setPixelColor(29, strip.Color(0, 255, 255));
		strip.setPixelColor(30, strip.Color(0, 255, 255));
		strip.setPixelColor(31, strip.Color(0, 255, 255));
		break;
	case 4: //left
		strip.setPixelColor(24, strip.Color(0, 255, 255));
		strip.setPixelColor(25, strip.Color(0, 255, 255));
		strip.setPixelColor(26, strip.Color(0, 255, 255));
		strip.setPixelColor(27, strip.Color(0, 255, 255));
		break;
	case 5: //upleft
		strip.setPixelColor(9, strip.Color(0, 255, 255));
		strip.setPixelColor(10, strip.Color(0, 255, 255));
		strip.setPixelColor(17, strip.Color(0, 255, 255));
		strip.setPixelColor(18, strip.Color(0, 255, 255));
		strip.setPixelColor(27, strip.Color(0, 255, 255));
		break;

	}

	strip.show();

}

#endif	// NEO_ON

#if GPS_ON
/*
Get valid GPS message. This function returns ONLY once a second.

void getGPSMessage(void)

Side affects:
Message is placed in global "cstr" string buffer.

Input:
none

Return:
none

*/
void getGPSMessage(void)
{
	uint8_t x = 0, y = 0, isum = 0;
	memset(cstr, 0, sizeof(cstr));

	// get nmea string
	while (true)
	{
		//Serial.println(gps.peek());
		if (gps.peek() != -1)
		{
			cstr[x] = gps.read();
			// if multiple inline messages, then restart
			if ((x != 0) && (cstr[x] == '$'))
			{
				x = 0;
				cstr[x] = '$';
			}

			// if complete message
			if ((cstr[0] == '$') && (cstr[x++] == '\n'))
			{
				// nul terminate string before /r/n
				cstr[x - 2] = 0;

				// if checksum not found
				if (cstr[x - 5] != '*')
				{
					x = 0;
					continue;
				}

				// convert hex checksum to binary
				isum = strtol(&cstr[x - 4], NULL, 16);

				// reverse checksum
				for (y = 1; y < (x - 5); y++) isum ^= cstr[y];

				// if invalid checksum
				if (isum != 0)
				{
					x = 0;
					continue;
				}

				// else valid message
				break;
			}
		}
	}
}

#else
/*
Get simulated GPS message once a second.

This is the same message and coordinates as described at the top of this
file.  You could edit these coordinates to point to the tree out front (GEOLAT0,
GEOLON0) to test your distance and direction calculations.  Just note that the
tree coordinates are in Decimal Degrees format, and the message coordinates are
in Degrees Minutes format.

void getGPSMessage(void)

Side affects:
Static GPRMC message is placed in global "cstr" string buffer.

Input:
none

Return:
none

*/
void getGPSMessage(void)
{
	static unsigned long gpsTime = 0;

	// simulate waiting for message
	while (gpsTime > millis()) delay(100);

	// do this once a second
	gpsTime = millis() + 1000;

	memcpy(cstr, "$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C", sizeof(cstr));

	return;
}

#endif	// GPS_ON

void setup(void)
{
#if TRM_ON
	// init serial interface
	Serial.begin(115200);
#endif	

#if ONE_ON
	// init OneShield Shield
#endif

#if NEO_ON
	// init NeoPixel Shield
	strip.begin();
	strip.setBrightness(10);
	strip.show();
#endif	

#if SDC_ON
	/*
	Initialize the SecureDigitalCard and open a numbered sequenced file
	name "MyMapNN.txt" for storing your coordinates, where NN is the
	sequential number of the file.  The filename can not be more than 8
	chars in length (excluding the ".txt").
	*/

	// see if the card is present and can be initialized:
	if (!SD.begin()) {
		//Serial.println("Card failed, or not present");
	}
	//Serial.println("card initialized.");

	String filename;
	for (size_t i = 0; i < 100; i++)
	{

		if (i < 10)
			filename = "MyFile0" + (String)i + ".txt";
		else
			filename = "MyFile" + (String)i + ".txt";
		if (!SD.exists(filename))
		{
			break;
		}
	}
	dataFile = SD.open(filename, FILE_WRITE);

#endif

#if GPS_ON
	// enable GPS sending GPRMC message
	gps.begin(9600);
	gps.println(PMTK_SET_NMEA_UPDATE_1HZ);
	gps.println(PMTK_API_SET_FIX_CTL_1HZ);
	gps.println(PMTK_SET_NMEA_OUTPUT_RMC);
#endif		

	// init target button here
	pinMode(button, INPUT_PULLUP);

	//Set Tartgets
	//ex:
	//	GEOLAT0 28.594532
	//	GEOLON0 -81.304437
	targets[0].lat = 28.59544f;
	targets[0].lon = -81.30397f;
	targets[1].lat = 28.59208f;
	targets[1].lon = -81.30407f;
	targets[2].lat = 28.59671f;
	targets[2].lon = -81.30205f;
	targets[3].lat = 28.5962f;
	targets[3].lon = -81.30538f;
}

void loop(void)
{
	// if button pressed, set new target
	if (debounce(button))
	{
		target + 1 > 3 ? target = 0 : target++;
		//Serial.println("Set Target");
	}

	//strip.setBrightness(~map(analogRead(A0), 0, 1023, 0, 255));

	// returns with message once a second
	getGPSMessage();


	// if GPRMC message (3rd letter = R)
	if (cstr[3] == 'R' && cstr[18] == 'A')
	{
		// parse message parameters

		/*******************************************************************************

		Following is the GPS Shield "GPRMC" Message Structure.This message is received
		once a second.You must parse the message to obtain the parameters required for
		the GeoCache project.GPS provides coordinates in Degrees Minutes(DDDMM.MMMM).
		The coordinates in the following GPRMC sample message, after converting to Decimal
		Degrees format(DDD.DDDDDD) is latitude(23.118757) and longitude(120.274060).By
		the way, this coordinate is GlobaTop Technology in Tiawan, who designed and
		manufactured the GPS Chip.
		 0123456789
		"$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C/r/n"

		$GPRMC,         // GPRMC Message
		064951.000,     // utc time hhmmss.sss
		A,              // status A=data valid or V=data not valid
		2307.1256,      // Latitude 2307.1256 (degrees minutes format dddmm.mmmm)
		N,              // N/S Indicator N=north or S=south
		12016.4438,     // Longitude 12016.4438 (degrees minutes format dddmm.mmmm)
		E,              // E/W Indicator E=east or W=west
		0.03,           // Speed over ground knots
		165.48,         // Course over ground (decimal degrees format ddd.dd)
		260406,         // date ddmmyy
		3.05,           // Magnetic variation (decimal degrees format ddd.dd)
		W,              // E=east or W=west
		A               // Mode A=Autonomous D=differential E=Estimated
		* 2C             // checksum
		/ r / n            // return and newline

		******************************************************************************/

#pragma region Parse Vars

		char GPRMCMessage[6] = { NULL };
		char utctime[11] = { NULL };
		char Status[2] = { NULL };
		char latitude[11] = { NULL };
		char latIndicator[2] = { NULL };
		char longitude[11] = { NULL };
		char longIndicator[2] = { NULL };
		char speed[5] = { NULL };
		char course[7] = { NULL };
		char date[7] = { NULL };
		char magvar[7] = { NULL };
		char WE[2] = { NULL };
		char Mode[2] = { NULL };
		int i = 0;

#pragma endregion

#pragma region Parsing

		int j = 0;
		for (j = 0; cstr[i] != ','; j++)
		{
			GPRMCMessage[j] = cstr[i];
			i++;
		}
		i++;
		//Serial.println("GP Message " + (String)GPRMCMessage);
		//delay(100);
		for (j = 0; cstr[i] != ','; j++)
		{
			utctime[j] = cstr[i];
			i++;
		}
		i++;
	//	Serial.println("Utctime " + (String)utctime);
		//delay(100);
		for (j = 0; cstr[i] != ','; j++)
		{
			Status[j] = cstr[i];
			i++;
		}
		i++;
		//Serial.println("Status " + (String)Status);
		//delay(100);
		for (j = 0; cstr[i] != ','; j++)
		{
			latitude[j] = cstr[i];
			i++;
		}
		i++;
		//Serial.println("Lat " + (String)latitude);
		//delay(100);
		for (j = 0; cstr[i] != ','; j++)
		{
			latIndicator[j] = cstr[i];
			i++;
		}
		i++;
	//	Serial.println("LatI " + (String)latIndicator);
	//	delay(100);
		for (j = 0; cstr[i] != ','; j++)
		{
			longitude[j] = cstr[i];
			i++;
		}
		i++;
		//Serial.println("Long " + (String)longitude);
		//delay(100);
		for (j = 0; cstr[i] != ','; j++)
		{
			longIndicator[j] = cstr[i];
			i++;
		}
		i++;
		//Serial.println("LongI " + (String)longIndicator);
	//	delay(100);
		for (j = 0; cstr[i] != ','; j++)
		{
			speed[j] = cstr[i];
			i++;
		}
		i++;
	//	Serial.println("Speed " + (String)speed);
	//	delay(100);
		for (j = 0; cstr[i] != ','; j++)
		{
			course[j] = cstr[i];
			i++;
		}
		i++;
		//Serial.println("Course " + (String)course);
		//delay(100);
		for (j = 0; cstr[i] != ','; j++)
		{
			date[j] = cstr[i];
			i++;
		}
		i++;
		//Serial.println("Date " + (String)date);
		//delay(100);
		for (j = 0; cstr[i] != ','; j++)
		{
			magvar[j] = cstr[i];
			i++;
		}
		i++;
		//Serial.println("Magvar " + (String)magvar);
		//delay(100);
		for (j = 0; cstr[i] != ','; j++)
		{
			WE[j] = cstr[i];
			i++;
		}
		i++;
		//Serial.println("WE " + (String)WE);
		//delay(100);
		for (j = 0; j < 1; j++)
		{
			Mode[j] = cstr[i];
			i++;
		}
		i++;
		//Serial.println("Mode " + (String)Mode);
		//delay(100);
		//Serial.println((String)cstr);
		//delay(1000);
#pragma endregion
		//Serial.println("Parsing");

		// calculated destination heading
		CourseOverGround = atof(course);

		heading = calcBearing(degMin2DecDeg(latIndicator, latitude), degMin2DecDeg(longIndicator, longitude), targets[target].lat, targets[target].lon);

		// calculated destination distance
		//distance = calcDistance(28.593936, -81.304642, GEOLAT0, GEOLON0);
		distance = calcDistance(degMin2DecDeg(latIndicator, latitude), degMin2DecDeg(longIndicator, longitude), targets[target].lat, targets[target].lon);

		//Serial.println("Set Distance and Bearing");
		//Serial.println(String(distance) + "\n" + String(heading));

#if SDC_ON
		// write current position to SecureDigital then flush
		if (latitude[0] != NULL && longitude[0] != NULL)
		{
			dataFile.println(String(degMin2DecDeg(longIndicator, longitude)) + ',' + String(degMin2DecDeg(latIndicator, latitude)) + ',' + distance);
			dataFile.flush();
		}

#endif
	}

#if NEO_ON
	// set NeoPixel target display
	setNeoPixel(target, heading, distance);
#endif		

#if TRM_ON
	// print debug information to Serial Terminal
	Serial.println(cstr);
#endif		

#if ONE_ON
	// print debug information to OneSheeld Terminal
	if (serialEventRun) serialEventRun();
#endif
}

bool debounce(int pin)
{
	for (uint16_t i = 0; i < 1000; i++)
	{
		if (digitalRead(pin))
		{
			return false;
		}
	}
	return true;
}
#pragma endregion