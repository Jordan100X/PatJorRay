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
#include <SD.h>
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
#define NEO_ON 0		// NeoPixelShield
#define TRM_ON 1		// SerialTerminal
#define ONE_ON 0		// 1Sheeld
#define SDC_ON 0		// SecureDigital
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
int button = 15; //Replace with actual button number
File dataFile;

#if GPS_ON
#include "SoftwareSerial.h"
SoftwareSerial gps(GPS_RX, GPS_TX);
#endif

#if NEO_ON
#include "NeoPixel.h"
Adafruit_NeoPixel strip = Adafruit_NeoPixel(40, NEO_TX, NEO_GRB + NEO_KHZ800);
#endif

#if ONE_ON
#define CUSTOM_SETTINGS
#define INCLUDE_TERMINAL_SHIELD
#include "OneSheeld.h"
#endif

#if SDC_ON
#include "SecureDigital.h"
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

/*************************************************
**** GEO FUNCTIONS - BEGIN ***********************
*************************************************/

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

	// add code here

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

	// add code here

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

	// add code here

	return(bearing);
}

/*************************************************
**** GEO FUNCTIONS - END**************************
*************************************************/

#if NEO_ON
/*
Sets target number, heading and distance on NeoPixel Display
*/
void setNeoPixel(uint8_t target, float heading, float distance)
{
	// add code here
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
#endif	

#if SDC_ON
	/*
	Initialize the SecureDigitalCard and open a numbered sequenced file
	name "MyMapNN.txt" for storing your coordinates, where NN is the
	sequential number of the file.  The filename can not be more than 8
	chars in length (excluding the ".txt").
	*/

	// see if the card is present and can be initialized:
	if (!SD.begin(sdChip)) {
		Serial.println("Card failed, or not present");
	}
	Serial.println("card initialized.");

	String filename;
	for (size_t i = 0; i < 99; i++)
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

	// don't do anything more:
	return;

#endif

#if GPS_ON
	// enable GPS sending GPRMC message
	gps.begin(9600);
	gps.println(PMTK_SET_NMEA_UPDATE_1HZ);
	gps.println(PMTK_API_SET_FIX_CTL_1HZ);
	gps.println(PMTK_SET_NMEA_OUTPUT_RMC);
#endif		

	// init target button here

}

void loop(void)
{
	// if button pressed, set new target
	if (debounce(button))
	{
		target++;
		if (target > 3)
			target = 0;
	}
	// returns with message once a second
	getGPSMessage();



	// if GPRMC message (3rd letter = R)
	while (cstr[3] == 'R')
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

		char* utctime[11] = { NULL };
		char* latitude[11] = { NULL };
		char*  longitude[11] = { NULL };
		char*  speed[5] = { NULL };
		char*  course[7] = { NULL };
		char*  date[7] = { NULL };
		char*  magvar[7] = { NULL };
		int i = 0;
		while (cstr[i] != ',')
		{
			i++;
		}
		i++;

		for (int j = 0; cstr[i] != ','; j++)
		{
			*utctime[j] = cstr[i];
			i++;
		}
		i++;
		while (cstr[i] != ',')
		{
			i++;
		}
		i++;

		for (int j = 0; cstr[i] != ','; j++)
		{
			*latitude[j] = cstr[i];
			i++;
		}
		i++;
		while (cstr[i] != ',')
		{
			i++;
		}
		i++;

		for (int j = 0; cstr[i] != ','; j++)
		{
			*longitude[j] = cstr[i];
			i++;
		}
		i++;
		while (cstr[i] != ',')
		{
			i++;
		}
		i++;

		for (int j = 0; cstr[i] != ','; j++)
		{
			*speed[j] = cstr[i];
			i++;
		}
		i++;

		for (int j = 0; cstr[i] != ','; j++)
		{
			*course[j] = cstr[i];
			i++;
		}
		i++;

		for (int j = 0; cstr[i] != ','; j++)
		{
			*date[j] = cstr[i];
			i++;
		}
		i++;

		for (int j = 0; cstr[i] != ','; j++)
		{
			*magvar[j] = cstr[i];
			i++;
		}
		i++;


		// calculated destination heading

		// calculated destination distance

#if SDC_ON
		// write current position to SecureDigital then flush

		if (dataFile)
		{
			dataFile.print(cstr);
		}

#endif

		break;
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