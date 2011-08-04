
#define a	6378137.0								// earth semimajor axis in meters
#define f 	0.0033528106647474807198455286185206	// reciprocal flattening
#define e2 	2*f-f*f									// eccentricity squared
#define MEAN_EARTH_RADIUS 	6378137.0
#define MEAN_EARTH_DIAMETER	12756274.0
#define UMR	0.017453292519943295769236907684886		//PI/180


struct GPS_DATA
{	
//latitude/longitude in deg * 10^7
	int latitude;
	int longitude;
//GPS height in mm 
	int height;
//speed in x (E/W) and y(N/S) in mm/s	
	int speed_x;
	int speed_y;
//GPS heading in deg * 1000
	int heading; 
	
//accuracy estimates in mm and mm/s
	unsigned int horizontal_accuracy;
	unsigned int vertical_accuracy;
	unsigned int speed_accuracy;

//number of satellite vehicles used in NAV solution
	unsigned int numSV;

// GPS status information; Bit7...Bit3: 0 Bit 2: longitude direction Bit1: latitude direction Bit 0: GPS lock
	int status; 		
};
extern struct GPS_DATA GPS_Data;

struct GPS_TIME 
{
	unsigned int time_of_week;	//[ms]
	unsigned short week;		//[1..52]
};
extern struct GPS_TIME GPS_Time;

//trigger's new gps data transmission
extern unsigned int gpsDataOkTrigger;

