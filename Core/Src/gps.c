#include "gps.h"
#include <string.h>
#include <stdlib.h>

int parseNMEA(char* nmea, GPSData* gps) {
	// validate
	if (*nmea != '$') {
		return G_PARSE;
	}
	if (!strncmp(nmea,"$GNRMC",6)) {
		printf("RMC data\r\n");
		int comma_count = 0;
		char* ind = nmea;

		char* start_utc = ind;
		while (comma_count < 2) {
			ind++;
			if (*ind == ',') {
				comma_count++;
				if (comma_count == 1) {
					start_utc = ind + 1;
				}
			}
		}
		ind++;
		if (*ind == 'V') {
			printf("Invalid data\r\n");
			return G_NOLOC;
		}

		char hour[3];
		memcpy(hour, start_utc, 2);
		hour[2] = 0;
		uint8_t iHour = atoi(hour);
		(gps->ti).Hours = iHour%12;
		(gps->ti).TimeFormat = iHour/12;

		char min[3];
		memcpy(min, start_utc + 2, 2);
		min[2] = 0;
		(gps->ti).Minutes = atoi(min);

		char sec[3];
		memcpy(sec, start_utc + 4, 2);
		sec[2] = 0;
		(gps->ti).Seconds = atoi(sec);

		while (comma_count < 3) {
			ind++;
			if (*ind == ',') {
				comma_count++;
			}
		}
		ind++;

		char lat_deg[3];
		memcpy(lat_deg,ind,2);
		ind += 2;
		lat_deg[2] = 0;
		uint16_t iLaDeg = atoi(lat_deg);

		char lat_min[8];
		memcpy(lat_min,ind,7);
		ind += 8;
		lat_min[7] = 0;
		float decLa = atof(lat_min);
		decLa /= 60.0;
		decLa += iLaDeg;
		decLa *= *ind == 'S' ? -1 : 1;
		ind += 3;


		char lon_deg[4];
		memcpy(lon_deg,ind,3);
		ind += 3;
		lon_deg[3] = 0;
		uint16_t iLoDeg = atoi(lon_deg);

		char lon_min[8];
		memcpy(lon_min,ind,7);
		ind += 9;
		lon_min[7] = 0;
		float decLo = atof(lon_min);
		decLo /= 60.0;
		decLo += iLoDeg;
		decLo *= *ind == 'W' ? -1 : 1;

		gps->la = decLa;
		gps->lo = decLo;

		int postLocCommas = 0;
		while (postLocCommas < 3) {
			ind++;
			if (*ind == ',') {
				postLocCommas++;
			}
		}
		ind++;
		char* start_date = ind;

		char day[3];
		memcpy(day, start_date, 2);
		day[2] = 0;
		(gps->da).Date = atoi(day);

		char month[3];
		memcpy(month, start_date + 2, 2);
		month[2] = 0;
		(gps->da).Month = atoi(month);

		char year[3];
		memcpy(year, start_date + 4, 2);
		year[2] = 0;
		(gps->da).Year = atoi(year);

		return G_OK;
	}

}
