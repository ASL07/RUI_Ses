/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic driver for the GPS receiver UP501

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __GPS_H__
#define __GPS_H__

/* Structure to handle the GPS parsed data in ASCII */
typedef struct
{
    char NmeaDataType[12];
    char NmeaUtcTime[22];
    char NmeaDataStatus[4];
    char NmeaLatitude[20];
    char NmeaLatitudePole[4];
    char NmeaLongitude[22];
    char NmeaLongitudePole[4];
    char NmeaFixQuality[4];
    char NmeaSatelliteTracked[6];
    char NmeaHorizontalDilution[12];
    char NmeaAltitude[16];
    char NmeaAltitudeUnit[4];
    char NmeaHeightGeoid[16];
    char NmeaHeightGeoidUnit[4];
    char NmeaSpeed[16];
    char NmeaDetectionAngle[16];
    char NmeaDate[16];
}tNmeaGpsData;

extern tNmeaGpsData NmeaGpsData;


#endif  // __GPS_H__
