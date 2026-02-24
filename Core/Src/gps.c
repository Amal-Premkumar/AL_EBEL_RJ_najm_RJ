/*
 * gps.c
 *
 *  Created on: Feb 6, 2026
 *      Author: Amal
 */


#include "gps.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

extern UART_HandleTypeDef huart4;

/* ===== INTERNAL ===== */
 GPS_Data_t gps = {0};

static char nmea_buf[128];
static uint8_t nmea_idx = 0;

static float last_lat = 0.0f;
static float last_lon = 0.0f;

/* ===== UTILS ===== */
static float degmin_to_deg(float dm)
{
    int deg = (int)(dm / 100);
    float min = dm - (deg * 100);
    return deg + (min / 60.0f);
}

static float haversine(float lat1, float lon1, float lat2, float lon2)
{
    const float R = 6371000.0f; // Earth radius (m)

    float dlat = (lat2 - lat1) * 0.0174533f;
    float dlon = (lon2 - lon1) * 0.0174533f;

    lat1 *= 0.0174533f;
    lat2 *= 0.0174533f;

    float a = sinf(dlat/2)*sinf(dlat/2) +
              cosf(lat1)*cosf(lat2) *
              sinf(dlon/2)*sinf(dlon/2);

    return 2 * R * asinf(sqrtf(a));
}

/* ===== NMEA PARSER ===== */
static void GPS_ParseGPRMC(char *s)
{
    char *tok;

    tok = strtok(s, ",");        // $GPRMC
    tok = strtok(NULL, ",");     // time
    tok = strtok(NULL, ",");     // status

    if (*tok != 'A')
    {
        gps.fix_valid = 0;
        return;
    }

    gps.fix_valid = 1;

    tok = strtok(NULL, ",");     // latitude
    float lat_dm = atof(tok);
    tok = strtok(NULL, ",");     // N/S
    char ns = *tok;

    tok = strtok(NULL, ",");     // longitude
    float lon_dm = atof(tok);
    tok = strtok(NULL, ",");     // E/W
    char ew = *tok;

    tok = strtok(NULL, ",");     // speed (knots)
    float speed_knots = atof(tok);

    float lat = degmin_to_deg(lat_dm);
    float lon = degmin_to_deg(lon_dm);

    if (ns == 'S') lat = -lat;
    if (ew == 'W') lon = -lon;

    gps.speed_kmh = speed_knots * 1.852f;

    if (last_lat != 0.0f)
    {
        gps.distance_m += haversine(last_lat, last_lon, lat, lon);
    }

    last_lat = lat;
    last_lon = lon;

    gps.latitude  = lat;
    gps.longitude = lon;
}

/* ===== API ===== */
void GPS_Init(void)
{
    memset(&gps, 0, sizeof(gps));
}

void GPS_Process(void)
{
    uint8_t ch;

    if (HAL_UART_Receive(&huart4, &ch, 1, 0) != HAL_OK)
        return;

    if (ch == '\n')
    {
        nmea_buf[nmea_idx] = 0;

        if (strstr(nmea_buf, "$GPRMC"))
            GPS_ParseGPRMC(nmea_buf);

        nmea_idx = 0;
    }
    else if (nmea_idx < sizeof(nmea_buf) - 1)
    {
        nmea_buf[nmea_idx++] = ch;
    }
}

GPS_Data_t GPS_GetData(void)
{
    return gps;
}


