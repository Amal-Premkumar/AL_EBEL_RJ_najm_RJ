/*
 * gps.h
 *
 *  Created on: Feb 6, 2026
 *      Author: Amal
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>


#define PKT_TELEMETRY  0xA1
/* ===== GPS DATA STRUCT ===== */
typedef struct
{
    float latitude;
    float longitude;
    float speed_kmh;
    float distance_m;     // total distance travelled
    uint8_t fix_valid;
} GPS_Data_t;

/* ===== API ===== */
void GPS_Init(void);
void GPS_Process(void);

 extern GPS_Data_t GPS_GetData(void);
 extern GPS_Data_t gps;

#endif /* INC_GPS_H_ */
