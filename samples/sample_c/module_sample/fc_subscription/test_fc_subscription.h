/**
 ********************************************************************
 * @file    test_fc_subscription.h
 * @brief   This is the header file for "test_fc_subscription.c", defining the structure and
 * (exported) function prototypes.
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TEST_FC_SUBSCRIPTION_H
#define TEST_FC_SUBSCRIPTION_H

/* Includes ------------------------------------------------------------------*/
#include "dji_typedef.h"
#include "dji_fc_subscription.h"
#include <pthread.h>
#include "cjson/cJSON.h"
#include <MQTTAsync.h>
#define RADIUS_EARTH 6371000 // 地球半径，单位：米
#define M_PI		3.14159265358979323846
#define TOPIC_REPLY       "gcs_reply/1/progress"
#define QOS         2

typedef struct {
    dji_f64_t rtkLongitude;
    dji_f64_t rtkLatitude; 
    dji_f32_t relativeHeight;
    float process;
} DroneStatus;

// typedef struct {
//     dji_f64_t homeLongitude;
//     dji_f64_t homeLatitude;
// } HomePoint;

typedef struct {
    cJSON *data;
    MQTTAsync client;
} ThreadParams;


#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
extern uint8_t is_RTK_ready;
extern uint8_t remainingBattery;
extern DroneStatus droneStatus;
// HomePoint homePoint;
extern pthread_mutex_t statusMutex;
// pthread_mutex_t homeMutex;
extern bool finishedMission;
extern pthread_mutex_t mqtt_publish_mutex;
extern bool isin_mission;
extern dji_f64_t schedule;
extern dji_f64_t targetLat, targetLon;
extern uint8_t userID;
extern bool stationary;
extern bool stopview;
extern dji_f64_t distance_safe;

/* Exported functions --------------------------------------------------------*/
T_DjiReturnCode DjiTest_FcSubscriptionStartService(void* arg);
T_DjiReturnCode DjiTest_FcSubscriptionRunSample(void);
T_DjiReturnCode DjiTest_FcSubscriptionDataShowTrigger(void);
T_DjiReturnCode DjiTest_FcSubscriptionGetTotalSatelliteNumber(uint8_t *number);
void replyProgress(MQTTAsync client, bool missionOK, bool inMission, float progress, int gateway);

#ifdef __cplusplus
}
#endif

#endif // TEST_FC_SUBSCRIPTION_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
