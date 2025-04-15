/**
 ********************************************************************
 * @file    test_fc_subscription.c
 * @brief
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

/* Includes ------------------------------------------------------------------*/
#include <utils/util_misc.h>
#include <math.h>
#include "test_fc_subscription.h"
#include "dji_logger.h"
#include "dji_platform.h"
// #include "widget_interaction_test/test_widget_interaction.h"
#include <sys/time.h>

/* Private constants ---------------------------------------------------------*/
#define FC_SUBSCRIPTION_TASK_FREQ         (1)
#define FC_SUBSCRIPTION_TASK_STACK_SIZE   (1024)
#define RADIUS 6371 // 地球平均半径，单位为公里

// #define TOPIC_REPLY       "gcs_reply/1/process"
// #define QOS         2
// #define M_PI		3.14159265358979323846
// #define RADIUS_EARTH 6371000 // 地球半径，单位：米
/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/
static void *UserFcSubscription_Task(void *arg);
static T_DjiReturnCode DjiTest_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data, uint16_t dataSize,
                                                                       const T_DjiDataTimestamp *timestamp);
static dji_f64_t computeProgress(dji_f64_t lat1, dji_f64_t lon1, dji_f64_t lat2, dji_f64_t lon2);
static dji_f64_t deg2rad(dji_f64_t deg);
static dji_f64_t haversine(dji_f64_t lat1, dji_f64_t lon1, dji_f64_t lat2, dji_f64_t lon2);
static dji_f64_t point_to_segment_distance(dji_f64_t latA, dji_f64_t lonA, dji_f64_t latB, dji_f64_t lonB, dji_f64_t latP, dji_f64_t lonP);

/* Private variables ---------------------------------------------------------*/
static T_DjiTaskHandle s_userFcSubscriptionThread;
static bool s_userFcSubscriptionDataShow = false;
static uint8_t s_totalSatelliteNumberUsed = 0;
static uint32_t s_userFcSubscriptionDataCnt = 0;
static MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
static MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
// static dji_f64_t targetLat=0, targetLon=0;
static dji_f64_t distanceTotal = 0;
static MQTTAsync client;
// static bool in_air = false;

bool finishedMission = false;
pthread_mutex_t mqtt_publish_mutex = PTHREAD_MUTEX_INITIALIZER;
bool isin_mission = false;
dji_f64_t schedule = 0;

uint8_t is_RTK_ready = 0;
uint8_t remainingBattery = 0;
DroneStatus droneStatus =  {0};
pthread_mutex_t statusMutex = PTHREAD_MUTEX_INITIALIZER;
dji_f64_t targetLat=0, targetLon=0;
uint8_t userID=0;
bool stationary=true;
bool stopview=false;
dji_f64_t distance_safe=0;

/* Exported functions definition ---------------------------------------------*/
T_DjiReturnCode DjiTest_FcSubscriptionStartService(void* arg)
{
    // ThreadParams *params = (ThreadParams*)arg;  // 将传入的void*转换为ThreadParams*
    // cJSON *data = params->data;
    client = (MQTTAsync) arg;
    // cJSON *latitude_json = cJSON_GetObjectItemCaseSensitive(data, "latitude");
    // if (latitude_json && cJSON_IsNumber(latitude_json)) {
    //     targetLat = latitude_json->valuedouble;
    //     printf("Latitude: %d\n", latitude_json->valuedouble);
    // }
    // cJSON *longitude_json = cJSON_GetObjectItemCaseSensitive(data, "longitude");
    // if (longitude_json && cJSON_IsNumber(longitude_json)) {
    //     targetLon = longitude_json->valuedouble;
    //     printf("Longitude: %d\n", longitude_json->valuedouble);
    // }


    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = NULL;

    osalHandler = DjiPlatform_GetOsalHandler();

    s_userFcSubscriptionDataShow = true;

    djiStat = DjiFcSubscription_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("init data subscription module error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    // djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
    //                                            DjiTest_FcSubscriptionReceiveQuaternionCallback);
    // if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Subscribe topic quaternion error.");
    //     return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    // } else {
    //     USER_LOG_DEBUG("Subscribe topic quaternion success.");
    // }

    // djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
    //                                            NULL);
    // if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Subscribe topic velocity error.");
    //     return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    // } else {
    //     USER_LOG_DEBUG("Subscribe topic velocity success.");
    // }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps position error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic gps position success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps details error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic gps details success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic rtk position info error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic rtk position info success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic rtk position error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic rtk position success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude of homepoint error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic altitude of homepoint success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic battery info error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic battery info success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic home point info error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic home point info success.");
    }
    
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic status flight error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic status flight success.");
    }

    printf("test_fc_subscription.c");

    if (osalHandler->TaskCreate("user_subscription_task", UserFcSubscription_Task,
                                FC_SUBSCRIPTION_TASK_STACK_SIZE, NULL, &s_userFcSubscriptionThread) !=
        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("user data subscription task create error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FcSubscriptionRunSample(void)
{
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiFcSubscriptionVelocity velocity = {0};
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
    T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo = {0};

    USER_LOG_INFO("Fc subscription sample start");
    s_userFcSubscriptionDataShow = true;

    USER_LOG_INFO("--> Step 1: Init fc subscription module");
    djiStat = DjiFcSubscription_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("init data subscription module error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--> Step 2: Subscribe the topics of quaternion, velocity and gps position");
    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               DjiTest_FcSubscriptionReceiveQuaternionCallback);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic velocity error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps position error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--> Step 3: Get latest value of the subscribed topics in the next 10 seconds\r\n");

    for (int i = 0; i < 10; ++i) {
        osalHandler->TaskSleepMs(1000 / FC_SUBSCRIPTION_TASK_FREQ);
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                          (uint8_t *) &velocity,
                                                          sizeof(T_DjiFcSubscriptionVelocity),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic velocity error.");
        } else {
            USER_LOG_INFO("velocity: x = %f y = %f z = %f healthFlag = %d, timestamp ms = %d us = %d.", velocity.data.x,
                          velocity.data.y,
                          velocity.data.z, velocity.health, timestamp.millisecond, timestamp.microsecond);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                          (uint8_t *) &gpsPosition,
                                                          sizeof(T_DjiFcSubscriptionGpsPosition),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps position error.");
        } else {
            USER_LOG_INFO("gps position: x = %d y = %d z = %d.", gpsPosition.x, gpsPosition.y, gpsPosition.z);
        }

        //Attention: if you want to subscribe the single battery info on M300 RTK, you need connect USB cable to
        //OSDK device or use topic DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO instead.
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1,
                                                          (uint8_t *) &singleBatteryInfo,
                                                          sizeof(T_DjiFcSubscriptionSingleBatteryInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic battery single info index1 error.");
        } else {
            USER_LOG_INFO(
                "battery single info index1: capacity percent = %ld% voltage = %ldV temperature = %.2f degree.",
                singleBatteryInfo.batteryCapacityPercent,
                singleBatteryInfo.currentVoltage / 1000,
                (dji_f32_t) singleBatteryInfo.batteryTemperature / 10);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX2,
                                                          (uint8_t *) &singleBatteryInfo,
                                                          sizeof(T_DjiFcSubscriptionSingleBatteryInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic battery single info index2 error.");
        } else {
            USER_LOG_INFO(
                "battery single info index2: capacity percent = %ld% voltage = %ldV temperature = %.2f degree.\r\n",
                singleBatteryInfo.batteryCapacityPercent,
                singleBatteryInfo.currentVoltage / 1000,
                (dji_f32_t) singleBatteryInfo.batteryTemperature / 10);
        }
    }

    USER_LOG_INFO("--> Step 4: Unsubscribe the topics of quaternion, velocity and gps position");
    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--> Step 5: Deinit fc subscription module");

    djiStat = DjiFcSubscription_DeInit();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit fc subscription error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    s_userFcSubscriptionDataShow = false;
    USER_LOG_INFO("Fc subscription sample end");

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FcSubscriptionDataShowTrigger(void)
{
    s_userFcSubscriptionDataShow = !s_userFcSubscriptionDataShow;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FcSubscriptionGetTotalSatelliteNumber(uint8_t *number)
{
    *number = s_totalSatelliteNumberUsed;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* Private functions definition-----------------------------------------------*/
#ifndef __CC_ARM
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-noreturn"
#pragma GCC diagnostic ignored "-Wreturn-type"
#endif

static void *UserFcSubscription_Task(void *arg)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionVelocity velocity = {0};
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
    T_DjiFcSubscriptionGpsDetails gpsDetails = {0};
    T_DjiFcSubscriptionRtkPositionInfo rtkPositionInfo = {0};
    T_DjiFcSubscriptionRtkPosition rtkPosition = {0};
    T_DjiFcSubscriptionAltitudeOfHomePoint altitudeOfHomepoint = {0};
    T_DjiFcSubscriptionWholeBatteryInfo batteryInfo = {0};
    T_DjiFcSubscriptionHomePointInfo homepointInfo = {0};
    T_DjiFcSubscriptionFlightStatus flightStatus = {0};
    T_DjiOsalHandler *osalHandler = NULL;

    // uint8_t is_RTK_ready = 0;
    // uint8_t remainingBattery = 0;
    // DroneStatus droneStatus =  {0};
    // // HomePoint homePoint = {0};
    // pthread_mutex_t statusMutex = PTHREAD_MUTEX_INITIALIZER;
    // // pthread_mutex_t homeMutex = PTHREAD_MUTEX_INITIALIZER;

    USER_UTIL_UNUSED(arg);
    osalHandler = DjiPlatform_GetOsalHandler();

    while (1) {
        osalHandler->TaskSleepMs(1000 / FC_SUBSCRIPTION_TASK_FREQ);

        // djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
        //                                                   (uint8_t *) &velocity,
        //                                                   sizeof(T_DjiFcSubscriptionVelocity),
        //                                                   &timestamp);
        // if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        //     USER_LOG_ERROR("get value of topic velocity error.");
        // }

        // if (s_userFcSubscriptionDataShow == true) {
        //     USER_LOG_INFO("velocity: x %f y %f z %f, healthFlag %d.", velocity.data.x, velocity.data.y,
        //                   velocity.data.z, velocity.health);
        //     printf("test_fc_subscription.c++++++++++++++++++++++++++++++++\n");
        // }
        printf("test_fc_subscription.c----------------------------------\n");

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                          (uint8_t *) &gpsPosition,
                                                          sizeof(T_DjiFcSubscriptionGpsPosition),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps position error.");
        }

        if (s_userFcSubscriptionDataShow == true) {
            USER_LOG_INFO("gps position: x %f y %f z %f.", gpsPosition.x*1e-7, gpsPosition.y*1e-7, gpsPosition.z*1e-3);
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
                                                          (uint8_t *) &gpsDetails,
                                                          sizeof(T_DjiFcSubscriptionGpsDetails),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps details error.");
        }

        if (s_userFcSubscriptionDataShow == true) {
            USER_LOG_INFO("gps total satellite number used: %d %d %d.\nGPS fixState: %f.",
                          gpsDetails.gpsSatelliteNumberUsed,
                          gpsDetails.glonassSatelliteNumberUsed,
                          gpsDetails.totalSatelliteNumberUsed,
                          gpsDetails.fixState);
            s_totalSatelliteNumberUsed = gpsDetails.totalSatelliteNumberUsed;
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO,
                                                          (uint8_t *) &homepointInfo,
                                                          sizeof(T_DjiFcSubscriptionHomePointInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of homepoint info error.");
        }

        if (s_userFcSubscriptionDataShow == true) {
            USER_LOG_INFO("Homepoint info: lat->%f, lon->%f.", homepointInfo.latitude*(180.0/M_PI), homepointInfo.longitude*(180.0/M_PI));
        }
        if(isin_mission)
        {
            distanceTotal = computeProgress(homepointInfo.latitude*(180.0/M_PI), homepointInfo.longitude*(180.0/M_PI), targetLat, targetLon);
            USER_LOG_INFO("distanceTotal: lat->%f, lon->%f. lat->%f, lon->%f.", homepointInfo.latitude*(180.0/M_PI), homepointInfo.longitude*(180.0/M_PI), targetLat, targetLon);
        }
        // pthread_mutex_lock(&homeMutex); // 加锁以保护对共享资源的访问
        // homePoint.homeLatitude = homepointInfo.latitude;
        // homePoint.homeLongitude = homepointInfo.longitude;
        // pthread_mutex_unlock(&homeMutex); // 解锁
        // if(isin_mission && in_air)
        // {
        //     distanceTotal = computeProgress(homepointInfo.latitude, homepointInfo.longitude, targetLat, targetLon);
        // }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
                                                          (uint8_t *) &rtkPositionInfo,
                                                          sizeof(T_DjiFcSubscriptionRtkPositionInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic rtk position info error.");
        }

        if (s_userFcSubscriptionDataShow == true) {
            USER_LOG_INFO("RTK position solution state: %d.", rtkPositionInfo);
        }
        if(rtkPositionInfo != is_RTK_ready)
        {
            is_RTK_ready = rtkPositionInfo;
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
                                                          (uint8_t *) &rtkPosition,
                                                          sizeof(T_DjiFcSubscriptionRtkPosition),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of rtk position error.");
        }

        if (s_userFcSubscriptionDataShow == true) {
            USER_LOG_INFO("RTK position: %lf %lf %f.",
                          rtkPosition.longitude,
                          rtkPosition.latitude,
                          rtkPosition.hfsl);
        }
        pthread_mutex_lock(&statusMutex); // 加锁以保护对共享资源的访问
        droneStatus.rtkLongitude = rtkPosition.longitude;
        droneStatus.rtkLatitude = rtkPosition.latitude;
        droneStatus.relativeHeight = rtkPosition.hfsl - altitudeOfHomepoint;
        droneStatus.process = schedule;
        printf("test_fc_subscription.c------------droneStatus----------------------%f,  %f,  %f\n", droneStatus.rtkLongitude, droneStatus.rtkLatitude, rtkPosition.hfsl);
        pthread_mutex_unlock(&statusMutex); // 解锁
        if(isin_mission)
        {
            dji_f64_t distanceCurrent = computeProgress(homepointInfo.latitude*(180.0/M_PI), homepointInfo.longitude*(180.0/M_PI), rtkPosition.latitude, rtkPosition.longitude);
            schedule = distanceCurrent / distanceTotal;
            printf("test_fc_subscription.c------------isin_mission--------------replyProgress--------\n");
            printf("distanceCurrent： %f;  distanceTotal: %f\n", distanceCurrent, distanceTotal);
            replyProgress(client, true, true, schedule, 1);
            distance_safe = point_to_segment_distance(homepointInfo.latitude*(180.0/M_PI), homepointInfo.longitude*(180.0/M_PI), targetLat, targetLon, rtkPosition.latitude, rtkPosition.longitude);
        } 
        


        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                                                          (uint8_t *) &altitudeOfHomepoint,
                                                          sizeof(T_DjiFcSubscriptionAltitudeOfHomePoint),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of altitude of homepoint error.");
        }

        if (s_userFcSubscriptionDataShow == true) {
            USER_LOG_INFO("Altitude of homepoint: %f.", altitudeOfHomepoint);

        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO,
                                                          (uint8_t *) &batteryInfo,
                                                          sizeof(T_DjiFcSubscriptionWholeBatteryInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of battery info error.");
        }

        if (s_userFcSubscriptionDataShow == true) {
            USER_LOG_INFO("Battery info: %d.", batteryInfo.percentage);
        }
        remainingBattery = batteryInfo.percentage;
        printf("test_fc_subscription.c------------remainingBattery----------------------%d\n", remainingBattery);

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                          (uint8_t *) &flightStatus,
                                                          sizeof(T_DjiFcSubscriptionFlightStatus),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of flight status error.");
        }

        if (s_userFcSubscriptionDataShow == true) {
            USER_LOG_INFO("Flight status: %d.", flightStatus);
        }
        if(flightStatus==0) stationary=true;
        else stationary=false;
        if(finishedMission && flightStatus==0) {
            printf("test_fc_subscription.c------------finishedMission--------------replyProgress--------\n");
            replyProgress(client, true, false, 1, 1); //无人机完成任务并降落在地面且锁桨
            finishedMission = false;
            stopview = true;
        }
        

        // 判断GPS和RTK差别大不
        // 航点飞行过程中的RTK固定解情况
        // RTK固定解T_DjiFcSubscriptionRtkPosition——E_DjiFcSubscriptionPositionSolutionProperty——DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_NARROW_INT和GPS固定解T_DjiFcSubscriptionGpsDetails——fixState——E_DjiFcSubscriptionGpsFixState——DJI_FC_SUBSCRIPTION_GPS_FIX_STATE_3D_FIX  
        // 看一下flight_control和 waypoint_v2    https://developer.dji.com/doc/payload-sdk-tutorial/cn/function-set/advanced-function/waypoint-mission.html
        // DjiFlightController_SetRtkPositionEnableStatus(DJI_FLIGHT_CONTROLLER_ENABLE_RTK_POSITION);
    }
}

#ifndef __CC_ARM
#pragma GCC diagnostic pop
#endif

static T_DjiReturnCode DjiTest_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data, uint16_t dataSize,
                                                                       const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionQuaternion *quaternion = (T_DjiFcSubscriptionQuaternion *) data;
    dji_f64_t pitch, yaw, roll;

    USER_UTIL_UNUSED(dataSize);

    pitch = (dji_f64_t) asinf(-2 * quaternion->q1 * quaternion->q3 + 2 * quaternion->q0 * quaternion->q2) * 57.3;
    roll = (dji_f64_t) atan2f(2 * quaternion->q2 * quaternion->q3 + 2 * quaternion->q0 * quaternion->q1,
                             -2 * quaternion->q1 * quaternion->q1 - 2 * quaternion->q2 * quaternion->q2 + 1) * 57.3;
    yaw = (dji_f64_t) atan2f(2 * quaternion->q1 * quaternion->q2 + 2 * quaternion->q0 * quaternion->q3,
                             -2 * quaternion->q2 * quaternion->q2 - 2 * quaternion->q3 * quaternion->q3 + 1) *
          57.3;

    if (s_userFcSubscriptionDataShow == true) {
        if (s_userFcSubscriptionDataCnt++ % DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ == 0) {
            USER_LOG_INFO("receive quaternion data.");
            USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond,
                          timestamp->microsecond);
            USER_LOG_INFO("quaternion: %f %f %f %f.", quaternion->q0, quaternion->q1, quaternion->q2,
                          quaternion->q3);

            USER_LOG_INFO("euler angles: pitch = %.2f roll = %.2f yaw = %.2f.\r\n", pitch, roll, yaw);
            //DjiTest_WidgetLogAppend("pitch = %.2f roll = %.2f yaw = %.2f.", pitch, roll, yaw);
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


void replyProgress(MQTTAsync client, bool missionOK, bool inMission, float progress, int gateway)
{
    // 创建一个reply JSON对象
    cJSON *reply = cJSON_CreateObject();
	// 检查是否成功创建了JSON对象
	if (reply == NULL) {
		const char *error_ptr = cJSON_GetErrorPtr();
		if (error_ptr != NULL) {
			fprintf(stderr, "Error before: %s\n", error_ptr);
		}
		return 1;
	}
	// 添加键值对到JSON对象
	cJSON_AddNumberToObject(reply, "gateway", gateway);
    cJSON_AddNumberToObject(reply, "userid", userID);
    struct timeval tv = {0};
    // 获取当前时间（包括秒和微秒）
	gettimeofday(&tv, NULL);
	// 计算毫秒
	long milliseconds = tv.tv_sec * 1000 + tv.tv_usec / 1000;
	printf("Timestamp in milliseconds: %ld\n", milliseconds);
	cJSON_AddNumberToObject(reply, "timestamp", milliseconds);
    if(!missionOK)
    {
        cJSON_AddNumberToObject(reply, "result", 4); // 4 表示任务执行中段（无人机未完成某个指令）
        printf("--------------------------------------- mission interrupt ----------");
        isin_mission = false;
    }
    else if(inMission)
    {
        cJSON_AddNumberToObject(reply, "result", 5); // 5 表示任务执行正常（执行中)
    }
    else
    {
        cJSON_AddNumberToObject(reply, "result", 0); // 0 表示任务执行正常（执行完成，落地锁桨叶)
        isin_mission = false;
    }
    // cJSON_AddBoolToObject(reply, "isin_mission", inMission); // bool missionOK, bool inMission一起判断的结果给"result"
    cJSON_AddNumberToObject(reply, "progress", progress);

    // 将JSON对象转换为字符串以便打印或保存
	char *json_string = cJSON_Print(reply);
	if (json_string == NULL) {
		USER_LOG_ERROR("Failed to create JSON string.");
		cJSON_Delete(reply);
		return;
	}
	printf("JSON string: %s\n", json_string);
	// 在这里你可以将json_string保存到文件或发送到网络等
	int rc;
	pubmsg.payload = (void *)json_string;
	pubmsg.payloadlen = strlen(json_string);
	pubmsg.qos = QOS;
	pubmsg.retained = 0;

	// 对client的修改要加互斥锁
	// 看MQTTAsync_sendMessage   github上面是否有说线程安全性
	pthread_mutex_lock(&mqtt_publish_mutex);
	if ((rc = MQTTAsync_sendMessage(client, TOPIC_REPLY, &pubmsg, &opts)) != MQTTASYNC_SUCCESS) {
		printf("Failed to start sendMessage, return code %d\n", rc);
	} else {
		printf("Message published to topic %s\n", TOPIC_REPLY);
	}
	pthread_mutex_unlock(&mqtt_publish_mutex);
	cJSON_Delete(reply);
	cJSON_free(json_string);
}

static dji_f64_t computeProgress(dji_f64_t lat1, dji_f64_t lon1, dji_f64_t lat2, dji_f64_t lon2)
{
    // 计算当前位置与起始点的距离
    // 将经纬度转换为弧度
    dji_f64_t phi1 = lat1 * M_PI / 180;
    dji_f64_t phi2 = lat2 * M_PI / 180;
    dji_f64_t delta_phi = (lat2 - lat1) * M_PI / 180;
    dji_f64_t delta_lambda = (lon2 - lon1) * M_PI / 180;

    // 计算大圆距离
    dji_f64_t a = sin(delta_phi/2) * sin(delta_phi/2) + cos(phi1) * cos(phi2) * sin(delta_lambda/2) * sin(delta_lambda/2);
    dji_f64_t c = 2 * atan2(sqrt(a), sqrt(1-a));
    dji_f64_t distance = RADIUS_EARTH * c;

    return distance;
}

// 将角度转换成弧度
static dji_f64_t deg2rad(dji_f64_t deg) {
    return (deg * M_PI / 180);
}

// 计算两个经纬度点之间的距离
static dji_f64_t haversine(dji_f64_t lat1, dji_f64_t lon1, dji_f64_t lat2, dji_f64_t lon2) {
    dji_f64_t dLat = deg2rad(lat2 - lat1);
    dji_f64_t dLon = deg2rad(lon2 - lon1);
    dji_f64_t a = sin(dLat / 2) * sin(dLat / 2) +
               cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
               sin(dLon / 2) * sin(dLon / 2);
    dji_f64_t c = 2 * atan2(sqrt(a), sqrt(1-a));
    return RADIUS * c;
}

// 计算点到线段的最短距离
static dji_f64_t point_to_segment_distance(dji_f64_t latA, dji_f64_t lonA, dji_f64_t latB, dji_f64_t lonB, dji_f64_t latP, dji_f64_t lonP) {
    dji_f64_t distAB = haversine(latA, lonA, latB, lonB);
    if (distAB == 0) return haversine(latA, lonA, latP, lonP);

    // 计算点P到线段AB上的投影点D的比例
    dji_f64_t ratio = ((lonP - lonA) * (lonB - lonA) + (latP - latA) * (latB - latA)) /
                   (distAB * distAB);
    dji_f64_t latD = latA + ratio * (latB - latA);
    dji_f64_t lonD = lonA + ratio * (lonB - lonA);

    // 如果投影点不在AB上，则选择最近的端点
    if (ratio < 0) {
        return haversine(latA, lonA, latP, lonP);
    } else if (ratio > 1) {
        return haversine(latB, lonB, latP, lonP);
    } else {
        return haversine(latD, lonD, latP, lonP);
    }
}
/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
