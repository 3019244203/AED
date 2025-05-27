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
#include <dji_flight_controller.h>
#include <unistd.h> // 包含对于write和sleep函数的声明

/* Private constants ---------------------------------------------------------*/
#define FC_SUBSCRIPTION_TASK_FREQ         (1)
#define FC_SUBSCRIPTION_TASK_STACK_SIZE   (1024)
// #define RADIUS 6371000 // 地球平均半径，单位为米
// #define GoHomeAti 50 // 返航高度

// #define TOPIC_REPLY       "gcs_reply/1/process"
// #define QOS         2
// #define M_PI		3.14159265358979323846
// #define RADIUS_EARTH 6371000 // 地球半径，单位：米
/* Private types -------------------------------------------------------------*/
// typedef struct {
//     E_DjiFcSubscriptionDisplayMode displayMode;
//     char *displayModeStr;
// } T_DjiTestFlightControlDisplayModeStr;

/* Private functions declaration ---------------------------------------------*/
static void *UserFcSubscription_Task(void *arg);
static void *DjiTest_FlightReturnAfterOpenCabinTask(void *arg);
static T_DjiReturnCode DjiTest_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data, uint16_t dataSize,
                                                                       const T_DjiDataTimestamp *timestamp);
// static dji_f64_t computeProgress(dji_f64_t lat1, dji_f64_t lon1, dji_f64_t lat2, dji_f64_t lon2);
static dji_f64_t deg2rad(dji_f64_t deg);
static void send_command(const char* command);
dji_f64_t haversine(dji_f64_t lat1, dji_f64_t lon1, dji_f64_t lat2, dji_f64_t lon2);
void control_airport(int choice);
static void latLonToXYZ(double lat_deg, double lon_deg, double* x, double* y, double* z);
static dji_f64_t point_to_segment_distance(dji_f64_t latA, dji_f64_t lonA, dji_f64_t latB, dji_f64_t lonB, dji_f64_t latP, dji_f64_t lonP);
// static uint8_t DjiTest_FlightControlGetDisplayModeIndex(E_DjiFcSubscriptionDisplayMode displayMode);

/* Private variables ---------------------------------------------------------*/
static T_DjiTaskHandle s_userFcSubscriptionThread;
static T_DjiTaskHandle s_resumegohomeThread;
static T_DjiTaskHandle s_gohomeafterhoverThread;
static T_DjiTaskHandle s_returnafteropencabinThread;
static bool s_userFcSubscriptionDataShow = false;
static uint8_t s_totalSatelliteNumberUsed = 0;
static uint32_t s_userFcSubscriptionDataCnt = 0;
static MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
static MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
// static dji_f64_t targetLat=0, targetLon=0;
// static dji_f64_t distanceTotal = 0;
static MQTTAsync client;
static time_t last_time;
static time_t hoverNow;
static time_t hoverLast_time;
static dji_f64_t accumulHoverTime = 0;
static bool hoverDueCode = false;
static bool sentResumeCommand = false;

static dji_f64_t last_lon = 120.8777396; /*!< Longitude, unit: deg. */
static dji_f64_t last_lat = 18.3650976;  /*!< Latitude, unit: deg. */
static dji_f32_t last_hei = 0;
// static bool in_air = false;
// static const T_DjiTestFlightControlDisplayModeStr s_flightControlDisplayModeStr[] = {
const T_DjiTestFlightControlDisplayModeStr s_flightControlDisplayModeStr[] = {
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE, .displayModeStr = "attitude mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS, .displayModeStr = "p_gps mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF, .displayModeStr = "assisted takeoff mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF, .displayModeStr = "auto takeoff mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING, .displayModeStr = "auto landing mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME, .displayModeStr = "go home mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING, .displayModeStr = "force landing mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ENGINE_START, .displayModeStr = "engine start mode"},
    {.displayMode = 0xFF, .displayModeStr = "unknown mode"}
};


bool finishedMission = false;
pthread_mutex_t mqtt_publish_mutex = PTHREAD_MUTEX_INITIALIZER;
bool isin_mission = false;
dji_f64_t schedule = 0;

uint8_t is_RTK_ready = 0;
uint8_t remainingBattery = 0;
DroneStatus droneStatus =  {0};
pthread_mutex_t statusMutex = PTHREAD_MUTEX_INITIALIZER;
dji_f64_t targetLat=0, targetLon=0;
uint32_t orderID=0;
uint8_t droneID=0;
uint8_t stationary=0;
uint8_t dMode=0;
bool stopview=false;
dji_f64_t distance_safe=0;
dji_f32_t relHeight=0;
dji_f32_t closeCabinHeight=0;
int16_t yaw = 0;
dji_f32_t vel=0;
dji_f32_t remainTime=0;
time_t start_time;
bool first_reply = true;
dji_f64_t distanceTotal = 0;
dji_f64_t cruiseSpeed = 1;
bool airportOpen = false;
int serial_port;
bool useCabin = false;
bool initializedController = false;
bool initializedWaypointV2 = false;

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

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic velocity error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic velocity success.");
    }

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

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic status displaymode error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } else {
        USER_LOG_DEBUG("Subscribe topic status displaymode success.");
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic avoid data failed,error code:0x%08llX", djiStat);
        return djiStat;
    }

    djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                               NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic rtk yaw failed,error code:0x%08llX", djiStat);
        return djiStat;
    }

    printf("test_fc_subscription.c");
    time(&hoverLast_time);

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
    T_DjiFcSubscriptionDisplaymode displayMode = {0};
    T_DjiFcSubscriptionHeightFusion heightFusion = {0};
    T_DjiFcSubscriptionRtkYaw rtkYaw = {0};
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
        // printf("test_fc_subscription.c----------------------------------\n");

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                          (uint8_t *) &velocity,
                                                          sizeof(T_DjiFcSubscriptionVelocity),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic velocity error.");
        }
        // if (s_userFcSubscriptionDataShow == true) {
        //     USER_LOG_INFO("velocity: x %f y %f z %f, healthFlag %d.", velocity.data.x, velocity.data.y,
        //                   velocity.data.z, velocity.health);
        // }
        vel = sqrt(pow(velocity.data.x,2)+pow(velocity.data.y,2));
        // if (s_userFcSubscriptionDataShow == true) {
        //     USER_LOG_INFO("Current speed: %f, speedZ: %f.\n", vel, velocity.data.z);
        // }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                          (uint8_t *) &gpsPosition,
                                                          sizeof(T_DjiFcSubscriptionGpsPosition),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps position error.");
        }
        closeCabinHeight = gpsPosition.z*1e-3;

        // if (s_userFcSubscriptionDataShow == true) {
        //     USER_LOG_INFO("gps position: x %f y %f z %f.", gpsPosition.x*1e-7, gpsPosition.y*1e-7, gpsPosition.z*1e-3);
        // }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
                                                          (uint8_t *) &gpsDetails,
                                                          sizeof(T_DjiFcSubscriptionGpsDetails),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic gps details error.");
        }

        // if (s_userFcSubscriptionDataShow == true) {
        //     USER_LOG_INFO("gps total satellite number used: %d %d %d.\nGPS fixState: %f.",
        //                   gpsDetails.gpsSatelliteNumberUsed,
        //                   gpsDetails.glonassSatelliteNumberUsed,
        //                   gpsDetails.totalSatelliteNumberUsed,
        //                   gpsDetails.fixState);
        // }
        s_totalSatelliteNumberUsed = gpsDetails.totalSatelliteNumberUsed;

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO,
                                                          (uint8_t *) &homepointInfo,
                                                          sizeof(T_DjiFcSubscriptionHomePointInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of homepoint info error.");
        }

        // if (s_userFcSubscriptionDataShow == true) {
        //     USER_LOG_INFO("Homepoint info: lat->%f, lon->%f.", homepointInfo.latitude*(180.0/M_PI), homepointInfo.longitude*(180.0/M_PI));
        // }
        // if(isin_mission)
        // {
        //     distanceTotal = haversine(homepointInfo.latitude*(180.0/M_PI), homepointInfo.longitude*(180.0/M_PI), targetLat, targetLon);
        //     USER_LOG_INFO("distanceTotal: lat->%f, lon->%f. lat->%f, lon->%f.", homepointInfo.latitude*(180.0/M_PI), homepointInfo.longitude*(180.0/M_PI), targetLat, targetLon);
        // }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
                                                          (uint8_t *) &rtkPositionInfo,
                                                          sizeof(T_DjiFcSubscriptionRtkPositionInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of topic rtk position info error.");
        }

        // if (s_userFcSubscriptionDataShow == true) {
        //     USER_LOG_INFO("RTK position solution state: %d.", rtkPositionInfo);
        // }
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

        // if (s_userFcSubscriptionDataShow == true) {
        //     USER_LOG_INFO("RTK position: %.9lf %.9lf %.9f %.9f.",
        //                   rtkPosition.longitude,
        //                   rtkPosition.latitude,
        //                   rtkPosition.hfsl,
        //                   altitudeOfHomepoint);
        // }
        pthread_mutex_lock(&statusMutex); // 加锁以保护对共享资源的访问
        // droneStatus.rtkLongitude = rtkPosition.longitude;
        // droneStatus.rtkLatitude = rtkPosition.latitude;
        // droneStatus.relativeHeight = gpsPosition.z*1e-3;
	    droneStatus.rtkLongitude =  gpsPosition.x*1e-7;
        droneStatus.rtkLatitude = gpsPosition.y*1e-7;
        droneStatus.relativeHeight = gpsPosition.z*1e-3;
        printf("test_fc_subscription.c------------droneStatus----------------------%f,  %f,  %f\n", droneStatus.rtkLongitude, droneStatus.rtkLatitude, droneStatus.relativeHeight);
        pthread_mutex_unlock(&statusMutex); // 解锁
        if(isin_mission)
        {
            
            // dji_f64_t distanceCurrent = haversine(homepointInfo.latitude*(180.0/M_PI), homepointInfo.longitude*(180.0/M_PI), rtkPosition.latitude, rtkPosition.longitude);
            // schedule = distanceCurrent / distanceTotal;
            // printf("test_fc_subscription.c------------isin_mission--------------replyProgress--------\n");
            // printf("distanceCurrent： %f;  distanceTotal: %f\n", distanceCurrent, distanceTotal);
            // replyProgress(client, true, true, schedule, 1);
            // distance_safe = point_to_segment_distance(homepointInfo.latitude*(180.0/M_PI), homepointInfo.longitude*(180.0/M_PI), targetLat, targetLon, rtkPosition.latitude, rtkPosition.longitude);
            // dji_f64_t distanceCurrent = haversine(homepointInfo.latitude*(180.0/M_PI), homepointInfo.longitude*(180.0/M_PI), gpsPosition.y*1e-7, gpsPosition.x*1e-7);
            distanceTotal = haversine(homepointInfo.latitude*(180.0/M_PI), homepointInfo.longitude*(180.0/M_PI), targetLat, targetLon);
            // dji_f64_t distanceremain = haversine(rtkPosition.latitude, rtkPosition.longitude, targetLat, targetLon);
            dji_f64_t distanceremain = haversine(gpsPosition.y*1e-7, gpsPosition.x*1e-7, targetLat, targetLon);
            cruiseSpeed = distanceTotal / 240;
            if(cruiseSpeed<3)  cruiseSpeed=3;
            else if(cruiseSpeed>13)    cruiseSpeed=13;
            remainTime = distanceremain / cruiseSpeed; //这里的速度不用vel，因为takeoff时vel水平速度接近0，时间就会显示很大
            // USER_LOG_INFO("distanceTotal: %f, remainTime: %f, cruiseSpeed: %f.\n", distanceTotal, remainTime, cruiseSpeed);

            // 获取当前时间
			time_t now;
			time(&now);
            if(first_reply)
            {
                first_reply = false;
                time(&last_time);
                // 转换为本地时间结构
                struct tm *local = localtime(&now);
                // 格式化输出时间，精确到秒
                USER_LOG_INFO("当前时间是: %04d-%02d-%02d %02d:%02d:%02d\n",
                    local->tm_year + 1900,
                    local->tm_mon + 1,
                    local->tm_mday,
                    local->tm_hour,
                    local->tm_min,
                    local->tm_sec);
                dji_f64_t diff = difftime(now, start_time);
                USER_LOG_INFO("时间差是: %.0f 秒\n", diff);
                replyProgress(client, true, true, schedule, orderID, droneID); //无人机正在执行任务
            } else {
                dji_f64_t diff = difftime(now, last_time);
                if(diff >= 5)
                {
                    time(&last_time);
                    replyProgress(client, true, true, schedule, orderID, droneID); //无人机正在执行任务
                }
            }
			

            
            // schedule = distanceCurrent / distanceTotal;
            // printf("test_fc_subscription.c------------isin_mission--------------replyProgress--------\n");
            // printf("distanceCurrent： %f;  distanceTotal: %f\n", distanceCurrent, distanceTotal);
            // replyProgress(client, true, true, schedule, orderid, droneid);
            // distance_safe = point_to_segment_distance(homepointInfo.latitude*(180.0/M_PI), homepointInfo.longitude*(180.0/M_PI), targetLat, targetLon, rtkPosition.latitude, rtkPosition.longitude);
            distance_safe = point_to_segment_distance(homepointInfo.latitude*(180.0/M_PI), homepointInfo.longitude*(180.0/M_PI), targetLat, targetLon, gpsPosition.y*1e-7, gpsPosition.x*1e-7);
            printf("distance_safe: %f\n", distance_safe);
            // USER_LOG_INFO("(distance_safe: %f)  (Homepoint_info: lat->%f, lon->%f.)  (targetPoint: lat->%f, lon->%f.)  (currentPoint: lat->%f, lon->%f.)\n", distance_safe, homepointInfo.latitude*(180.0/M_PI), homepointInfo.longitude*(180.0/M_PI), targetLat, targetLon, rtkPosition.latitude, rtkPosition.longitude);
            // USER_LOG_INFO("Homepoint info: lat->%f, lon->%f.", homepointInfo.latitude*(180.0/M_PI), homepointInfo.longitude*(180.0/M_PI));
        } 

        if(!hoverDueCode)
        {
            if(accumulHoverTime >= 60)
            {
                USER_LOG_INFO("accumulHoverTime: %f.", accumulHoverTime);
                // 悬停超过1分钟执行返航
                if (osalHandler->TaskCreate("gohome_afterhover_task", DjiTest_FlightControlGoHomeForceLandingTask,
                    FC_SUBSCRIPTION_TASK_STACK_SIZE, NULL, &s_gohomeafterhoverThread) !=
                    DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("user gohome afterhover task create error.");
                } else {
                    USER_LOG_INFO("user gohome afterhover task create successfully.");
                }
            } else {
                dji_f64_t hoverDistanceHorizon = haversine(last_lat, last_lon, gpsPosition.y*1e-7, gpsPosition.x*1e-7);
                dji_f32_t hoverDistanceVetical = fabs(last_hei-gpsPosition.z*1e-3);
                time(&hoverNow);
                dji_f64_t diff = difftime(hoverNow, hoverLast_time);
                if(diff >= 5)
                {
                    last_lat = gpsPosition.y*1e-7;
                    last_lon = gpsPosition.x*1e-7;
                    USER_LOG_INFO("accumulHoverTime: %f, diff: %f, hoverDistanceHorizon: %f, hoverDistanceVetical: %f, last_lat: %f, last_lon: %f, gpsPosition.y*1e-7: %f, gpsPosition.x*1e-7: %f.", accumulHoverTime, diff, hoverDistanceHorizon, hoverDistanceVetical, last_lat, last_lon, gpsPosition.y*1e-7, gpsPosition.x*1e-7);
                    if(flightStatus==DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR && hoverDistanceHorizon<=2.0 && hoverDistanceVetical<=2.0)
                    {
                        accumulHoverTime += 5;
                    } else {
                        accumulHoverTime = 0;
                    }
                    time(&hoverLast_time);
                }
            }
        }
        
        


        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                                                          (uint8_t *) &altitudeOfHomepoint,
                                                          sizeof(T_DjiFcSubscriptionAltitudeOfHomePoint),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of altitude of homepoint error.");
        }

        // if (s_userFcSubscriptionDataShow == true) {
        //     USER_LOG_INFO("Altitude of homepoint: %f.", altitudeOfHomepoint);

        // }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO,
                                                          (uint8_t *) &batteryInfo,
                                                          sizeof(T_DjiFcSubscriptionWholeBatteryInfo),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of battery info error.");
        }

        // if (s_userFcSubscriptionDataShow == true) {
        //     USER_LOG_INFO("Battery info: %d.", batteryInfo.percentage);
        // }
        remainingBattery = batteryInfo.percentage;
        // printf("test_fc_subscription.c------------remainingBattery----------------------%d\n", remainingBattery);

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                          (uint8_t *) &flightStatus,
                                                          sizeof(T_DjiFcSubscriptionFlightStatus),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of flight status error.");
        }

        // if (s_userFcSubscriptionDataShow == true) {
        //     USER_LOG_INFO("Flight status: %d.", flightStatus);
        // }
        stationary=flightStatus;
        USER_LOG_INFO("finishedMission: %d-----------flightStatus: %d--------------isin_mission: %d--------\n", finishedMission, flightStatus, isin_mission);
        if(finishedMission && flightStatus==0 && isin_mission) {
            USER_LOG_INFO("test_fc_subscription.c------------finishedMission--------------replyProgress--------\n");
            replyProgress(client, true, false, 1, orderID, droneID); //无人机完成任务并降落在地面且锁桨
            finishedMission = false;
            stopview = true;
        }

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                                          (uint8_t *) &displayMode,
                                                          sizeof(T_DjiFcSubscriptionDisplaymode),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("get value of displaymode status error.");
        }
        if (s_userFcSubscriptionDataShow == true) {
            USER_LOG_INFO("Displaymode status: %d.", displayMode);
        }
        dMode = displayMode;

        dji_f64_t distanceCurrent = haversine(homepointInfo.latitude*(180.0/M_PI), homepointInfo.longitude*(180.0/M_PI), gpsPosition.y*1e-7, gpsPosition.x*1e-7);
        // if(displayMode==DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING && !airportOpen && closeCabinHeight<=20 && distanceCurrent<=1 && !sentCloseCommandToCabin && !sentResumeCommand)
        if((displayMode==DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING || displayMode==DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING) && !airportOpen && closeCabinHeight<=20 && distanceCurrent<=1 && !sentResumeCommand)
        {
            hoverDueCode = true;
            if (osalHandler->TaskCreate("return_afteropencabin_task", DjiTest_FlightReturnAfterOpenCabinTask,
                FC_SUBSCRIPTION_TASK_STACK_SIZE, NULL, &s_returnafteropencabinThread) !=
                DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                hoverDueCode = false;
                USER_LOG_ERROR("user return after open cabin task create error.");
            } else {
                USER_LOG_INFO("user return after open cabin task create successfully.");
            }
        }

        
        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                                          (uint8_t *) &heightFusion,
                                                          sizeof(T_DjiFcSubscriptionHeightFusion),
                                                          &timestamp);

        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Get value of topic height fusion error, error code: 0x%08X", djiStat);
        }
        // if (s_userFcSubscriptionDataShow == true) {
        //     USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", timestamp.millisecond, timestamp.microsecond);
        //     USER_LOG_DEBUG("Relative height fusion is %f m", heightFusion);
        // }
        relHeight = heightFusion;  // 虽然heightFusion是用超声波传感器，在高度过高时是0，但是relHeight只用于0.65到0.75之间距离地面较近的判断，是可以的

        djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW,
                                                          (uint8_t *) &rtkYaw,
                                                          sizeof(T_DjiFcSubscriptionRtkYaw),
                                                          &timestamp);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Get value of topic height fusion error, error code: 0x%08X", djiStat);
        }
        // if (s_userFcSubscriptionDataShow == true) {
        //     USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", timestamp.millisecond, timestamp.microsecond);
        //     USER_LOG_DEBUG("Relative height fusion is %f m", heightFusion);
        // }
        yaw = rtkYaw;
        

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


void replyProgress(MQTTAsync client, bool missionOK, bool inMission, float progress, uint32_t order, uint8_t gateway)
{
    struct timeval tv = {0};
    // 创建一个reply JSON对象
    cJSON *reply = cJSON_CreateObject();
	// 检查是否成功创建了JSON对象
	if (reply == NULL) {
		const char *error_ptr = cJSON_GetErrorPtr();
		if (error_ptr != NULL) {
			fprintf(stderr, "Error before: %s\n", error_ptr);
		}
		return;
	}

    // 添加键值对到JSON对象
    char url[50];
    snprintf(url, sizeof(url), "rtmp://39.105.20.55:1935/live/streamtjyx%d", gateway);
    cJSON_AddStringToObject(reply, "rtmpUrl", url);
    cJSON_AddNumberToObject(reply, "helpOrderId", order);
    cJSON_AddNumberToObject(reply, "droneId", gateway);   // 后续多台无人机根据距离判断修改droneId

    cJSON *log = cJSON_CreateObject();
    if (log == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            fprintf(stderr, "Error before: %s\n", error_ptr);
        }
        // cJSON_Delete(root);
        // return 1;
        cJSON_Delete(reply);
        return;
    }
    cJSON_AddNumberToObject(log, "orderId", order);
    cJSON_AddNumberToObject(log, "droneId", gateway);   // 后续多台无人机根据距离判断修改droneId
    cJSON_AddNumberToObject(log, "battery", remainingBattery);
    cJSON_AddNumberToObject(log, "yaw", yaw);
    pthread_mutex_lock(&statusMutex); // 加锁以保护对共享资源的访问
    dji_f64_t currentLon = droneStatus.rtkLongitude;
    dji_f64_t currentLat = droneStatus.rtkLatitude;
    dji_f32_t curHei = droneStatus.relativeHeight;
    pthread_mutex_unlock(&statusMutex); // 解锁
    char str[100]; // 确保数组足够大以容纳结果字符串
    snprintf(str, sizeof(str), "POINT(%.9f %.9f)", currentLon, currentLat);
    cJSON_AddStringToObject(log, "coordinate", str);
    cJSON_AddStringToObject(log, "photoUrl", "");
    // 获取当前时间（包括秒和微秒）
    gettimeofday(&tv, NULL);
    // 计算毫秒
    long milliseconds = tv.tv_sec * 1000 + tv.tv_usec / 1000;
    printf("Timestamp in milliseconds: %ld\n", milliseconds);
    cJSON_AddNumberToObject(log, "timestamp", milliseconds);
    cJSON_AddNumberToObject(log, "altitude", curHei);
    cJSON_AddNumberToObject(log, "speed", vel);
    snprintf(str, sizeof(str), "LINESTRING(%.9f %.9f, %.9f %.9f)", currentLat, currentLon, targetLat, targetLon);
    cJSON_AddStringToObject(log, "remainingPath", str);
    cJSON_AddNumberToObject(log, "remainingTime", remainTime);

    if(dMode==41||dMode==11)    cJSON_AddStringToObject(log, "flightMode", "TAKEOFF");
    else if(dMode==14)  cJSON_AddStringToObject(log, "flightMode", "CRUISE");
    else if(dMode==12 || dMode==33) cJSON_AddStringToObject(log, "flightMode", "LAND");
    // else    cJSON_AddStringToObject(log, "flightMode", "");
    cJSON_AddBoolToObject(reply, "result", true);
    if(!missionOK)
    {
        // cJSON_AddNumberToObject(reply, "result", 4); // 4 表示任务执行中段（无人机未完成某个指令）
        // cJSON_AddBoolToObject(reply, "result", false); // false表示任务执行中段（无人机未完成某个指令）
        cJSON_AddStringToObject(log, "workStatus", "INTERRUPTED");
        cJSON_AddItemToObject(reply, "droneLog", log);
        USER_LOG_INFO("--------------------------------------- mission interrupt ----------");
        isin_mission = false;
        stopview = true;
        first_reply = true;
        accumulHoverTime = 0;
    }
    else if(inMission)
    {
        // cJSON_AddNumberToObject(reply, "result", 5); // 5 表示任务执行正常（执行中)
        // cJSON_AddBoolToObject(reply, "result", true); // true表示务执行正常（执行中)
        cJSON_AddStringToObject(log, "workStatus", "EXECUTING");
        cJSON_AddItemToObject(reply, "droneLog", log);
        // USER_LOG_INFO("--------------------------------------- mission ing…… ----------");
    }
    else
    {
        // cJSON_AddNumberToObject(reply, "result", 0); // 0 表示任务执行正常（执行完成，落地锁桨叶)
        // cJSON_AddBoolToObject(reply, "result", true); // true表示务执行正常（结束)
        cJSON_AddStringToObject(log, "workStatus", "COMPLETED");
        cJSON_AddItemToObject(reply, "droneLog", log);
        isin_mission = false;
        first_reply = true;
        accumulHoverTime = 0;
        USER_LOG_INFO("--------------------------------------- mission finished!!! ----------");
    }
    
    
    // // cJSON_AddBoolToObject(reply, "isin_mission", inMission); // bool missionOK, bool inMission一起判断的结果给"result"
    // cJSON_AddNumberToObject(reply, "progress", progress);

    // 将JSON对象转换为字符串以便打印或保存
	char *json_string = cJSON_Print(reply);
	if (json_string == NULL) {
		USER_LOG_ERROR("Failed to create JSON string.");
        cJSON_Delete(log);
		cJSON_Delete(reply);
		return;
	}
    printf("LOG JSON string: %s\n", json_string);
	// USER_LOG_INFO("JSON string: %s\n", json_string);
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
		USER_LOG_ERROR("Failed to start sendMessage, return code %d\n", rc);
	} 
    // else {
	// 	USER_LOG_INFO("Message published to topic %s\n", TOPIC_REPLY);
	// }
	pthread_mutex_unlock(&mqtt_publish_mutex);
    cJSON_free(json_string);
    // 由于log对象被添加到了reply对象中（通过cJSON_AddItemToObject(reply, "droneLog", log);），一旦log被加入到reply中，其所有权就转移给了reply。这意味着当调用cJSON_Delete(reply);时，reply及其所有子项（包括log）都将被释放。因此，在最终释放reply之前不应该再单独调用cJSON_Delete(log);，否则会导致重复释放log导致未定义行为。
    // cJSON_Delete(log);
	cJSON_Delete(reply);
	
}

// static dji_f64_t computeProgress(dji_f64_t lat1, dji_f64_t lon1, dji_f64_t lat2, dji_f64_t lon2)
// {
//     // 计算当前位置与起始点的距离
//     // 将经纬度转换为弧度
//     dji_f64_t phi1 = lat1 * M_PI / 180;
//     dji_f64_t phi2 = lat2 * M_PI / 180;
//     dji_f64_t delta_phi = (lat2 - lat1) * M_PI / 180;
//     dji_f64_t delta_lambda = (lon2 - lon1) * M_PI / 180;

//     // 计算大圆距离
//     dji_f64_t a = sin(delta_phi/2) * sin(delta_phi/2) + cos(phi1) * cos(phi2) * sin(delta_lambda/2) * sin(delta_lambda/2);
//     dji_f64_t c = 2 * atan2(sqrt(a), sqrt(1-a));
//     dji_f64_t distance = RADIUS_EARTH * c;

//     return distance;
// }

// 将角度转换成弧度
static dji_f64_t deg2rad(dji_f64_t deg) {
    return (deg * M_PI / 180);
}

// 计算两个经纬度点之间的距离
dji_f64_t haversine(dji_f64_t lat1, dji_f64_t lon1, dji_f64_t lat2, dji_f64_t lon2) {
    dji_f64_t dLat = deg2rad(lat2 - lat1);
    dji_f64_t dLon = deg2rad(lon2 - lon1);
    dji_f64_t a = sin(dLat / 2) * sin(dLat / 2) +
               cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
               sin(dLon / 2) * sin(dLon / 2);
    dji_f64_t c = 2 * atan2(sqrt(a), sqrt(1-a));
    return RADIUS_EARTH * c;
}

// 将经纬度转换为三维笛卡尔坐标
static void latLonToXYZ(double lat_deg, double lon_deg, double* x, double* y, double* z) {
    double lat = deg2rad(lat_deg);
    double lon = deg2rad(lon_deg);
    *x = RADIUS_EARTH * cos(lat) * cos(lon);
    *y = RADIUS_EARTH * cos(lat) * sin(lon);
    *z = RADIUS_EARTH * sin(lat);
}

// // 计算点到线段的最短距离
// static dji_f64_t point_to_segment_distance(dji_f64_t latA, dji_f64_t lonA, dji_f64_t latB, dji_f64_t lonB, dji_f64_t latP, dji_f64_t lonP) {
//     dji_f64_t distAB = haversine(latA, lonA, latB, lonB);
//     if (distAB == 0) return haversine(latA, lonA, latP, lonP);

//     // 计算点P到线段AB上的投影点D的比例
//     dji_f64_t ratio = ((lonP - lonA) * (lonB - lonA) + (latP - latA) * (latB - latA)) /
//                    (distAB * distAB);
//     dji_f64_t latD = latA + ratio * (latB - latA);
//     dji_f64_t lonD = lonA + ratio * (lonB - lonA);

//     USER_LOG_INFO("(distance_safe: %f)  (Homepoint_info: lat->%f, lon->%f.)  (targetPoint: lat->%f, lon->%f.)  (currentPoint: lat->%f, lon->%f.)  (projectionPoint: lat->%f, lon->%f.)  ratio: %f.\n", distance_safe, latA, lonA, latB, lonB, latP, lonP, latD, lonD, ratio);
//     // 如果投影点不在AB上，则选择最近的端点
//     if (ratio < 0) {
//         return haversine(latA, lonA, latP, lonP);
//     } else if (ratio > 1) {
//         return haversine(latB, lonB, latP, lonP);
//     } else {
//         return haversine(latD, lonD, latP, lonP);
//     }
// }

// 计算点到线段的最短距离
static dji_f64_t point_to_segment_distance(dji_f64_t latA, dji_f64_t lonA, dji_f64_t latB, dji_f64_t lonB, dji_f64_t latP, dji_f64_t lonP) {
    dji_f64_t Ax, Ay, Az, Bx, By, Bz, Px, Py, Pz;
    latLonToXYZ(latA, lonA, &Ax, &Ay, &Az);
    latLonToXYZ(latB, lonB, &Bx, &By, &Bz);
    latLonToXYZ(latP, lonP, &Px, &Py, &Pz);

    // 向量 AB 和 AP
    dji_f64_t ABx = Bx - Ax, ABy = By - Ay, ABz = Bz - Az;
    dji_f64_t APx = Px - Ax, APy = Py - Ay, APz = Pz - Az;
    // 点积和长度平方
    dji_f64_t ab2 = ABx*ABx + ABy*ABy + ABz*ABz;
    dji_f64_t ap_ab = APx*ABx + APy*ABy + APz*ABz;
    // 投影参数
    dji_f64_t t = ap_ab / ab2; 

    dji_f64_t closestLat, closestLon;
    // 计算投影点的坐标
    double projX = Ax + t * ABx;
    double projY = Ay + t * ABy;
    double projZ = Az + t * ABz;
    // 将投影点转换为经纬度（反解）
    double norm = sqrt(projX*projX + projY*projY + projZ*projZ);
    projX /= norm;
    projY /= norm;
    projZ /= norm;

    closestLat = asin(projZ) * 180.0 / M_PI;
    closestLon = atan2(projY, projX) * 180.0 / M_PI;
    USER_LOG_INFO("(distance_safe: %f)  (Homepoint_info: lat->%f, lon->%f.)  (targetPoint: lat->%f, lon->%f.)  (currentPoint: lat->%f, lon->%f.)  (projectionPoint: lat->%f, lon->%f.)  ratio: %f.\n", distance_safe, latA, lonA, latB, lonB, latP, lonP, closestLat, closestLon, t);

    if (t < 0.0) {
        // 最接近点A
        return haversine(latP, lonP, latA, lonA);
    } else if (t > 1.0) {
        // 最接近点B
        return haversine(latP, lonP, latB, lonB);
    } else {
        return haversine(latP, lonP, closestLat, closestLon);
    }
}


uint8_t DjiTest_FlightControlGetDisplayModeIndex(E_DjiFcSubscriptionDisplayMode displayMode)
{
    uint8_t i;

    for (i = 0; i < sizeof(s_flightControlDisplayModeStr) / sizeof(T_DjiTestFlightControlDisplayModeStr); i++) {
        if (s_flightControlDisplayModeStr[i].displayMode == displayMode) {
            return i;
        }
    }

    return i;
}

void *DjiTest_FlightControlGoHomeForceLandingTask(void *arg)
{
    USER_LOG_DEBUG("Init flight Control Sample");
    T_DjiReturnCode returnCode;
    T_DjiFlightControllerRidInfo ridInfo = {0};

    T_DjiOsalHandler *s_osalHandler  = DjiPlatform_GetOsalHandler();
    if (!s_osalHandler) 
    {
        USER_LOG_ERROR("DjiPlatform_GetOsalHandler error.");

        return NULL;
    }

    ridInfo.latitude = 22.542812;
    ridInfo.longitude = 113.958902;
    ridInfo.altitude = 10;
    if(!initializedController)
    {
        returnCode = DjiFlightController_Init(ridInfo);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Init flight controller module failed, error code:0x%08llX", returnCode);
            // goto out;
            return NULL;
        } else {
            initializedController = true;
        }
    }
    // returnCode = DjiFlightController_RegJoystickCtrlAuthorityEventCallback(
    //     DjiTest_FlightControlJoystickCtrlAuthSwitchEventCallback);
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT) {
    //     USER_LOG_ERROR("Register joystick control authority event callback failed,error code:0x%08llX", returnCode);
    //     return returnCode;
    // }

    USER_LOG_INFO("Flight control go-home-force-landing sample start");
    // // RC must be in p-mode.
    // USER_LOG_INFO("--> Step 1: Obtain joystick control authority");
    // returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    // 	USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
    // 	goto out;
    // }
    // s_osalHandler->TaskSleepMs(1000);
    // USER_LOG_INFO("--> Set go home altitude to %d(m)\r\n", GoHomeAti);
    // returnCode = DjiFlightController_SetGoHomeAltitude(GoHomeAti);
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Set go home altitude to %d(m) failed, error code: 0x%08X", GoHomeAti, returnCode);
    //     goto out;
    // }
    /*! get go home altitude */
    E_DjiFlightControllerGoHomeAltitude goHomeAltitude;
    returnCode = DjiFlightController_GetGoHomeAltitude(&goHomeAltitude);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get go home altitude failed, error code: 0x%08X", returnCode);
        goto out;
    }
    USER_LOG_INFO("Current go home altitude is %d m\r\n", goHomeAltitude);

    USER_LOG_INFO("--> Step 6: Set go home altitude to 60(m)\r\n");
    if(goHomeAltitude!=60)
    {
        returnCode = DjiFlightController_SetGoHomeAltitude(60);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Set go home altitude to 60(m) failed, error code: 0x%08X", returnCode);
            goto out;
        }
    }
    

    USER_LOG_INFO("--> Go home and confirm force landing\r\n");

    E_DjiFlightControllerObstacleAvoidanceEnableStatus enableStatus;
    returnCode = DjiFlightController_GetDownwardsVisualObstacleAvoidanceEnableStatus(&enableStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get downwards visual obstacle avoidance enable status error");
    }
    USER_LOG_INFO("Start go home action");
    returnCode = DjiFlightController_StartGoHome();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Start to go home failed, error code: 0x%08X", returnCode);
        goto out;
    }
    
    int actionNotStarted = 0;
    int timeoutCycles = 20;
    while (dMode != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME && actionNotStarted < timeoutCycles) {
        actionNotStarted++;
        s_osalHandler->TaskSleepMs(100);
    }

    if (actionNotStarted == timeoutCycles) {
        USER_LOG_ERROR("%s start failed, now flight is in %s.",
                        s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME)].displayModeStr,
                        s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(
                            dMode)].displayModeStr);
        goto out;
    } else {
        USER_LOG_INFO("Now flight is in %s.",
                        s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(dMode)].displayModeStr);
        while (stationary == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
            dMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME) {
            s_osalHandler->TaskSleepMs(1000);// waiting for this action finished
        }
    }
    // if (!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME)) {
    //     goto out;
    // } else {
    //     while (stationary == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
    //         dMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME) {
    //         s_osalHandler->TaskSleepMs(1000);// waiting for this action finished
    //     }
    // }
    /*Start landing */
    USER_LOG_INFO("Start landing action");
    actionNotStarted = 0;
    timeoutCycles = 20;
    while (dMode != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING && actionNotStarted < timeoutCycles) {
        actionNotStarted++;
        s_osalHandler->TaskSleepMs(100);
    }

    if (actionNotStarted == timeoutCycles) {
        USER_LOG_ERROR("%s start failed, now flight is in %s.",
                        s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING)].displayModeStr,
                        s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(
                            dMode)].displayModeStr);
        goto out;
    } else {
        USER_LOG_INFO("Now flight is in %s.",
                        s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(dMode)].displayModeStr);
        while (dMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING &&
            stationary == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
            s_osalHandler->TaskSleepMs(1000);
            if ((dji_f64_t) 0.65 < relHeight && relHeight < (dji_f64_t) 0.75) {
                break;
            }
        }
    }
    // if (!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING)) {
    //     USER_LOG_ERROR("Fail to execute Landing action");
    //     goto out;
    // } else {
    //     while (dMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING &&
    //             stationary == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
    //         s_osalHandler->TaskSleepMs(1000);
    //         if ((dji_f64_t) 0.65 < relHeight && relHeight < (dji_f64_t) 0.75) {
    //             break;
    //         }
    //     }
    // }
    /*Confirm Landing */
    USER_LOG_INFO("Start confirm Landing and avoid ground action");
    returnCode = DjiFlightController_StartConfirmLanding();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Fail to execute confirm landing avoid ground action, error code: 0x%08X", returnCode);
        goto out;
    }
    if (enableStatus == DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE) {
        actionNotStarted = 0;
        timeoutCycles = 20;
        while (dMode != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING && actionNotStarted < timeoutCycles) {
            actionNotStarted++;
            s_osalHandler->TaskSleepMs(100);
        }

        if (actionNotStarted == timeoutCycles) {
            USER_LOG_ERROR("%s start failed, now flight is in %s.",
                            s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING)].displayModeStr,
                            s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(
                                dMode)].displayModeStr);
            goto out;
        } else {
            USER_LOG_INFO("Now flight is in %s.",
                            s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(dMode)].displayModeStr);
            while (dMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING &&
                stationary == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
                s_osalHandler->TaskSleepMs(1000);
            }
        }
    } else {
        while (dMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING &&
            stationary == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
            s_osalHandler->TaskSleepMs(1000);
        }
    }
    /*Landing finished check*/
    if (dMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
        dMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE) {
        USER_LOG_INFO("Successful landing");
    } else {
        USER_LOG_ERROR("Landing finished, but the aircraft is in an unexpected mode. "
                       "Please connect DJI Assistant.");
        // return false;
        goto out;
    }
    USER_LOG_INFO("Successful go home and confirm force landing\r\n");

    // USER_LOG_INFO("-> Step 9: Release joystick authority");
    // returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    // 	USER_LOG_ERROR("Release joystick authority failed, error code: 0x%08X", returnCode);
    // 	goto out;
    // }
    // replyProgress(client, false, false, schedule, 1);

    // USER_LOG_DEBUG("Deinit Flight Control Sample");
    // returnCode = DjiFlightController_DeInit();
    // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    //     USER_LOG_ERROR("Deinit flight controller module failed, error code:0x%08llX",
    //                    returnCode);
    //     goto out;
    // }

out:
    USER_LOG_INFO("Deinit Flight Control Sample");
    if(initializedController)
    {
        returnCode = DjiFlightController_DeInit();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Deinit flight controller module failed, error code:0x%08llX",
                        returnCode);
            return NULL;
        } else {
            initializedController = false;
        }
    }
    USER_LOG_INFO("Flight control go-home-force-landing sample end");
}

static void send_command(const char* command) {
    write(serial_port, command, strlen(command));
}

void control_airport(int choice) {

        switch (choice) {
            case 1:
                send_command("<CO>\r\n");
                break;
            case 2:
                send_command("<CC>\r\n");
                break;
            case 3:
                send_command("<CP>\r\n");
                break;
            default:
                printf("Invalid choice. Please try again.\n");
        }
}

static void *DjiTest_FlightReturnAfterOpenCabinTask(void *arg)
{
    T_DjiReturnCode returnCode;
    // 控制无人机悬停或者停止返航的动作
    returnCode = DjiFlightController_CancelLanding();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Control hover failed, error code: 0x%08X", returnCode);
    } else {
        sentResumeCommand = true;
        USER_LOG_INFO("Control hover successfully.");
    }
    
    // 给机舱发送打开的命令
    if(useCabin)
    {
        control_airport(1);
    }
    // 有一个标志记录已经发送，之后不再发送
    // sentCloseCommandToCabin = true;
    USER_LOG_INFO("Have sent close command to cabin, sentResumeCommand: %d.\n", sentResumeCommand);

    // while(!airportOpen)
    // {
    //     sleep(1);
    // }

    USER_LOG_INFO("Resume go home action");
    
    
    T_DjiOsalHandler *s_osalHandler  = DjiPlatform_GetOsalHandler();
    if (!s_osalHandler){
        USER_LOG_ERROR("DjiPlatform_GetOsalHandler error.");
        sentResumeCommand = false;
        return NULL;
    }

    T_DjiFlightControllerRidInfo ridInfo = {0};
    ridInfo.latitude = 22.542812;
    ridInfo.longitude = 113.958902;
    ridInfo.altitude = 10;
    if(!initializedController)
    {
        returnCode = DjiFlightController_Init(ridInfo);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Init flight controller module failed, error code:0x%08llX", returnCode);
            sentResumeCommand = false;
            return NULL;
        } else {
            initializedController = true;
        }
    }
    E_DjiFlightControllerGoHomeAltitude goHomeAltitude;
    returnCode = DjiFlightController_GetGoHomeAltitude(&goHomeAltitude);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get go home altitude failed, error code: 0x%08X", returnCode);
    }
    USER_LOG_INFO("Current go home altitude is %d m\r\n", goHomeAltitude);
    if(goHomeAltitude!=60)
    {
        returnCode = DjiFlightController_SetGoHomeAltitude(60);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Set go home altitude to 60(m) failed, error code: 0x%08X", returnCode);
        }
    }
    E_DjiFlightControllerObstacleAvoidanceEnableStatus enableStatus;
    returnCode = DjiFlightController_GetDownwardsVisualObstacleAvoidanceEnableStatus(&enableStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get downwards visual obstacle avoidance enable status error");
    }
    returnCode = DjiFlightController_StartGoHome();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Resume go home failed, error code: 0x%08X", returnCode);
        goto out;
    } else {    // 恢复返航成功
        USER_LOG_INFO("user resume gohome task create successfully.");
    }
    int actionNotStarted = 0;
    int timeoutCycles = 20;
    while (dMode != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME && actionNotStarted < timeoutCycles) {
        actionNotStarted++;
        s_osalHandler->TaskSleepMs(100);
    }
    if (actionNotStarted == timeoutCycles) {
        USER_LOG_ERROR("%s start failed, now flight is in %s.",
                        s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME)].displayModeStr,
                        s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(
                            dMode)].displayModeStr);
        goto out;
    } else {
        USER_LOG_INFO("Now flight is in %s.",
                        s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(dMode)].displayModeStr);
        while (stationary == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
            dMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME) {
            s_osalHandler->TaskSleepMs(1000);// waiting for this action finished
        }
    }
    USER_LOG_INFO("Start landing action");
    actionNotStarted = 0;
    timeoutCycles = 20;
    while (dMode != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING && actionNotStarted < timeoutCycles) {
        actionNotStarted++;
        s_osalHandler->TaskSleepMs(100);
    }
    if (actionNotStarted == timeoutCycles) {
        USER_LOG_ERROR("%s start failed, now flight is in %s.",
                        s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING)].displayModeStr,
                        s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(
                            dMode)].displayModeStr);
        goto out;
    } else {
        USER_LOG_INFO("Now flight is in %s.",
                        s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(dMode)].displayModeStr);
        while (dMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING &&
            stationary == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
            s_osalHandler->TaskSleepMs(1000);
            if ((dji_f64_t) 0.65 < relHeight && relHeight < (dji_f64_t) 0.75) {
                break;
            }
        }
    }
    USER_LOG_INFO("Start confirm Landing and avoid ground action");
    returnCode = DjiFlightController_StartConfirmLanding();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Fail to execute confirm landing avoid ground action, error code: 0x%08X", returnCode);
        goto out;
    }
    if (enableStatus == DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE) {
        actionNotStarted = 0;
        timeoutCycles = 20;
        while (dMode != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING && actionNotStarted < timeoutCycles) {
            actionNotStarted++;
            s_osalHandler->TaskSleepMs(100);
        }
        if (actionNotStarted == timeoutCycles) {
            USER_LOG_ERROR("%s start failed, now flight is in %s.",
                            s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING)].displayModeStr,
                            s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(
                                dMode)].displayModeStr);
            goto out;
        } else {
            USER_LOG_INFO("Now flight is in %s.",
                            s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(dMode)].displayModeStr);
            while (dMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING &&
                stationary == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
                s_osalHandler->TaskSleepMs(1000);
            }
        }
    } else {
        while (dMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING &&
            stationary == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
            s_osalHandler->TaskSleepMs(1000);
        }
    }
    /*Landing finished check*/
    if (dMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
        dMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE) {
        USER_LOG_INFO("Successful landing");
        hoverDueCode = false;
        // sentResumeCommand = true;
    } else {
        USER_LOG_ERROR("Landing finished, but the aircraft is in an unexpected mode. "
                       "Please connect DJI Assistant.");
        // return false;
    }

out:
    sentResumeCommand = false;
    // hoverDueCode = false;
    USER_LOG_INFO("Deinit Flight Control Sample");
    if(initializedController)
    {
        returnCode = DjiFlightController_DeInit();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Deinit flight controller module failed, error code:0x%08llX",
                        returnCode);
            return NULL;
        } else {
            initializedController = false;
        }
    }
    USER_LOG_INFO("Successful resume go home and confirm force landing\r\n");

    USER_LOG_INFO("--------------------------------------------------------8: %d.\n", sentResumeCommand);
}
    
/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
