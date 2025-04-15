/**
 ********************************************************************
 * @file    test_liveview.c
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
#include "test_liveview.h"
#include "dji_liveview.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "dji_aircraft_info.h"
#include "time.h"
#include <pthread.h>  // 引入线程库
#include <fc_subscription/test_fc_subscription.h>

/* Private constants ---------------------------------------------------------*/
#define TEST_LIVEVIEW_STREAM_FILE_PATH_STR_MAX_SIZE             256
#define TEST_LIVEVIEW_STREAM_STROING_TIME_IN_SECONDS            20

#define TEST_LIVEVIEW_STREAM_REQUEST_I_FRAME_ON                 1
#define TEST_LIVEVIEW_STREAM_REQUEST_I_FRAME_TICK_IN_SECONDS    5

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/
static char s_fpvCameraStreamFilePath[TEST_LIVEVIEW_STREAM_FILE_PATH_STR_MAX_SIZE];
static char s_payloadCameraStreamFilePath[TEST_LIVEVIEW_STREAM_FILE_PATH_STR_MAX_SIZE];

FILE *ffmpeg_pipe;

/* Private functions declaration ---------------------------------------------*/
static void DjiTest_FpvCameraStreamCallback(E_DjiLiveViewCameraPosition position, const uint8_t *buf,
                                            uint32_t bufLen);
static void DjiTest_PayloadCameraStreamCallback(E_DjiLiveViewCameraPosition position, const uint8_t *buf,
                                                uint32_t bufLen);
static void* liveviewThreadFunc(void* arg);

                                                /* Exported functions definition ---------------------------------------------*/
T_DjiReturnCode DjiTest_LiveviewRunSample(E_DjiMountPosition mountPosition)
{
    pthread_t liveviewThread;
    T_DjiReturnCode returnCode;

    USER_LOG_INFO("Liveview sample start");

    // 创建并启动新线程来处理视频流
    if (pthread_create(&liveviewThread, NULL, liveviewThreadFunc, (void*)&mountPosition) != 0) {
        USER_LOG_ERROR("Failed to create liveview thread");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    // 主线程可以继续执行其他任务或立即返回
    USER_LOG_INFO("Liveview sample launched in a separate thread");

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;  // 返回成功状态
}


/* Exported functions definition ---------------------------------------------*/
/* 线程入口函数 */
static void* liveviewThreadFunc(void* arg)
{
    E_DjiMountPosition mountPosition = *(E_DjiMountPosition*)arg;
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    time_t currentTime = time(NULL);
    struct tm *localTime = NULL;
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo = {0};

    USER_LOG_INFO("Liveview sample start");

    returnCode = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get aircraft base info error");
        // return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        goto out;
    }

    USER_LOG_INFO("--> Step 1: Init liveview module");
    returnCode = DjiLiveview_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Liveview init failed, error code: 0x%08X", returnCode);
        goto out;
    }

    const char *command = "ffmpeg -re -i - -vcodec copy -f flv rtmp://39.105.20.55:1935/live/streamtjyx";
    ffmpeg_pipe = popen(command, "w");
    if (!ffmpeg_pipe) {
        printf("Failed to start ffmpeg\n");
    }

    USER_LOG_INFO("--> Step 2: Start h264 stream of the fpv and selected payload\r\n");

    if (aircraftInfoBaseInfo.aircraftSeries == DJI_AIRCRAFT_SERIES_M300 ||
        aircraftInfoBaseInfo.aircraftSeries == DJI_AIRCRAFT_SERIES_M350 ||
        aircraftInfoBaseInfo.aircraftSeries == DJI_AIRCRAFT_SERIES_M30) {

        localTime = localtime(&currentTime);
        sprintf(s_fpvCameraStreamFilePath, "fpv_stream_%04d%02d%02d_%02d-%02d-%02d.h264",
                localTime->tm_year + 1900, localTime->tm_mon + 1, localTime->tm_mday,
                localTime->tm_hour, localTime->tm_min, localTime->tm_sec);

        returnCode = DjiLiveview_StartH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_FPV, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT,
                                                 DjiTest_FpvCameraStreamCallback);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Request h264 of fpv failed, error code: 0x%08X", returnCode);
            goto out;
        }

        while (true) {
            if(stopview)
            {
                break;
            }
            USER_LOG_INFO("Storing camera h264 stream, second:.");
    #if TEST_LIVEVIEW_STREAM_REQUEST_I_FRAME_ON
            // if (i % TEST_LIVEVIEW_STREAM_REQUEST_I_FRAME_TICK_IN_SECONDS == 0) {
                returnCode = DjiLiveview_RequestIntraframeFrameData(DJI_LIVEVIEW_CAMERA_POSITION_FPV,
                                                                    DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Request stream I frame of fpv failed, error code: 0x%08X", returnCode);
                }
            // }
    #endif
            osalHandler->TaskSleepMs(1000);
        }
    }

    
    USER_LOG_INFO("--> Step 3: Stop h264 stream of the fpv and selected payload\r\n");
    if (aircraftInfoBaseInfo.aircraftSeries == DJI_AIRCRAFT_SERIES_M300 ||
        aircraftInfoBaseInfo.aircraftSeries == DJI_AIRCRAFT_SERIES_M350 ||
        aircraftInfoBaseInfo.aircraftSeries == DJI_AIRCRAFT_SERIES_M30) {
        returnCode = DjiLiveview_StopH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_FPV, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Request to stop h264 of fpv failed, error code: 0x%08X", returnCode);
            goto out;
        }
    }
    USER_LOG_INFO("Fpv stream is saved to file: %s", s_fpvCameraStreamFilePath);

    

    USER_LOG_INFO("--> Step 4: Deinit liveview module");
    returnCode = DjiLiveview_Deinit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Liveview deinit failed, error code: 0x%08X", returnCode);
        goto out;
    }

    if (ffmpeg_pipe) {
        pclose(ffmpeg_pipe);
        ffmpeg_pipe = NULL;
    }

out:
    USER_LOG_INFO("Liveview sample end");
    pthread_exit(NULL);  // 结束线程
    // return returnCode;
    return NULL;
}

/* Private functions definition-----------------------------------------------*/
static void DjiTest_FpvCameraStreamCallback(E_DjiLiveViewCameraPosition position, const uint8_t *buf,
                                            uint32_t bufLen)
{
    // FILE *fp = NULL;
    // size_t size;

    // fp = fopen(s_fpvCameraStreamFilePath, "ab+");
    // if (fp == NULL) {
    //     printf("fopen failed!\n");
    //     return;
    // }

    // size = fwrite(buf, 1, bufLen, fp);
    // if (size != bufLen) {
    //     fclose(fp);
    //     return;
    // }

    // fflush(fp);
    // fclose(fp);
    // 确保ffmpeg_pipe已成功创建
    if (!ffmpeg_pipe) {
        printf("ffmpeg pipe is not initialized.\n");
        return;
    }
    fwrite(buf, 1, bufLen, ffmpeg_pipe);
    fflush(ffmpeg_pipe);

}

static void DjiTest_PayloadCameraStreamCallback(E_DjiLiveViewCameraPosition position, const uint8_t *buf,
                                                uint32_t bufLen)
{
    FILE *fp = NULL;
    size_t size;

    fp = fopen(s_payloadCameraStreamFilePath, "ab+");
    if (fp == NULL) {
        printf("fopen failed!\n");
        return;
    }

    size = fwrite(buf, 1, bufLen, fp);
    if (size != bufLen) {
        fclose(fp);
        return;
    }

    fflush(fp);
    fclose(fp);
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
