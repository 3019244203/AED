// mqtt_function.h
#ifndef MQTT_FUNCTION_H
#define MQTT_FUNCTION_H

#include "MQTTAsync.h"
#include <stdbool.h>
#define ADDRESS     "mqtt://39.105.20.55:1883"
#define CLIENTID    "ExampleClientSub"
#define USERNAME    "AEDMatrice350"
#define PASSWORD    "AEDMatrice350tjyx"
#define TOPIC       "dispatch/topic"
#define TOPIC_CANCEL       "aid_cancel/topic"
#define TOPIC_responseCabins       "responseDronesByAirport/topic"
#define TOPIC_requestCabins       "requestDronesByAirport/topic"
#define TOPIC_responsePlacementPoint       "responsePlacementPoint/topic"
#define TOPIC_requestPlacementPoint       "requestPlacementPoint/topic"
// #define TOPIC_REPLY       "gcs_reply/1/process"
#define PAYLOAD     "Hello World!"
// #define QOS         2
#define TIMEOUT     10000L

// 定义一个名为 Coordinate 的结构体
typedef struct {
    double latitude;  // 纬度
    double longitude; // 经度
} Coordinate;

typedef struct {
    int index; // 对应于dronesArray中的索引
    double distance;
} IndexedDistance;


typedef struct {
    int droneId;
    double lat;
    double lng;
} DroneInfo;


extern int finished;
extern int subscribed;
extern int disc_finished;


// 函数声明
void connlost(void *context, char *cause);
int msgarrvd(void *context, char *topicName, int topicLen, MQTTAsync_message *message);
void onConnect(void* context, MQTTAsync_successData* response);
void onConnectFailure(void* context, MQTTAsync_failureData* response);


#endif // MYHEADER_H
