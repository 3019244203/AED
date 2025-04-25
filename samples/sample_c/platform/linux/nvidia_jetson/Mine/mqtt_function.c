#include "MQTTAsync.h"
#include "mqtt_function.h"
#include "cjson/cJSON.h"
#include <sys/time.h>
#include <fc_subscription/test_fc_subscription.h>  // 为了ThreadParams, droneStatus
#include <waypoint_v2/test_waypoint_v2.h>
#include <math.h>
#include "dji_logger.h"
#include <dji_aircraft_info.h>
#include <liveview/test_liveview.h>
// #define RADIUS_EARTH 6371000 // 地球半径，单位：米
// #define M_PI		3.14159265358979323846
#define DJI_TEST_GOHOME_FORCELAND_TASK_STACK_SIZE   (1024)


static T_DjiTaskHandle s_gohomeForcelandThread;

int finished = 0;
int subscribed = 0;
int disc_finished = 0;
// bool isin_mission = false;
Coordinate stationPos = {39.126794, 117.120695};
MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
// pthread_mutex_t mqtt_publish_mutex = PTHREAD_MUTEX_INITIALIZER;

// 计算无人机当前位置和机巢的距离
float calculateGreatCircleDistance(double lat1, double lon1, double lat2, double lon2) {
    // 将经纬度转换为弧度
    double phi1 = lat1 * M_PI / 180;
    double phi2 = lat2 * M_PI / 180;
    double delta_phi = (lat2 - lat1) * M_PI / 180;
    double delta_lambda = (lon2 - lon1) * M_PI / 180;
    // 计算大圆距离
    double a = sin(delta_phi/2) * sin(delta_phi/2) + cos(phi1) * cos(phi2) * sin(delta_lambda/2) * sin(delta_lambda/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    float distance = RADIUS_EARTH * c;
    return distance;
}


void connlost(void *context, char *cause)
{
	MQTTAsync client = (MQTTAsync)context;
	MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
	int rc;

	printf("\nConnection lost\n");
	if (cause)
		printf("     cause: %s\n", cause);

	printf("Reconnecting\n");
	conn_opts.keepAliveInterval = 20;
	conn_opts.cleansession = 1;
	conn_opts.onSuccess = onConnect;
	conn_opts.onFailure = onConnectFailure;
	if ((rc = MQTTAsync_connect(client, &conn_opts)) != MQTTASYNC_SUCCESS)
	{
		printf("Failed to start connect, return code %d\n", rc);
		finished = 0;
	} else {
		finished = 1;
	}
}


int msgarrvd(void *context, char *topicName, int topicLen, MQTTAsync_message *message)
{
	struct timeval tv = {0};
	MQTTAsync client = (MQTTAsync)context;

    printf("Message arrived\n");
    printf("     topic: %s\n", topicName);
    printf("   message: %.*s\n", message->payloadlen, (char*)message->payload);
	// usleep(15000000L);   // 在sleep的时间段内，若收到话题，会存到队列中，等这次回调完成后，会对队列中刚才的话题调用这个回调。只不过30>20,可能多次后会造成超时断开连接‘15秒就不会。
    // 解析消息内容，假设消息是JSON格式
    cJSON *root = cJSON_Parse((char *)message->payload);
	if (root != NULL) {
		if(strcmp(topicName, TOPIC) == 0)
		{
			// 这里可以进一步解析 JSON 对象
			// 例如获取特定字段的值
			cJSON *item = cJSON_GetObjectItem(root, "gateway");
			if (item != NULL && cJSON_IsNumber(item)) {
				printf("Parsed value for 'gateway': %d\n", item->valueint);
			} else {
				goto out;
			}
			cJSON *useritem = cJSON_GetObjectItem(root, "userid");
			if (useritem != NULL && cJSON_IsNumber(useritem)) {
				printf("Parsed value for 'userid': %d\n", useritem->valueint);
			} else {
				goto out;
			}
			cJSON *data = cJSON_GetObjectItem(root, "data");
			// cJSON *data_copy;
			if (data != NULL && cJSON_IsObject(data)) { // 检查是否为Object类型
				printf("Parsed value for 'data': %s\n", data->string);
				cJSON *latitude_json = cJSON_GetObjectItemCaseSensitive(data, "latitude");
				if (latitude_json && cJSON_IsNumber(latitude_json)) {
					targetLat = latitude_json->valuedouble;
					printf("Latitude: %f\n", latitude_json->valuedouble);
				}
				cJSON *longitude_json = cJSON_GetObjectItemCaseSensitive(data, "longitude");
				if (longitude_json && cJSON_IsNumber(longitude_json)) {
					targetLon = longitude_json->valuedouble;
					printf("Longitude: %f\n", longitude_json->valuedouble);
				}
				// data_copy = cJSON_Duplicate(data, true);
				// if (data_copy == NULL) {
				// 	goto out;
				// }
			} else {
				goto out;
			}
			printf("--------------------------------------- test test test test test ----------");

			// 任务------------------------
			// 创建一个reply JSON对象
			cJSON *reply = cJSON_CreateObject();
			// 检查是否成功创建了JSON对象
			if (reply == NULL) {
				const char *error_ptr = cJSON_GetErrorPtr();
				if (error_ptr != NULL) {
					fprintf(stderr, "Error before: %s\n", error_ptr);
				}
				// cJSON_Delete(root);
				// return 1;
				goto out;
			}
			// 添加键值对到JSON对象
			cJSON_AddNumberToObject(reply, "gateway", item->valueint);
			cJSON_AddNumberToObject(reply, "userid", useritem->valueint);
			// 获取当前时间（包括秒和微秒）
			gettimeofday(&tv, NULL);
			// 计算毫秒
			long milliseconds = tv.tv_sec * 1000 + tv.tv_usec / 1000;
			printf("Timestamp in milliseconds: %ld\n", milliseconds);
			cJSON_AddNumberToObject(reply, "timestamp", milliseconds);

			// 此时该id在执行任务，不重复执行任务
			if(isin_mission) // 这个是针对开发者
			{
				cJSON_AddNumberToObject(reply, "result", 2); // 2 表示任务不开始执行（因为已经在执行任务）
				// cJSON_AddBoolToObject(reply, "isin_mission", false); // 这个是针对用户
				cJSON_AddNumberToObject(reply, "progress", 0);
				// 将JSON对象转换为字符串以便打印或保存
				char *json_string = cJSON_Print(reply);
				if (json_string == NULL) {
					USER_LOG_ERROR("Failed to create JSON string.");
					cJSON_Delete(reply);
					// cJSON_Delete(root);
					// return;
					goto out;
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
				printf("--------------------------------------- mission denied because uav is in mission ----------");
				cJSON_Delete(reply);
				cJSON_free(json_string);
				// cJSON_Delete(root);
				// return 1;
				goto out;
			}
			// 判断电池和RTK是否支持mission
			// 电池要看文档和在订阅里面订阅
			pthread_mutex_lock(&statusMutex); // 加锁以保护对共享资源的访问
			dji_f64_t currentLon = droneStatus.rtkLongitude;
			dji_f64_t currentLat = droneStatus.rtkLatitude;
			pthread_mutex_unlock(&statusMutex); // 解锁
			float disBposTstation = calculateGreatCircleDistance(currentLat, currentLon, stationPos.latitude, stationPos.longitude); //后续要改station的坐标，可以开机从数据库读取
			float disTarposTstation = calculateGreatCircleDistance(currentLat, currentLon, targetLat, targetLon); //地面站处理当收到目标点时计算目标点距离无人机当前位置的距离，如果过远则拒绝执行任务（防止接收到的目标点有问题）
			if(disTarposTstation > 3.4028235E38f || remainingBattery < 10 || disBposTstation > 3.4028235E38f || is_RTK_ready==50 || stationary!=0)
			{
				USER_LOG_INFO("----disTarposTstation--%f----remainingBattery--%d----disBposTstation--%f----is_RTK_ready--%d\n", disTarposTstation, remainingBattery, disBposTstation, is_RTK_ready);
				cJSON_AddNumberToObject(reply, "result", 3); // 3 表示任务不开始执行（无人机自身原因或者目标航点太远）
				// cJSON_AddBoolToObject(reply, "isin_mission", false);
				cJSON_AddNumberToObject(reply, "progress", 0);
				// 将JSON对象转换为字符串以便打印或保存
				char *json_string = cJSON_Print(reply);
				if (json_string == NULL) {
					USER_LOG_ERROR("Failed to create JSON string.");
					cJSON_Delete(reply);
					// cJSON_Delete(root);
					// return;
					goto out;
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
				printf("--------------------------------------- mission denied because uav is not correct ----------");
				cJSON_Delete(reply);
				// cJSON_Delete(root);
				cJSON_free(json_string);
				// return 1;
				goto out;
			}

			printf("Ready to execute_mission!\n");
			userID = useritem->valueint;
			// ThreadParams *params = malloc(sizeof(ThreadParams));  // 分配内存用于存储参数
			// params->client = client;
			// params->data = data_copy;  // cJSON *data = cJSON_GetObjectItem(root, "data");params->data = data;如果cJSON_Delete(root);，会影响DjiTest_WaypointV2RunSample线程的正常执行。
			// 在 DjiTest_WaypointV2RunSample 线程中，params->data 是指向 root 子对象的指针。
			// 如果在主线程中调用了 cJSON_Delete(root);，则 params->data 指向的内存将被释放，导致线程中的 params->data 成为悬空指针。
			// 解决办法：复制数据。你可以将 data 的内容复制到一个新的 cJSON 对象中，并将其传递给线程
			// GPT
			pthread_t thread_id;
			// 创建线程，传递给threadFunction作为线程执行的函数
			// if (pthread_create(&thread_id, NULL, DjiTest_WaypointV2RunSample, (void*)params) != 0) {
			if (pthread_create(&thread_id, NULL, DjiTest_WaypointV2RunSample, (void*)client) != 0) {
				printf("线程创建失败\n");
				cJSON_Delete(reply);
				// cJSON_Delete(data_copy); // 清理复制的对象
				// free(params);
				// cJSON_Delete(root);
				// return 1;
				goto out;
			}


			pthread_detach(thread_id);
			// 等待线程结束，避免主线程先于子线程结束
			// pthread_join(thread_id, NULL);

			cJSON_Delete(reply);

			T_DjiReturnCode returnCode = DjiTest_LiveviewRunSample(DJI_MOUNT_POSITION_UNKNOWN);
			if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
				USER_LOG_ERROR("live view sample init error\n");
			}
			// free(params);  // 不能在这里释放内存，否则线程执行有问题，需要在线程中释放内存
			// cJSON_Delete(root); // 解析完成后记得释放内存
		} 
		else if(strcmp(topicName, TOPIC_CANCEL) == 0)
		{
			T_DjiOsalHandler *osalHandler = NULL;
			osalHandler = DjiPlatform_GetOsalHandler();
			if (osalHandler->TaskCreate("gohome_forceland_task", DjiTest_FlightControlGoHomeForceLandingTask,
                                DJI_TEST_GOHOME_FORCELAND_TASK_STACK_SIZE, NULL, &s_gohomeForcelandThread) !=
				DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
				USER_LOG_ERROR("user gohome forceland task create error.");
				goto out;
			}
		}
	} else {
		// 如果解析失败，可能不是有效的 JSON 或者有其他问题
		printf("Failed to parse JSON message.\n");
		MQTTAsync_freeMessage(&message);
    	MQTTAsync_free(topicName);
		return 1;
	}
out:
	cJSON_Delete(root);
	MQTTAsync_freeMessage(&message);
    MQTTAsync_free(topicName);
    return 1;
}

void onDisconnectFailure(void* context, MQTTAsync_failureData* response)
{
	printf("Disconnect failed, rc %d\n", response->code);
	disc_finished = 2;
}

void onDisconnect(void* context, MQTTAsync_successData* response)
{
	printf("Successful disconnection\n");
	disc_finished = 1;
}

void onSubscribe(void* context, MQTTAsync_successData* response)
{
	printf("Subscribe succeeded\n");
	subscribed = 1;
}

void onSubscribeFailure(void* context, MQTTAsync_failureData* response)
{
	printf("Subscribe failed, rc %d\n", response->code);
	subscribed = 2;
}


void onConnectFailure(void* context, MQTTAsync_failureData* response)
{
	printf("Connect failed, rc %d\n", response->code);
	finished = 2;
}


void onConnect(void* context, MQTTAsync_successData* response)
{
	MQTTAsync client = (MQTTAsync)context;
	MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
	int rc;

	printf("Successful connection\n");

	printf("Subscribing to topic %s\nfor client %s using QoS%d\n\n"
           "Press Q<Enter> to quit\n\n", TOPIC, CLIENTID, QOS);
	opts.onSuccess = onSubscribe; // 即使执行到MQTTAsync_subscribe还没有话题到来，也能订阅成功.
	opts.onFailure = onSubscribeFailure;
	opts.context = client;
	if ((rc = MQTTAsync_subscribe(client, TOPIC, QOS, &opts)) != MQTTASYNC_SUCCESS)
	{
		printf("Failed to start subscribe %s, return code %d\n", TOPIC, rc);
		finished = 2;
	} else {
		finished = 1;
	}
	if ((rc = MQTTAsync_subscribe(client, TOPIC_CANCEL, QOS, &opts)) != MQTTASYNC_SUCCESS)
	{
		printf("Failed to start subscribe %s, return code %d\n", TOPIC_CANCEL, rc);
		finished = 2;
	} else {
		finished = 1;
	}
}

