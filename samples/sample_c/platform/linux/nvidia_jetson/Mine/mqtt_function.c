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
int dronesCount = 0;
DroneInfo dronesArray[100]; // 假设最多有100个无人机
IndexedDistance indexedDistances[100];
// pthread_mutex_t mqtt_publish_mutex = PTHREAD_MUTEX_INITIALIZER;

// // 计算无人机当前位置和机巢的距离
// float calculateGreatCircleDistance(double lat1, double lon1, double lat2, double lon2) {
//     // 将经纬度转换为弧度
//     double phi1 = lat1 * M_PI / 180;
//     double phi2 = lat2 * M_PI / 180;
//     double delta_phi = (lat2 - lat1) * M_PI / 180;
//     double delta_lambda = (lon2 - lon1) * M_PI / 180;
//     // 计算大圆距离
//     double a = sin(delta_phi/2) * sin(delta_phi/2) + cos(phi1) * cos(phi2) * sin(delta_lambda/2) * sin(delta_lambda/2);
//     double c = 2 * atan2(sqrt(a), sqrt(1-a));
//     float distance = RADIUS_EARTH * c;
//     return distance;
// }

static void sortDronesByDistanceTo(double pointALat, double pointALng);
void connlost(void *context, char *cause)
{
	MQTTAsync client = (MQTTAsync)context;
	MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
	int rc;

	USER_LOG_ERROR("\nConnection lost\n");
	if (cause)
		USER_LOG_ERROR("     cause: %s\n", cause);

	USER_LOG_ERROR("Reconnecting\n");
	conn_opts.keepAliveInterval = 20;
	conn_opts.cleansession = 1;
	conn_opts.username = USERNAME;
	conn_opts.password = PASSWORD;
	conn_opts.onSuccess = onConnect;
	conn_opts.onFailure = onConnectFailure;
	if ((rc = MQTTAsync_connect(client, &conn_opts)) != MQTTASYNC_SUCCESS)
	{
		USER_LOG_ERROR("Failed to start connect, return code %d\n", rc);
		finished = 0;
	} else {
		USER_LOG_ERROR("Reconnected successfully! ");
		finished = 1;
	}
}


int msgarrvd(void *context, char *topicName, int topicLen, MQTTAsync_message *message)
{
	struct timeval tv = {0};
	MQTTAsync client = (MQTTAsync)context;

    USER_LOG_INFO("Message arrived\n");
    USER_LOG_INFO("     topic: %s\n", topicName);
    USER_LOG_INFO("   message: %.*s\n", message->payloadlen, (char*)message->payload);
	// usleep(15000000L);   // 在sleep的时间段内，若收到话题，会存到队列中，等这次回调完成后，会对队列中刚才的话题调用这个回调。只不过30>20,可能多次后会造成超时断开连接‘15秒就不会。
    // 解析消息内容，假设消息是JSON格式
    cJSON *root = cJSON_Parse((char *)message->payload);
	if (root != NULL) {
		if(strcmp(topicName, TOPIC) == 0)
		{
			// 获取当前时间
			time(&start_time);
			// 转换为本地时间结构
			struct tm *local = localtime(&start_time);
			// 格式化输出时间，精确到秒
			USER_LOG_INFO("当前时间是: %04d-%02d-%02d %02d:%02d:%02d\n",
				local->tm_year + 1900,
				local->tm_mon + 1,
				local->tm_mday,
				local->tm_hour,
				local->tm_min,
				local->tm_sec);

			// 这里可以进一步解析 JSON 对象
			// 例如获取特定字段的值
			cJSON *item = cJSON_GetObjectItem(root, "airportId");
			if (item != NULL && cJSON_IsNumber(item)) {
				printf("Parsed value for 'airportId': %d\n", item->valueint);
				if(item->valueint!=1)	goto out;  // 不是这个机场地面站的信息，不处理
			} else {
				goto out;
			}

			cJSON *order = cJSON_GetObjectItem(root, "helpOrderId");
			if (order != NULL && cJSON_IsNumber(order)) {
				printf("Parsed value for 'order': %d\n", order->valueint);
			} else {
				goto out;
			}

			dji_f64_t targetLatTemp, targetLonTemp;  // 临时存储目标投放点位置，在这里不能直接给targetLat赋值，防止下一次循环又赋新的值
			cJSON *coord = cJSON_GetObjectItem(root, "deliveryPointCoordinate");
			// cJSON *data_copy;
			if (coord != NULL && cJSON_IsObject(coord)) { // 检查是否为Object类型
				printf("Parsed value for 'coord': %s\n", coord->string);
				cJSON *latitude_json = cJSON_GetObjectItemCaseSensitive(coord, "lat");
				if (latitude_json && cJSON_IsNumber(latitude_json)) {
					targetLatTemp = latitude_json->valuedouble;
					printf("Latitude: %.9f\n", latitude_json->valuedouble);
				} else {
					goto out;
				}
				cJSON *longitude_json = cJSON_GetObjectItemCaseSensitive(coord, "lng");
				if (longitude_json && cJSON_IsNumber(longitude_json)) {
					targetLonTemp = longitude_json->valuedouble;
					printf("Longitude: %.9f\n", longitude_json->valuedouble);
				} else {
					goto out;
				}
			} else {
				goto out;
			}

			sortDronesByDistanceTo(targetLatTemp, targetLonTemp);
			
			printf("--------------------------------------- test test test test test ----------");
			/* 
			将所有无人机机舱位置计算和投放点的距离，从近到远依次排序，判断若第一个无人机不能用，则用第二台……依此类推
			现在只有一台，默认不用判断距离，只判断能不能用*/
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
			cJSON_AddStringToObject(reply, "rtmpUrl", "");
			cJSON_AddNumberToObject(reply, "helpOrderId", order->valueint);
			// cJSON_AddNumberToObject(reply, "droneId", 1);   // 后续多台无人机根据距离判断修改droneId

			cJSON *log = cJSON_CreateObject();
			if (log == NULL) {
				const char *error_ptr = cJSON_GetErrorPtr();
				if (error_ptr != NULL) {
					fprintf(stderr, "Error before: %s\n", error_ptr);
				}
				// cJSON_Delete(root);
				// return 1;
				cJSON_Delete(reply);
				goto out;
			}
			cJSON_AddNumberToObject(log, "orderId", order->valueint);
			// cJSON_AddNumberToObject(log, "droneId", 1);   // 后续多台无人机根据距离判断修改droneId
			// cJSON_AddNumberToObject(log, "battery", remainingBattery);
			// pthread_mutex_lock(&statusMutex); // 加锁以保护对共享资源的访问
			// dji_f64_t currentLon = droneStatus.rtkLongitude; //
			// dji_f64_t currentLat = droneStatus.rtkLatitude;
			// dji_f32_t curHei = droneStatus.relativeHeight;
			// pthread_mutex_unlock(&statusMutex); // 解锁
			// cJSON_AddStringToObject(log, "coordinate", "POINT("+currentLon+" "+currentLat+")");
			cJSON_AddStringToObject(log, "photoUrl", "123.jpg");
			// 获取当前时间（包括秒和微秒）
			gettimeofday(&tv, NULL);
			// 计算毫秒
			long milliseconds = tv.tv_sec * 1000 + tv.tv_usec / 1000;
			printf("Timestamp in milliseconds: %ld\n", milliseconds);
			cJSON_AddNumberToObject(log, "timestamp", milliseconds);
			// cJSON_AddNumberToObject(log, "altitude", curHei);
			// cJSON_AddNumberToObject(log, "speed", vel);
			// cJSON_AddStringToObject(log, "remainingPath", "LINESTRING("+currentLon+" "+currentLat+", "+targetLat+" "+targetLat+")");
			// cJSON_AddNumberToObject(log, "remainingTime", remainTime);

			USER_LOG_INFO("--dronesCount--%d\n", dronesCount);
			/*按照无人机机舱距离投放点/目标点的距离从小到大排序*/
			for (int i = 0; i < dronesCount; i++) {
				int originalIndex = indexedDistances[i].index;
				printf("Drone ID: %d, Distance: %.2f meters\n", dronesArray[originalIndex].droneId, indexedDistances[i].distance);	// dronesArray[originalIndex].droneId是距离最近的无人机的droneId
				
				pthread_mutex_lock(&statusMutex); // 加锁以保护对共享资源的访问
				dji_f64_t currentLon = droneStatus.rtkLongitude; //
				dji_f64_t currentLat = droneStatus.rtkLatitude;
				dji_f32_t curHei = droneStatus.relativeHeight;
				pthread_mutex_unlock(&statusMutex); // 解锁
				float disBposTstation = haversine(currentLat, currentLon, dronesArray[originalIndex].lat, dronesArray[originalIndex].lng); //后续要改station的坐标，可以开机从数据库读取
				if(isin_mission || remainingBattery < 10 || disBposTstation > 7.5028235f || is_RTK_ready!=50 || stationary!=0) // 这个是针对开发者   后续多台无人机改成map
				{
					if(i==dronesCount-1)
					{
						USER_LOG_INFO("--remainingBattery--%d----disBposTstation--%f----is_RTK_ready--%d\n", remainingBattery, disBposTstation, is_RTK_ready);
						cJSON_AddBoolToObject(reply, "result", false); // false表示任务不开始执行（因为已经在执行任务或有问题）
						cJSON_AddNumberToObject(reply, "droneId", dronesArray[originalIndex].droneId);   // 后续多台无人机根据距离判断修改droneId
						cJSON_AddNumberToObject(log, "droneId", dronesArray[originalIndex].droneId);   // 后续多台无人机根据距离判断修改droneId
						cJSON_AddNumberToObject(log, "battery", remainingBattery);
						cJSON_AddNumberToObject(log, "yaw", yaw);
						char str[100]; // 确保数组足够大以容纳结果字符串
						// 直接格式化两个double值并用空格分隔
						snprintf(str, sizeof(str), "POINT(%.9f %.9f)", currentLon, currentLat);
						// snprintf(str, sizeof(str), "%f", currentLon); // 安全地将double值格式化为字符串
						cJSON_AddStringToObject(log, "coordinate", str);
						cJSON_AddNumberToObject(log, "altitude", curHei);
						cJSON_AddNumberToObject(log, "speed", vel);
						
						snprintf(str, sizeof(str), "LINESTRING(%.9f %.9f, %.9f %.9f)", currentLon, currentLat, targetLon, targetLat);
						cJSON_AddStringToObject(log, "remainingPath", str);
						cJSON_AddNumberToObject(log, "remainingTime", remainTime);
						cJSON_AddStringToObject(log, "workStatus", "INTERRUPTED");
						cJSON_AddStringToObject(log, "flightMode", "");
						cJSON_AddItemToObject(reply, "droneLog", log);

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
						printf("--------------------------------------- mission denied because uav is in mission or other problems.----------");
						// cJSON_free(json_string);
						// cJSON_Delete(log);
						// cJSON_Delete(reply);
						
						// cJSON_Delete(root);
						// return 1;
						goto out;
					}
				} else {
					printf("Ready to execute_mission!\n");
					targetLat = targetLatTemp;	// 不用担心下一次循环这个赋值改变而影响其他文件中对targetLat的访问，因为下一次循环到这里时，无人机肯定处于空闲状态而不是mission状态（isin_mission=false），所以是开始新的任务，旧的任务已经结束，不存在影响就任务目标投放点的问题。但是当有多个无人机可以同时执行任务时，就得用map防止多个无人机的目标投放点互相影响这个赋值
					targetLon = targetLonTemp;
					orderID = order->valueint;
					droneID = dronesArray[originalIndex].droneId;
					ThreadParams *params = malloc(sizeof(ThreadParams));
					params->client = client;
					params->orderID = order->valueint;
					params->droneID = dronesArray[originalIndex].droneId;
					pthread_t thread_id;
					// 创建线程，传递给threadFunction作为线程执行的函数
					if (pthread_create(&thread_id, NULL, DjiTest_WaypointV2RunSample, (void*)params) != 0) {
					// if (pthread_create(&thread_id, NULL, DjiTest_WaypointV2RunSample, (void*)client) != 0) {
						printf("线程创建失败\n");
						cJSON_Delete(log);
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
					cJSON_Delete(log);
					cJSON_Delete(reply);

					stopview=false;
					T_DjiReturnCode returnCode = DjiTest_LiveviewRunSample(dronesArray[originalIndex].droneId);
					if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
						USER_LOG_ERROR("live view sample init error\n");
					}
					// free(params);  // 不能在这里释放内存，否则线程执行有问题，需要在线程中释放内存
					// cJSON_Delete(root); // 解析完成后记得释放内存

					break;
				}
			}
			// float disBposTstation = haversine(currentLat, currentLon, stationPos.latitude, stationPos.longitude); //后续要改station的坐标，可以开机从数据库读取
			// // 此时该id在执行任务，不重复执行任务
			// if(isin_mission || remainingBattery < 10 || disBposTstation > 3.4028235E38f || is_RTK_ready==50 || stationary!=0) // 这个是针对开发者   后续多台无人机改成map
			// {
			// 	USER_LOG_INFO("--remainingBattery--%d----disBposTstation--%f----is_RTK_ready--%d\n", remainingBattery, disBposTstation, is_RTK_ready);
			// 	cJSON_AddBoolToObject(reply, "result", false); // false表示任务不开始执行（因为已经在执行任务或有问题）
			// 	cJSON_AddStringToObject(log, "workStatus", "interrupt");
			// 	cJSON_AddObjectToObject(reply, "droneLog", log);

			// 	// 将JSON对象转换为字符串以便打印或保存
			// 	char *json_string = cJSON_Print(reply);
			// 	if (json_string == NULL) {
			// 		USER_LOG_ERROR("Failed to create JSON string.");
			// 		cJSON_Delete(log);
			// 		cJSON_Delete(reply);
			// 		// cJSON_Delete(root);
			// 		// return;
			// 		goto out;
			// 	}
			// 	printf("JSON string: %s\n", json_string);
			// 	// 在这里你可以将json_string保存到文件或发送到网络等
			// 	int rc;
			// 	pubmsg.payload = (void *)json_string;
			// 	pubmsg.payloadlen = strlen(json_string);
			// 	pubmsg.qos = QOS;
			// 	pubmsg.retained = 0;

			// 	// 对client的修改要加互斥锁
			// 	// 看MQTTAsync_sendMessage   github上面是否有说线程安全性
			// 	pthread_mutex_lock(&mqtt_publish_mutex);
			// 	if ((rc = MQTTAsync_sendMessage(client, TOPIC_REPLY, &pubmsg, &opts)) != MQTTASYNC_SUCCESS) {
			// 		printf("Failed to start sendMessage, return code %d\n", rc);
			// 	} else {
			// 		printf("Message published to topic %s\n", TOPIC_REPLY);
			// 	}
			// 	pthread_mutex_unlock(&mqtt_publish_mutex);
			// 	printf("--------------------------------------- mission denied because uav is in mission or other problems.----------");
			// 	cJSON_Delete(log);
			// 	cJSON_Delete(reply);
			// 	cJSON_free(json_string);
			// 	// cJSON_Delete(root);
			// 	// return 1;
			// 	goto out;
			// }

			// printf("Ready to execute_mission!\n");
			// ThreadParams *params = malloc(sizeof(ThreadParams));
			// params->client = client;
			// params->orderID = order->valueint;
			// pthread_t thread_id;
			// // 创建线程，传递给threadFunction作为线程执行的函数
			// if (pthread_create(&thread_id, NULL, DjiTest_WaypointV2RunSample, (void*)params) != 0) {
			// // if (pthread_create(&thread_id, NULL, DjiTest_WaypointV2RunSample, (void*)client) != 0) {
			// 	printf("线程创建失败\n");
			// 	cJSON_Delete(log);
			// 	cJSON_Delete(reply);
			// 	// cJSON_Delete(data_copy); // 清理复制的对象
			// 	// free(params);
			// 	// cJSON_Delete(root);
			// 	// return 1;
			// 	goto out;
			// }


			// pthread_detach(thread_id);
			// // 等待线程结束，避免主线程先于子线程结束
			// // pthread_join(thread_id, NULL);

			// cJSON_Delete(reply);

			// stopview=false;
			// T_DjiReturnCode returnCode = DjiTest_LiveviewRunSample(1);
			// if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
			// 	USER_LOG_ERROR("live view sample init error\n");
			// }
			// // free(params);  // 不能在这里释放内存，否则线程执行有问题，需要在线程中释放内存
			// // cJSON_Delete(root); // 解析完成后记得释放内存
		} 
		else if(strcmp(topicName, TOPIC_CANCEL) == 0)
		{
			cJSON *item = cJSON_GetObjectItem(root, "airportId");
			if (item != NULL && cJSON_IsNumber(item)) {
				USER_LOG_INFO("Parsed value for 'airportId': %d\n", item->valueint);
				if(item->valueint!=1)	goto out;  // 不是这个机场地面站的信息，不处理
			} else {
				goto out;
			}
			T_DjiOsalHandler *osalHandler = osalHandler = DjiPlatform_GetOsalHandler();
			if (osalHandler->TaskCreate("gohome_forceland_task", DjiTest_FlightControlGoHomeForceLandingTask,
                                DJI_TEST_GOHOME_FORCELAND_TASK_STACK_SIZE, NULL, &s_gohomeForcelandThread) !=
				DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
				USER_LOG_ERROR("user gohome forceland task create error.");
				goto out;
			}
		}
		else if(strcmp(topicName, TOPIC_responseCabins) == 0)
		{
			cJSON *item = cJSON_GetObjectItem(root, "airportId");
			if (item != NULL && cJSON_IsNumber(item)) {
				USER_LOG_INFO("Parsed value for 'airportId': %d\n", item->valueint);
				if(item->valueint!=1)	goto out;  // 不是这个机场地面站的信息，不处理
			} else {
				goto out;
			}

			cJSON *drones = cJSON_GetObjectItemCaseSensitive(root, "drones");
			if (drones != NULL && cJSON_IsArray(drones)) { // 检查是否为Object类型
				dronesCount = cJSON_GetArraySize(drones);
				USER_LOG_INFO("--dronesCount--%d\n", dronesCount);
				// DistanceInfo distances[dronesCount];

				double latitude, longitude;
				for (int i = 0; i < dronesCount; ++i) {
					cJSON *drone = cJSON_GetArrayItem(drones, i);
					cJSON *coordinate = cJSON_GetObjectItemCaseSensitive(drone, "coordinate");
					if (cJSON_IsString(coordinate) && (coordinate->valuestring != NULL)) {
						// 假设格式为 "POINT(lat lon)"
						sscanf(coordinate->valuestring, "POINT(%lf %lf)", &latitude, &longitude);
						USER_LOG_INFO("Drone ID: %d, Latitude: %f, Longitude: %f\n",
							   cJSON_GetObjectItemCaseSensitive(drone, "droneId")->valueint,
							   latitude, longitude);
					} else {
						goto out;
					}

					dronesArray[i].droneId = cJSON_GetObjectItemCaseSensitive(drone, "droneId")->valueint;
					dronesArray[i].lat = latitude;
            		dronesArray[i].lng = longitude;
				}
			} else {
				goto out;
			}
		}
		else if(strcmp(topicName, TOPIC_requestPlacementPoint) == 0)
		{
			cJSON *item = cJSON_GetObjectItem(root, "airportId");
			if (item != NULL && cJSON_IsNumber(item)) {
				USER_LOG_INFO("Parsed value for 'airportId': %d\n", item->valueint);
				if(item->valueint!=1)	goto out;  // 不是这个机场地面站的信息，不处理
			} else {
				goto out;
			}

			cJSON *reply = cJSON_CreateObject();
			if (reply == NULL) {
				const char *error_ptr = cJSON_GetErrorPtr();
				if (error_ptr != NULL) {
					fprintf(stderr, "Error before: %s\n", error_ptr);
				}
				goto out;
			}
			cJSON_AddNumberToObject(reply, "airportId", item->valueint);
			pthread_mutex_lock(&statusMutex); // 加锁以保护对共享资源的访问
			dji_f64_t currentLon = droneStatus.rtkLongitude; //
			dji_f64_t currentLat = droneStatus.rtkLatitude;
			dji_f32_t curHei = droneStatus.relativeHeight;
			pthread_mutex_unlock(&statusMutex); // 解锁
			char str[50]; // 确保数组足够大以容纳结果字符串
			snprintf(str, sizeof(str), "POINT(%.9f %.9f)", currentLat, currentLon);
			cJSON_AddStringToObject(reply, "coordinate", str);
			char *json_string = cJSON_Print(reply);
			if (json_string == NULL) {
				USER_LOG_ERROR("Failed to create JSON string.");
				cJSON_Delete(reply);
				goto out;
			}
			printf("JSON string: %s\n", json_string);
			int rc;
			pubmsg.payload = (void *)json_string;
			pubmsg.payloadlen = strlen(json_string);
			pubmsg.qos = QOS;
			pubmsg.retained = 0;
			pthread_mutex_lock(&mqtt_publish_mutex);
			if ((rc = MQTTAsync_sendMessage(client, TOPIC_responsePlacementPoint, &pubmsg, &opts)) != MQTTASYNC_SUCCESS) {
				printf("Failed to start sendMessage, return code %d\n", rc);
			} else {
				printf("Message published to topic %s\n", TOPIC_responsePlacementPoint);
			}
			pthread_mutex_unlock(&mqtt_publish_mutex);
			cJSON_Delete(reply);
			cJSON_free(json_string);
			goto out;
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
	if ((rc = MQTTAsync_subscribe(client, TOPIC_responseCabins, QOS, &opts)) != MQTTASYNC_SUCCESS)
	{
		printf("Failed to start subscribe %s, return code %d\n", TOPIC_responseCabins, rc);
		finished = 2;
	} else {
		finished = 1;
	}
	if ((rc = MQTTAsync_subscribe(client, TOPIC_requestPlacementPoint, QOS, &opts)) != MQTTASYNC_SUCCESS)
	{
		printf("Failed to start subscribe %s, return code %d\n", TOPIC_requestPlacementPoint, rc);
		finished = 2;
	} else {
		finished = 1;
	}

	// ===== 添加发布消息的代码在这里 =====
    const char *payload = "{\"airportId\": 1}";
    int payloadLen = strlen(payload);

    MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
    pubmsg.payload = (void *)payload;
    pubmsg.payloadlen = payloadLen;
    pubmsg.qos = QOS; // 使用全局定义的QOS等级
    pubmsg.retained = 0;

    if ((rc = MQTTAsync_sendMessage(client, TOPIC_requestCabins, &pubmsg, NULL)) != MQTTASYNC_SUCCESS)
    {
        printf("Failed to publish message to topic %s, return code %d\n", TOPIC_requestCabins, rc);
    }
    else
    {
        printf("Published message to topic: %s\n", TOPIC_requestCabins);
    }
}

// 比较函数用于qsort
int compareByDistance(const void *a, const void *b) {
    IndexedDistance idxA = *(IndexedDistance *)a;
    IndexedDistance idxB = *(IndexedDistance *)b;
    if (idxA.distance < idxB.distance) return -1;
    if (idxA.distance > idxB.distance) return 1;
    return 0;
}

static void sortDronesByDistanceTo(double pointALat, double pointALng) {
    // IndexedDistance indexedDistances[dronesCount];
    
    for (int i = 0; i < dronesCount; ++i) {
        indexedDistances[i].index = i;
        indexedDistances[i].distance = haversine(pointALat, pointALng, dronesArray[i].lat, dronesArray[i].lng);
    }

    qsort(indexedDistances, dronesCount, sizeof(IndexedDistance), compareByDistance);

    printf("Sorted by distance:\n");
    // for (int i = 0; i < dronesCount; i++) {
    //     int originalIndex = indexedDistances[i].index;
    //     printf("Drone ID: %d, Distance: %.2f meters\n", dronesArray[originalIndex].id, indexedDistances[i].distance);
    // }
}


