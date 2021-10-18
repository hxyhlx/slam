/***************************************
@作者：复旦大学
@日期：2020.12.15
@备注：MQTT传参
***************************************/
#include <stdio.h>
#include <stdlib.h>
#include <mosquitto.h>
#include <string.h>
#include <iostream>
#include <iomanip>
#include "navigation/web.h"
#include "navigation/coorChange.h"
#include "jsoncpp/json/json.h"
using namespace std;
string web_buff;
bool get_flag=0;
string taskId;
void my_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
    if(message->payloadlen){ 
	Json::Reader reader;  
	Json::Value root;
	string str=((char *)message->payload);
	reader.parse(str, root);//printf("%s %s", message->topic, message->payload);

	if(root["cmd"].asString()=="ROUTE_DESIGN"){
    	web_buff=((char *)message->payload);
	    get_flag=1; cout<<"get msg "<<get_flag<<endl;
	}
        //printf("%s %s", message->topic, message->payload);
    }else{
        cout<<message->topic<<message->payloadlen<<endl;
    }
    fflush(stdout);
}

void my_connect_callback(struct mosquitto *mosq, void *userdata, int result)
{
    int i;
    if(!result){
        /* Subscribe to broker information topics on successful connect. */
        mosquitto_subscribe(mosq, NULL, "alg-center", 1);
        mosquitto_subscribe(mosq, NULL, "topic1", 1);
    }else{
        fprintf(stderr, "Connect failed\n");
    }
}

void my_subscribe_callback(struct mosquitto *mosq, void *userdata, int mid, int qos_count, const int *granted_qos)
{
    int i;
    printf("Subscribed (mid: %d): %d", mid, granted_qos[0]);
    for(i=1; i<qos_count; i++){
        printf(", %d", granted_qos[i]);
    }
    printf("\n");
}

void my_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
    /* Pring all log messages regardless of level. */
    printf("%s\n", str);
}



template <typename T>
string to_string_with_precision(const T a_value, const int n)
{
    std::ostringstream out;
    out << std::setprecision(n) << a_value;
    return out.str();
}

string produce_pos(vector<gps_p> playload,string msg)
{
	Json::Value root;
	root["cmd"]="ROUTE_DESIGN_RES"; root["state"]=1; root["taskId"]=taskId;
	Json::Value pos_array;
	Json::Value object;
	
	vector<gps_p>::iterator p;
    for(p=playload.begin();p!=playload.end();p++)
    {
	object["lng"]=p->lon; object["lat"]=p->lat; object["ht"]=p->alt;
	pos_array.append(object);
	}
	root["pts"]=pos_array;
	root["msg"]=msg;
	
	Json::FastWriter fast_writer;
	string str=fast_writer.write(root);
	cout<<str<<endl;
	return str;
}


/*
string produce_STAC()
{
	Json::Value root;
	root["cmd"]="MAP_OBSTACLE"; root["yx"]="alg-center"; root["obstacleId"]=taskId;
	Json::Value pos_array;
	Json::Value object;
	
	vector<gps_p>::iterator p;
   
	object["lng"]=121.33; object["lat"]=31.355; object["ht"]=4;
	pos_array.append(object);
	
	root["pos"]=pos_array;
	
	Json::StyledWriter fast_writer;
	string str=fast_writer.write(root);
	cout<<str<<endl;
	return str;
}
*/



void webserver::pub(struct mosquitto *mosq,vector<gps_p> payload,string msg)
{
    //buff=to_string_with_precision(playload.lon,11);
    //cout<<buff<<  &buff<<endl;
    //mosquitto_publish(mosq,NULL,"gps info",sizeof(buff),&buff,0,0);
	/*(memset(temp, 0, sizeof(temp)) ;
	snprintf(temp, sizeof(temp), "%f ,%f ,%f",playload.lon, playload.lat, playload.alt);
	//printf("%s\n", temp) ;
    mosquitto_publish(mosq,NULL,"gps info",strlen(temp)+1,temp,0,0);*/
	string str=produce_pos(payload,msg);
	//string OBS=produce_STAC();
	//memset(buff2, 0, sizeof(buff2)) ;
	//snprintf(buff2, sizeof(buff2), "%s",str);
	//strcpy(ros_buff, (str.c_str()));
	char* c = const_cast<char *>(str.c_str());
	//strcpy(OBSTACLE,OBS.c_str());
	//cout<<"buff is:"<<ros_buff<<endl;
	mosquitto_publish(mosq,NULL,"webgis",strlen(c),c,1,0);
	//mosquitto_publish(mosq,NULL,"MAP_OBSTACLE",strlen(OBSTACLE)+1,OBSTACLE,0,0);

}






webserver::webserver()
{

    
    //libmosquitto 库初始化
    mosquitto_lib_init();
    //创建mosquitto客户端
    mosq = mosquitto_new(NULL,session,NULL);
    if(!mosq){
        printf("create client failed..\n");
        mosquitto_lib_cleanup();
       // return 1;
    }else{cout<<"Connect to web server!!"<<endl;}
    //设置回调函数，需要时可使用
    //mosquitto_log_callback_set(mosq, my_log_callback);
    mosquitto_connect_callback_set(mosq, my_connect_callback);
    mosquitto_message_callback_set(mosq, my_message_callback);
    //mosquitto_subscribe_callback_set(mosq, my_subscribe_callback);
    //客户端连接服务器
    int ret = mosquitto_connect_async(mosq, HOST, PORT, KEEP_ALIVE);
        if(ret){
                printf("Connect server error!\n");
                mosquitto_destroy(mosq);
                mosquitto_lib_cleanup();
               // return 0;
        }

       int loop = mosquitto_loop_start(mosq); 
	   if(loop != MOSQ_ERR_SUCCESS)
	   {
			printf("mosquitto loop error\n");
			//return 1;
	   }

        // 开始通信：循环执行、直到运行标志g_iRunFlag被改变
        printf("Start!\n");

}

webserver::~webserver()
{
	// 结束后的清理工作
        mosquitto_loop_stop(mosq, false);
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        printf("End!\n");

}

vector<gps_p> webserver::extract_coor(string buff)
{
	Json::Reader reader;  
	Json::Value root;
	vector<gps_p> pos;
	if (reader.parse(buff, root))  // reader将Json字符串解析到root，root将包含Json里所有子元素  
    { 
	taskId=root["taskId"].asString();
	Json::Value arrayObj = root["pts"];
	//cout<<arrayObj.size()<<endl;
	for (int i=0; i<arrayObj.size(); i++)
		{
			double lng = arrayObj[i]["lng"].asDouble();
			double lat = arrayObj[i]["lat"].asDouble();
			double ht = arrayObj[i]["ht"].asFloat();
			//cout <<"Extract Coor:"<<endl<<setprecision(11)<<lng<<","<<lat<<","<<ht<<endl;
			gps_p p;
			p.lon=lng,p.lat=lat,p.alt=ht;
			pos.push_back(p);
		}return pos;
     }else return pos;
}
