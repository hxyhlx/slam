#ifndef GLOBAL_PLANNER_WEB_H
#define GLOBAL_PLANNER_WEB_H

#include <stdio.h>
#include <stdlib.h>
#include <mosquitto.h>
#include <string.h>
#include <iostream>
#include <vector>

#define  SIMULATION 1    //0 表示连接webgis

#define HOST "47.110.134.250"
#define PORT  21883
#define KEEP_ALIVE 60
#define MSG_MAX_SIZE  512

bool session = true;
char temp[512] ;

//vector<double> buff;
class gps_p;
class webserver
{
public: 
	struct mosquitto *mosq = NULL;
	webserver();
	~webserver();
	void pub(struct mosquitto *mosq, std::vector<gps_p> playload, std::string msg);
	int coor_message_get=0;
	std::vector<gps_p> extract_coor(std::string buff); 

};
#endif 


