#ifndef _COORCHANGE_H
#define _COORCHANGE_H


#include<iostream>
#include<math.h>
#include "octomap_msgs/Octomap.h"
#include "octomap/octomap.h"
#include "octomap/OcTree.h"

#define M_PI       3.14159265358979323846

double R=6378137; 
double origin_lon=121.49928079021212, origin_lat=31.29857230433321;
double HD=M_PI/180;

class gps_p
{
public:
 double lon;
 double lat;
 double alt;

};


//coordinate of PX4 is North/West, our coordinate is West/North

gps_p coor2gpsPx4(const octomap::point3d p)
{
    double px=p.y(),py=p.x();
    double x_rad = px/R , y_rad = py/R;
    double c = sqrt(x_rad*x_rad + y_rad*y_rad);
    double sin_lat = sin(origin_lat*HD) , cos_lat = cos(origin_lat*HD);
    double lon_rad = origin_lon*HD , lat_rad = origin_lat*HD;
    double lon,lat;
    if (abs(c)>0)
    {
        double sin_c = sin(c) , cos_c = cos(c);
	lat_rad = asin(cos_c * sin_lat + (x_rad*sin_c*cos_lat)/c);
	lon_rad = (lon_rad + atan2(y_rad * sin_c, c*cos_lat*cos_c - x_rad*sin_lat*sin_c));

	lat = lat_rad/HD;
	lon = lon_rad/HD;
    }
    else{
	lat = lat_rad/HD;
	lon = lon_rad/HD;
    }
    gps_p gps;
    gps.lon=lon; gps.lat=lat; gps.alt=p.z();
    return gps;
}

octomap::point3d gps2coorPx4(gps_p gps)
{
    double lat_rad = gps.lat*HD , lon_rad = gps.lon*HD;
    double sin_lat = sin(lat_rad) , cos_lat = cos(lat_rad);
    double lon_rad_ref = origin_lon*HD , lat_rad_ref = origin_lat*HD;
    double sin_lat_ref = sin(lat_rad_ref) , cos_lat_ref = cos(lat_rad_ref);
    double cos_d_lon = cos(lon_rad - lon_rad_ref);
    double arg = sin_lat_ref*sin_lat + cos_lat_ref*cos_lat*cos_d_lon;
    
    if(arg>1){arg=1;}
    else if(arg<-1){arg=-1;}
    
    double c = acos(arg);
    double k = 1;
    
    if(abs(c)>0){k=(c/sin(c));}
      
    double x = k*(cos_lat_ref*sin_lat - sin_lat_ref*cos_lat*cos_d_lon) * R;
    double y = k*cos_lat*sin(lon_rad - lon_rad_ref) * R;
    return octomap::point3d(y,x,gps.alt);
    
}
/*gps_p coor2gps(const point3d p)
{
    double _lat=p.y() /(2*R*M_PI/360); //cout<<"  _lat is: "<<_lat<<endl;
    double _lon=p.x() /(R*cos(origin_lat)*2*M_PI/360); //cout<<"  _lnt is: "<<_lng<<endl;
    double lat=origin_lat+_lat;
    double lon=origin_lon+_lon;
    gps_p gps;
    gps.lon=lon;gps.lat=lat;gps.alt=p.z();
    return gps;
}
*/



#endif
