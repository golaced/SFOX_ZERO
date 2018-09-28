#include <stdint.h>
#include "cmsis_os.h"

typedef struct {
	//时间
	uint8_t _UTC[20];
	float UTC;
	//纬度
	uint8_t _Lat[20];
	double latitude;
	uint8_t Northing_Indicator;
	//经度
	uint8_t _Lon[20];
	double longitude;
	uint8_t Easting_Indicator;
    //相对于home点的pos_north,单位m
    float pos_north;
    //相对于home点的pos_east,单位m
    float pos_east;
	//高度(HAE)
	uint8_t _Alt[20];
	float alt;
	//水平定位精度
	uint8_t _Horizontal_Accuracy[10];
	float horizontal_accuracy;
	//定位状态
	uint8_t status[10];
	//竖直定位精度
	uint8_t _Vertical_Accuracy[10];
	float vertical_accuracy;
	//speed over ground 对地航速
	uint8_t _SOG[10];
	float SOG;
    float vel_north;
    float vel_east;
	//course over ground 对地航向
	uint8_t _COG_TURE[10];
	float COG_TURE;
	//向下速度
	uint8_t _VD[10];
	float vel_down;
	//水平精度因子 horizontal dillution of precision
	uint8_t _HDOP[10];
	float HDOP;
	//竖直精度因子  vertical dillution of precision
	uint8_t _VDOP[10];
	float VDOP;
	//时间精度因子  time dillution of precision
	uint8_t _TDOP[10];
	float TDOP;
	//可观测卫星数
	uint8_t _SVs_Used[10];
	int SVs_Used;
}_GPS_M8N;

//存储M8N模块获取的数据
extern _GPS_M8N gps_m8n;
//-1——无GPS数据
//0——数据较差，不能起飞或者飞行导航；
//1——数据良好，可以利用GPS起飞及飞行导航；
extern int GPS_M8N_STATUS;
//0：设定home点后，而后取消了；   1：home点状态正常
extern int GPS_M8N_HOME_POINT_STATUS;
extern double home_point_latitude;
extern double home_point_longitude;

int gps_m8n_receive(uint8_t data);

int gps_m8n_data_extraction(uint8_t *data);

int gps_m8n_set_home_point(void);//1:设定成功；0：未设定成功

int gps_m8n_status_preparation(float dT);

int gps_m8n_thread(float dT);

int gps_m8n_wgs84_to_ned_haversine(double lat, double lon, float alt, float * x, float * y);

