#include "gps_m8n.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "time.h"
#include "cmsis_os.h"

#define DEG_TO_RAD 0.01745329251994329576f
#define RAD_TO_DEG 57.29577951308232087679f
#define CONSTANTS_RADIUS_OF_EARTH 6371000       /* meters (m)*/
#define DBL_EPSILON 2.2204460492503131e-16      /* 1E-9 */

extern osSemaphoreId myBinarySem07GPSUpdateHandle;
extern osMessageQId myQueue02GPSM8NToInsHandle;

#define GPS_M8N_BUF_LENGTH  150
#define NUM_OF_COMMA_IN_PUBX00  20
#define PI 3.1415926535

uint8_t gps_m8n_buffer[GPS_M8N_BUF_LENGTH];

_GPS_M8N gps_m8n;
int GPS_M8N_STATUS = -1;
int GPS_M8N_HOME_POINT_STATUS = 0;
double home_point_latitude = 0.0;
double home_point_longitude = 0.0;

int gps_m8n_thread(float dT)
{   
    gps_m8n_status_preparation(dT);   
    return 0;
}

int gps_m8n_receive(uint8_t data)
{
    static int data_receive_stage = 0;
    static int _data_len = 0;
    static int _data_cnt = 0;
    if((data_receive_stage == 0))
    {
        if(data == '$')
        {
            gps_m8n_buffer[data_receive_stage] = data;
            data_receive_stage = 1;
        }
        else
        {
            data_receive_stage = 0;
            memset(gps_m8n_buffer,0,sizeof(uint8_t)*GPS_M8N_BUF_LENGTH);
        }
        return 0;
    }
    if((data_receive_stage == 1))
    {
        if(data == 'P')
        {
            gps_m8n_buffer[data_receive_stage] = data;
            data_receive_stage = 2;
        }
        else
        {
            data_receive_stage = 0;
            memset(gps_m8n_buffer,0,sizeof(uint8_t)*GPS_M8N_BUF_LENGTH);
        }
        return 0;
    }
    if((data_receive_stage == 2))
    {
        if(data == 'U')
        {
            gps_m8n_buffer[data_receive_stage] = data;
            data_receive_stage = 3;
        }
        else
        {
            data_receive_stage = 0;
            memset(gps_m8n_buffer,0,sizeof(uint8_t)*GPS_M8N_BUF_LENGTH);
        }
        return 0;
    }
    if((data_receive_stage == 3))
    {
        if(data == 'B')
        {
            gps_m8n_buffer[data_receive_stage] = data;
            data_receive_stage = 4;
        }
        else
        {
            data_receive_stage = 0;
            memset(gps_m8n_buffer,0,sizeof(uint8_t)*GPS_M8N_BUF_LENGTH);
        }
        return 0;
    }
    if((data_receive_stage == 4))
    {
        if(data == 'X')
        {
            gps_m8n_buffer[data_receive_stage] = data;
            data_receive_stage = 5;
        }
        else
        {
            data_receive_stage = 0;
            memset(gps_m8n_buffer,0,sizeof(uint8_t)*GPS_M8N_BUF_LENGTH);
        }
        return 0;
    }
    if((data_receive_stage == 5))
    {
        if(data == ',')
        {
            gps_m8n_buffer[data_receive_stage] = data;
            data_receive_stage = 6;
        }
        else
        {
            data_receive_stage = 0;
            memset(gps_m8n_buffer,0,sizeof(uint8_t)*GPS_M8N_BUF_LENGTH);
        }
        return 0;
    }
    if((data_receive_stage == 6))
    {
        if(data == '0')
        {
            gps_m8n_buffer[data_receive_stage] = data;
            data_receive_stage = 7;
        }
        else
        {
            data_receive_stage = 0;
            memset(gps_m8n_buffer,0,sizeof(uint8_t)*GPS_M8N_BUF_LENGTH);
        }
        return 0;
    }
    if((data_receive_stage == 7))
    {
       if(data == '0')
        {
            gps_m8n_buffer[data_receive_stage] = data;
            data_receive_stage = 8;
        }
        else
        {
            data_receive_stage = 0;
            memset(gps_m8n_buffer,0,sizeof(uint8_t)*GPS_M8N_BUF_LENGTH);
        }
        return 0;
    }
    if((data_receive_stage == 8))
    {
        if(data == ',')
        {
            gps_m8n_buffer[data_receive_stage] = data;
            data_receive_stage = 9;
            _data_len = NUM_OF_COMMA_IN_PUBX00 - 2;
            _data_cnt = 8;
        }
        else
        {
            data_receive_stage = 0;
            memset(gps_m8n_buffer,0,sizeof(uint8_t)*GPS_M8N_BUF_LENGTH);
        }
        return 0;
    }
    else if((data_receive_stage == 9) && (_data_len >= 0))
    {
        _data_cnt++;
        gps_m8n_buffer[_data_cnt] = data;
        if(data == ',')
        {
            _data_len--;
            if(_data_len == 0)
            {
                gps_m8n_buffer[_data_cnt] = '*';    //帧尾，自定义以 '*' 结尾
                //解析接收到的数据
                gps_m8n_data_extraction(gps_m8n_buffer);
                //清零接收缓存
                memset(gps_m8n_buffer,0,sizeof(uint8_t)*GPS_M8N_BUF_LENGTH);
                //接收状态位置0
                data_receive_stage = 0;
            }
        }
    }
    return 0;
}


int gps_m8n_data_extraction(uint8_t *data)
{
	//memset(&gps_m8n, 0, sizeof(gps_m8n));   //清除上一次的数据
	int comma = 0;      //逗号数
	int sub_index = 0;  //帧字符串索引
	int buf_index = 0;  
	uint8_t *Pstr;  //字符指针
	uint8_t Sbuf;   
	//$PUBX,00,071802.60,3101.66654,N,12126.18801,E,41.051,G3,1.7,3.2,0.035,3.41,0.012,,0.84,1.31,0.89,12,0,0*60
	uint8_t *target = (uint8_t*)strstr((char*)data, (char*)"$PUBX,");
	if (target != NULL)
	{
		buf_index = target - data;  //uint8_t类型地址分配为1个字节，两者差值正好就是索引位
	}
	if(buf_index==0)
	{
		comma = 0;
		Pstr = data + buf_index + 9;
		do {
			Sbuf = *Pstr++;
			switch (Sbuf)
			{
                case ',':comma++;   //通过逗号的数目来进行状态分类
                    sub_index = 0;
                    break;
                default:
                {
                    switch (comma)
                    {
                        //UTC
                        case 0:
                        {
                            gps_m8n._UTC[sub_index] = Sbuf;
                            if (*Pstr == ',')
                            {
                                gps_m8n._UTC[sub_index + 1] = '\0';
                            }
                            break;
                        }
                        //Latitude
                        case 1:
                        {
                            gps_m8n._Lat[sub_index] = Sbuf;
                            if (*Pstr == ',')
                            {
                                gps_m8n._Lat[sub_index + 1] = '\0';
                            }
                            break;
                        }
                        //Northing Indicator
                        case 2:
                        {
                            gps_m8n.Northing_Indicator = Sbuf;
                            break;
                        }
                        //Longitude
                        case 3:
                        {
                            gps_m8n._Lon[sub_index] = Sbuf;
                            if (*Pstr == ',')
                            {
                                gps_m8n._Lon[sub_index + 1] = '\0';
                            }
                            break;
                        }
                        //Easting Indicator
                        case 4:
                        {
                            gps_m8n.Easting_Indicator = Sbuf;
                            break;
                        }
                        //Alt
                        case 5:
                        {
                            gps_m8n._Alt[sub_index] = Sbuf;
                            if (*Pstr == ',')
                            {
                                gps_m8n._Alt[sub_index + 1] = '\0';
                            }
                            break;
                        }
                        //status
                        case 6:
                        {
                            gps_m8n.status[sub_index] = Sbuf;
                            if (*Pstr == ',')
                            {
                                gps_m8n.status[sub_index + 1] = '\0';
                            }
                            break;
                        }
                        //Horizontal Accuracy
                        case 7:
                        {
                            gps_m8n._Horizontal_Accuracy[sub_index] = Sbuf;
                            if (*Pstr == ',')
                            {
                                gps_m8n._Horizontal_Accuracy[sub_index + 1] = '\0';
                            }
                            break;
                        }
                        //Vertical Accuracy
                        case 8:
                        {
                            gps_m8n._Vertical_Accuracy[sub_index] = Sbuf;
                            if (*Pstr == ',')
                            {
                                gps_m8n._Vertical_Accuracy[sub_index + 1] = '\0';
                            }
                            break;
                        }
                        //SOG
                        case 9:
                        {
                            gps_m8n._SOG[sub_index] = Sbuf;
                            if (*Pstr == ',')
                            {
                                gps_m8n._SOG[sub_index + 1] = '\0';
                            }
                            break;
                        }
                        //COG(true))
                        case 10:
                        {
                            gps_m8n._COG_TURE[sub_index] = Sbuf;
                            if (*Pstr == ',')
                            {
                                gps_m8n._COG_TURE[sub_index + 1] = '\0';
                            }
                            break;
                        }
                        //VD : velocity down
                        case 11:
                        {
                            gps_m8n._VD[sub_index] = Sbuf;
                            if (*Pstr == ',')
                            {
                                gps_m8n._VD[sub_index + 1] = '\0';
                            }
                            break;
                        }
                        //HDOP : horizontal dillution of precision
                        case 13:
                        {
                            gps_m8n._HDOP[sub_index] = Sbuf;
                            if (*Pstr == ',')
                            {
                                gps_m8n._HDOP[sub_index + 1] = '\0';
                            }
                            break;
                        }
                        //VDOP : vertical dillution of precision
                        case 14:
                        {
                            gps_m8n._VDOP[sub_index] = Sbuf;
                            if (*Pstr == ',')
                            {
                                gps_m8n._VDOP[sub_index + 1] = '\0';
                            }
                            break;
                        }
                        //TDOP : time dillution of precision
                        case 15:
                        {
                            gps_m8n._TDOP[sub_index] = Sbuf;
                            if (*Pstr == ',')
                            {
                                gps_m8n._TDOP[sub_index + 1] = '\0';
                            }
                            break;
                        }
                        //SVs Used
                        case 16:
                        {
                            gps_m8n._SVs_Used[sub_index] = Sbuf;
                            if (*Pstr == ',')
                            {
                                gps_m8n._SVs_Used[sub_index + 1] = '\0';
                            }
                            break;
                        }
                        default:break;
                    }
                    sub_index++;
                    break;
                }
			}
		} while (Sbuf != '*');
	}
	//数据解析
	gps_m8n.UTC = atof((char*)gps_m8n._UTC);
	double lat_temp = strtod((char*)gps_m8n._Lat, NULL)*0.01; //ddmm.mmmm
	double lon_temp = strtod((char*)gps_m8n._Lon, NULL)*0.01; //dddmm.mmmm
	gps_m8n.latitude = (double)((int)(lat_temp)) + 100 * (lat_temp - (double)((int)(lat_temp))) / 60.0;
	gps_m8n.longitude = (double)((int)(lon_temp)) + 100 * (lon_temp - (double)((int)(lon_temp))) / 60.0;
	gps_m8n.alt = atof((char*)gps_m8n._Alt);
	gps_m8n.horizontal_accuracy = atof((char*)gps_m8n._Horizontal_Accuracy);
	gps_m8n.vertical_accuracy = atof((char*)gps_m8n._Vertical_Accuracy);
	gps_m8n.SOG = atof((char*)gps_m8n._SOG)*0.277778f;    //原始数据是km/h，换算成 m/s  1000/3600 = 5/18;//
	gps_m8n.COG_TURE = atof((char*)gps_m8n._COG_TURE);      //真北0 东90 南180 西270
    gps_m8n.vel_north = gps_m8n.SOG * cos(gps_m8n.COG_TURE/180.0f*PI);
    gps_m8n.vel_east = gps_m8n.SOG * sin(gps_m8n.COG_TURE/180.0f*PI);
	gps_m8n.vel_down = atof((char*)gps_m8n._VD);
	gps_m8n.HDOP = atof((char*)gps_m8n._HDOP);
	gps_m8n.VDOP = atof((char*)gps_m8n._VDOP);
	gps_m8n.TDOP = atof((char*)gps_m8n._TDOP);
	gps_m8n.SVs_Used = atoi((char*)gps_m8n._SVs_Used);
    if((int)gps_m8n.UTC!=0)
    {
        osSemaphoreRelease(myBinarySem07GPSUpdateHandle);   //GPS数据更新
    }
	return 0;
}

int gps_m8n_status_preparation(float dT)
{
    //判断GPS数据是否按照正常频率输出
    static int gps_update_watchdog;
    if(osOK == osSemaphoreWait(myBinarySem07GPSUpdateHandle,0))
    {
        det_t_s.det_t_gps_m8n_update_s = get_cycle_time(gps_m8n_update_time_index);
        if(fabs(det_t_s.det_t_gps_m8n_update_s - dT)<=0.2f) //GPS最低5Hz更新(200ms一次)，本函数20Hz执行(50ms一次),两者每次时间差不能超过200ms,否则认为不正常
        {
            gps_update_watchdog = 0;    //GPS更新——喂狗
            //判断GPS数据是否满足导航的精度要求
            if(gps_m8n.SVs_Used>=8)    //卫星数目是否满足要求
            {
                if(strcmp((char*)gps_m8n.status,(char*)"NF")!=0)    //GPS定位状态是否为G3
                {
                    if(gps_m8n.HDOP<2.0 && gps_m8n.VDOP<2.5 && gps_m8n.TDOP<2.0)    //HDOP VDOP TDOP精度是否满足要求
                    {
                        GPS_M8N_STATUS = 1;
                        //设定home点
                        if(GPS_M8N_HOME_POINT_STATUS==0)
                        {
                            gps_m8n_set_home_point();
                        }
                        //计算pos_north,pos_east
                        if(GPS_M8N_HOME_POINT_STATUS==1)
                        {
                            gps_m8n_wgs84_to_ned_haversine(gps_m8n.latitude, gps_m8n.longitude, gps_m8n.alt, &gps_m8n.pos_north, &gps_m8n.pos_east);
                            osMessagePut(myQueue02GPSM8NToInsHandle,(uint32_t)&gps_m8n,0);
                        }
                    }
                    else
                    {
                        GPS_M8N_STATUS = 0;
                    }
                }
                else
                {
                    GPS_M8N_STATUS = 0;
                }
            }
            else
            {
                GPS_M8N_STATUS = 0;
            }
        }
    }
    else
    {
        gps_update_watchdog++;
        if(gps_update_watchdog > 10)    //500ms内GPS模块没有正常输出，说明GPS模块坏了
        {
            GPS_M8N_STATUS = -1;
            memset(&gps_m8n, 0, sizeof(gps_m8n));   //清除数据
        }
    }
    return 0;
}

//1:设定成功；0：未设定成功
int gps_m8n_set_home_point(void)
{
    static double temp_home_point_latitude = 0.0;
    static double temp_home_point_longitude = 0.0;
    static int gps_valid_data_cnt = 0;
    //判断GPS数据是否满足定位要求
    if(GPS_M8N_STATUS == 1)
    {
        temp_home_point_latitude += gps_m8n.latitude;
        temp_home_point_longitude += gps_m8n.longitude;
        gps_valid_data_cnt++;
        if(gps_valid_data_cnt==10)
        {
            home_point_latitude = temp_home_point_latitude / gps_valid_data_cnt;
            home_point_longitude = temp_home_point_longitude / gps_valid_data_cnt;
            gps_valid_data_cnt = 0;
            GPS_M8N_HOME_POINT_STATUS = 1;
        }
    }
    else
    {
        //若定位过程中，数据出现异常，重新设定home点
        GPS_M8N_HOME_POINT_STATUS = 0;  
        gps_valid_data_cnt = 0;
        home_point_latitude = 0.0;
        home_point_longitude = 0.0;
        temp_home_point_latitude = 0.0;
        temp_home_point_longitude = 0.0;
    }
    return 0;
}

int gps_m8n_wgs84_to_ned_haversine(double lat, double lon, float alt, float * x, float * y)
{
    double lat_rad = lat * DEG_TO_RAD;
    double lon_rad = lon * DEG_TO_RAD;

    double home_lat_rad = home_point_latitude * DEG_TO_RAD;
    double home_lon_rad = home_point_longitude * DEG_TO_RAD;

    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double cos_d_lon = cos(lon_rad - home_lon_rad);

    double d_lat_half = (lat_rad - home_lat_rad) * 0.5;
    double d_lon_half = (lon_rad - home_lon_rad) * 0.5;

    double sin_power_d_lat_half = sin(d_lat_half) * sin(d_lat_half);
    double sin_power_d_lon_half = sin(d_lon_half) * sin(d_lon_half);

    double home_sin_lat=sin(home_lat_rad);
    double home_cos_lat=cos(home_lat_rad);
    double home_sin_lon=sin(home_lon_rad);
    double home_cos_lon=cos(home_lon_rad);

    double arg = sqrt(sin_power_d_lat_half + home_cos_lat*cos_lat*sin_power_d_lon_half);

    if (arg > 1.0) {
            arg = 1.0;
    } else if (arg < -1.0) {
            arg = -1.0;
    }

    double c = 2*asin(arg);

    double k = (fabs(c) < DBL_EPSILON) ? 1.0 : (c / sin(c));

    *x = k * (home_cos_lat * sin_lat - home_sin_lat * cos_lat * cos_d_lon) * (CONSTANTS_RADIUS_OF_EARTH + alt);
    *y = k * cos_lat * sin(lon_rad - home_lon_rad) * (CONSTANTS_RADIUS_OF_EARTH + alt);

    return 0;
}


