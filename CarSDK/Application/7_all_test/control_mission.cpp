/*******************************************************************************
 *  INCLUDE FILES
 *******************************************************************************
 */
#include <termios.h>
#include <unistd.h> 
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <time.h>
#include <string.h>
#include <pthread.h> 
#include <dlfcn.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <stdlib.h>
#include "control_mission.h"
#include "car_lib.h"



extern "C" {

	static int dist_table[6][28] = {
	{4095, 4063, 3400, 2940, 2516, 2211, 1897, 1712,
	1488, 1331, 1233, 1133, 1038, 974, 920, 879, 783, 750,
	688, 640, 580, 551, 519, 486, 455, 422, 390, 357},
	{4095, 4070, 3580, 3059, 2667, 2333, 2076, 1890,
	1708, 1566, 1440, 1312, 1210, 1115, 1020, 956, 890, 827,
	796, 765, 700, 666, 632, 600, 566, 520, 460, 430},
	{4095,4090,3520,3040,2669,2330,2060,1894,
	1708,1553,1440,1313,1219,1125,1030,958,925,893,
	830,798,702,669,637,605,573,541,470,437},
	{4095,3415,2935,2560,2260,2008,1790,1605,
	1470,1375,1244,1117,1016,952,890,820,750,710,
	665,610,553,495,474,456,427,410,393,380},
	{4095,4080,3617,3112,2685,2360,2080,1851,
	1665,1534,1404,1260,1153,1090,1020,987,955,910,
	890,860,828,795,773,765,740,725,700,690},
	{4095,4080,3648,3084,2663,2334,2050,1837,
	1706,1520,1395,1299,1200,1135,1085,1040,1008,979,
	945,914,882,850,820,805,790,773,757,720}
	};

	int DistanceSensor_cm(int channel)
	{
		return sensor_dist(channel, DistanceSensor(channel));
	}

	int sensor_dist(int channel, int input) {
		int position = 0;
		int i = 0;
		for (i = 0; i < 28; i++) {
			if (input > dist_table[channel - 1][i]) {
				position = i + 3;
				break;
			}
			else {}
		}
		if (position == 0) {
			position = 50;
		}
		return position;
	}

	int StopLine(int Lineflag) {
		char sensor;
		char byte = 0x80;
		sensor = LineSensor_Read();        // black:1, white:0
	//	printf("LineSensor_Read() = ");
		int flag = 0;
		int i;
		for (i = 0; i < 8; i++)
		{
			if ((i % 4) == 0) printf(" ");
			if ((sensor & byte)) {
				//			printf("1");
				flag++;
			}//byte == 0x80 == 0111 0000 (2)
	//		else printf("0");
			sensor = sensor << 1;
		}
		//	printf("\n");
		//	printf("LineSensor_Read() = %d \n", sensor);
		if (flag > Lineflag) {
			return 1;
		}
		return 0;
	}

	void DesiredDistance(int SettingSpeed, int SettingDistance)
	{
		int init_encoder = 0;
		int on_encoder = 0;
		EncoderCounter_Write(init_encoder);
		DesireSpeed_Write(SettingSpeed);
		while (1) {
			on_encoder = abs(EncoderCounter_Read());
			if (on_encoder != 65278) printf("encoder : %-4d\n", on_encoder);
			if (on_encoder >= SettingDistance && on_encoder != 65278) {
				DesireSpeed_Write(0);
				break;
			}
			usleep(100000);
		}
	}

	
	


}//extern "C"


/*/////////////////////////////
		ROUNDABOUT START
*//////////////////////////////
int flag_go, flag_wait, flag_stop, flag_end;
int check_start;
int lower_StopDistance = 25;
int uper_StopDistance = 40;
int lower_RoundDistance = 20;
int uper_RoundDistance = 40;
int THR_RoundAbout_END = 200;
int first_RoundAbout = 0;

void RoundAbout_Init() {
	flag_go = -1;
	check_start = 0;
	flag_wait = -1;
	flag_stop = -1;
	flag_end = -1;
}

int RoundAbout_isStart(const int Distance1) {
	if (!first_RoundAbout++) RoundAbout_Init();

	if (flag_go > 0) 
	{
		if (Distance1 >= uper_StopDistance)
			flag_go--;
	}
	else if (flag_go == 0) 
	{ 
		if (!check_start) 
		{
			check_start = 1;
			flag_wait = -1;
		}
	}
	else
	{
		if (Distance1 < lower_StopDistance)
			if (flag_wait < 2)
				flag_wait++;
		else
			if (flag_wait > 0)
				flag_wait--;

		if (flag_wait == 2) {
			flag_wait = -1;
			flag_go = 35; // 해당 프레임이 지난 후 출발할 것임
		}
	}
	return check_start;
}

int RoundAbout_isDelay(const int Distance1, int& speed) {
	if (Distance1 < lower_RoundDistance) {
		if (flag_wait < 2)
			flag_wait++;

		if (flag_wait == 2) {
			flag_wait = -1;
			flag_stop = 30; // 해당 프레임만큼 정지
			return 1;
		}
		return 0;
	}
	else {
		if (flag_wait > 0)
			flag_wait--;
		
		if (flag_stop = 0) {
			flag_stop = -1;
			if (speed > 30) { // 일정 속도 이상일 경우 감속
				speed -= 5;
			}
			return 0;
		}
		else if (flag_stop > 0) {
			if (Distance1 >= uper_RoundDistance) {
				flag_stop--;
			}
			return 1;
		}
		else
		{
			return 0;
		}
	}
}

int RoundAbout_End(const int Distance1, const int Distance2) {
	if ((Distance1 > 40) && (Distance2 > 40)) { // 거리 센서에 아무것도 잡히지 않을 때
		if (flag_end < THR_RoundAbout_END) // 해당 프레임이 지나면 분기 벗어남
			flag_end++;
	}
	else {
		if (flag_end > 0)
			flag_end--;
	}

	if (flag_end > THR_RoundAbout_END) {
		flag_end = -1;
		return 1;
	}
	return 0;
}
/*/////////////////////////////
		ROUNDABOUT END
*//////////////////////////////