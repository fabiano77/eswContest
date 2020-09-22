/*******************************************************************************
 *  INCLUDE FILES
 *******************************************************************************
 */
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include <pthread.h>
#include <dlfcn.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "control_mission.h"
#include "car_lib.h"

using namespace std;
///////////////////////////////////////////////////////////////////////////////////
int flag_line;
int flag_go, flag_wait, flag_stop, flag_end;
int check_start;
int lower_RoundDistance = 20;
int uper_RoundDistance = 30;
int THR_RoundAbout_END = 80;
int first_RoundAbout = 0;
int first_Line = 0;

void RoundAbout_Init();

int flag_Tunnel;
int first_Tunnel = 0;
int absDist;
int steerVal;
int flag_steer[5];

bool sensor_using_flag = false;

extern "C"
{

	static int dist_table[6][28] = {
		{4095, 4063, 3400, 2940, 2516, 2211, 1897, 1712,
		 1488, 1331, 1233, 1133, 1038, 974, 920, 879, 783, 750,
		 688, 640, 580, 551, 519, 486, 455, 422, 390, 357},
		{4095, 4070, 3580, 3059, 2667, 2333, 2076, 1890,
		 1708, 1566, 1440, 1312, 1210, 1115, 1020, 956, 890, 827,
		 796, 765, 700, 666, 632, 600, 566, 520, 460, 430},
		{4095, 4090, 3520, 3040, 2669, 2330, 2060, 1894,
		 1708, 1553, 1440, 1313, 1219, 1125, 1030, 958, 925, 893,
		 830, 798, 702, 669, 637, 605, 573, 541, 470, 437},
		{4095, 3415, 2935, 2560, 2260, 2008, 1790, 1605,
		 1470, 1375, 1244, 1117, 1016, 952, 890, 820, 750, 710,
		 665, 610, 553, 495, 474, 456, 427, 410, 393, 380},
		{4095, 4080, 3617, 3112, 2685, 2360, 2080, 1851,
		 1665, 1534, 1404, 1260, 1153, 1090, 1020, 987, 955, 910,
		 890, 860, 828, 795, 773, 765, 740, 725, 700, 690},
		{4095, 4080, 3648, 3084, 2663, 2334, 2050, 1837,
		 1706, 1520, 1395, 1299, 1200, 1135, 1085, 1040, 1008, 979,
		 945, 914, 882, 850, 820, 805, 790, 773, 757, 720}};

	int DistanceSensor_cm(int channel)
	{
		while (sensor_using_flag)
		{
			usleep(5000); //5ms
		}
		sensor_using_flag = true;
		int retval = sensor_dist(channel, DistanceSensor(channel));
		sensor_using_flag = false;
		return retval;
	}

	int sensor_dist(int channel, int input)
	{
		int position = 0;
		int i = 0;
		for (i = 0; i < 28; i++)
		{
			if (input > dist_table[channel - 1][i])
			{
				position = i + 3;
				break;
			}
			else
			{
			}
		}
		if (position == 0)
		{
			position = 50;
		}
		return position;
	}

	signed int Encoder_Read(void)
	{
		while (sensor_using_flag)
		{
			usleep(5000); //5ms
		}
		sensor_using_flag = true;
		int retval = EncoderCounter_Read();
		sensor_using_flag = false;
		return retval;
	}

	int StopLine(int Lineflag)
	{ // black 1, white 0
		char sensor;
		char byte = 0x80;

		while (sensor_using_flag)
		{
			usleep(5000); //5ms
		}
		sensor_using_flag = true;
		sensor = LineSensor_Read();
		sensor_using_flag = false;
		int flag = 0;
		int i;
		for (i = 0; i < 8; i++)
		{
			if (i == 0)
				printf(" "); //ì²? ë²ˆì§¸ ë¹„íŠ¸ ë¬´ì‹œ
			else
			{
				if ((i % 4) <= 1)
					printf(" ");
				if ((sensor & byte))
				{
					//printf("1");	// black
					printf("_");
				}
				else
				{
					//printf("0");	// white
					printf("w");
					flag++;
				}
			}
			sensor = sensor << 1;
		}
		cout << " // cnt white = " << flag << endl;
		//printf("LineSensor_Read() = %d \n", sensor);
		if (flag > Lineflag)
		{
			return 1;
		}
		return 0;
	}

	int STOP_WhiteLine(int Lineflag)
	{
		if (!first_Line++)
			flag_line = 0;

		if (StopLine(Lineflag))
		{
			flag_line++;
			if (flag_line > 2)
			{
				flag_line = 0;
				return 1;
			}
		}
		else
		{
			if (flag_line > 0)
				flag_line--;
		}
		return 0;
	}

	void SteeringServo_Write_uart(signed short angle)
	{
		while (sensor_using_flag)
		{
			usleep(500); //0.5ms
		}
		sensor_using_flag = true;
		SteeringServoControl_Write(angle);
		sensor_using_flag = false;
	}

	void DesireSpeed_Write_uart(signed short speed)
	{
		while (sensor_using_flag)
		{
			usleep(500); //0.5ms
		}
		sensor_using_flag = true;
		DesireSpeed_Write(speed);
		sensor_using_flag = false;
	}

	void DesiredDistance(int SettingSpeed, int SettingDistance, int SettingSteering)
	{
		//cout << "speed = " << SettingSpeed << "dist = " << SettingDistance << "steer = " << SettingSteering << endl;
		DesireSpeed_Write(0);
		usleep(5000);
		int init_encoder = 0;
		int on_encoder = 0;
		EncoderCounter_Write(init_encoder);
		usleep(10000);
		int read_encoder = Encoder_Read();
		int error_flag = 0;
		while (read_encoder != 0)
		{
			//cout << "[]read_encoder" << read_encoder << endl;
			read_encoder = Encoder_Read();
			usleep(50000);
			EncoderCounter_Write(init_encoder);
			usleep(50000);
			if (error_flag++ > 10)
				printf("DesireDistance(): encoder ERROR!\n");
		}
		SteeringServo_Write_uart(SettingSteering);
		usleep(80000);
		DesireSpeed_Write(SettingSpeed);
		while (1)
		{
			if (SettingSpeed > 0)
			{
				//? •ë©? ì¶©ëŒ ê°ì??
				if (DistanceSensor_cm(1) <= 6)
				{
					DesireSpeed_Write(0);
					cout << "\tDesiredDistance() : front detection!" << endl;
					break;
				}
			}
			else
			{
				//?›„ë©? ì¶©ëŒ ê°ì??
				if (DistanceSensor_cm(4) <= 7)
				{
					DesireSpeed_Write(0);
					cout << "\tDesiredDistance() : rear detection!" << endl;
					break;
				}
			}

			on_encoder = abs(Encoder_Read());
			if (on_encoder != CHECKSUMERROR) // 65278
			{
				//printf("encoder : %-4d\n", on_encoder);
				if (on_encoder >= SettingDistance)
				{
					DesireSpeed_Write(0);
					break;
				}
			}
			usleep(10000); // 50->20 ms ë³?ê²? 09/14
		}
		//cout << "\tDesiredDistance() :encoder = " << on_encoder << endl;
	}

	void onlyDistance(int SettingSpeed, int SettingDistance)
	{
		int init_encoder = 0;
		int on_encoder = 0;
		EncoderCounter_Write(init_encoder);
		DesireSpeed_Write(SettingSpeed);
		if (SettingSpeed < 0)
			CarLight_Write(0x02);
		while (1)
		{
			if (SettingSpeed > 0)
			{
				//? •ë©? ì¶©ëŒ ê°ì??
				if (DistanceSensor_cm(1) <= 5)
				{
					cout << "onlyDistance() : front detection!" << endl;
					DesireSpeed_Write(0);
					break;
				}
			}
			else
			{
				//?›„ë©? ì¶©ëŒ ê°ì??
				if (DistanceSensor_cm(4) <= 5)
				{
					cout << "onlyDistance() : rear detection!" << endl;
					DesireSpeed_Write(0);
					break;
				}
			}

			on_encoder = abs(Encoder_Read());
			if (on_encoder != CHECKSUMERROR) // 65278
			{
				//printf("encoder : %-4d\n", on_encoder);
				if (on_encoder >= SettingDistance)
				{
					DesireSpeed_Write(0);
					break;
				}
			}
			usleep(20000); // 10ms
		}
		if (SettingSpeed < 0)
			CarLight_Write(0x00);
	}

	int RoundAbout_isStart(const int Distance1)
	{
		if (!first_RoundAbout++)
			RoundAbout_Init();
		int lower_StopDistance = 25;
		int uper_StopDistance = 30;

		if (flag_go > 0)
		{
			if (Distance1 > uper_StopDistance)
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
			{
				if (flag_wait < 2)
					flag_wait++;
				else if (flag_wait > 0)
					flag_wait--;
			}
			if (flag_wait == 2)
			{
				flag_wait = -1;
				flag_go = 25; // ï¿½Ø´ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿? ï¿½ï¿½ï¿½ï¿½
			}
		}
		return check_start;
	}

	int Tunnel_isEnd(const int Distance1, const int Distance2, const int Distance3, const int Distance4)
	{
		/* 2,6 - 3,5 (1,2 - 3,4)
		1. ï¿½ï¿½(2, 6)
		2. ï¿½ï¿½(3, 5)
		*/
		int i;
		if (first_Tunnel == 1)
		{
			flag_Tunnel = 0;
			steerVal = 0;
			for (i = 0; i < 5; i++)
			{
				flag_steer[i] = 0;
			}
		}

		if ((Distance1 > 30) && (Distance2 > 30))
		{
			if (flag_Tunnel < 2)
				flag_Tunnel++;
		}
		else
		{
			if (flag_Tunnel > 0)
				flag_Tunnel--;
		}

		if ((Distance3 > 30) && (Distance4 > 30))
		{
			if ((flag_Tunnel >= 2) && (flag_Tunnel < 4))
				flag_Tunnel++;
		}
		else
		{
			if (flag_Tunnel >= 2)
				flag_Tunnel--;
		}

		if (flag_Tunnel == 4)
		{
			return 1;
		}
		return 0;
	}

	int Tunnel_SteerVal(const int Distance1, const int Distance2)
	{

		absDist = abs(Distance1 - Distance2);

		if (absDist < 2)
		{
			steerVal = 0;
		}
		else if (absDist < 4)
		{
			steerVal = 100;
		}
		else if (absDist < 6)
		{
			steerVal = 220;
		}
		else if (absDist < 8)
		{
			steerVal = 350;
		}
		else
		{
			steerVal = 480;
		}

		if (Distance1 < Distance2)
			steerVal = -steerVal;

		printf("steer : %d, [%d, %d]\n", steerVal, Distance2, Distance1);
		return 1500 - steerVal;
	}

	void frontLightOnOff(unsigned short lightFlag, int on)
	{
		if (on == 1)
		{
			if (!(lightFlag & 0x01))		  //1ï¿½ï¿½ ï¿½ï¿½Æ®ï¿½ï¿½ 1ï¿½ï¿½ ï¿½Æ´Ò°ï¿½ï¿?
				lightFlag = lightFlag ^ 0x01; //XORï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ 1ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ø´ï¿?.
		}
		else if (on == 0)
		{
			if (lightFlag & 0x01)			  //1ï¿½ï¿½ ï¿½ï¿½Æ®ï¿½ï¿½ 1ï¿½Ï°ï¿½ï¿?
				lightFlag = lightFlag ^ 0x01; //XORï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ 0ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ø´ï¿?.
		}
		CarLight_Write(lightFlag);
	}

	void rearLightOnOff(unsigned short lightFlag, int on)
	{
		if (on == 1)
		{
			if (!(lightFlag & 0x02))		  //2ï¿½ï¿½ ï¿½ï¿½Æ®ï¿½ï¿½ 1ï¿½ï¿½ ï¿½Æ´Ò°ï¿½ï¿?
				lightFlag = lightFlag ^ 0x02; //XORï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ 1ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ø´ï¿?.
		}
		else if (on == 0)
		{
			if (lightFlag & 0x02)			  //2ï¿½ï¿½ ï¿½ï¿½Æ®ï¿½ï¿½ 1ï¿½Ï°ï¿½ï¿?
				lightFlag = lightFlag ^ 0x02; //XORï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ 0ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ø´ï¿?.
		}
		CarLight_Write(lightFlag);
	}

	int auto_speedMapping(int steerVal, const int basicSpeed)
	{
		int absSteer = abs(steerVal);

		// if (absSteer < 200)
		// 	return basicSpeed;
		// else if (absSteer < 300)
		// 	return int(basicSpeed * 1.15);
		// else if (absSteer < 400)
		// 	return int(basicSpeed * 1.25);
		// else if (absSteer <= 500)
		// 	return int(basicSpeed * 1.35);
		// else
		// 	return basicSpeed;

		if (absSteer < 200)
			return basicSpeed;
		else
			return basicSpeed * (1 + 0.3 * (absSteer - 200) / 300);
	}

	void buzzer(int numOfTimes, int interval_us, int pulseWidth_us)
	{
		for (int i = 0; i < numOfTimes; i++)
		{
			Alarm_Write(ON);
			usleep(pulseWidth_us);
			Alarm_Write(OFF);
			usleep(interval_us);
		}
	}

	void manualControl(struct ControlData *cdata, char key)
	{
		int i;
		switch (key)
		{
		case 'a': //steering left		: servo ì¡°í–¥ê°? (2000(ì¢?) ~ 1500(ì¤?) ~ 1000(?š°)
			cdata->steerVal += 50;
			SteeringServo_Write_uart(cdata->steerVal);
			printf("angle_steering = %d\n", cdata->steerVal);
			printf("SteeringServoControl_Read() = %d\n", SteeringServoControl_Read()); //default = 1500, 0x5dc
			break;

		case 'd': //steering right	: servo ì¡°í–¥ê°? (2000(ì¢?) ~ 1500(ì¤?) ~ 1000(?š°)
			cdata->steerVal -= 50;
			SteeringServo_Write_uart(cdata->steerVal);
			printf("angle_steering = %d\n", cdata->steerVal);
			printf("SteeringServoControl_Read() = %d\n", SteeringServoControl_Read()); //default = 1500, 0x5dc
			break;

		case 's':
			DesireSpeed_Write_uart(0);
			break;

		case 'w': //go forward

			cdata->desireSpeedVal = cdata->settingSpeedVal;
			DesireSpeed_Write_uart(cdata->desireSpeedVal);
			break;

		case 'x': //go backward	speed ?Œ?ˆ˜ ?¸ê°??•˜ë©? ?›„ì§?.

			cdata->desireSpeedVal = -cdata->settingSpeedVal;
			DesireSpeed_Write_uart(cdata->desireSpeedVal);
			break;

			// case 'j':	//cam left
			// 	angle_cameraX += 50;
			// 	CameraXServoControl_Write(angle_cameraX);
			// 	printf("angle_cameraX = %d\n", angle_cameraX);
			// 	printf("CameraXServoControl_Read() = %d\n", CameraXServoControl_Read());    //default = 1500, 0x5dc
			// 	break;

			// case 'l':	//cam right
			// 	angle_cameraX -= 50;
			// 	CameraXServoControl_Write(angle_cameraX);
			// 	printf("angle_cameraX = %d\n", angle_cameraX);
			// 	printf("CameraXServoControl_Read() = %d\n", CameraXServoControl_Read());    //default = 1500, 0x5dc
			// 	break;

		case 'i': //cam up
			cdata->cameraY -= 50;
			CameraYServoControl_Write(cdata->cameraY);
			printf("cdata->cameraY = %d\n", cdata->cameraY);
			printf("CameraYServoControl_Read() = %d\n", CameraYServoControl_Read()); //default = 1500, 0x5dc
			break;

		case 'k': //cam down
			cdata->cameraY += 50;
			CameraYServoControl_Write(cdata->cameraY);
			printf("angle_cameraY = %d\n", cdata->cameraY);
			printf("CameraYServoControl_Read() = %d\n", CameraYServoControl_Read()); //default = 1500, 0x5dc
			break;

		case '1': //speed up		ìµœë?? ?Š¤?”¼?“œ 500
			cdata->settingSpeedVal += 10;
			printf("speed = %d\n", cdata->settingSpeedVal);
			break;

		case '2': //speed down
			cdata->settingSpeedVal -= 10;
			printf("speed = %d\n", cdata->settingSpeedVal);
			break;

		case 'q': //Flashing left winker 3 s

			Winker_Write(LEFT_ON);
			usleep(3000000); // 3 000 000 us
			Winker_Write(ALL_OFF);
			break;

		case 'e': //Flashing right winker 3 s

			Winker_Write(RIGHT_ON);
			usleep(3000000); // 3 000 000 us
			Winker_Write(ALL_OFF);
			break;

		case 'z':										//front lamp on/off
			cdata->lightFlag = cdata->lightFlag ^ 0x01; // 00000000 ^ 00000001 (XOR)ï¿½ï¿½ï¿½ï¿½ : 0ï¿½ï¿½ï¿½ï¿½Æ®ï¿½ï¿½ XORï¿½ï¿½ï¿½ï¿½ï¿½Ñ´ï¿½.
			CarLight_Write(cdata->lightFlag);
			break;

		case 'c':										//rear lamp on/off
			cdata->lightFlag = cdata->lightFlag ^ 0x02; // 00000000 ^ 00000010 (XOR)ï¿½ï¿½ï¿½ï¿½ : 1ï¿½ï¿½ï¿½ï¿½Æ®ï¿½ï¿½ XORï¿½ï¿½ï¿½ï¿½ï¿½Ñ´ï¿½.
			CarLight_Write(cdata->lightFlag);
			break;

		case ' ': //alarm
			for (i = 0; i < 2; i++)
			{
				Alarm_Write(ON);
				usleep(200000);
				Alarm_Write(OFF);
				usleep(200000);
			}
			break;

		case '0':
			SpeedPIDProportional_Write(40);
			SpeedPIDIntegral_Write(40);
			SpeedPIDProportional_Write(40);
			break;
		case '\n':
			break;

		default:
			printf("wrong key input.\n");
			break;
		}
	}

	uint32_t timeCheck(struct timeval *tempTime)
	{
		struct timeval prevTime = *tempTime;
		struct timeval nowTime;
		gettimeofday(&nowTime, NULL);

		uint32_t retVal = ((nowTime.tv_sec - prevTime.tv_sec) * 1000) + ((int)nowTime.tv_usec / 1000 - (int)prevTime.tv_usec / 1000);
		if ((*tempTime).tv_sec == 0)
			retVal = 0;

		*tempTime = nowTime;

		return retVal;
	}

} //extern "C"

void RoundAbout_Init()
{
	flag_go = -1;
	check_start = 0;
	flag_wait = -1;
	flag_stop = -1;
	flag_end = -1;
}

/* 

	int RoundAbout_isDelay(const int Distance1)
	{
		if (Distance1 < lower_RoundDistance)
		{
			if (flag_wait < 2)
				flag_wait++;

			if (flag_wait == 2)
			{
				flag_wait = -1;
				flag_stop = 25; // ï¿½Ø´ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½Ó¸ï¿½Å­ ï¿½ï¿½ï¿½ï¿½
				return 1;
			}
			return 0;
		}
		else
		{
			if (flag_wait > 0)
				flag_wait--;

			if (flag_stop == 0)
			{
				flag_stop = -1;
				return 0;
			}
			else if (flag_stop > 0)
			{
				if (Distance1 >= uper_RoundDistance)
				{
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

	int RoundAbout_isEnd(const int Distance1, const int Distance2)
	{
		if ((Distance1 > 40) && (Distance2 > 40))
		{									   // ï¿½Å¸ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½Æ¹ï¿½ï¿½Íµï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½
			if (flag_end < THR_RoundAbout_END) // ï¿½Ø´ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½Ğ±ï¿½ ï¿½ï¿½ï¿½î³²
				flag_end++;
		}
		else
		{
			if (flag_end > 0)
				flag_end--;
		}

		if (flag_end > THR_RoundAbout_END)
		{
			flag_end = -1;
			return 1;
		}
		return 0;
	}

	int Tunnel_isTunnel(const int Distance1, const int Distance2, const int Distance3, const int Distance4) {
		if (!first_Tunnel++) flag_Tunnel = 0;

		if ((Distance1 < 30) && (Distance2 < 30)) {
			if(flag_Tunnel < 2)
				flag_Tunnel++;
		}
		else {
			if (flag_Tunnel > 0)
				flag_Tunnel--;
		}

		if ((Distance3 < 30) && (Distance4 < 30)) {
			if ((flag_Tunnel >= 2) && (flag_Tunnel < 4))
				flag_Tunnel++;
		}
		else {
			if (flag_Tunnel >= 2)
				flag_Tunnel--;
		}

		if (flag_Tunnel == 4) {
			return 1;
		}
		return 0;
	}

int Tunnel_isStart(const int Distance2, const int Distance6, const int Distance3, const int Distance5)
{
	if (Distance2 <= 30 && Distance3 <= 30 && Distance5 <= 30 && Distance6 <= 30)
		return 1;
	else
		return 0;
}

int Tunnel_SteerVal(const int Distance1, const int Distance2)
	{
		// ï¿½ï¿½ï¿½ï¿½ 19 , ï¿½ï¿½ï¿½ï¿½ 40
		// ï¿½ß¾ï¿½ï¿½ï¿½ 10, 10ï¿½ï¿½ ï¿½ï¿½ï¿½Í¾ï¿½ï¿½ï¿½
		absDist = abs(Distance1 - Distance2);
		int i;
		if (absDist < 2)
		{
			i = 0;
			if (++flag_steer[0] == 2)
			{
				steerVal = 0;
			}
		}
		else if (absDist < 4)
		{
			i = 1;
			if (++flag_steer[1] == 2)
			{
				steerVal = 80;
			}
		}
		else if (absDist < 6)
		{
			i = 2;
			if (++flag_steer[2] == 2)
			{
				steerVal = 180;
			}
		}
		else if (absDist < 8)
		{
			i = 3;
			if (++flag_steer[3] == 2)
			{
				steerVal = 300;
			}
		}
		else
		{
			i = 4;
			if (++flag_steer[4] == 2)
			{
				steerVal = 420;
			}
		}

		if (flag_steer[i] == 2)
		{
			flag_steer[i] = 0;
			if (Distance1 < Distance2)
				steerVal = -steerVal;
		}
		for (int j = 0; j < 5; j++)
		{
			if ((flag_steer[j] > 0) && (j != i))
				flag_steer[j]--;
		}
		printf("steer : %d, [%d, %d]\n", steerVal, Distance2, Distance1);
		return 1500 - steerVal;
	}


*/
