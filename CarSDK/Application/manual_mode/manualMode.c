/*******************************************************************************
 *  INCLUDE FILES
 *******************************************************************************
 */
#include <stdio.h>
#include <sys/time.h>
#include "car_lib.h"

 /*******************************************************************************
  *  Functions
  *******************************************************************************
  */

void clrscr(void)
{
	write(1, "\033[1;1H\033[2J", 10);
}

void main(void)
{
	unsigned char status;
	short speed;
	unsigned char gain;
	int position, posInit, posDes, posRead;
	short angle_steering, angle_cameraX, angle_cameraY;
	int channel;
	int data;
	char sensor;
	int i, j;
	int tol;
	char byte = 0x80;

	CarControlInit();

	int angle = 1500;
	SteeringServoControl_Write(angle);
	CameraXServoControl_Write(angle);
	CameraYServoControl_Write(angle);

	angle_steering = SteeringServoControl_Read();
	printf("SteeringServoControl_Read() = %d\n", angle_steering);    //default = 1500, 0x5dc
	angle_cameraX = CameraXServoControl_Read();
	printf("CameraXServoControl_Read() = %d\n", angle_cameraX);    //default = 1500, 0x5dc
	angle_cameraY = CameraYServoControl_Read();
	printf("CameraYServoControl_Read() = %d\n", angle_cameraY);    //default = 1500, 0x5dc

	clrscr();
	printf("---------------------[control key]------------------	\n");
	printf("    w      : forward          |   i      : up			\n");
	printf("  a s d    : left, stop ,right| j   l    : left, right	\n");
	printf("    x      : backward         |   k      : down			\n");
	printf("  (move)                      | (camera)				\n");
	printf("----------------------------------------------------	\n");
	printf("	1 = speed up, 2 = speed down						\n");
	printf("----------------------------------------------------	\n");
	printf("	q, e = winker || z, c = front, rear lamp			\n");
	printf("----------------------------------------------------	\n");
	printf("	space bar = Alarm									\n");
	printf("----------------------------------------------------	\n");
	printf("(테스트)");
	//jobs to be done beforehand;
	PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!
	//jobs to be done beforehand;
	SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!!
	speed = 0; // speed set     --> speed must be set when using position controller
	DesireSpeed_Write(speed);

	unsigned short lightFlag = 0x00;
	while (0)
	{
		char key;
		printf("input :");
		scanf("%c", &key);
		getchar();

		switch (key)
		{
		case 'a':	//steering left		: servo 조향값 (2000(좌) ~ 1500(중) ~ 1000(우)
			angle_steering += 50;
			SteeringServoControl_Write(angle_steering);
			printf("angle_steering = %d\n", angle_steering);
			printf("SteeringServoControl_Read() = %d\n", SteeringServoControl_Read());    //default = 1500, 0x5dc
			break;

		case 'd':	//steering right	: servo 조향값 (2000(좌) ~ 1500(중) ~ 1000(우)
			angle_steering -= 50;
			SteeringServoControl_Write(angle_steering);
			printf("angle_steering = %d\n", angle_steering);
			printf("SteeringServoControl_Read() = %d\n", SteeringServoControl_Read());    //default = 1500, 0x5dc
			break;

		case 's':	//stop
			DesireSpeed_Write(0);
			printf("DC motor stop\n");
			break;

		case 'w':	//go forward
			DesireSpeed_Write(speed);
			usleep(100000);	//1 000 000 us
			printf("DesireSpeed_Read() = %d\n", DesireSpeed_Read());
			break;

		case 'x':	//go backward	speed 음수 인가하면 후진.
			DesireSpeed_Write(0 - speed);
			usleep(100000);	//1 000 000 us
			printf("DesireSpeed_Read() = %d\n", DesireSpeed_Read());
			break;

		case 'j':	//cam left
			angle_cameraX += 50;
			CameraXServoControl_Write(angle_cameraX);
			printf("angle_cameraX = %d\n", angle_cameraX);
			printf("CameraXServoControl_Read() = %d\n", CameraXServoControl_Read());    //default = 1500, 0x5dc
			break;

		case 'l':	//cam right
			angle_cameraX -= 50;
			CameraXServoControl_Write(angle_cameraX);
			printf("angle_cameraX = %d\n", angle_cameraX);
			printf("CameraXServoControl_Read() = %d\n", CameraXServoControl_Read());    //default = 1500, 0x5dc
			break;

		case 'i':	//cam up
			angle_cameraY -= 50;
			CameraYServoControl_Write(angle_cameraY);
			printf("angle_cameraY = %d\n", angle_cameraY);
			printf("CameraYServoControl_Read() = %d\n", CameraYServoControl_Read());    //default = 1500, 0x5dc
			break;

		case 'k':	//cam down
			angle_cameraY += 50;
			CameraYServoControl_Write(angle_cameraY);
			printf("angle_cameraY = %d\n", angle_cameraY);
			printf("CameraYServoControl_Read() = %d\n", CameraYServoControl_Read());    //default = 1500, 0x5dc
			break;

		case '1':	//speed up		최대 스피드 500
			speed += 20;
			printf("speed = %d\n", speed);
			break;

		case '2':	//speed down
			speed -= 20;
			printf("speed = %d\n", speed);
			break;

		case 'q':	//Flashing left winker 3 times 
			for (i = 0; i < 3; i++)
			{
				Winker_Write(LEFT_ON);
				usleep(600000);		// 300 000 us
				Winker_Write(ALL_OFF);
				usleep(600000);
			}
			break;

		case 'e':	//Flashing right winker 3 times 
			for (i = 0; i < 3; i++)
			{
				Winker_Write(RIGHT_ON);
				usleep(600000);		// 300 000 us
				Winker_Write(ALL_OFF);
				usleep(600000);
			}
			break;

		case 'z':	//front lamp on/off
			lightFlag = lightFlag ^ 0x01;	// 00000000 ^ 00000001 (XOR)���� : 0����Ʈ�� XOR�����Ѵ�.
			CarLight_Write(lightFlag);
			break;

		case 'c':	//rear lamp on/off
			lightFlag = lightFlag ^ 0x02;	// 00000000 ^ 00000010 (XOR)���� : 1����Ʈ�� XOR�����Ѵ�.
			CarLight_Write(lightFlag);
			break;

		case ' ':	//alarm 
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


		struct timeval st, et;
		int currentSpeed;
		double err_P = 0;
		double err_I = 0;
		double err_D = 0;
		double err_B = 0;
		int goal;

	while (1)
	{

		SpeedPIDProportional_Write(20);
		SpeedPIDIntegral_Write(20);
		SpeedPIDProportional_Write(20);

		printf("goal : ");
		scanf("%d", &goal);
		DesireSpeed_Write(goal);
		usleep(100000);

		gettimeofday(&st, NULL);
		while (1)
		{
			currentSpeed = DesireSpeed_Read();
			if(currentSpeed<-500 || currentSpeed>500) continue;
			err_P = currentSpeed - goal;
			err_I += err_P;
			err_D = err_B - err_P;
			err_B = err_P;

			int writeVal = goal - (err_P*0.2 + err_I*0.15 + err_D*0.1);
			printf("currentSpeed = %d, writeVal = %d\n", currentSpeed, writeVal);
			printf("errP = %4.1f, errI = %4.1f, errD = %4.1f\n\n",err_P, err_I, err_D);
			DesireSpeed_Write(writeVal);
			gettimeofday(&et, NULL);
			int time_tmp = ((et.tv_sec - st.tv_sec) * 1000) + ((int)et.tv_usec / 1000 - (int)st.tv_usec / 1000);
			if (time_tmp > 4000) break; // 4s
			usleep(10000); //10ms
		}
	}



}



