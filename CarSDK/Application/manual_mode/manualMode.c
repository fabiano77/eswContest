/*******************************************************************************
 *  INCLUDE FILES
 *******************************************************************************
 */
#include <stdio.h>
#include "car_lib.h"

 /*******************************************************************************
  *  Functions
  *******************************************************************************
  */
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

	angle = 1500;
	SteeringServoControl_Write(angle);
	CameraXServoControl_Write(angle);
	CameraYServoControl_Write(angle);

	angle_steering = SteeringServoControl_Read();
	printf("SteeringServoControl_Read() = %d\n", angle_steering);    //default = 1500, 0x5dc
	angle_cameraX = CameraXServoControl_Read();
	printf("CameraXServoControl_Read() = %d\n", angle_cameraX);    //default = 1500, 0x5dc
	angle_cameraY = CameraYServoControl_Read();
	printf("CameraYServoControl_Read() = %d\n", angle_cameraY);    //default = 1500, 0x5dc

	printf("---------------------[control key]------------------	\n");
	printf("    w      : forward          |   i      : up			\n");
	printf("  a s d    : left, stop ,right| j   l    : left, right	\n");
	printf("    x      : backward         |   k      : down			\n");
	printf("  (move)                      | (camera)				\n");
	printf("----------------------------------------------------	\n");
	printf("	1 = speed up, 2 = speed down						\n");
	printf("----------------------------------------------------	\n");
	printf("	z, x = winker || c, v = front, rear lamp			\n");
	printf("----------------------------------------------------	\n");
	printf("	space bar = Alarm									\n");
	printf("----------------------------------------------------	\n");

	//jobs to be done beforehand;
	PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!
	//control on
	SpeedControlOnOff_Write(CONTROL);
	//jobs to be done beforehand;
	SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!!
	speed = 0; // speed set     --> speed must be set when using position controller
	DesireSpeed_Write(speed);

	char key;
	unsigned short lightFlag = 0x00;
	while (true)
	{
		printf("input :");
		scanf("%c", &key);

		switch (key)
		{
		case 'a':	//steering left
			angle_steering -= 20;
			SteeringServoControl_Write(angle_steering);
			printf("angle_steering = %d", angle_steering);
			printf("SteeringServoControl_Read() = %d\n", SteeringServoControl_Read());    //default = 1500, 0x5dc
			break;
		case 'd':	//steering right
			angle_steering += 20;
			SteeringServoControl_Write(angle_steering);
			printf("angle_steering = %d", angle_steering);
			printf("SteeringServoControl_Read() = %d\n", SteeringServoControl_Read());    //default = 1500, 0x5dc
			break;
		case 's':	//stop
			DesireSpeed_Write(0);
			break;
		case 'w':	//go forward
			DesireSpeed_Write(speed);
			printf("DesireSpeed_Read() = %d\n", DesireSpeed_Read());
			break;
		case 'x':	//go backward
			DesireSpeed_Write(0 - speed);
			printf("DesireSpeed_Read() = %d\n", DesireSpeed_Read());
			break;
		case 'j':	//cam left
			angle_cameraX -= 20;
			CameraXServoControl_Write(angle_cameraX);
			printf("angle_cameraX = %d", angle_cameraX);
			printf("CameraXServoControl_Read() = %d\n", CameraXServoControl_Read());    //default = 1500, 0x5dc
			break;
		case 'l':	//cam right
			angle_cameraX += 20;
			CameraXServoControl_Write(angle_cameraX);
			printf("angle_cameraX = %d", angle_cameraX);
			printf("CameraXServoControl_Read() = %d\n", CameraXServoControl_Read());    //default = 1500, 0x5dc
			break;
		case 'i':	//cam up
			angle_cameraY -= 20;
			CameraYServoControl_Write(angle_cameraY);
			printf("angle_cameraY = %d", angle_cameraY);
			printf("CameraYServoControl_Read() = %d\n", CameraYServoControl_Read());    //default = 1500, 0x5dc
			break;
		case 'k':	//cam down
			angle_cameraY += 20;
			CameraYServoControl_Write(angle_cameraY);
			printf("angle_cameraY = %d", angle_cameraY);
			printf("CameraYServoControl_Read() = %d\n", CameraYServoControl_Read());    //default = 1500, 0x5dc
			break;
		case '1':	//speed up
			speed += 10;
			printf("speed = %d\n", speed);
			break;
		case '2':	//speed down
			speed -= 10;
			printf("speed = %d\n", speed);
			break;
		case 'z':	//Flashing left winker 3 times 
			for (int i = 0; i < 3; i++)
			{
				Winker_Write(LEFT_ON);
				usleep(300000);		// 300 000 us
				Winker_Write(ALL_OFF);
				usleep(300000);
			}
			break;
		case 'x':	//Flashing right winker 3 times 
			for (int i = 0; i < 3; i++)
			{
				Winker_Write(RIGHT_ON);
				usleep(300000);		// 300 000 us
				Winker_Write(ALL_OFF);
				usleep(300000);
			}
			break;
		case 'c':	//front lamp on/off
			lightFlag = lightFlag ^ 0x01;	// 00000000 ^ 00000001 (XOR)연산 : 0번비트와 XOR연산한다.
			CarLight_Write(lightFlag);
			break;
		case 'v':	//rear lamp on/off
			lightFlag = lightFlag ^ 0x02;	// 00000000 ^ 00000010 (XOR)연산 : 1번비트와 XOR연산한다.
			CarLight_Write(lightFlag);
			break;
		case ' ':	//alarm 
			for (int i = 0; i < 2; i++)
			{
				Alarm_Write(ON);
				usleep(200000);
				Alarm_Write(OFF);
				usleep(200000);
			}
			break;
		default:
			cout << "wrong key input." << endl;
			break;
		}

	}

}



