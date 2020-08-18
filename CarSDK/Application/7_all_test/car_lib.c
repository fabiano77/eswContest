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
#include "car_lib.h"

 /*******************************************************************************
  *  Defines
  *******************************************************************************
  */
  //#define BAUDRATE B115200
  //#define MODEDEVICE "/dev/ttyS0" // "/dev/ttyS0" is used for debug terminal

#define BAUDRATE        B19200
#define SERIAL_DEVICE   "/dev/ttyS2"  // ttyHS0, ttyHS1, ttyHS3 are available
#define I2C_DEVICE      "/dev/i2c-2"

static int uart_fd;
static int i2c_fd;
static int dist_table[6][28] = {
	{4095, 4063, 3400, 2940, 2516, 2211, 1897, 1712,1488,1331,1233,1133,1038,974,920,879,783,750,688,640,580,551,519,486,455,422,390,357},
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

/*******************************************************************************
 *  Functions
 *******************************************************************************
 */
	void CarControlInit(void)
{
	char fd_serial[20];
	struct termios oldtio, newtio;

	char* dev = I2C_DEVICE;
	int addr = 0x4b;
	int r;

	// UART configuration
	strcpy(fd_serial, SERIAL_DEVICE); //FFUART

	uart_fd = open(fd_serial, O_RDWR | O_NOCTTY);
	if (uart_fd < 0) {
		printf("Serial %s  Device Err\n", fd_serial);
		exit(1);
	}
	printf("CarControlInit(void) Uart Device : %s\n", SERIAL_DEVICE);

	tcgetattr(uart_fd, &oldtio); /* save current port settings */
	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD; // | CRTSCTS;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME] = 0;   /* inter-character timer unused */
	newtio.c_cc[VMIN] = 1;   /* blocking read until 8 chars received */

	tcflush(uart_fd, TCIFLUSH);
	tcsetattr(uart_fd, TCSANOW, &newtio);

	// I2C configuration
	i2c_fd = open(dev, O_RDWR);
	if (i2c_fd < 0)
	{
		perror("Opening i2c device node\n");
		exit(1);
	}

	r = ioctl(i2c_fd, I2C_SLAVE, addr);
	if (r < 0)
	{
		perror("Selecting i2c device\n");
	}
}

void CarLight_Write(char status)
{
	unsigned char buf[8];

	buf[0] = 0xa0;
	if (status == ALL_ON)
	{
		buf[1] = 0x03; //code length (1) + 2
		buf[2] = 0x01; //write(1)
		buf[3] = ALL_ON;    // all on = 3
		buf[4] = 0xa7; //checksum = a0 + 03 + 01 + 03

		printf("Front and Rear Light ON\n");
	}
	else if (status == FRONT_ON)
	{
		buf[1] = 0x03; //code length (1) + 2
		buf[2] = 0x01; //write(1)
		buf[3] = FRONT_ON;    // front on = 1
		buf[4] = 0xa5; //checksum = a0 + 03 + 01 + 01

		printf("Front Light ON\n");
	}
	else if (status == REAR_ON)
	{
		buf[1] = 0x03; //code length (1) + 2
		buf[2] = 0x01; //write(1)
		buf[3] = REAR_ON;    // rear on = 2
		buf[4] = 0xa6; //checksum = a0 + 03 + 01 + 02

		printf("Rear Light ON\n");
	}
	else
	{
		buf[1] = 0x03; //code length (1) + 2
		buf[2] = 0x01; //write(1)
		buf[3] = ALL_OFF;    // all off = 0
		buf[4] = 0xa4; //checksum = a0 + 03 + 01 + 00

		printf("Front and Rear Light OFF\n");
	}
	write(uart_fd, &buf[0], 5);
}

void Alarm_Write(char status)
{
	unsigned char buf[8];

	buf[0] = 0xa2;
	if (status == ON)
	{
		buf[1] = 0x03; //code length (1) + 2
		buf[2] = 0x01; //write(1)
		buf[3] = 0x64; //0.01s (1sec = 100 = 0x64)
		buf[4] = 0x0a; //checksum = a2 + 03 + 01 + 64 = 10a

		printf("Alarm ON\n");
	}
	else
	{
		buf[1] = 0x03; //code length (1) + 2
		buf[2] = 0x01; //write(1)
		buf[3] = 0x00; //0.00s
		buf[4] = 0xa6; //checksum = a2 + 03 + 01 + 00

		printf("Alarm OFF\n");
	}
	write(uart_fd, &buf[0], 5);
}

void Winker_Write(char status)
{
	unsigned char buf[8];

	buf[0] = 0xa1;
	if (status == ALL_ON)
	{
		buf[1] = 0x03; //code length (1) + 2
		buf[2] = 0x01; //write(1)
		buf[3] = ALL_ON;    // all on = 3
		buf[4] = 0xa8; //checksum = a1 + 03 + 01 + 03

		printf("Left and Right Winker ON\n");
	}
	else if (status == RIGHT_ON)
	{
		buf[1] = 0x03; //code length (1) + 2
		buf[2] = 0x01; //write(1)
		buf[3] = RIGHT_ON;    // right on = 1
		buf[4] = 0xa6; //checksum = a1 + 03 + 01 + 01

		printf("Right Winker ON\n");
	}
	else if (status == LEFT_ON)
	{
		buf[1] = 0x03; //code length (1) + 2
		buf[2] = 0x01; //write(1)
		buf[3] = LEFT_ON;    // left on = 2
		buf[4] = 0xa7; //checksum = a1 + 03 + 01 + 02

		printf("Left Winker ON\n");
	}
	else
	{
		buf[1] = 0x03; //code length (1) + 2
		buf[2] = 0x01; //write(1)
		buf[3] = ALL_OFF;    // all off = 0
		buf[4] = 0xa5; //checksum = a1 + 03 + 01 + 00

		printf("Left and Right Winker OFF\n");
	}
	write(uart_fd, &buf[0], 5);
}

char SpeedControlOnOff_Read(void)
{
	unsigned char buf[8];
	unsigned char read_buf[8];

	buf[0] = 0x90;
	buf[1] = 0x02; //code length (0) + 2
	buf[2] = 0x02; //read(2)
	buf[3] = 0x94; //checksum = 90 + 02 + 02

	//  printf("Read Speed Control OnOff\n");
	write(uart_fd, &buf[0], 4);
	read(uart_fd, &read_buf[0], 4);

	return read_buf[2];
}

void SpeedControlOnOff_Write(char status)
{
	unsigned char buf[8];

	buf[0] = 0x90;
	buf[1] = 0x03; //code length (1) + 2
	buf[2] = 0x01; //write(1)
	buf[3] = status; //UNCONTROL=0 CONTROL=1
	buf[4] = 0x94 + buf[3]; //checksum = 90 + 03 + 01 + buf[3]

	//printf("SpeedControlOnOff_Write(void) = %d\n", buf[3]);
	write(uart_fd, &buf[0], 5);
}

signed short DesireSpeed_Read(void)
{
	unsigned char buf[8];
	unsigned char read_buf[8];

	buf[0] = 0x91;
	buf[1] = 0x02; //code length (0) + 2
	buf[2] = 0x02; //read(2)
	buf[3] = 0x95; //checksum = 91 + 02 + 02

//  printf("Read Desire Speed\n");
	write(uart_fd, &buf[0], 4);
	read(uart_fd, &read_buf[0], 5);

	return ((signed short)(read_buf[3] << 8) + (signed short)(read_buf[2]));
}

void DesireSpeed_Write(signed short speed)
{
	unsigned char buf[8];

	buf[0] = 0x91;
	buf[1] = 0x04; //code length (2) + 2
	buf[2] = 0x01; //write(1)
	buf[3] = speed & 0x00ff; //bottom byte
	buf[4] = (speed >> 8) & 0x00ff; //top byte
	buf[5] = 0x96 + buf[3] + buf[4]; //checksum

	//printf("DesireSpeed_Write(void) = %d\n", speed);
	write(uart_fd, &buf[0], 6);
}

unsigned char SpeedPIDProportional_Read(void)
{
	unsigned char buf[8];
	unsigned char read_buf[8];

	buf[0] = 0x92;
	buf[1] = 0x02; //code length (0) + 2
	buf[2] = 0x02; //read(2)
	buf[3] = 0x96; //checksum = 92 + 02 + 02

//  printf("Read Speed PID Proportional\n");
	write(uart_fd, &buf[0], 4);
	read(uart_fd, &read_buf[0], 4);

	return read_buf[2];
}

void SpeedPIDProportional_Write(unsigned char gain)
{
	unsigned char buf[8];

	buf[0] = 0x92;
	buf[1] = 0x03; //code length (1) + 2
	buf[2] = 0x01; //write(1)
	buf[3] = gain;
	buf[4] = 0x96 + buf[3]; //checksum = 90 + 03 + 01 + buf[3]

	printf("SpeedPIDProportional_WriteH = %d\n", buf[3]);
	write(uart_fd, &buf[0], 5);
}

unsigned char SpeedPIDIntegral_Read(void)
{
	unsigned char buf[8];
	unsigned char read_buf[8];

	buf[0] = 0x93;
	buf[1] = 0x02; //code length (0) + 2
	buf[2] = 0x02; //read(2)
	buf[3] = 0x97; //checksum = 93 + 02 + 02

//  printf("Read Speed PID Integral\n");
	write(uart_fd, &buf[0], 4);
	read(uart_fd, &read_buf[0], 4);

	return read_buf[2];
}

void SpeedPIDIntegral_Write(unsigned char gain)
{
	unsigned char buf[8];

	buf[0] = 0x93;
	buf[1] = 0x03; //code length (1) + 2
	buf[2] = 0x01; //write(1)
	buf[3] = gain;
	buf[4] = 0x97 + buf[3]; //checksum = 93 + 03 + 01 + buf[3]

	printf("SpeedPIDIntegral_Write(void) = %d\n", buf[3]);
	write(uart_fd, &buf[0], 5);
}

unsigned char SpeedPIDDifferential_Read(void)
{
	unsigned char buf[8];
	unsigned char read_buf[8];

	buf[0] = 0x94;
	buf[1] = 0x02; //code length (0) + 2
	buf[2] = 0x02; //read(2)
	buf[3] = 0x98; //checksum = 94 + 02 + 02

//  printf("Read Speed PID Differential\n");
	write(uart_fd, &buf[0], 4);
	read(uart_fd, &read_buf[0], 4);

	return read_buf[2];
}

void SpeedPIDDifferential_Write(unsigned char gain)
{
	unsigned char buf[8];

	buf[0] = 0x94;
	buf[1] = 0x03; //code length (1) + 2
	buf[2] = 0x01; //write(1)
	buf[3] = gain;
	buf[4] = 0x98 + buf[3]; //checksum = 94 + 03 + 01 + buf[3]

	printf("SpeedPIDDifferential_Write(void) = %d\n", buf[3]);
	write(uart_fd, &buf[0], 5);
}

char PositionControlOnOff_Read(void)
{
	unsigned char buf[8];
	unsigned char read_buf[8];

	buf[0] = 0x96;
	buf[1] = 0x02; //code length (0) + 2
	buf[2] = 0x02; //read(2)
	buf[3] = 0x9a; //checksum = 96 + 02 + 02

//  printf("Read Position Control OnOff\n");
	write(uart_fd, &buf[0], 4);
	read(uart_fd, &read_buf[0], 4);

	return read_buf[2];

}

void PositionControlOnOff_Write(char status)
{
	unsigned char buf[8];

	buf[0] = 0x96;
	buf[1] = 0x03; //code length (1) + 2
	buf[2] = 0x01; //write(1)
	buf[3] = status; //UNCONTROL=0 CONTROL=1
	buf[4] = 0x9a + buf[3]; //checksum = 9a + 03 + 01 + buf[3]

	printf("PositionControlOnOff_Write(void) = %d\n", buf[3]);
	write(uart_fd, &buf[0], 5);
}

unsigned char PositionProportionPoint_Read(void)
{
	unsigned char buf[8];
	unsigned char read_buf[8];

	buf[0] = 0x98;
	buf[1] = 0x02; //code length (0) + 2
	buf[2] = 0x02; //read(2)
	buf[3] = 0x9c; //checksum = 98 + 02 + 02

//  printf("Read Position Proportion Point\n");
	write(uart_fd, &buf[0], 4);
	read(uart_fd, &read_buf[0], 4);

	return read_buf[2];
}

void PositionProportionPoint_Write(unsigned char gain)
{
	unsigned char buf[8];

	buf[0] = 0x98;
	buf[1] = 0x03; //code length (1) + 2
	buf[2] = 0x01; //write(1)
	buf[3] = gain;
	buf[4] = 0x9c + buf[3]; //checksum = 98 + 03 + 01 + buf[3]

	printf("PositionProportionPoint_Write(void) = %d\n", buf[3]);
	write(uart_fd, &buf[0], 5);
}

signed int DesireEncoderCount_Read(void)
{
	unsigned char buf[8];
	unsigned char read_buf[8];

	buf[0] = 0x97;
	buf[1] = 0x02; //code length (0) + 2
	buf[2] = 0x02; //read(2)
	buf[3] = 0x9b; //checksum = 97 + 02 + 02

//  printf("Read Desire position\n");
	write(uart_fd, &buf[0], 4);
	read(uart_fd, &read_buf[0], 7);

	return ((signed int)(read_buf[5] << 24) + (signed int)(read_buf[4] << 16) + (signed int)(read_buf[3] << 8) + (signed int)(read_buf[2]));
}

void DesireEncoderCount_Write(signed int position)
{
	unsigned char buf[8];

	buf[0] = 0x97;
	buf[1] = 0x06; //code length (6) + 2
	buf[2] = 0x01; //write(1)
	buf[3] = position & 0x000000ff; //bottom byte
	buf[4] = (position >> 8) & 0x000000ff; //3rd byte
	buf[5] = (position >> 16) & 0x000000ff; //2nd byte
	buf[6] = (position >> 24) & 0x000000ff; //top byte
	buf[7] = 0x9e + buf[3] + buf[4] + buf[5] + buf[6]; //checksum

	//printf("DesireEncoderCount_Write(void) = %d\n", position);
	write(uart_fd, &buf[0], 8);
}

signed int EncoderCounter_Read(void)
{
	unsigned char buf[8];
	unsigned char read_buf[8];

	buf[0] = 0xb0;
	buf[1] = 0x02; //code length (0) + 2
	buf[2] = 0x02; //read(2)
	buf[3] = 0xb4; //checksum = b0 + 02 + 02

//  printf("Read Encoder Counter\n");
	write(uart_fd, &buf[0], 4);
	read(uart_fd, &read_buf[0], 7);

	if (read_buf[6] != ((read_buf[0] + read_buf[1] + read_buf[2] + read_buf[3] + read_buf[4] + read_buf[5]) % 256))
	{
		return CHECKSUMERROR;
	}
	else
	{
		return ((signed int)(read_buf[5] << 24) + (signed int)(read_buf[4] << 16) + (signed int)(read_buf[3] << 8) + (signed int)(read_buf[2]));
	}
}

void EncoderCounter_Write(signed int position)
{
	unsigned char buf[8];

	buf[0] = 0xb0;
	buf[1] = 0x06; //code length (6) + 2
	buf[2] = 0x01; //write(1)
	buf[3] = position & 0x000000ff; //bottom byte
	buf[4] = (position >> 8) & 0x000000ff; //3rd byte
	buf[5] = (position >> 16) & 0x000000ff; //2nd byte
	buf[6] = (position >> 24) & 0x000000ff; //top byte
	buf[7] = 0xb7 + buf[3] + buf[4] + buf[5] + buf[6]; //checksum

	//printf("EncoderCounter_Write(void) = %d\n", position);
	write(uart_fd, &buf[0], 8);
}

signed short SteeringServoControl_Read(void)
{
	unsigned char buf[8];
	unsigned char read_buf[8];

	buf[0] = 0xa3;
	buf[1] = 0x02; //code length (0) + 2
	buf[2] = 0x02; //read(2)
	buf[3] = 0xa7; //checksum = a3 + 02 + 02

//  printf("Read Steering Servo Angle\n");
	write(uart_fd, &buf[0], 4);
	read(uart_fd, &read_buf[0], 5);

	return ((signed short)(read_buf[3] << 8) + (signed short)(read_buf[2]));
}

void SteeringServoControl_Write(signed short angle)
{
	unsigned char buf[8];

	buf[0] = 0xa3;
	buf[1] = 0x04; //code length (2) + 2
	buf[2] = 0x01; //write(1)
	buf[3] = angle & 0x00ff; //bottom byte
	buf[4] = (angle >> 8) & 0x00ff; //top byte
	buf[5] = 0xa8 + buf[3] + buf[4]; //checksum

	//printf("SteeringServoControl_Write(void) = %d\n", angle);
	write(uart_fd, &buf[0], 6);
}

signed short CameraXServoControl_Read(void)
{
	unsigned char buf[8];
	unsigned char read_buf[8];

	buf[0] = 0xa5;
	buf[1] = 0x02; //code length (0) + 2
	buf[2] = 0x02; //read(2)
	buf[3] = 0xa9; //checksum = a5 + 02 + 02

//  printf("Read Camera X Servo Angle\n");
	write(uart_fd, &buf[0], 4);
	read(uart_fd, &read_buf[0], 5);

	return ((signed short)(read_buf[3] << 8) + (signed short)(read_buf[2]));
}

void CameraXServoControl_Write(signed short angle)
{
	unsigned char buf[8];

	buf[0] = 0xa5;
	buf[1] = 0x04; //code length (2) + 2
	buf[2] = 0x01; //write(1)
	buf[3] = angle & 0x00ff; //bottom byte
	buf[4] = (angle >> 8) & 0x00ff; //top byte
	buf[5] = 0xaa + buf[3] + buf[4]; //checksum

	//printf("CameraXServoControl_Write(void) = %d\n", angle);
	write(uart_fd, &buf[0], 6);
}

signed short CameraYServoControl_Read(void)
{
	unsigned char buf[8];
	unsigned char read_buf[8];

	buf[0] = 0xa7;
	buf[1] = 0x02; //code length (0) + 2
	buf[2] = 0x02; //read(2)
	buf[3] = 0xab; //checksum = a7 + 02 + 02

//  printf("Read Camera Y Angle\n");
	write(uart_fd, &buf[0], 4);
	read(uart_fd, &read_buf[0], 5);

	return ((signed short)(read_buf[3] << 8) + (signed short)(read_buf[2]));
}

void CameraYServoControl_Write(signed short angle)
{
	unsigned char buf[8];

	buf[0] = 0xa7;
	buf[1] = 0x04; //code length (2) + 2
	buf[2] = 0x01; //write(1)
	buf[3] = angle & 0x00ff; //bottom byte
	buf[4] = (angle >> 8) & 0x00ff; //top byte
	buf[5] = 0xac + buf[3] + buf[4]; //checksum

	//printf("CameraYServoControl_Write(void) = %d\n", angle);
	write(uart_fd, &buf[0], 6);
}

unsigned char LineSensor_Read(void)
{
	unsigned char buf[8];
	unsigned char read_buf[8];

	buf[0] = 0xb1;
	buf[1] = 0x02; //code length (0) + 2
	buf[2] = 0x02; //read(2)
	buf[3] = 0xb5; //checksum = b1 + 02 + 02

//  printf("Read Line Trace Sensor\n");
	write(uart_fd, &buf[0], 4);
	read(uart_fd, &read_buf[0], 4);

	return read_buf[2];
}

int DistanceSensor(int channel)
{
	unsigned char buf[8];
	unsigned char command;
	unsigned char value[2];
	useconds_t delay = 2000;
	int data;
	int r;

	switch (channel)
	{
	case 1: command = 0x8c; break;
	case 2: command = 0xcc; break;
	case 3: command = 0x9c; break;
	case 4: command = 0xdc; break;
	case 5: command = 0xac; break;
	case 6: command = 0xec; break;
	default: printf("channel error.\n"); break;
	}
	r = write(i2c_fd, &command, 1);
	usleep(delay);

	r = read(i2c_fd, value, 2);
	if (r != 2)
	{
		perror("reading i2c device\n");
	}
	usleep(delay);

	data = (int)((value[0] & 0b00001111) << 8) + value[1];

	return data;
}

int DistanceSensor_cm(int channel)
{
	return sensor_dist(channel, DistanceSensor(channel));
}

int sensor_dist(int channel, int input) {
	int position = 0;
	int i = 0;
	for (i = 0; i < 28; i++) {
		if (input > dist_table[channel-1][i]) {
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

void DesiredDistance(int SettingSpeed, int SettingDistance) {
	int init_encoder = 0;
	int on_encoder = 0;
	EncoderCounter_Write(init_encoder);
	DesireSpeed_Write(SettingSpeed);
	while (1) {
		on_encoder = abs(EncoderCounter_Read());
		if (on_encoder != 65278) printf("encoder : %-4d\n", on_encoder);
		if (on_encoder >= desire_encoder && on_encoder != 65278) {
			DesireSpeed_Write(0);
			break;
		}
		usleep(100000);
	}
}
