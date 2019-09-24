#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <unistd.h>
#include <pthread.h>

#include "dataproc.h"
#include "device_ft232hq.h"
#include "sdl_opt.h"
#include "cap_save.h"

extern unsigned int app_mode;
extern unsigned int cap_ir_falg;


pthread_t pid_device_ft232hq;
volatile bool bRun_dataproc = false;

SENSOR sensor;
SerialPortReceive receivedata;
USART_TR_DATA usart_rx_data;

//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½160*120ï¿½ï¿½ï¿?120*120ï¿½ï¿½ï¿?80*80ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?160*120
#define    MaxSizeW          160
#define    MaxSizeH           120
unsigned int DataBuffer[MaxSizeH][MaxSizeW];
unsigned char  Vediobuffer[MaxSizeH * MaxSizeW];

void ImageDisplay()
{
	unsigned int *data = (unsigned int *)DataBuffer;

	for (unsigned int sizey = 0; sizey < sensor.high; sizey++)
	{
		for (unsigned int sizex = 0; sizex <sensor.width; sizex++)
		{
			int locate = sizey * sensor.width + sizex;
			Vediobuffer[locate] = *(data + sizey*MaxSizeW + sizex) >> 6;    //14bit to 0-255
		}
	}

	//sdl_show_buf(Vediobuffer);
	Mat img_ir_gray(MaxSizeH, MaxSizeW, CV_8UC1, Vediobuffer);
	//threshold(img_ir_gray, img_ir_gray, 150, 255, CV_THRESH_BINARY);
	Mat img_ir_rgb;
	cvtColor(img_ir_gray, img_ir_rgb, CV_GRAY2BGR);
	Mat img_ir_rgb_times;
	resize(img_ir_rgb, img_ir_rgb_times, Size(),2,2);
	if(app_mode == 1)
		sdl_show_rgb(img_ir_rgb_times);

	if(cap_ir_falg == 1)
	{
		cap_ir_save(img_ir_rgb_times);
		cap_ir_falg = 0;
	}
}

//command analysis
void Packet_analysis()
{
	switch (usart_rx_data.command)
	{
		case 0x24:		//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
		{
					 //obtain Resolution of detector
					 if (usart_rx_data.length == 321)  // 160*2+1
					 {
						 sensor.high = 120;
						 sensor.width = 160;
					 }
					 else if (usart_rx_data.length == 241)  //120*2+1
					 {
						 sensor.high = 120;
						 sensor.width = 120;
					 }
					 else if (usart_rx_data.length == 161)   //80*2+1
					 {
						 sensor.high = 80;
						 sensor.width = 80;
					 }

					 unsigned int line;
					 line = usart_rx_data.data[0];  //first data is row number
					 unsigned int count = 0;
					 if (usart_rx_data.length > 0 && usart_rx_data.length < 1000)
					 {
						 for (unsigned int i = 1; i < usart_rx_data.length; i++)
						 {
							 //14bitsï¿½ï¿½ï¿?
							 if (i & 0x01)
							 {
								 DataBuffer[line][count] = usart_rx_data.data[i] << 8;
							 }
							 else
							 {
								 DataBuffer[line][count] |= usart_rx_data.data[i];
								 count++;
							 }
						 }
					 }

					 if (line == sensor.high - 1 && count == sensor.width)
					 {
						 ImageDisplay();
					 }
					 break;
		}
		default:
		{
				   break;
		}
	}
}

//Packet analysis
unsigned int length = 0;
int Statetransition = 0;
void DataProcess(void)
{
	for (unsigned int len = 0; len < receivedata.len; len++)
	{
		switch (Statetransition)
		{
			case STARTAA:
			{
				if (receivedata.data[len] == 0xAA)
				{
					usart_rx_data.frame_start = 0xAA00;
					length = 0;
					Statetransition = START55;
				}
				break;
			}
			case START55:
			{
					if (receivedata.data[len] == 0x55 && length == 1)
					{
						usart_rx_data.frame_start |= 0x55;
						Statetransition = STATUS;
					}
					else
						Statetransition = STARTAA;
					break;
			}
			case STATUS:
			{
						   if (receivedata.data[len] == 0x00 && length == 2)
						   {
							   usart_rx_data.status = 0x00;
							   Statetransition = COMMOND;
						   }
						   else
							   Statetransition = STARTAA;
						   break;
			}
			case COMMOND:
			{
							if (length == 3)
							{
								usart_rx_data.command = receivedata.data[len];
								Statetransition = LENGTH;
							}
							else
								Statetransition = STARTAA;
							break;
			}
			case LENGTH:
			{
						   if (length == 4)                                       //length
							   usart_rx_data.length = receivedata.data[len] << 8;
						   else if (length == 5)
						   {
							   usart_rx_data.length |= receivedata.data[len];
							   Statetransition = CRC1;
						   }
						   else
							   Statetransition = STARTAA;
						   break;
			}
			case CRC1:
			{
						 if (length == 6)                                       //CRC1
						 {
							 usart_rx_data.crc1 = receivedata.data[len] << 8;
						 }
						 else if (length == 7)
						 {
							 usart_rx_data.crc1 |= receivedata.data[len];
							 Statetransition = DATA;
						 }
						 else
							 Statetransition = STARTAA;
						 break;
			}
			case DATA:
			{
						 if (length < usart_rx_data.length + 8)
							 usart_rx_data.data[length - 8] = receivedata.data[len];
						 if (length == usart_rx_data.length + 7)
							 Statetransition = CRC2;
						 break;
			}
			case CRC2:
			{
						 if (length == usart_rx_data.length + 8)                     //CRC2
						 {
							 usart_rx_data.crc2 = receivedata.data[len] << 8;
						 }
						 else if (length == usart_rx_data.length + 9)
						 {
							 usart_rx_data.crc2 |= receivedata.data[len];
							 Statetransition = END0D;
						 }
						 break;
			}
			case END0D:
			{
						  if (length == usart_rx_data.length + 10 && receivedata.data[len] == 0x0D)
						  {
							  usart_rx_data.frame_end = 0x0D00;
							  Statetransition = END0A;
						  }
						  else
							  Statetransition = STARTAA;
						  break;
			}
			case END0A:
			{
						  if (length == usart_rx_data.length + 11 && receivedata.data[len] == 0x0A)  //0x0A
						  {
							  usart_rx_data.frame_end |= 0x0A;
							  Statetransition = STARTAA;
							  length = 0;
							  Packet_analysis();
						  }
						  else
						  {
							  length = 0;
							  Statetransition = STARTAA;
						  }
						  break;
			}
			default:
			{
					   Statetransition = STARTAA;
					   length = 0;
					   break;
			}
		}
		length++;
	}
}

void *thread_device_ft232hq(void *arg)
{
    while(bRun_dataproc)
    {
    	FT_STATUS ftStatus = device_ft232hq_read(receivedata.data, &(receivedata.len));
		if (ftStatus == FT_OK)
		{
			DataProcess();
		}
    }
    return NULL;
}

int dataproc_init()
{
	receivedata.len = 0;
	receivedata.data = (unsigned char *)malloc(65536 * sizeof(unsigned char));
	if (receivedata.data == NULL)
	{
		printf("receivedata.data malloc error!\n");
		return -1;
	}

	usart_rx_data.data = (unsigned char *)malloc(65536 * sizeof(unsigned char));;
	if (usart_rx_data.data == NULL)
	{
		printf("usart_rx_data.data malloc error!\n");
		return -1;
	}

	bRun_dataproc = true;
    int ret = pthread_create(&pid_device_ft232hq, NULL, &thread_device_ft232hq, NULL);
    if(ret)
    {
        printf("create pthread:pid_device_ft232hq error!\n");
        return -1;
    }
    return 0;
}

void dataproc_exit()
{
	bRun_dataproc = false;
	 pthread_join(pid_device_ft232hq, NULL);
	 printf("%s\n", __func__);
}

bool is_dataproc_on()
{
	return bRun_dataproc;
}
