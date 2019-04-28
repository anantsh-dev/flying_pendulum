// UARTTest.cpp : 

#include "stdafx.h"
#include "Com.h"
#include "windows.h"
#include "time.h"
#include "stdio.h"
#include "iostream"
#include "fstream"
#include "string"
#include "ctime"
#include "chrono"
using namespace std;

unsigned char ucComNo[2] ={0,0};
signed char OpenCom(void)
{	static unsigned long ulNo=0;
	signed char cResult= 0;	
	printf("waiting for Com%d! ",ucComNo[0]);
	do
	{
		cResult = OpenCOMDevice(ucComNo[0],115200);
	}while(cResult!=0);
	printf("Com%d has been plugged in\r\n",ucComNo[0]);
	
	return 0;
}
double a[3],w[3],Angle[3],T;
void DecodeIMUData(unsigned char chrTemp[], ofstream& myfile)
{
	auto end = std::chrono::system_clock::now();
	std::time_t end_time = std::chrono::system_clock::to_time_t(end);
	time_t my_time = time(NULL);
	switch(chrTemp[1])
	{
	case 0x51:
		a[0] = (short(chrTemp[3]<<8|chrTemp[2]))/32768.0*16;
		a[1] = (short(chrTemp[5]<<8|chrTemp[4]))/32768.0*16;
		a[2] = (short(chrTemp[7]<<8|chrTemp[6]))/32768.0*16;
		T = (short(chrTemp[9]<<8|chrTemp[8]))/340.0+36.25;
		printf("a = %4.3f\t%4.3f\t%4.3f\t\r\n",a[0],a[1],a[2]);
		break;
	case 0x52:
		w[0] = (short(chrTemp[3]<<8|chrTemp[2]))/32768.0*2000;
		w[1] = (short(chrTemp[5]<<8|chrTemp[4]))/32768.0*2000;
		w[2] = (short(chrTemp[7]<<8|chrTemp[6]))/32768.0*2000;
		T = (short(chrTemp[9]<<8|chrTemp[8]))/340.0+36.25;
//		printf("w = %4.3f\t%4.3f\t%4.3f\t\r\n",w[0],w[1],w[2]);
		break;
	case 0x53:
		Angle[0] = (short(chrTemp[3]<<8|chrTemp[2]))/32768.0*180;
		Angle[1] = (short(chrTemp[5]<<8|chrTemp[4]))/32768.0*180;
		Angle[2] = (short(chrTemp[7]<<8|chrTemp[6]))/32768.0*180;
		T = (short(chrTemp[9]<<8|chrTemp[8]))/340.0+36.25;
		printf("Angle = %4.2f\t%4.2f\t%4.2f\t %s\r\n", Angle[0], Angle[1], Angle[2], ctime(&my_time));
		string b= "angle "+std::to_string(Angle[0])+" "+ std::to_string(Angle[1])+" "+ std::to_string( Angle[2])+" "+ std::ctime(&end_time) +"\n";
		printf("%s\n",b);
		myfile << b;
		myfile.flush();
		break;
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	char chrBuffer[1000];
	unsigned char chrTemp[1000];
	signed char cResult[2] = {0};
	unsigned short usLength=0,usRxLength=0;
	FILE *fp;
	fp = fopen("Com.ini","r");
	if (1!=fscanf(fp,"Com = %d",&ucComNo[0]))
	{
		printf("Port config file wrong");
		Sleep(5000);
	}
	fclose(fp);
	OpenCom();
	printf("hello");
	chrBuffer[0] = 0x01;
	SendUARTMessageLength(ucComNo[0],chrBuffer,1);
	printf("hi");
	ofstream myfile;
	myfile.open("angles.txt");
	ofstream& a = myfile;

	while(1)
	{
		usLength = CollectUARTData(ucComNo[0],chrBuffer);
		if (usLength>0)
		{
			usRxLength += usLength;
                while (usRxLength >= 11)
                {
                    memcpy(chrTemp,chrBuffer,usRxLength);
                    if (!((chrTemp[0] == 0x55) & ((chrTemp[1] == 0x51) | (chrTemp[1] == 0x52) | (chrTemp[1] == 0x53))))
                    {
                        for (int i = 1; i < usRxLength; i++) chrBuffer[i - 1] = chrBuffer[i];
                        usRxLength--;
                        continue;
                    }
					DecodeIMUData(chrTemp,a);
                    for (int i = 11; i < usRxLength; i++) chrBuffer[i - 11] = chrBuffer[i];
                    usRxLength -= 11;
                }
		}
		
		Sleep(200);
	}
	myfile.close();
		return 0;
}

