#include "Gettime.h"

int Gettime()
{
	LARGE_INTEGER limtp, freq;
	//while (1)
	{
		QueryPerformanceFrequency(&freq);//获得当前的计数频率,即每秒进行多少次计数
		//cout << freq.QuadPart << endl;
		QueryPerformanceCounter(&limtp);//获取当前计数次数
		//cout << limtp.QuadPart<<endl;
		//cout << (limtp.QuadPart/1000) / (freq.QuadPart/1000) << "   " << limtp.QuadPart * 1000 / freq.QuadPart % 1000 << "   " << limtp.QuadPart * 10000000 / freq.QuadPart % 1000 << endl;
		//Sleep(1);
		//cout << limtp.QuadPart / freq.QuadPart + limtp.QuadPart * 1000 / freq.QuadPart % 1000 << endl;
		return limtp.QuadPart / freq.QuadPart * 1000 + limtp.QuadPart * 1000 / freq.QuadPart % 1000;
	}
}

double THGettime()
{
	static LARGE_INTEGER limtp, freq;
	static bool Once_TH = true;
	if (Once_TH)
	{
		QueryPerformanceFrequency(&freq);//获得当前的计数频率,即每秒进行多少次计数
		Once_TH = false;
	}
	QueryPerformanceCounter(&limtp);//获取当前计数次数
	return ((double)limtp.QuadPart / freq.QuadPart * 1000);
}


double GETTIME()
{
	static LARGE_INTEGER limtp, freq;
	static bool Once_TH = true;
	//第一次进入，获取外部频率
	if (Once_TH)
	{
		QueryPerformanceFrequency(&freq);
		Once_TH = false;
	}
	QueryPerformanceCounter(&limtp);
	//输出单位 ms
	return ((double)(limtp.QuadPart * 1000) / freq.QuadPart);
}