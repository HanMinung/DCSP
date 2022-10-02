#pragma once
#define _CRT_SECURE_NO_WARNINGS
#define	_USE_MATH_DEFINES

#include "NIDAQmx.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <windows.h>
#include <time.h>
#include <conio.h>    /* _kbhit() */
#include <memory.h>

/*=================================================================================*/
/*									VARIABLES			                           */
/*=================================================================================*/

#define         UNIT_PI               (double)(         3.1415926535897     )
#define         FINAL_TIME            (double)(             30.0  	        )    
#define         SAMPLING_FREQ         (double)(             200.0           )    
#define         SAMPLING_TIME         (double)(   1 / SAMPLING_FREQ         )      
#define         N_STEP                (int)   ( FINAL_TIME / SAMPLING_TIME  )
#define         N_DATA                (int)   ( FINAL_TIME / SAMPLING_TIME  )    
#define         HIGH                  (int)   (				5				)
#define         LOW                   (int)   (				0				)
#define         N                     (int)   (             132             )    
#define			Unit_Conv_coef		  (double)(		  295.67/2.495		    )

/* =========================================================================================================== */


#define         ENTER_BREAK            if (_kbhit())   {if (_getch() == '\r') {break;}}
#define         EMERGENCY_STOP         if (_kbhit())   {if (_getch() == ' ') { AnalogWrite[0] = 0.0; AnalogWrite[1] = 2.5; \
																			   DAQmxCreateAOVoltageChan((taskA_OUT), ("Dev2/ao0:1"), (""), (0.0), (5.0), (DAQmx_Val_Volts), ("")); return 0;}}
#define         FOR_LOOP(i, N)         for (int i = 0; i < N; i ++)


/* =========================================================================================================== */


#define	  DAQ_ANALOG_INPUT_PORT(task)         (DAQmxCreateTask((""), (&task)));\
                                            (DAQmxCreateAIVoltageChan((task), ("Dev2/ai0:3"), (""), (DAQmx_Val_Diff), (0.0), (5.0), (DAQmx_Val_Volts), ("")));\
                                            (DAQmxStartTask(task))

#define   DAQ_ANALOG_OUTPUT_PORT(task)        (DAQmxCreateTask((""), (&task)));\
                                            (DAQmxCreateAOVoltageChan((task), ("Dev2/ao0:1"), (""), (0.0), (5.0), (DAQmx_Val_Volts), ("")));\
                                            (DAQmxStartTask(task))

#define   DAQ_ANALOG_STOPCLEAR(task)			(DAQmxStopTask(task));\
										    (DAQmxClearTask(task))

#define   WRITE_ANALOG(Task , data)			DAQmxWriteAnalogF64(Task ,1 ,1 , FINAL_TIME , DAQmx_Val_GroupByChannel ,  data , NULL , NULL )
#define   READ_ANALOG(Task , data)          DAQmxReadAnalogF64(Task , -1 , FINAL_TIME  , DAQmx_Val_GroupByChannel , data , sizeof(float64)*2  , NULL , NULL )


/* =========================================================================================================== */


FILE*		pFile;

double		Time ;
double      Time_prev ;
double      Time_curr ;
double      Time_idle ;
double      Time_check ;

//char		OutFileName[100];

int			idx;
int			count;
unsigned	i;

/* =========================================================================================================== */

double		DIR;
double		DOA;
double		Kp_Outer;
double		Epsylon;
double		Epsylon2DEG;

/* =========================================================================================================== */

double		Voltage_offset;
double		freq;
double		Standard;
double		Sinusoid;

/* =========================================================================================================== */

double      Ref_Voltage;
double		Linearized_Vcmd;

double      W_in ;
double      Unit_conversion_coef ;
double      W_Error ;
double      W_reference ;

float64     Voltage_Gyro ;
double      W_gyro ;
double		V_gyro_offset;
double		V_gyro_off_avg;

/* =========================================================================================================== */

double      Alpha_L ;
double      Alpha_R ;
double      Beta_L ;
double      Beta_R ;

double      a_L ;
double      a_R ;
double      b_L ;
double      b_R ;

/* =========================================================================================================== */

double     sum, sumX2, sumY, sumXY , a2 , b2  ;

/* =========================================================================================================== */

int			alpha;
double		Tau;
double		K_P, K_D, K_I;
double		Error_Curr;
double		P_Curr_OUT;
double		I_Curr_OUT;
double		D_Curr_OUT;

double		I_past;
double		D_past;

double		PID_W_OUT;
double		W_Error_past;

/* =========================================================================================================== */

double      W_gyro_LPF;
double		W_gyro_PID;

double		xk0, xk1, xk2;
double		b2, b1, b0;
double		a1, a0 ;
double		Vin, Vout ;

/* =========================================================================================================== */

char        OutFileName_1[50];
char        OutFileTag_1[20];

double      Buf_Time[N_DATA] ;
double      Buf_Error[N_DATA] ;
double		Buf_P_Out[N_DATA] ;
double		Buf_I_Out[N_DATA] ;
double		Buf_D_Out[N_DATA] ;
double		Buf_PID_Out[N_DATA] ;
double      Buf_Wgyro[N_DATA] ;

/*=================================================================================*/
/*                                 ABOUT DAQ			                           */
/*=================================================================================*/

/* Channel Creation */

TaskHandle   taskAI0 ;
TaskHandle   taskAI1 ;
TaskHandle   taskAI2 ;
TaskHandle   taskAI3 ;
TaskHandle   taskAO0 ;
TaskHandle   taskAO1 ;

extern	float64	 AnalogRead[4] = {0,0, };
extern	float64	 AnalogWrite[2] = { 0,0, };

TaskHandle   taskA_IN ;
TaskHandle   taskA_OUT ;

/*=================================================================================*/

void DAQ_channel_creation(void) {

	DAQmxCreateTask("", &taskAI0);
	DAQmxCreateTask("", &taskAI1);
	//DAQmxCreateTask("", &taskAI2);
	DAQmxCreateTask("", &taskAI3);
	DAQmxCreateTask("", &taskAO0);
	DAQmxCreateTask("", &taskAO1);

};


/* channel configuration */

void DAQ_channel_config(void) {

	DAQmxCreateAIVoltageChan(taskAI0, "Dev2/ai0", "", DAQmx_Val_Diff, 0, 10.0, DAQmx_Val_Volts, "");
	DAQmxCreateAIVoltageChan(taskAI1, "Dev2/ai1", "", DAQmx_Val_Diff, 0, 10.0, DAQmx_Val_Volts, "");
	//DAQmxCreateAIVoltageChan(taskAI2, "Dev2/ai2", "", DAQmx_Val_Diff, 0, 10.0, DAQmx_Val_Volts, "");
	DAQmxCreateAIVoltageChan(taskAI3, "Dev2/ai3", "", DAQmx_Val_Diff, 0, 10.0, DAQmx_Val_Volts, "");
	DAQmxCreateAOVoltageChan(taskAO0, "Dev2/ao0", "", 0.0, 5.0, DAQmx_Val_Volts, "");
	DAQmxCreateAOVoltageChan(taskAO1, "Dev2/ao1", "", 0.0, 5.0, DAQmx_Val_Volts, "");

	//  DAQmxWriteDigitalScalarU32(taskDO0, 0.0, 1.0, 0, NULL);
	//  DAQmxWriteDigitalScalarU32(taskDO1, 0.0, 1.0, 0, NULL);
};

/* Start the Channel */
void DAQ_channel_start(void) {

	DAQmxStartTask(taskAI0);
	DAQmxStartTask(taskAI1);
	//DAQmxStartTask(taskAI2);
	DAQmxStartTask(taskAI3);
	DAQmxStartTask(taskAO0);
	DAQmxStartTask(taskAO1);

};


void DAQ_stop_task(void) {
	DAQmxStopTask(taskAI0);
	DAQmxStopTask(taskAI1);
	//DAQmxStopTask(taskAI2);
	DAQmxStopTask(taskAI3);
	DAQmxStopTask(taskAO0);
	DAQmxStopTask(taskAO1);
}


void DAQ_clear_task(void) {
	DAQmxClearTask(taskAI0);
	DAQmxClearTask(taskAI1);
	//DAQmxClearTask(taskAI2);
	DAQmxClearTask(taskAI3);
	DAQmxClearTask(taskAO0);
	DAQmxClearTask(taskAO1);
}


void DAQ_read_set(void) {
	DAQmxReadAnalogScalarF64(taskAI0, 10.0, &Vin, NULL);
}


void DAQ_write_set(void) {

	DAQmxWriteAnalogScalarF64(taskAI0, 1.0, 5.0, 0.0, NULL);
	DAQmxWriteAnalogScalarF64(taskAI1, 1.0, 5.0, 0.0, NULL);
	DAQmxWriteAnalogScalarF64(taskAI2, 1.0, 5.0, 0.0, NULL);
	DAQmxWriteAnalogScalarF64(taskAI3, 1.0, 5.0, 0.0, NULL);
	// FLAG OFF & GET 2.5 Voltage
	DAQmxWriteAnalogScalarF64(taskAO0, 1.0, 5.0, 0, NULL);
	DAQmxWriteAnalogScalarF64(taskAO1, 1.0, 5.0, 2.5, NULL);

}

void Initialization(void) {

	DAQ_channel_creation();

	DAQ_channel_config();

	DAQ_channel_start();

	// MOTOR FLAG ON & Initialization
	DAQmxWriteAnalogScalarF64(taskAO0, 0.0, 5.0, 5.0, NULL);
	DAQmxWriteAnalogScalarF64(taskAO1, 0.0, 5.0, 2.5, NULL);

}

void Terminate_Task(void) {

	DAQmxWriteAnalogScalarF64(taskAO1, 0.0, 5.0, 2.5, NULL);
	DAQmxWriteAnalogScalarF64(taskAO0, 0.0, 5.0, 0, NULL);

	DAQ_write_set();
	DAQ_stop_task();
	DAQ_clear_task();

}

/* =========================================================================================================== */

double Set_sinusoidal(double Standard, double Freq) {

	Sinusoid = Standard * 1.0 * (sin(2 * UNIT_PI * Freq * Time)) ;
	
	return Sinusoid;
};

double Set_sinusoidal_offset(double Voltage_offset, double Standard, double Freq) {

	Sinusoid = Voltage_offset + Standard * 1.0 * (sin(2 * UNIT_PI * Freq * Time)) ;
	
	return Sinusoid;
};

/* =========================================================================================================== */

void Start_presskey(void) {

	printf("Press any key to start the program.... \n");

	getchar();

}

void Start_Next_Frequency(void) {

	printf("\n\nPress any key to start the NEXT FREQUENCY.... \n");

	getchar();

}

void GET_average_offset(void) {

	FOR_LOOP(i, 15) {

		READ_ANALOG(taskA_IN, &AnalogRead);

		Voltage_Gyro = AnalogRead[2];

		//EMERGENCY_STOP;

		printf("%lf \n", Voltage_Gyro);
		Sleep(1000);

		sum += Voltage_Gyro;

		V_gyro_off_avg = sum / 15.0;

	}

}

/* =========================================================================================================== */


/*=================================================================================*/
/*                                 ABOUT TIME			                           */
/*=================================================================================*/

void Time_Synchronization() {

	Time_prev = Time_curr;

	count ++;

	Time = SAMPLING_TIME * count;

}

double GetWindowTime(void)
{
	LARGE_INTEGER	liEndCounter, liFrequency;

	QueryPerformanceCounter(&liEndCounter);
	QueryPerformanceFrequency(&liFrequency);

	return(liEndCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);
};

/* ===================================================================================*/

void Idle_time(void) {

	while (1)
	{
		Time_curr = GetWindowTime();

		if (Time_curr - Time_prev >= (SAMPLING_TIME * 1000.0))	break;
	}

};


/*=================================================================================*/
/*                             FILTER IMPLEMENTATION			                   */
/*=================================================================================*/

double First_order_Filter(double a0, double b1, double b0, double Vin) {

	xk0 = Vin - a0 * xk1 ;
	Vout = b1 * xk0 + b0 * xk1;
	xk1 = xk0;
	
	return Vout;

}

double Second_order_Filter(double a1, double a0, double b2, double b1, double b0, double Vin) {

	xk0 = Vin - a1 * xk1 - a0 * xk2;
	Vout = b2 * xk0 + b1 * xk1 + b0 * xk2;
	xk2 = xk1;
	xk1 = xk0;

	return Vout;

};

/* ===================================================================================*/

double P_Controller(double Error_curr,double K_P) {

	P_Curr_OUT = Error_curr * K_P;
	
	return P_Curr_OUT;

}

double I_Controller(double _error_past, double _error_curr, double I_past, double K_I) {

	I_Curr_OUT = (K_I * SAMPLING_TIME * (_error_curr + _error_past ) / 2) + I_past;

	I_past = I_Curr_OUT;																// UPDATE CURR_output --> Past_output

	return I_Curr_OUT;

}

double D_Controller(double _error_past, double _error_curr, double D_past, double K_D) {

	D_Curr_OUT = (2 * K_D * (_error_curr - _error_past) / (2*Tau + SAMPLING_TIME) + ( 2*Tau - SAMPLING_TIME) * D_past / (2*Tau + SAMPLING_TIME) );

	D_past = D_Curr_OUT; 

	return D_Curr_OUT;
}

/*=================================================================================*/

void Linearization() {

	Ref_Voltage = W_in * (1 / Unit_conversion_coef) + 2.5;

	if (0 <= Ref_Voltage && Ref_Voltage <= 2.50) {

		Linearized_Vcmd = (Alpha_L * Ref_Voltage + Beta_L - b_L) / a_L;

	}

	if (2.50 < Ref_Voltage && Ref_Voltage <= 5.00) {

		Linearized_Vcmd = (Alpha_R * Ref_Voltage + Beta_R - b_R) / a_R;

	}
}

/*=================================================================================*/

void Import_data(void) {

	//DAQmxReadAnalogScalarF64(taskAI2, -1, &Voltage_Gyro, NULL);
	READ_ANALOG(taskA_IN, &AnalogRead);

	Voltage_Gyro = AnalogRead[2];

	DIR = AnalogRead[0];
	DOA = AnalogRead[1];
	Epsylon2DEG = (DOA / 2.5) * 20.0;


	if (DOA > 0.02) {

		if (DIR < 0.02) {

			W_reference = Kp_Outer * Epsylon2DEG;

			printf("Epsylon = 오른쪽으로 ");

		}
		else {

			W_reference = -Kp_Outer * Epsylon2DEG;

			printf("Epsylon = 왼쪽으로 ");

		}

	}
	else     W_reference = 0.0;

	printf("%lf   DOA = %lf   DIR = %lf   W_Ref = %lf\n\n", Epsylon2DEG, AnalogRead[1], AnalogRead[0], W_reference);

	//printf("Gyro_Voltage Value : %lf\n", Voltage_Gyro);

	W_gyro = (Voltage_Gyro - V_gyro_off_avg) * (1000 / 0.67);

	W_gyro_LPF = Second_order_Filter(-0.5094, 0.0, 0.2453, 0.2453, 0.0, W_gyro);                    // LPF implementation

	W_Error = W_reference - W_gyro_LPF;

	/*====================================================================================================*/

	P_Controller(W_Error, 1.106);                                               // IDEAL VALUE : 1.236
																				// P gain을 조금 0.2 정도 키움 : overshoot 감소 rising time은 거의 비슷함
	I_Controller(W_Error_past, W_Error, I_Curr_OUT, 43.73);

	D_Controller(W_Error_past, W_Error, D_Curr_OUT, 0.02071);

	W_in = P_Curr_OUT + I_Curr_OUT + D_Curr_OUT;

	W_Error_past = W_Error;

}

/*=================================================================================*/

void Export_data(void) {

	Linearization();

	//DAQmxWriteAnalogScalarF64(taskAO1, 0.0, 5.0, Linearized_Vcmd, NULL);
	AnalogWrite[1] = Linearized_Vcmd;
	
	WRITE_ANALOG(taskA_OUT, AnalogWrite);

}


// D - controller

/*=================================================================================*/
/*                                ABOUT BUFFER		                               */
/*=================================================================================*/


/* ===================================================================================*/
// Memory 초기화

void Buffer_memset(double array_1[], double array_2[], double array_3[], double array_4[], double array_5[], double array_6[], double array_7[]) {
	
	memset(array_1, 0, sizeof(double) * N_DATA);
	memset(array_2, 0, sizeof(double) * N_DATA);
	memset(array_3, 0, sizeof(double) * N_DATA);
	memset(array_4, 0, sizeof(double) * N_DATA);
	memset(array_5, 0, sizeof(double) * N_DATA);
	memset(array_6, 0, sizeof(double) * N_DATA);
	memset(array_7, 0, sizeof(double) * N_DATA);

}

//void Print_data(void) {
//
//	sprintf(OutFileName_1, "Filter_Implementation");
//
//	pFile = fopen(strcat(OutFileName_1, "_data.txt"), "w+t");
//
//	for (int idx = 0; idx < N_DATA; idx++)
//	{
//		fprintf(pFile, "%20.10f\t%20.10f \n", Buf_Time[idx], Buf_Error[idx]);
//	}
//
//}


//void Print_data_2(void) {
//
//	sprintf(OutFileName_1, "Experimental_Result(6.23)");
//
//	pFile = fopen(strcat(OutFileName_1, "_data.txt"), "w+t");
//
//	for (int idx = 0; idx < N_DATA; idx++)
//	{
//		fprintf(pFile, "%20.10f\t%20.10f\t%20.10f\t%20.10f\t%20.10f\t%20.10f\t%20.10f \n", Buf_Time[idx], Buf_Error[idx], Buf_P_Out[count], Buf_I_Out[count], Buf_D_Out[count], Buf_PID_Out[count], Buf_Wgyro[count]);
//	}
//
//}

/*=================================================================================*/
/*                                 LINEAR REGRESSION                               */
/*=================================================================================*/

	/* Calculating Required Sum */

// 선언
// double     sumX = 0, sumX2 = 0, sumY = 0, sumXY = 0 , a2 = 0 , b2 = 0 ;
// double     a1 = 0;

// index number 중요
//for (i = 35; i < N; i++)
//{
//	sumX = sumX + Buf_Inputdata[i];
//	sumX2 = sumX2 + Buf_Inputdata[i] * Buf_Inputdata[i];
//	sumY = sumY + Buf_Outdata[i];
//	sumXY = sumXY + Buf_Inputdata[i] * Buf_Outdata[i];
//}
//
///* Calculating a and b */
//
//a2 = (66 * sumXY - sumX * sumY) / (66 * sumX2 - sumX * sumX);
//b2 = (sumX2 * sumY - sumX * sumXY) / (66 * sumX2 - sumX * sumX);
//
//printf("\na1은 IDEAL한 선형화 직선의 기울기 !\n\n");
//
//a1 = Buf_Outdata[100] / Buf_Inputdata[100];
//printf("VALUE OF a1 :%lf , a2 : %lf , b2 : %lf", a1, a2, b2);


/*=================================================================================*/
/*                                 MANY SIGNALS                                    */
/*=================================================================================*/

 //	Value of M_PI = -PI
 
double Triangular_wave(double x)
{
	if (x >= 0 && x < M_PI)
	{
		return 2.0 * (x / M_PI) - 1.0;
	}

	else if (x >= M_PI && x < 2 * M_PI)
	{
		return 2.0 * ((2 * M_PI - x) / M_PI) - 1.0;
	}
	else if (x >= 2 * M_PI)
	{
		return (Triangular_wave(x - 2 * M_PI));
	}
	else if (x < 0)
	{
		return (Triangular_wave(x + 2 * M_PI));
	}
}

/* ===================================================================================*/

double Square_wave(double x)
{
	if (x >= 0 && x < M_PI)
	{
		return 1.0;
	}

	else if (x >= M_PI && x < 2 * M_PI)
	{
		return -1.0;
	}
	else if (x >= 2 * M_PI)
	{
		return Square_wave(x - 2 * M_PI);
	}
	else if (x < 0)
	{
		return Square_wave(x + 2 * M_PI);
	}
}