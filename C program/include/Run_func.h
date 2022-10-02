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
#define         FINAL_TIME            (double)(				30.0            )    
#define         SAMPLING_FREQ         (double)(             200.0           )    
#define         SAMPLING_TIME         (double)(   1 / SAMPLING_FREQ         )    
#define         N_STEP                (int)   ( FINAL_TIME / SAMPLING_TIME  )    
#define         N_DATA                (int)   ( FINAL_TIME / SAMPLING_TIME  )    
#define         HIGH                  (int)   (				5				)
#define         LOW                   (int)   (				0				)
#define         N                     (int)                 132                    
#define			Unit_Conv_coef		  (double)(		  295.67/2.495		    )

/* =========================================================================================================== */

#define         ENTER_BREAK            if (_kbhit())    {if (_getch() == '\r') {break;}}
#define         EMERGENCY_STOP         if (_kbhit())   {if (_getch() == ' ') { AnalogWrite[0] = 0.0; AnalogWrite[1] = 2.5; \
																			   DAQmxCreateAOVoltageChan((taskA_OUT), ("Dev2/ao0:1"), (""), (0.0), (5.0), (DAQmx_Val_Volts), ("")); return 0;}}
#define         FOR_LOOP(i, N)         for (int i = 0; i < N; i ++)

/* =========================================================================================================== */

#define   DAQ_ANALOG_INPUT_PORT(task)         (DAQmxCreateTask((""), (&task)));\
                                              (DAQmxCreateAIVoltageChan((task), ("Dev2/ai0:3"), (""), (DAQmx_Val_Diff), (0.0), (5.0), (DAQmx_Val_Volts), ("")));\
                                              (DAQmxStartTask(task))

#define   DAQ_ANALOG_OUTPUT_PORT(task)        (DAQmxCreateTask((""), (&task)));\
                                              (DAQmxCreateAOVoltageChan((task), ("Dev2/ao0:1"), (""), (0.0), (5.0), (DAQmx_Val_Volts), ("")));\
                                              (DAQmxStartTask(task))

#define   DAQ_ANALOG_STOPCLEAR(task)	      (DAQmxStopTask(task));\
										      (DAQmxClearTask(task))

#define   WRITE_ANALOG(Task , data)           DAQmxWriteAnalogF64(Task ,1 ,1 , FINAL_TIME , DAQmx_Val_GroupByChannel ,  data , NULL , NULL )
#define   READ_ANALOG(Task , data)            DAQmxReadAnalogF64(Task , -1 , FINAL_TIME  , DAQmx_Val_GroupByChannel , data , sizeof(float64)*2  , NULL , NULL )

/* =========================================================================================================== */

// ABOUT DAQ
extern 	TaskHandle   taskAI0 = 0;
extern 	TaskHandle   taskAI1 = 0;
extern 	TaskHandle   taskAI2 = 0;
extern 	TaskHandle   taskAI3 = 0;
extern 	TaskHandle   taskAO0 = 0;
extern 	TaskHandle   taskAO1 = 0;

extern	float64		 AnalogRead[4] ;
extern	float64		 AnalogWrite[2] ;

extern  TaskHandle   taskA_IN = 0 ;
extern  TaskHandle   taskA_OUT = 0 ;

/* =========================================================================================================== */
extern	FILE* pFile;

extern  double      Time = 0.0;
extern	double      Time_prev = 0.0;
extern	double      Time_curr = 0.0;
extern	double      Time_idle = 0.0;
extern	double      Time_init = 0.0;

extern	double		x = 0.0;

extern	double		Alpha_L = 0.08314345;
extern	double		Alpha_R = 0.08153374;
extern	double		Beta_L = 1.01385;
extern	double		Beta_R = 1.00539128;

extern	double		a_L = 0.446;
extern	double		b_L = 0.19741;
extern	double		a_R = 0.44489;
extern	double		b_R = 0.016618;

/* =========================================================================================================== */

extern  double		DIR = 0.0 ;
extern  double		DOA = 0.0 ;
extern  double      Kp_Outer = 1.3;
extern  double		Epsylon = 0.0;
extern  double      Epsylon2DEG = 0.0;

/* =========================================================================================================== */

extern	double		Linearized_Vcmd = 0.0;
extern	double		Ref_Voltage = 0.0;

extern	double      W_in = 0.0;
extern	double      Unit_conversion_coef = 300.00 / 2.50;
extern	double      W_Error = 0.0;
extern	double      W_reference = 0.0;

extern	float64     Voltage_Gyro = 0.0;
extern	double      W_gyro = 0.0;
extern  double		V_gyro_offset = 0.0;
extern  double      V_gyro_off_avg = 0.0;

extern	char		OutFileName[100] = { "" };

/*=================================================================================*/

extern	double 		Freq = 0.0;
extern	double		Voltage_offset = 0.0;
extern  double		Standard = 0.0;

/*=================================================================================*/

extern	int			idx = 0;
extern	int			count = 0;

extern	unsigned	i = 0;

extern	int			alpha = 3;
extern	double		Tau = 1 / 157;
extern	double		Error_Curr = 0.0;
extern	double		P_Curr_OUT = 0.0;
extern  double		I_Curr_OUT = 0.0;
extern  double		D_Curr_OUT = 0.0;

extern	double		K_P = 0.0;
extern	double		K_D = 0.0;
extern	double		K_I = 0.0;

extern  double		W_gyro_LPF = 0.0;
extern  double		W_gyro_PID = 0.0;
extern  double		W_Error_past = 0.0;

extern  double		I_past = 0.0;
extern  double		D_past = 0.0;

extern  double		PID_W_OUT = 0.0;

/*=================================================================================*/

extern	double		triangle_freq = 0.0;
extern  double		square_freq = 0.0;

extern	char        OutFileName_1[50] = { "" };
extern	char        OutFileTag_1[20] = { "" };

extern	double      Buf_Time[N_DATA] = { 0,0, };
extern	double      Buf_Error[N_DATA] = { 0,0, };
extern  double		Buf_P_Out[N_DATA] = { 0,0, };
extern  double		Buf_I_Out[N_DATA] = { 0,0, };
extern  double		Buf_D_Out[N_DATA] = { 0,0, };
extern  double		Buf_PID_Out[N_DATA] = { 0,0, };
extern  double      Buf_Wgyro[N_DATA] = { 0,0, };

extern	double		b2 = 0.0, b1 = 0.0, b0 = 0.0;
extern	double		xk0 = 0.0, xk1 = 0.0, xk2 = 0.0;
extern	double		a1 = 0.0, a0 = 0.0;
extern	double		Vin = 0.0, Vout = 0.0;

/*=================================================================================*/

extern  double      sum = 0, sumX2 = 0, sumY = 0, sumXY = 0, a2 = 0;

/*=================================================================================*/
/*                                 ABOUT DAQ			                           */
/*=================================================================================*/

/* Channel Creation */
void DAQ_channel_creation(void);

/* channel configuration */
void DAQ_channel_config(void);

/* Start the Channel */
void DAQ_channel_start(void);

/* DAQ channel clearance */
void DAQ_stop_task(void);

void DAQ_clear_task(void);

void DAQ_read_set(void);

void DAQ_write_set(void);

void DAQ_stop_task(void);

void DAQ_clear_task(void);

void Initialization(void);

void Terminate_Task(void);

void GET_average_offset(void);

/*=================================================================================*/

void Linearization(void);

void Import_data(void);

void Export_data(void);

void Print_data(void);

/*=================================================================================*/

double Set_sinusoidal(double Standard, double Freq);

double Set_sinusoidal_offset(double Voltage_offset, double Standard, double Freq);

/*=================================================================================*/

void Start_presskey(void);

void Start_Next_Frequency(void);

/*=================================================================================*/

void Memory_write(void);

/*=================================================================================*/
/*                                 ABOUT TIME			                           */
/*=================================================================================*/

void Time_Synchronization(void);

double GetWindowTime(void);

void Idle_time(void);

/*=================================================================================*/
/*						    DIGITAL FILTER IMPLEMENTATION				           */
/*=================================================================================*/

double First_order_Filter(double a1, double b1, double b0, double Vin);

double Second_order_Filter(double a1, double a0, double b2, double b1, double b0, double Vin);

double P_Controller(double Error_curr, double K_P);

double I_Controller(double _error_past, double _error_curr, double _I_past, double K_I);

double D_Controller(double _error_past, double _error_curr, double _D_past, double K_D);

/*=================================================================================*/
/*                                 ABOUT BUFFER			                           */
/*=================================================================================*/

void Buffer_memset(double array_1[], double array_2[], double array_3[], double array_4[], double array_5[]);

/*=================================================================================*/
/*                                 ABOUT VALIDATION                                */
/*=================================================================================*/

double Triangular_wave(double x);

double Square_wave(double x);