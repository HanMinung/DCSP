#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

/*===========================================================================================================================*/
/*                                                 안되면 되게 하라 반드시.                                                  */
/*===========================================================================================================================*/

/*===========================================================================================================================*/
/*                                                 GIMBALL MOTOR CONTROL                                                     */
/*===========================================================================================================================*/

#include "Run_func.h"

double      Buf_Time[N_DATA] = { 0,0 };
double      Buf_Input_Voltage[N_DATA] = { 0,0 };
double      Buf_Input_Linear[N_DATA] = { 0,0 };
double      Buf_Input_Fmtr[N_DATA] = { 0,0 };
double      Buf_Output_Voltage[N_DATA] = { 0,0 };
double      Buf_Output_Gyro[N_DATA] = { 0,0 };

//double      Ref_Voltage = 0.0;
//double      Linearized_Vcmd = 0.0;
double      W_in = 0.0;

float64     Voltage_Gyro = 0.0;
double      W_gyro = 0.0;
double      Out_Voltage_mean = 0.0;

/* forloop buffer */

double      Buf_SumData[N_DATA] = { 0.0, };
double      Buf_SumAngVel[N_DATA] = { 0.0, };
double      TaskTime[N_DATA] = { 0.0, };
double      Buf_SumOffset[N_DATA] = { 0,0 };

/* while loop buffer */

double      Buf_Outdata[N_DATA] = { 0.0, };
double      Buf_Inputdata[N_DATA] = { 0.0, };

double      Buf_Wgyro[N] = { 0,0 };

TaskHandle   taskAI2 = 0.0;

/*===========================================================================================================================*/
/*                                                  START MAIN CODE                                                          */
/*===========================================================================================================================*/

void main(void) {

    FILE* pFile;
    //FILE* fFile;

    DAQmxCreateTask("", &taskAI2);

    DAQmxCreateAIVoltageChan(taskAI2, "Dev2/ai2", "", DAQmx_Val_Diff, 0, 10.0, DAQmx_Val_Volts, "");

    DAQmxStartTask(taskAI2);

    /* Channel Creation , Channel Configuration , Starting the channel */
    DAQ_channel_creation();

    DAQ_channel_config();

    DAQ_channel_start();

    // MOTOR FLAG ON & Initialization
    DAQmxWriteAnalogScalarF64(taskAO0, 0.0, 5.0, 5.0, NULL);
    DAQmxWriteAnalogScalarF64(taskAO1, 0.0, 5.0, 2.5, NULL);

    Start_presskey();

    EMERGENCY_STOP;

    /*=======================================================================================================*/

    Alpha_L = 0.08314345;
    Alpha_R = 0.08153374;
    Beta_L = 1.01385;
    Beta_R = 1.00539128;

    a_L = 0.446;
    b_L = 0.19741;
    a_R = 0.44489;
    b_R = 0.016618;

    /*=======================================================================================================*/

        do {

            Time_prev = Time_curr;

            Time = SAMPLING_TIME * count;

            //  Peak to Peak : 1V & OFFSET : 1.2148V
            Ref_Voltage = Set_sinusoidal_offset(2.50, 2.50, 10.0);
            //  Ref_Voltage = 2.5 + 2.5 * Triangular_wave(2 * UNIT_PI * 0.1 * Time);
            //  Ref_Voltage = 2.5 + 2.5 * Square_wave(2 * UNIT_PI * 0.2 * Time);


            if (0 <= Ref_Voltage && Ref_Voltage <= 2.50) {

                Linearized_Vcmd = (Alpha_L * Ref_Voltage + Beta_L - b_L) / a_L;

            }

            if (2.50 < Ref_Voltage && Ref_Voltage <= 5.00) {

                Linearized_Vcmd = (Alpha_R * Ref_Voltage + Beta_R - b_R) / a_R;

            }

            DAQmxWriteAnalogScalarF64(taskAO1, 0.0, 5.0, Linearized_Vcmd, NULL);

            DAQmxWriteAnalogScalarF64(taskAO1, 0.0, 5.0, Linearized_Vcmd, NULL);

            DAQmxReadAnalogScalarF64(taskAI2, -1, &Voltage_Gyro, NULL);

            /*===========================================================================================================================*/

            Buf_Time[count] = Time ;
            Buf_Input_Voltage[count] = Ref_Voltage ;
            Buf_Input_Linear[count] = Linearized_Vcmd ;
            Buf_Output_Voltage[count] = Voltage_Gyro ;
            Buf_Output_Gyro[count] = ((Buf_Output_Voltage[count]- 1.21418) * 1000) / 0.67 ;

            EMERGENCY_STOP;

            Idle_time();

            count++;

            Time = SAMPLING_TIME * count;

            printf("Time : %lf , Ref Voltage : %2lf , Read DATA : %lf \n\n",Time, Ref_Voltage, Buf_Output_Voltage[count]);

        } while (Time < FINAL_TIME);

        EMERGENCY_STOP;

        DAQmxWriteAnalogScalarF64(taskAO0, 0.0, 5.0, 0, NULL);
        DAQmxWriteAnalogScalarF64(taskAO1, 0.0, 5.0, 2.5, NULL);


    /*===========================================================================================================================*/
    /*                                             Please FINISH with 2.5V AND FLAG OFF                                          */
    /*===========================================================================================================================*/

    void DAQ_write_set(void);

    // DAQ stop task 
    void DAQ_stop_task(void);

    void DAQ_clear_task(void);

    sprintf(OutFileName_1, "Modeling_sine_10.0Hz");

    pFile = fopen(strcat(OutFileName_1, "_data.txt"), "w+t");

    for (int idx = 0; idx < N_DATA; idx++)
    {
        fprintf(pFile, "%20.10f\t%20.10f\t%20.10f\t%20.10f \n", Buf_Time[idx], Buf_Input_Voltage[idx], Buf_Output_Voltage[idx], Buf_Output_Gyro[idx]);
    }

}

