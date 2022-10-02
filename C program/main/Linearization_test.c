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
double      Buf_Input_Vout[N_DATA] = { 0,0 };
double      Buf_Input_Fmtr[N_DATA] = { 0,0 };
double      Buf_Output_Vss[N_DATA] = { 0,0 };
double      Buf_Output_Vcmd[N_DATA] = { 0,0 };

double      Ref_Voltage = 0.0;
double      Linearized_Vcmd = 0.0;

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

    FOR_LOOP(i, N) {    // Essential Option : Setting Proper Value of N

        /*Time = SAMPLING_TIME * count;*/

        // N값 수정 필요 : 101

        Ref_Voltage = 2.30 + 0.05 * i;

        if (0 <= Ref_Voltage && Ref_Voltage <= 2.50) {

            Linearized_Vcmd = 0.0680 * Ref_Voltage + 1.9400;

        }

        if (2.50 < Ref_Voltage && Ref_Voltage <= 5.00) {

            Linearized_Vcmd = 0.0795 * Ref_Voltage + 2.6525;

        }

        DAQmxWriteAnalogScalarF64(taskAO1, 0.0, 5.0, Linearized_Vcmd, NULL);

        count = 0; // do while 들어가기 전에 count 초기화


        do {

            // PIN 조정 필요 AI1 OR AI2
            DAQmxReadAnalogScalarF64(taskAI2, -1, &Voltage_Gyro, NULL);

            Buf_SumData[count] = Voltage_Gyro;

            EMERGENCY_STOP;

            Idle_time();

            Time_prev = Time_curr;

            Time = SAMPLING_TIME * count;

            count++;

        } while (Time < FINAL_TIME);

        // 1000개의 데이터 중 400 ~ 900까지 데이터를 본다

        EMERGENCY_STOP;

        // DATA SIZE : 1600개

        for (int i = 400; i < (N_DATA - 100); i++) {

            Out_Voltage_mean += Buf_SumData[i];

        }

        //// 이거 나누는 값 수동 조정 해야함
        Buf_Inputdata[i] = Ref_Voltage;
        Buf_Outdata[i] = Out_Voltage_mean / 1100;
        // Calculation of W_gyro
        Buf_Wgyro[i] = ((Buf_Outdata[i] - 1.2150) * 1000) / 0.67; //  [deg/sec]

        /*   Update Voltage Level   */

        //  DAQmxWriteAnalogScalarF64(taskAO1, 0.0, 5.0, 2.5, NULL);
        Out_Voltage_mean = 0.0;

        DAQmxWriteAnalogScalarF64(taskAO1, 0.0, 5.0, 2.5, NULL);

        Sleep(2500);

        printf("Vc : %lf , Vcmd : %lf , OutVol : %lf , Out_W_Gyro : %lf \n", Ref_Voltage, Linearized_Vcmd, Buf_Outdata[i], Buf_Wgyro[i]);


    }

    /*===========================================================================================================================*/
    /*                                             Please FINISH with 2.5V AND FLAG OFF                                          */
    /*===========================================================================================================================*/

    void DAQ_write_set(void);

    // DAQ stop task 
    void DAQ_stop_task(void);

    void DAQ_clear_task(void);


    sprintf(OutFileName_1, "Gimbal_Characteristic_Linearization(5.17)");

    pFile = fopen(strcat(OutFileName_1, "_data.txt"), "w+t");

    for (int idx = 0; idx < N; idx++)
    {
        fprintf(pFile, "%20.10f\t%20.10f\t%20.10f \n", Buf_Inputdata[idx], Buf_Outdata[idx], Buf_Wgyro[idx]);
    }

}
