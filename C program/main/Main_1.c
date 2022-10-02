#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

/*===========================================================================================================================*/
/*                                                 안되면 되게 하라 반드시.                                                  */
/*===========================================================================================================================*/

/*===========================================================================================================================*/
/*                                                 GIMBALL MOTOR 정적특성                                                    */
/*===========================================================================================================================*/

#include "Run_func.h"

double      Buf_Time[N_DATA] = { 0,0 };

float64     Voltage_Gyro = 0.0;
double      W_gyro = 0.0;
double      Out_Voltage_mean = 0.0;


/* forloop buffer */

double      Buf_SumData[N_DATA] = { 0.0, };
double      Buf_SumAngVel[N_DATA] = { 0.0, };
double      TaskTime[N_DATA] = { 0.0, };

double      Sum_Offset = 0.0;
double      Voltage_Offset = 0.0;

/* while loop buffer */

double       Buf_Outdata[N_DATA] = { 0.0, };
double       Buf_Inputdata[N_DATA] = { 0.0, };

double       Buf_Wgyro[N] = { 0,0 };

TaskHandle   taskAI2 = 0.0;

/*===========================================================================================================================*/
/*                                                  START MAIN CODE                                                          */
/*===========================================================================================================================*/

// N = 80

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

    /****************************************************************************************/
    /*                                  N VALUE    :  132                                   */
    /*                                  FINAL TIME :  6 sec                                 */              
    /****************************************************************************************/

    // INITIALIZATION

    Ref_Voltage = 1.79;

    FOR_LOOP(i, N) {    // Essential Option : Setting Proper Value of N

        //Time = SAMPLING_TIME * count;

        if (Ref_Voltage >= 1.790 && Ref_Voltage < 1.850)       Ref_Voltage += 0.005;                   // INDEX : 12개   ending : 1.850
        if (Ref_Voltage >= 1.850 && Ref_Voltage <= 2.10)      Ref_Voltage += 0.05;                     // INDEX : 5개   ending : 2.20
        /*    Dead Zone 이전   */
        if (Ref_Voltage > 2.10 && Ref_Voltage < 2.30)        Ref_Voltage += 0.005;                     // INDEX : 40개  ending : 2.285
        /*    Dead Zone        */
        if (Ref_Voltage >= 2.30 && Ref_Voltage <  2.65)      Ref_Voltage += 0.05;                      // INDEX : 7개   ending : 2.70
        /*    Dead Zone 이후   */
        if (Ref_Voltage >= 2.65 && Ref_Voltage < 2.90)       Ref_Voltage += 0.005;                     // INDEX : 50개 
        if (Ref_Voltage >= 2.90 && Ref_Voltage < 3.05)       Ref_Voltage += 0.05;                      // INDEX : 3개
        if (Ref_Voltage >= 3.05 && Ref_Voltage < 3.155)       Ref_Voltage += 0.005;                    // INDEX : 21개

        count = 0; // do while 들어가기 전에 count 초기화

        DAQmxWriteAnalogScalarF64(taskAO1, 0.0, 5.0, Ref_Voltage, NULL);

        do {

            // PIN 조정 필요 AI1 OR AI2
            DAQmxReadAnalogScalarF64(taskAI2, -1, &Voltage_Gyro, NULL);

            Buf_SumData[count] = Voltage_Gyro;

            EMERGENCY_STOP;

            Idle_time();

            Time_Synchronization();

        } while (Time < FINAL_TIME);

        // 1000개의 데이터 중 400 ~ 900까지 데이터를 본다

        EMERGENCY_STOP;

        for (int i = 400; i < (N_DATA - 100); i++) {

            Out_Voltage_mean += Buf_SumData[i];

        }

        //// 이거 나누는 값 수동 조정 해야함
        Buf_Inputdata[i] = Ref_Voltage;
        Buf_Outdata[i] = Out_Voltage_mean / 700;
        // Calculation of W_gyro
        Buf_Wgyro[i] = ((Buf_Outdata[i] - 1.21418) * 1000) / 0.67; //  [deg/sec]

        Out_Voltage_mean = 0.0;

        DAQmxWriteAnalogScalarF64(taskAO1, 0.0, 5.0, 2.5, NULL);

        Sleep(2000);

        printf("Vc : %lf , OutVol : %lf , Out_W_Gyro : %lf \n", Ref_Voltage, Buf_Outdata[i], Buf_Wgyro[i]);

        // MOTOR ENDING 2.5V 로 내리기

    }

    /*===========================================================================================================================*/
    /*                                             Please FINISH with 2.5V AND FLAG OFF                                          */
    /*===========================================================================================================================*/

    void DAQ_write_set(void);

    // DAQ stop task 
    void DAQ_stop_task(void);

    void DAQ_clear_task(void);

    // Create File
    sprintf(OutFileName_1, "Gimbal_Characteristic_(6.7)");

    pFile = fopen(strcat(OutFileName_1, "_data.txt"), "w+t");

    //  FOR_LOOP의 반복 횟수가 N번이기 때문에 
    for (int idx = 0; idx < N; idx++)
    {
        /*===============================출력할 내용 입력====================================*/
        fprintf(pFile, "%20.10f\t%20.10f\t%20.10f \n", Buf_Inputdata[idx], Buf_Outdata[idx], Buf_Wgyro[idx]);
        //fprintf(pFile, "%20.10f %20.10f %20.10f %20.10f\n", Buf_Inputdata[idx], Linearized_Vss[idx], Buf_Outdata[idx], Buf_Angvelocity[idx]);
    }

}
