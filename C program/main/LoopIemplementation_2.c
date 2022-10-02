#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include "Run_func.h"

/*===========================================================================================================================*/
/*                                                  START MAIN CODE                                                          */
/*===========================================================================================================================*/

void main(void) {

    char		OutFileName[100] = { "" };
    char        OutFileName_1[50] = { "" };
    char        OutFileTag_1[20] = { "" };

    double      Buf_Time[N_DATA] = { 0,0, };
    double      Buf_Error[N_DATA] = { 0,0, };
    double		Buf_P_Out[N_DATA] = { 0,0, };
    double		Buf_I_Out[N_DATA] = { 0,0, };
    double		Buf_D_Out[N_DATA] = { 0,0, };
    double		Buf_PID_Out[N_DATA] = { 0,0, };
    double      Buf_Wgyro[N_DATA] = {0,0, };

    DAQmxCreateTask((""), (&taskA_IN));
    DAQmxCreateAIVoltageChan((taskA_IN), ("Dev2/ai0:3"), (""), (DAQmx_Val_Diff), (0.0), (5.0), (DAQmx_Val_Volts), (""));
    DAQmxStartTask(taskA_IN);

    AnalogWrite[0] = 5.0;
    AnalogWrite[1] = 2.5;

    DAQmxCreateTask((""), (&taskA_OUT));
    DAQmxCreateAOVoltageChan((taskA_OUT), ("Dev2/ao0:1"), (""), (0.0), (5.0), (DAQmx_Val_Volts), (""));
    DAQmxStartTask(taskA_OUT);


    FOR_LOOP(i, 40) {

        READ_ANALOG(taskA_IN, &AnalogRead);

        Voltage_Gyro = AnalogRead[2];

        //EMERGENCY_STOP;

        printf("%lf \n", Voltage_Gyro);
        Sleep(1000);

        sum += Voltage_Gyro;

    }

    V_gyro_off_avg = sum / 40.0;

    printf("%lf\n", V_gyro_off_avg);

    printf("\n\n");

    Start_presskey();

    do {

        //Import_data();
        //DAQmxReadAnalogScalarF64(taskAI2, -1, &Voltage_Gyro, NULL);
        READ_ANALOG(taskA_IN, &AnalogRead);

        Voltage_Gyro = AnalogRead[2];

        printf("Gyro_Voltage Value : %lf\n", Voltage_Gyro);

        W_gyro = (Voltage_Gyro - V_gyro_off_avg) * (1000 / 0.67);

        W_gyro_LPF = Second_order_Filter(-0.5094, 0.0, 0.2453, 0.2453, 0.0, W_gyro);                    // LPF implementation

        W_Error = 200 * Square_wave(2 * UNIT_PI * 0.2 * Time) - W_gyro_LPF;

        /*====================================================================================================*/

        P_Controller(W_Error, 1.106);                                               // IDEAL VALUE : 1.236
                                                                                    // P gain을 조금 0.2 정도 키움 : overshoot 감소 rising time은 거의 비슷함
        I_Controller(W_Error_past, W_Error, I_Curr_OUT, 43.73);

        D_Controller(W_Error_past, W_Error, D_Curr_OUT, 0.02071);

        W_in = P_Curr_OUT + I_Curr_OUT + D_Curr_OUT;

        W_Error_past = W_Error;

        Ref_Voltage = W_in * (1 / Unit_conversion_coef) + 2.5;

        if (0 <= Ref_Voltage && Ref_Voltage <= 2.50) {

            Linearized_Vcmd = (Alpha_L * Ref_Voltage + Beta_L - b_L) / a_L;

        }

        if (2.50 < Ref_Voltage && Ref_Voltage <= 5.00) {

            Linearized_Vcmd = (Alpha_R * Ref_Voltage + Beta_R - b_R) / a_R;

        }

        //DAQmxWriteAnalogScalarF64(taskAO1, 0.0, 5.0, Linearized_Vcmd, NULL);
        AnalogWrite[1] = Linearized_Vcmd;

        WRITE_ANALOG(taskA_OUT, AnalogWrite);

        //printf("Error Value : %lf  &&  PID_out_W : %lf && Linearized Vcmd : %lf \n\n",W_Error, W_in,Linearized_Vcmd);

        EMERGENCY_STOP;

        Buf_Time[count] = Time;
        Buf_Error[count] = W_Error;
        Buf_P_Out[count] = P_Curr_OUT;
        Buf_I_Out[count] = I_Curr_OUT;
        Buf_D_Out[count] = D_Curr_OUT;
        Buf_PID_Out[count] = W_in;
        Buf_Wgyro[count] = W_gyro;

        while (1)
        {
            Time_curr = GetWindowTime();

            if (Time_curr - Time_prev >= (SAMPLING_TIME * 1000.0))	break;
        }

        Time_prev = Time_curr;

        count++;

        Time = SAMPLING_TIME * count;

    } while (Time < FINAL_TIME);

    //Terminate_Task();
    AnalogWrite[0] = 0.0;
    AnalogWrite[1] = 2.5;
    DAQmxCreateAOVoltageChan((taskA_OUT), ("Dev2/ao0:1"), (""), (0.0), (5.0), (DAQmx_Val_Volts), (""));

    DAQ_ANALOG_STOPCLEAR(taskA_IN);
    DAQ_ANALOG_STOPCLEAR(taskA_OUT);

    sprintf(OutFileName_1, "Experimental_Result(6.23)");

    pFile = fopen(strcat(OutFileName_1, "_data.txt"), "w+t");

    for (int idx = 0; idx < N_DATA; idx++)
    {
        fprintf(pFile, "%20.10f\t%20.10f\t%20.10f\t%20.10f\t%20.10f\t%20.10f\t%20.10f \n", Buf_Time[idx], Buf_Error[idx], Buf_P_Out[count], Buf_I_Out[count], Buf_D_Out[count], Buf_PID_Out[count], Buf_Wgyro[count]);
    }

}