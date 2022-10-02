#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include "Run_func.h"

/*===========================================================================================================================*/
/*                                                  START MAIN CODE                                                          */
/*===========================================================================================================================*/

void main(void) {

    DAQmxCreateTask((""),(&taskA_IN));
    DAQmxCreateAIVoltageChan((taskA_IN), ("Dev2/ai0:3"), (""), (DAQmx_Val_Diff), (0.0), (5.0), (DAQmx_Val_Volts), (""));
    DAQmxStartTask(taskA_IN);

    AnalogWrite[0] = 5.0;
    AnalogWrite[1] = 2.5;

    DAQmxCreateTask((""),(&taskA_OUT));
    DAQmxCreateAOVoltageChan((taskA_OUT), ("Dev2/ao0:1"), (""), (0.0), (5.0), (DAQmx_Val_Volts), (""));
    DAQmxStartTask(taskA_OUT);
    
    GET_average_offset();

    printf("\n\nAVERAGE GYRO__VOLTAGE :  %lf\n", V_gyro_off_avg);

    printf("\n\n");

    Start_presskey();

    do {

        Import_data();

        Export_data();

        //printf("Error Value : %lf  &&  PID_out_W : %lf && Linearized Vcmd : %lf \nWW\n",W_Error, W_in,Linearized_Vcmd);

        EMERGENCY_STOP;

        Idle_time();
        
        Time_Synchronization();


    } while (Time < FINAL_TIME);

    //Terminate_Task();
    AnalogWrite[0] = 0.0;
    AnalogWrite[1] = 2.5;
    DAQmxCreateAOVoltageChan((taskA_OUT), ("Dev2/ao0:1"), (""), (0.0), (5.0), (DAQmx_Val_Volts), (""));

    DAQ_ANALOG_STOPCLEAR(taskA_IN);
    DAQ_ANALOG_STOPCLEAR(taskA_OUT);


}