clear all; clc; close all;

% Right Side
Vc_dz_pl = 2.70;        % dead zone이 끝나는 위치
Vc_sat_pl = 5.0;       % sat zone이 끝나는 위치 

Vcmd_dz_pl = 2.60;
Vcmd_sat_pl = 3.1;

% Left Side
Vc_dz_mi = 2.40;        
Vc_sat_mi = 1.99;

Vcmd_dz_mi = 2.20;
Vcmd_sat_mi = 0.00;

% Right SIDE

matrixA_1 = [Vc_dz_pl 1 ;
             Vc_sat_pl 1]
matrixB_1 = [Vcmd_dz_pl ; Vcmd_sat_pl]

Variable_Right = inv(matrixA_1) * matrixB_1;

% Left SIDE

matrixA_2 = [Vc_dz_mi 1 ;
             Vc_sat_mi 1]
matrixB_2 = [Vcmd_dz_mi ; Vcmd_sat_mi]

Variable_Left = inv(matrixA_2) * matrixB_2;
