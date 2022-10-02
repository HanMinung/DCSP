%% MOTOR CHARACTERISTIC
% Gymball Characteristic Curve
clear all; close all; clc;
data_1 = load('Gimbal_Characteristic_(6.7)_data.txt');

V_cmd = data_1(:,1);
V_gyro = data_1(:,2);
W_gyro = data_1(:,3);

% PLOTTING
figure(1);
scatter(V_cmd,V_gyro);
grid on;
title("Vcmd - Vout Curve",'Fontsize',20);
xlabel("Vcmd [V]",'Fontsize',20);
ylabel("Vout [V]",'Fontsize',20);
legend("Vout",'Fontsize',20);

figure(2);
scatter(V_cmd,W_gyro);
grid on;
title("Vcmd - Wgyro Curve",'Fontsize',20);
xlabel("Vcmd [V]",'Fontsize',20);
ylabel("Wgyro [deg/sec]",'Fontsize',20);
legend("Wgyro",'Fontsize',20);

%% MOTOR CHARACTERISTIC & Ideal 선형화
% Gymball Characteristic Curve

data_1 = load('Gimbal_Characteristic_(5.25)_data.txt');

V_cmd = data_1(:,1);
V_gyro = data_1(:,2);
W_gyro = data_1(:,3);

t = linspace(0,5,99);
Alpha = 0.07603206;
Beta = 1.02311967;
y = Alpha*t+Beta;

k = 300.67/2.495;

% PLOTTING
figure(1);
scatter(V_cmd,V_gyro);
hold on;
scatter(t,y);
grid on;
title("Vcmd - Vout Curve",'Fontsize',20);
xlabel("Vcmd [V]",'Fontsize',20);
ylabel("Vout [V]",'Fontsize',20);
legend("Vout","Ideal Linearization",'Fontsize',20);

figure(2);
scatter(V_cmd,W_gyro);
hold on;
scatter(t, 1480 * (y-1.2148) );
grid on;
title("Vcmd - Wgyro Curve",'Fontsize',20);
xlabel("Vcmd [V]",'Fontsize',20);
ylabel("Wgyro [deg/sec]",'Fontsize',20);
legend("Wgyro","Ideal linearization",'Fontsize',20);
ylim([-300 300]);

%% Linearization
clc; close all; clear all; 

data_1 = load('Gimbal_Characteristic_Linearization(5.25)_data.txt');

Linear_Vcmd = data_1(:,1);
k = 295.67/2.495;
Linear_Win = k * (data_1(:,1) - 2.5) ;
Linear_Vgyro = data_1(:,2);
Linear_Wgyro = data_1(:,3);

figure(1);
scatter(Linear_Vcmd,Linear_Vgyro);
grid on;
title("Lienarzation : Vcmd - Vout Curve",'Fontsize',20);
xlabel("Vcmd [V]",'Fontsize',20);
ylabel("Vout [V]",'Fontsize',20);
legend("Vout",'Fontsize',20);

figure(2);
scatter(Linear_Vcmd,Linear_Wgyro);
grid on;
title("Lienarzation : Vcmd - Wout Curve",'Fontsize',20);
xlabel("Vcmd [V]",'Fontsize',20);
ylabel("Wout [deg/s]",'Fontsize',20);
legend("Wout",'Fontsize',20);
hold on;

figure(3);
scatter(Linear_Win,Linear_Wgyro);


%% Validation : Sine wave(0.5 , 2) , Triangle Wave(0.2) , Rectangular Wave(0.2)
clear all; close all; clc;

Alpha = 0.08314345;
Beta = 1.01385;

data_1 = load('Modeling_triangle_0.1Hz_data.txt');
data_2 = load('Modeling_sine_0.2Hz_data.txt');
% data_3 = load('Modeling_sine_2.0Hz_data.txt');
Time = data_1(:,1);
V_in = data_1(:,2);
Time_2 = data_2(:,1);
V_in_2 = data_2(:,2);
% V_in_3 = data_3(:,2);

V_cmd = (Alpha*V_in + Beta)

k = 298/2.50;
W_in = k * (V_in - 2.5) ;
W_in_2 = k * (V_in_2 - 2.5) ;
% W_in_3 = k * (V_in_3 - 2.5) ;

V_out = data_1(:,3);
W_out = data_1(:,4);

V_out_2 = data_2(:,3);
W_out_2 = data_2(:,4);

% V_out_3 = data_3(:,3);
% W_out_3 = data_3(:,4);

figure(1);
plot(Time,W_in,'linewidth',3);
hold on;
plot(Time,W_out,'linewidth',2);
hold off;

grid on;
title("Validation : 0.1Hz Triangle Input ",'Fontsize',20);
xlabel("time [sec]",'Fontsize',20);
ylabel("Anglular Velocity [deg/sec]",'Fontsize',20);
legend("Win","Wout",'Fontsize',20);


figure(2);
plot(Time_2,W_in_2,'linewidth',3);
hold on;
plot(Time_2,W_out_2,'linewidth',2);
hold off;

grid on;
title("Validation : 0.2Hz Sine Input ",'Fontsize',20);
xlabel("time [sec]",'Fontsize',20);
ylabel("Anglular Velocity [deg/sec]",'Fontsize',20);
legend("Win","Wout",'Fontsize',20);

% figure(3);
% plot(Time,W_in_3,'linewidth',3);
% hold on;
% plot(Time,W_out_3,'linewidth',2);
% hold off;
% 
% grid on;
% title("Validation : 0.2Hz Sine Input ",'Fontsize',12);
% xlabel("time [sec]",'Fontsize',12);
% ylabel("Anglular Velocity [deg/sec]",'Fontsize',12);
% legend("Win","Wout",'Fontsize',12);
% xlim([8 11.035]);


%% PLOT MOTOR Modeling TEST : SQUARE WAVE
clear all; close all; clc;

Alpha = 0.0819001;
Beta = 1.02724;

data_1 = load('Motor_square_0.1Hz_data.txt');

k = 295.67/2.495;

Time = data_1(:,1);
V_in = data_1(:,2);
W_in = k * (V_in - 2.5) ;
V_out = data_1(:,3);
W_out = data_1(:,4);

figure();
plot(Time,W_in,'linewidth',3);
hold on;
plot(Time,W_out,'linewidth',2);
hold off;

grid on;
title("Validation : 0.1Hz SQUARE Input ",'Fontsize',12);
xlabel("Time [sec]",'Fontsize',12);
ylabel("Anglular Velocity [deg/sec]",'Fontsize',12);
legend("Win","Wout",'Fontsize',12);


