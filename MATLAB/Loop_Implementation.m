%% Loop_Implementation : Analysis model
clear all; clc; close all;

s = tf('s');
T_f = 200;
Ts = 1/T_f; 

%-------------------------------------------------------------------------%
Num_c = [0.02071 1.036 45.73];
Den_c = [1 0];
G_controller = tf(Num_c,Den_c);

Num_m = [0 0 3126];
Den_m = [1 95.29 3651];
%%
% _ _ _ITALIC TEXT_ _ _ 
G_motor = tf(Num_m,Den_m);
G_cl = feedback(G_motor*G_controller,1,-1);

LPF_coef = 130.0;
Num_f = [LPF_coef];
Den_f = [1 LPF_coef];
G_lpf = tf(Num_f,Den_f);
%-------------------------------------------------------------------------%
G_lpf_z = c2d(G_lpf,Ts,'Tustin');

%-------------------------------------------------------------------------%
%% Step Response Plot according to the PID gain

% K_P는 어느 정도 늘리는게 좋을 것 같음. 1.036 --> 1.236
% K_I는 어느정도 줄이는게 좋을 것 같음.  45.74 --> 41.73  --> OVERSHOOT을 줄일 수 있다.
% K_D는 건드리지 않는 것이 좋을 것 같음.

% for K_D = 45.73 : -2 : 37.73

Num_m = [0 0 3126];
Den_m = [1 95.29 3651];
G_motor = tf(Num_m,Den_m);

for K_P = 1.036 : 0.2 : 2.036
%for K_I = 45.73 : -2 : 37.73
   
    figure(5),clf

    Num_c = [0.02071  K_P  45.74];
    Den_c = [1 0];
    G_controller = tf(Num_c,Den_c);
    
    G_cl = feedback(G_motor*G_controller,1,-1);
    [y, t] = step(G_cl, 0.2);
    
    figure(1);
    plot(t,y,'LineWidth',1.3);  hold on;
    xlabel("Time[sec]",'fontsize',20);
    ylabel("Amplitude[-]", 'fontsize',20);
    title ("Step Response", 'fontsize',20);
    xlim([0 0.17]);
    ylim([0 1.1]);
    
    hold on;
    grid on;
    set(gca,'Fontsize',14);
    
end

legend('K_P = 1.036','K_P = 1.236','K_P = 1.436','K_P = 1.636','K_P = 1.836','K_P = 2.036');
%% Simulink DATA plot : Analysis model

Simdata = sim('LoopStructure_Analysismodel.slx');
Time   = Simdata.time;
Output = Simdata.output;

plot(Time,Output,'linewidth',1.5);
grid on; box on;
xlabel("Time [sec]",'Fontsize',20);
ylabel("Angular velocity[deg/s]",'Fontsize',20);
legend("Input","Disturbance","Output",'Fontsize',20);
title("Disturbance rejection performance",'Fontsize',20);
xlim([0 5]);

%% 겹쳐 그리면서 PID gain 튜닝하기
clear all; close all; clc;

Num_c1 = [0.02071 1.036 45.73];
Num_c2 = [0.02071 1.406 53.73];  %% IDEAL VALUE
Den_c = [1 0];
G_controller_1 = tf(Num_c1,Den_c);
G_controller_2 = tf(Num_c2,Den_c);

Num_m = [0 0 3126];
Den_m = [1 95.29 3651];
G_motor = tf(Num_m,Den_m);

G_cl_1 = feedback(G_motor*G_controller_1,1,-1);
G_cl_2 = feedback(G_motor*G_controller_2,1,-1);

[y1,t1] = step(G_cl_1,0.5);
[y2,t2] = step(G_cl_2,0.5);

    figure(1);
    plot(t1,y1,'LineWidth',2);  hold on;
    plot(t2,y2,'Linewidth',2);  
    grid on;
    xlabel("Time[sec]",'fontsize',20);
    ylabel("Amplitude [-]", 'fontsize',20);
    title ("Step Response", 'fontsize',20);
    legend('Before adjusting','After adjusting','Fontsize',20);
    
    xlim([0 0.3]);
    ylim([0 1.1]);
    
%%  우주형 코드

clc; close all; clear all;
% A= readmatrix("DoriDori.txt");

fc_LPF = 10;%critical
wc_LPF = fc_LPF*2*pi;
fs=200;
[b,a] =butter(2,fc_LPF/(fs/2));


A = load("21800773_0622_2141.mat");
save_Wb_1 = A.save_movemean_Wb;
save_Wg_1 = A.save_movemean_Wg;

N_time_1 = length(save_Wb_1);
Final_time = 30;
SAMPLING_PERIOD = 0.005;
time_W_1 = 0:SAMPLING_PERIOD:Final_time-SAMPLING_PERIOD;

WgFiltered=filter(b,a,save_Wg_1);

fig1 = figure;
plot( time_W_1, save_Wb_1 );
hold on; plot( time_W_1, WgFiltered );

legend('Wb', 'Wg');
xlabel('Time [sec]')
ylabel('\omega [deg/sec]');
movegui(fig1,"southwest")
saveas(fig1, 'disturbance_1.fig'); %저장
    
    
%% Actual Experiment DATA
 
clc; clearvars; close all;
A=load("21800773_0622_2141.mat");
time = 0:0.005:(30-0.005);
time = time(625:4503);
time = transpose(time);
input = movmean(A.save_movemean_Wb(625:4503),5);
output = movmean(A.save_movemean_Wg(625:4503),5);

    fun = @(x, t) x(1)*cos(x(4)*t+x(2))+x(3);
    x0out = [100, -1, 0, 5];
    x0in = [200, 1, 0, 5];

   xout = lsqcurvefit(fun,x0out,time,output);
   xin = lsqcurvefit(fun,x0in,time,input);
plot(time, input, 'b-',time,1.5*output,'k','linewidth',0.8);
hold on;
plot(time,1.5*fun(xout,time), time,fun(xin,time), 'LineWidth',0.8);
grid on
box on

title('Disturbance Rejection Performance', 'FontSize',20, 'FontWeight','bold');
xlabel('Time [sec]','Fontsize',20); ylabel('angular velocity [deg/s]','Fontsize',20); 
legend('Actual input', 'Actual output', 'Fitted output', 'Fitted input','Fontsize',20);
xlim([5 10])


%% Tracking Performance : OUTER LOOP IMPLEMENTATION
clear all; close all; clc;

A = readmatrix('TRACKING_21800773_0624_0041.csv');
time = A(:,1);
whatisthis = A(:,2);
PTT = A(:,3);
DOA = A(:,4);
DIR = A(:,5);

PTT = PTT - 1.4;
Distance = PTT*28.5/2.5;
angle = atan(Distance/50.5)*(180/pi);
for i = 1:size(time)
    if(whatisthis(i) < 0)
        controlled(i) = (10.0 / 2.5)*DOA(i);
    end
    if(whatisthis(i) > 0)
        controlled(i) = -(10.0 / 2.5)*DOA(i);
    end
end 
head = transpose(angle)-controlled;

hold on;
grid on;

plot(time,angle);
plot(time,head);

INPUT_tri = timeseries(angle,time);
OUTPUT_tri = timeseries(head,time);
%%
figure(1); plot(A(:,3));
figure(2); plot(A(:,4));