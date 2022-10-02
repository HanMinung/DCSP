%% Controller Design
clear all; close all; clc;

s = tf('s');

%-------------------------------------------------------------------------%
%                                K_m*(W_m)^2
%    G_motor(s) =  --------------------------------------
%                       s^2 + 2*Zt_m*W_m*s + (W_m)^2    

% Num_m = [0 0 2677];
% Den_m = [1 113.4 3485];

Num_m = [0 0 3126];
Den_m = [1 95.29 3651];
%-------------------------------------------------------------------------%
W_m = sqrt(Den_m(3)) ;
K_m = 3126/Den_m(3) ;
Zt_m = 95.29/(2*W_m) ;

% Setting : 값이 정해지면 PID 제어 계수값들은 모두 Fix 된다.
Zt_c = 1/sqrt(2);
W_c = 0.4441 * W_m;
% R_c = 1;

%-------------------------------------------------------------------------%
tbllegend=[{'W_c=20'},{'W_c=25'},{'W_c=30'},{'W_c=35'},{'W_c=40'},{'W_c=45'},{'7'},{'8'},{'9'},{'10'},{'11'}];
figure(5),clf

Point_Start = 25; 
Point_Final = 45;
Point_Interval = 5;

for W_c = Point_Start : Point_Interval : Point_Final
    
    R_c = 3 * W_c ;

    % NEED UPDATE
    K_i = ( R_c*W_c^2 )/( K_m*W_m^2 );
    K_p = ((W_c)^2 - (W_m)^2 + 2*Zt_c*W_c*R_c )/( K_m*(W_m)^2 );
    K_d = (2*Zt_c*W_c - 2*Zt_m*W_m + R_c)/( K_m*(W_m)^2 );
    
    Num_c = [K_d K_p K_i];
    Den_c = [1 0];

    G_controller = tf(Num_c, Den_c);
    G_motor = tf(Num_m, Den_m);
    G_cl = feedback(G_motor*G_controller,1,-1);
    
    [y, t] = step(G_cl, 0.5);
    figure(1);
    plot(t,y,'LineWidth',2);  hold on;
    grid on;
    xlabel("Time[sec]",fontsize=20);
    ylabel("Amplitude [-]",fontsize=20);
    title ("Step Response",fontsize=20);
%     pzmap(G_cl);
%     bode(G_cl);
%     margin(G_cl);
%     Nyquist(G_cl);

    if W_c==Point_Final,legend(tbllegend);end
    
    hold on;
    grid on;
    set(gca,'Fontsize',14);
end

%% PLOTTING : Rising time, %os , phase margin, gain margin
clear all; close all; clc;

s = tf('s');

%-------------------------------------------------------------------------%
%                                K_m*(W_m)^2
%    G_motor(s) =  --------------------------------------
%                       s^2 + 2*Zt_m*W_m*s + (W_m)^2    

Num_m = [0 0 3126];
Den_m = [1 95.29 3651];
G_motor = tf(Num_m , Den_m);

%-------------------------------------------------------------------------%
W_m = sqrt(Den_m(3)) ;
K_m = 3126/Den_m(3) ;
Zt_m = 95.29/(2*W_m) ;

Zt_c = 1/sqrt(2);

%-------------------------------------------------------------------------%
c = 0:0.001:1;

Buf_Rising_time = zeros(length(c),1);
Buf_Overshoot = zeros(length(c),1);
Buf_gm = zeros(length(c),1);
Buf_pm = zeros(length(c),1);

%-------------------------------------------------------------------------%
count = 1;

for b = c
    
    W_c = b * W_m;
    R_c = alpha * W_c;          % Alpha value : 2,3,5
    
    K_i = ( R_c*W_c^2 )/( K_m*W_m^2 );
    K_p = ((W_c)^2 - (W_m)^2 + 2*Zt_c*W_c*R_c )/( K_m*(W_m)^2 );
    K_d = (2*Zt_c*W_c - 2*Zt_m*W_m + R_c)/( K_m*(W_m)^2 );
    
    Num_c = [K_d K_p K_i];
    Den_c = [1 0];

    G_controller = tf(Num_c, Den_c);
    G_o = G_controller * G_motor;
    
    G_cl = feedback(G_motor*G_controller,1,-1);
    
    Buf_Rising_time(count) = stepinfo(G_cl).RiseTime;
    Buf_Overshoot(count) = stepinfo(G_cl).Overshoot;
    [gm,pm] = margin(G_o);
    
    if ~isnan(pm)
        Buf_gm(count) = gm;
    end
    
    if ~isnan(gm)
        Buf_pm(count) = pm;
    end
    
    count = count+1;
end

figure(1); plot(c,Buf_Rising_time,'linewidth',2);
grid on;
title("Rising Time Plot",'Fontsize',14);
xlabel("W_c/W_m",'Fontsize',14);
ylabel("Rising Time[sec]",'Fontsize',14);

figure(2); plot(c,Buf_Overshoot,'linewidth',2);
grid on;
title("Overshoot Plot",'Fontsize',14);
xlabel("W_c/W_m",'Fontsize',14);
ylabel("Overshoot[%]",'Fontsize',14);

figure(3); plot(c,Buf_gm,'linewidth',2);
grid on;
title("Gain margin Plot",'Fontsize',14);
xlabel("W_c/W_m",'Fontsize',14);
ylabel("Gain margin[dB]",'Fontsize',14);

figure(4); plot(c,Buf_pm,'linewidth',2);
grid on;
title("Phase margin plot",'Fontsize',14);
xlabel("W_c/W_m",'Fontsize',14);
ylabel("Phase margin[deg]",'Fontsize',14);


%% STEP_response Validation & Nyquist plot
clear all; close all; clc;

s = tf('s');

Num_m = [0 0 3126];
Den_m = [1 95.29 3651];

W_m = sqrt(Den_m(3)) ;
K_m = 3126/Den_m(3) ;
Zt_m = 95.29/(2*W_m) ;

Zt_c = 1/sqrt(2);
W_c =  0.70 * W_m;       % Minimum value : 0.441
R_c = 3 * W_c;

K_i = ( R_c*W_c^2 )/( K_m*W_m^2 );
K_p = ((W_c)^2 - (W_m)^2 + 2*Zt_c*W_c*R_c )/( K_m*(W_m)^2 );
K_d = (2*Zt_c*W_c - 2*Zt_m*W_m + R_c)/( K_m*(W_m)^2 );

Num_c = [K_d K_p K_i];
Den_c = [0 1 0];

G_motor = tf(Num_m,Den_m);
G_controller = tf(Num_c,Den_c);

G_o = G_motor * G_controller;
G_cl = feedback(G_motor*G_controller,1,-1); 
[y, t] = step(G_cl, 0.5);

figure(1);
plot(t,y,'LineWidth',1);
grid on;
xlabel("Time[sec]",'fontsize',20);
ylabel("Amplitude [-]", 'fontsize',20);
title ("Step Response", 'fontsize',20);
legend("Step response : Gcl",'Fontsize',20)

figure(2);
nyquist(G_o); grid on; box on;

figure(3);
H_2 = 1/(1+G_controller*G_motor);
bode(H_2); grid on; box on;

pzmap(G_cl); grid on;


%% Call Simulink data : DESIGN MODEL

Simdata = sim('LoopStructure_Designmodel.slx');
Time   = Simdata.time;
Output = Simdata.output;

plot(Time,Output,'linewidth',1.5);
grid on; box on;
xlabel("Time [sec]",'Fontsize',20);
ylabel("Magnitude [-]",'Fontsize',20);
legend("Input","Disturbance","Output",'Fontsize',20);
title("Disturbance rejection analysis",'Fontsize',20);


%% Pole placement

G_cl = feedback(G_motor*G_controller,1,-1);
figure
pzmap(G_cl); sgrid(1/sqrt(2),[26.0339, 118.0678])
xlim([-200 0])

%%
num = []
den = []
