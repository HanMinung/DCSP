clear all; close all; clc;

%--------------------------------------------------------------------------
s = tf('s');
%-----------------  Transfer Function : Gm(s) -----------------------------
% Parameter Setting

%                       K_m * (W_m)^2
%    Gm(s) =  --------------------------------
%               s^2 + 2*Zt_m*W_m*s + (W_m)^2    

W_m = 1;
K_m = 1;
Zt_m = 1/sqrt(2);

%--------------------------------------------------------------------------

% Setting : 값이 정해지면 PID 제어 계수값들은 모두 Fix 된다.
Zt_c = 1/sqrt(2);
W_c = 1;
R_c = 1;              % 1st order pole

%--------------------------------------------------------------------------

% Zt_c * W_c에 의해서 2차 conjugate pole들이 결정
% R_c에 의해서 1차 pole이 결정
% if : abs(Zt_c * W_c) >> abs(R_c) --> Can approxiamte First order
% if : abs(Zt_c * W_c) << abs(R_c) --> Can approxiamte Second order

%-----------------  Transfer Function : Controller Gc(s) ------------------
%               s^2Kd + sKp + Ki
%    Gc(s) =  --------------------
%                       s    

Ki = ( R_c*W_c^2 )/( K_m*W_m^2 );
Kp = ((W_c)^2 - (W_m)^2 + 2*Zt_c*W_c*R_c )/( K_m*(W_m)^2 );
Kd = (2*Zt_c*W_c - 2*Zt_m*W_m + R_c)/( K_m*(W_m)^2 );

%--------------------------------------------------------------------------
% Definition of Gm(s)

num1 = [K_m * (W_m)^2];
den1 = [1 2*Zt_m*W_m (W_m)^2];

G_m = tf(num1, den1);

%--------------------------------------------------------------------------
% Definition of Gc(s) : PID controller

num2 = [Kd Kp Ki];
den2 = [1 0];

G_c = tf(num2, den2);

%--------------------------------------------------------------------------
% Definition of Gcl(s) : Closed loop Transfer Function

G_cl = feedback(G_m*G_c,1);

%--------------------------------------------------------------------------
%% : BODE PLOT

figure(1);
bode(G_m);
title("G_m"); grid on; box on;
figure(2);
bode(G_c);
title("G_c"); grid on; box on;
figure(3);
bode(G_cl);
title("G_cl"); grid on; box on;

%--------------------------------------------------------------------------
%% : Pole - Zero MAP   ---   Properties of Transfer function

% if : abs(Zt_c * W_c) >> abs(R_c) --> Can approxiamte First order TF
% if : abs(Zt_c * W_c) << abs(R_c) --> Can approxiamte Second order TF
% W_c 값을 점점 키운다 --> faster second order conjugate poles --> 1차 시스템으로 근사 
% bodeplot 해보자

tbllegend=[{'1'},{'2'},{'3'},{'4'},{'5'}];
figure(5),clf

for a = 1 : 1 : 5
    R_c = a * W_c ;
    
    % NEED UPDATE
    Ki = ( R_c*W_c^2 )/( K_m*W_m^2 );
    Kp = ((W_c)^2 - (W_m)^2 + 2*Zt_c*W_c*R_c )/( K_m*(W_m)^2 );
    Kd = (2*Zt_c*W_c - 2*Zt_m*W_m + R_c)/( K_m*(W_m)^2 );
    
    num2 = [Kd Kp Ki];
    den2 = [1 0];

    G_c = tf(num2, den2);
    G_cl = feedback(G_m*G_c,1);
    
    pzmap(G_cl);
    if a==5,legend(tbllegend);end
    hold on;
    grid on;
    
end

%--------------------------------------------------------------------------
%% : Step Response 

Buf_Rising_time = zeros(5,1);
Buf_Overshoot = zeros(5,1);

tbllegend=[{'1'},{'2'},{'3'},{'4'},{'5'}];
figure(5),clf

for a = 1 : 1 : 5
    R_c = a * W_c ;
    
    % NEED UPDATE
    Ki = ( R_c*W_c^2 )/( K_m*W_m^2 );
    Kp = ((W_c)^2 - (W_m)^2 + 2*Zt_c*W_c*R_c )/( K_m*(W_m)^2 );
    Kd = (2*Zt_c*W_c - 2*Zt_m*W_m + R_c)/( K_m*(W_m)^2 );
    
    num2 = [Kd Kp Ki];
    den2 = [1 0];

    G_c = tf(num2, den2);
    
    G_cl = feedback(G_m*G_c,1);
    
    step(G_cl);
    if a==5,legend(tbllegend);end
    
    % Check Rise Time
    Buf_Rising_time(a,1) = getfield(stepinfo(G_cl),'RiseTime');
    
    % Check Overshoot
    Buf_Overshoot(a,1) = getfield(stepinfo(G_cl),'Overshoot');
    
    hold on;
    grid on;
   
end

% 결과 : a 값이 커짐에 따라 rise time은 감소한다.
% 점점 1차 시스템으로 근사되면서 rising time이 감소해야함

%--------------------------------------------------------------------------

