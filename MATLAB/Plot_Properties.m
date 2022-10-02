%% PLOTTING : Rising time, %os , phase margin, gain margin
clear all; close all; clc;

s = tf('s');

Num_m = [0 0 3126];
Den_m = [1 95.29 3651];
G_motor = tf(Num_m , Den_m);

%-------------------------------------------------------------------------%
W_m = sqrt(Den_m(3)) ;
K_m = 3126/Den_m(3) ;
Zt_m = 95.29/(2*W_m) ;

Zt_c = 1/sqrt(2);

%-------------------------------------------------------------------------%
c = 0:0.001:1.5;

Buf_Rising_time = zeros(length(c),4);
Buf_Overshoot = zeros(length(c),4);
Buf_gm = zeros(length(c),4);
Buf_pm = zeros(length(c),4);

%-------------------------------------------------------------------------%
count = 1;

for alpha = 2:1:5
    
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
    
        Buf_Rising_time(count,alpha-1) = stepinfo(G_cl).RiseTime;
        Buf_Overshoot(count,alpha-1) = stepinfo(G_cl).Overshoot;
        [gm,pm] = margin(G_o);
    
        if ~isnan(pm)
        Buf_gm(count,alpha-1) = gm;
        end
    
        if ~isnan(gm)
        Buf_pm(count,alpha-1) = pm;
        end
    
    count = count+1;
    
    end
    
    count = 1;  % 1로 초기화 필요 !
end

figure(1); 
plot(c,Buf_Rising_time(:,1),'r-','linewidth',1);  hold on;
plot(c,Buf_Rising_time(:,2),'g-','linewidth',1);  hold on;
plot(c,Buf_Rising_time(:,3),'b-','linewidth',1);  hold on;
plot(c,Buf_Rising_time(:,4),'k-','linewidth',1);  hold on;
grid on; box on;
title("Rising Time Plot",'Fontsize',20);
xlabel("W_c/W_m",'Fontsize',20);
ylabel("Rising Time[sec]",'Fontsize',20);
legend('alpha = 2','alpha = 3','alpha = 4','alpha = 5','Fontsize',18);
xlim([0.1 1.5]);

figure(2); 
plot(c,Buf_Overshoot(:,1),'r-','linewidth',1);   hold on;
plot(c,Buf_Overshoot(:,2),'g-','linewidth',1);   hold on;
plot(c,Buf_Overshoot(:,3),'b-','linewidth',1);   hold on;
plot(c,Buf_Overshoot(:,4),'k-','linewidth',1);   hold on;
grid on; box on;
title("Overshoot Plot",'Fontsize',20);
xlabel("W_c/W_m",'Fontsize',20);
ylabel("Overshoot[%]",'Fontsize',20);
legend('alpha = 2','alpha = 3','alpha = 4','alpha = 5','Fontsize',18);
xlim([0.1 1.5]);
ylim([0 25]);


figure(3); 
plot(c,Buf_gm(:,1),'r-','linewidth',1);  hold on;
plot(c,Buf_gm(:,2),'g-','linewidth',1);  hold on;
plot(c,Buf_gm(:,3),'b-','linewidth',1);  hold on;
plot(c,Buf_gm(:,4),'k-','linewidth',1);  hold on;
grid on; box on;
title("Gain margin Plot",'Fontsize',20);
xlabel("W_c/W_m",'Fontsize',20);
ylabel("Gain margin[dB]",'Fontsize',20);
legend('alpha = 2','alpha = 3','alpha = 4','alpha = 5','Fontsize',18);
xlim([0.1 0.6]);
ylim([0 20]);

figure(4); 
plot(c,Buf_pm(:,1),'r-','linewidth',1);   hold on;
plot(c,Buf_pm(:,2),'g-','linewidth',1);   hold on;
plot(c,Buf_pm(:,3),'b-','linewidth',1);   hold on;
plot(c,Buf_pm(:,4),'k-','linewidth',1);   hold on;
grid on; box on;
title("Phase margin plot",'Fontsize',20);
xlabel("W_c/W_m",'Fontsize',20);
ylabel("Phase margin[deg]",'Fontsize',20);
legend('alpha = 2','alpha = 3','alpha = 4','alpha = 5','Fontsize',18);
xlim([0.1 1.5]);