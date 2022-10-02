%% Frequency Sweep & MODELING : ROUGH Bode Plot

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%                    CURVE FITTING                        %%%%

clc; clearvars; close all;
options = optimoptions('lsqcurvefit','Algorithm','levenberg-marquardt');

frequency=1.0:1.0:9.0;
Amplitude=zeros(1,length(frequency));
phase=zeros(1,length(frequency));

count = 1;

% plot(A(:,1),(1.247515/3.24975)*A(:,2)-0.2052496873,'Color','r','LineWidth',0.5);

for i=frequency
    filename="Modeling_sine_";
    filename=append(filename,num2str(i,'%.1f'));
    filename=append(filename,"Hz_data.txt");
    
%     fig1 = figure;
    data = readmatrix(filename);
%     set(fig1, 'OuterPosition', [3, 270, 500, 420])
    
    Time=data((500:2000),1);
    V_input= data((500:2000),2);
    V_output = data((500:2000),3);
    W_out = data((500:2000),4);
    
    k = 300/2.50;
    W_in = k * (V_input - 2.50) ;

    SysResp=@(x,t) x(1)*sin(2*pi*i*Time-x(2))+x(3);
    x0=[300,0.0,0.0];                                    %Amplitude, phase delay, offset
    x=lsqcurvefit(SysResp,x0,Time,W_out,[],[],options);  % non - linear least sauqres fit

    eMag = x(1) ;               % [V] : estimated magnitude of Vout
    ePhs = x(2) ;               % [rad] : estimated phase shift of Vout
    eBias = x(3) ;              % [V] : estimated value of Vout0
    ewout= eMag*sin(2*pi*frequency.*Time-ePhs) + eBias;

    figure()
    plot(Time, W_in,'b','LineWidth',0.005);
    hold on;
    plot(Time,ewout(:,i),'color','r','LineWidth',1.5);hold on;
    plot(Time,W_out,'Color','b','LineWidth',1.5);
    
    Amplitude(count) = eMag;
    phase(count) = -ePhs;
    count=count+1;
end

% f1=figure(1);
% subplot(2,1,1);
% plot((frequency),Amplitude,'ro');
% set(gca,'Xscale','log');
% set(gca,'Yscale','log');
% subplot(2,1,2);
% plot((frequency),rad2deg(-phase),'bo');
% set(gca,'Xscale','log');
% hold on; grid on;

tblOmega =2*pi*frequency; % [rad/s]
tblMagAtt = Amplitude/300.0 ; % [-] : non-dimensional
tblPhsDelay = phase ; % [rad] : note that tblPhsDelay generally, has negative values
tblFreqResp = tblMagAtt .* exp(1j*tblPhsDelay) ;

% [num,den]=invfreqs(tblFreqResp, tblOmega, 0, 2,[],30);
[num,den]=invfreqs(tblFreqResp, tblOmega, 0, 2);

model=tf(num,den);

figure()
bode(model);
grid on;


%% EXACT MODELING

clear all; close all; clc;
options = optimoptions('lsqcurvefit','Algorithm','levenberg-marquardt');

frequency=0.1:0.1:3.00;
Amplitude=zeros(1,length(frequency));
phase=zeros(1,length(frequency));
count=1;

for i=frequency
    filename="Modeling_sine_";
    filename=append(filename,num2str(i,'%.1f'));
    filename=append(filename,"Hz_data.txt");

    data = readmatrix(filename);

    Time=data((500:2000),1);
    V_input= data((500:2000),2);
    V_output = data((500:2000),3);
    W_out = data((500:2000),4);
    
    k = 295.67/2.50;
    W_in = k * (V_input - 2.50) ;

    SysResp=@(x,t) x(1)*sin(2*pi*i*Time-x(2))+x(3);
    x0=[300,0.0,0.0];                                        %Amplitude, phase delay, offset
    x=lsqcurvefit(SysResp,x0,Time,W_out,[],[],options);      % non - linear least sauqres fit
    
    eMag = x(1) ;               % [V] : estimated magnitude of Vout
    ePhs = x(2) ;               % [rad] : estimated phase shift of Vout
    eBias = x(3) ;              % [V] : estimated value of Vout0
    ewout= eMag*sin(2*pi*frequency.*Time+ePhs) + eBias;
    
%     figure;
%     plot(Time,fun(x,Time),'Color','r','LineWidth',2.5);
%     hold on;
%     plot(Time,data1,'b','LineWidth',0.005);
    hold on;

%     plot(time,data1,'Color','b','LineWidth',2.5);
    Amplitude(count)=x(1)/295.67;
    phase(count)=x(2);
    count=count+1;
end

num = [3126];
den = [1.0000   95.29  3651];
model=tf(num,den);

plot((2*pi*frequency),20*log10(Amplitude),'x');
set(gca,'Xscale','log');
% set(gca,'Yscale','log');
hold on;
bode(model);
hold on;

plot((2*pi*frequency),rad2deg(-phase),'x');
set(gca,'Xscale','log');
hold on;
bode(model);
hold on; grid on;
