%% Frequency Sweeper - using lsqcurvefit() for better high frequency performance

clc; clearvars; close all;

input_amp = 300;

frequency=0.50:0.5:10.00;
Amplitude=zeros(1,length(frequency));
phase=zeros(1,length(frequency));
count=1;

for i=frequency
    filename="Motor_Modeling_";
    filename=append(filename,num2str(i,'%.2f'));
    filename=append(filename,"Hz_data.txt");
    
    %파일명 생성
    data = readmatrix(filename);

    Time = data(:,1);
    V_input = data(:,2);
    V_output = data(:,3);
    W_out = data(:,4);
    
    k = 298.40723112/2.5;
    W_in = k * (V_input - 2.5) ;
    
    SysResp = @(x,t)x(1)*sin(2*pi.*frequency(count).*t-x(2)+x(3)); 
    wout0 = 0.0;
    
%     fun = @(x, t)x(1)*sin(x(2)*t-x(3));

%     x0 = [input_amp, 2*pi*frequency(count), 1];

    x = lsqcurvefit(SysResp,[300;0.0;wout0],Time,V_output)
    
    eMag = x(1);
    ePhs = x(2);
    eBias = x(3);
    ewout = eMag.*sin(2.*pi*frequency.*Time+ePhs)+eBias;
    
    plot(data(:,1),W_in,data(:,1),W_out);
    grid on; box on; hold on;
    plot(data(:,1),ewout)
    
    %==============         PLOT      ========================
    output_mag(count)=eMag/input_amp;
    output_phase(count)=-ePhs
   
    %출력 체크
    figure;
%     plot(t,output,'ko',t,fun(x,t),'b-', t, input-600);
     
%     title(num2str("i1",'%.2f'))
    count=count+1;
end

%실제 전달함수 계산
b = figure;
tblFreqResp = output_mag .* exp(1i*output_phase) ; %주파수응답 오일러 꼴로 만들어줌
[num, den] = invfreqs(tblFreqResp, 2*pi*frequency, 0, ); %전달1수 계산
EstTransFunc = tf(num,den)
hold on
plot(2*pi*frequency,mag2db(output_mag),'o');
bode(EstTransFunc)
plot(2*pi*frequency,rad2deg(output_phase),'o');

fb = bandwidth(EstTransFunc)
grid on