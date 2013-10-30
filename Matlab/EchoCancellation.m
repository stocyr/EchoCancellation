%% ECHO Cancellation Project 
clear all
close all;
% Parameter
td = 200E-3;        % Delay Time in [s]
fs = 44100/6;       % Sampling frequency
a = 0.4;            % Gain of the Echo Signal
tk = 10E-3;         % Lokale Korrelation am Anfang [s]
u = 0.0285;         % konvergenzgeschwindigkeit
NFIR = 2000;        % Filterlenght 
deltak = floor(tk*fs);  % Ignored Samples at the beginning


w_global = true;   % if true: all values of the filter are stored, 
                    % otherwise the actual sample is overwritten

% Load Signal
[sound, fswav, nbit]= wavread('Lorem_ipsum_3500.wav');
x = sound(round(1:fswav/fs:end));  % Undersampling
x = [x; x];
clearvars sound;
%soundsc(x, fs);   % play sound
u = 2/((NFIR+1)*std(x(3000:3400))^2);
%x=randn(size(x));

% Create Echo
nshift = floor(td*fs);

g = zeros(size(x));
g(1) = 1;
g(nshift) = a;
y=filter(g,1,x);

soundsc(y, fs);   % play sound + echo

%% Signal Processing

if w_global == true
    w = zeros(NFIR - deltak, length(x) - NFIR+2);
    w(1470-deltak,:) = 0.4;
else
    w = zeros(NFIR - deltak, 1);
end

err = zeros(size(x));

for k = NFIR:length(x);
    if w_global == true
        err(k)=y(k)-w(:,k-NFIR+1)'*x(k-deltak:-1:k-NFIR+1);
        w(:,k-NFIR+2) = w(:,k-NFIR+1) + u*x(k-deltak:-1:k-NFIR+1)*(err(k));
    else
        err(k)=y(k)-w'*x(k-deltak:-1:k-NFIR+1);
        w = w + u*x(k-deltak:-1:k-NFIR+1)*(err(k));
    end
end

%%
plot(err);
hold all;
plot(x);
title('Error over Time')
legend('echo-cancelled signal', 'original signal');
xlabel('Time [s]');
ylabel('Amplitude');

%%
if w_global == true
    figure
    opengl software;
    surf(w(7:10:end,1:500:end));   % , 'EdgeColor', 'none', 'LineStyle', 'none', 'FaceLighting', 'phong'
    title('Convergence of Filter')
    ylabel('Coeffitients');
    xlabel('Time [s]');
    set(gca, 'YDir', 'reverse')
    figure;
    plot(g(1:NFIR-deltak));
    stem(find(g(1:NFIR-deltak)>0), g(find(g(1:NFIR-deltak)>0)), 'linewidth', 2);
    hold all;
    plot([zeros(deltak, 1); w(:,end)], 'linewidth',2);
    title('Final Filter Coeffitients')
    legend('Original Echo', 'Reproduced Echo');
    xlabel('Coeffitients');
    grid on
    
else
    figure;
    plot([zeros(deltak,1); w]);
    hold all;
    plot(g(1:length(w)));
end

soundsc(err, fs);   % error signal

%%
% demonstration
soundsc(y(1:end/2), fs);   % play sound + echo
soundsc(err(1:end/2), fs);   % play sound with echo cancellation

%%
% grafik für PPT:

figure;
g_arm = zeros(1600,1);
g_arm(1) = 1;
g_arm(400) =1/2;
g_arm(800) = 1/4;
g_arm(1200) = 1/8;
g_arm(1600) = 1/16;

plot(g_arm);
stem(find(g_arm>0), g_arm(find(g_arm>0)), 'linewidth', 2);
title('Echo Filter Coeffitients')
xlabel('Coeffitients');
grid on
