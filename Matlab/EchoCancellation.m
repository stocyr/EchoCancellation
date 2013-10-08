%% ECHO Cancellation Project 
clear all
close all;
% Parameter
td = 200E-3;       % Delay Time in [s]
fs = 44100/6;      % Sampling frequency
a = 0.4;           % Gain of the Echo Signal
tk = 1/100;        % Lokale Korrelation am Anfang
u = 0.0002;          % konvergenzgeschwindigkeit

% Load Signal
[sound, fswav, nbit]= wavread('Lorem_ipsum_3500.wav');
x = sound(round(1:fswav/fs:end));  % Undersampling
clearvars sound;
soundsc(x, fs);   % play sound

x=randn(size(x));

% Create Echo
nshift = floor(td*fs);

g = zeros(size(x));
g(1) = 1;
g(10) = a;
y=filter(g,1,x);

 soundsc(y, fs);   % play sound + echo

%% Signal Processing
NFIR = 20;
%deltak = floor(tk*fs);
deltak = 1;
w = zeros(NFIR - deltak, 1);
err = zeros(size(x));


for k = NFIR:length(x);
    %w(:,k-NFIR+1) = w(:,k) + u*x(k-deltak:-1:k-NFIR+1)*(err(k));
    err(k)=y(k)-w'*x(k-deltak:-1:k-NFIR+1);
    w = w + u*x(k-deltak:-1:k-NFIR+1)*(err(k));
end

plot(err, 'r--');
hold on;
plot(x);

figure;
plot([zeros(deltak); w]);
hold all;
plot(g(1:length(w)));
