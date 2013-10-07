%% ECHO Cancellation Project 
clear all
close all
% Parameter
td = 200;       % Delay Time in [ms]
fs = 44100/6;   % Sampling frequency
a = 0.4;        % Gain of the Echo Signal

% Load Signal
[sound, fswav, nbit]= wavread('Lorem_ipsum_3500.wav');
x = sound(round(1:fswav/fs:end));  % Undersampling
clearvars sound;
soundsc(x, fs);   % play sound

% Create Echo
nshift = floor(td*10^-3*fs);

echo = [zeros(nshift,1)' a*x']';
x = [x' zeros(nshift,1)']';
y = x + echo;
soundsc(y, fs);   % play sound + echo

%% Signal Processing
NFIR = 2000;
%w = zeros(NFIR, length(x)-NFIR+1);         % soooo nicht auf ARM, 2 Spalten
w = zeros(NFIR, 1);         % soooo nicht auf ARM, 2 Spalten
u = 0.02;                   % konvergenzgeschwindigkeit
x = [zeros(NFIR,1)' x']';
y = [zeros(NFIR,1)' y']';

for k = 1:length(x)-NFIR;
    %w(:,k+1) = w(:,k) + u*x(k:k+NFIR-1)*(y(k+NFIR)-w(:,k)'*x(k:k+NFIR-1));
    w = w + u*x(k:k+NFIR-1)*(y(k+NFIR)-w'*x(k:k+NFIR-1));
end
%% Filtering
% theoretischer Filterkern:
% w = zeros(NFIR, 1);
% w(1) = 1;
% w(nshift) = a;
% w = flipud(w);

%rb_ = roots(w(:,end));
rb_ = roots(w);

rb = rb_;

% alle rb werden zu ra. ra dürfen aber nicht ausserhalb des einheitskreises
% sein. also müssen alle rb ausserhalb des einheitskreises in gleiche pole
% umgewandelt werden. dies wird gemacht, indem man den betrag genau
% invertiert und sie vom rb in die ra kiste verfrachtet.

ra = rb(find(abs(rb) >=1 ));
ra = 1/ra;
rb = rb(find(abs(rb) < 1));
x_filter = filter(rb, ra, y);

% normalize and make real
x_filter = real(x_filter)/max(abs(real(x_filter)));

%% Play sound and Plot results
soundsc(x_filter, fs);    % play sound filtered
figure

subplot(2, 1, 1);           
plot(linspace(-fs/2,fs/2,length(x_filter))', abs(fftshift(fft(x_filter))))
ylabel('Magnitude')
xlabel('Frequency [Hz]')

subplot(2, 1, 2);
plot(w)
ylabel('Magnitude')
xlabel('Filter coefficients')

% todo: u optimieren, mit referenz filterkern w ausprobieren, komplexeres
% echo generieren
