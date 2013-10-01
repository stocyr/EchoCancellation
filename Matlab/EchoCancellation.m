%% ECHO Cancellation Project 
% Parameter
td = 600;       % Delay Time in [ms]
fs = 8000;      % Sampling frequency
a = 0.4;        % Gain of the Echo Signal

% Load Signal
[sound, fswav, nbit]= wavread('Lorem_ipsum.wav');
sig = sound(1:ceil(fswav/fs):end);  % Undersampling
clearvars sound;

% Create Echo
nshift = floor(td*10^-3*fs)
echo = [zeros(nshift,1)' a*sig']';
d = [sig' zeros(nshift,1)']';
x = d + echo;

% Play signal
% soundsc(x, fs)
% plot(1:length(x), x)

%% Signal Processing
% NFIR = 200;
% y = zeros(NFIR,1);
% e = ones(NFIR,1);
% u = ones(NFIR,1);
% w = ones(NFIR,1);
% 
% % Estimation for adaptive FIR-Filter
% for n = 1:length(x)
%     y(n) = w(1)'*x(n);
%     e(n) = d(n)-y(n);
%    % u(n)=1/(x(n)'*x(n));
%    w = [w(2)+2*u(1)*e(n)*x(n) w(1:end-1)']';
% end

plot(y)
soundsc(y,fs)



