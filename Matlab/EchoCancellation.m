%% ECHO Cancellation Project 
% Parameter
td = 200;       % Delay Time in [ms]
fs = 8000;      % Sampling frequency
a = 0.4;        % Gain of the Echo Signal

% Load Signal
[sound, fswav, nbit]= wavread('Lorem_ipsum_4000.wav');
x = sound(round(1:fswav/fs:end));  % Undersampling
clearvars sound;

% Create Echo
nshift = floor(td*10^-3*fs)

echo = [zeros(nshift,1)' a*x']';
x = [x' zeros(nshift,1)']';
y = x + echo;


% Play signal
%  soundsc(y, fs)
% plot(1:length(y), y)

%% Signal Processing
NFIR = 2000;
w = zeros(NFIR, 1);         % soooo nicht auf ARM, 2 Spalten
u = 0.01;                   % konvergenzgeschwindigkeit
x = [zeros(NFIR,1)' x']';
y = [zeros(NFIR,1)' y']';

for k = 1:length(x)-NFIR;
w = w + u*x(k:k+NFIR-1)*(y(k+NFIR)-w'*x(k:k+NFIR-1));
end

w  = w(end:-1:1);
rb_=roots(flipud(w));


ra = rb_(find(abs(rb_) <=1 ));
ra = 1./ra;
rb = rb_(find(abs(rb_) > 1));
x_filter = filter(ra, rb, x);
soundsc(100*real(x_filter), 8000);

% R_xx = xcorr(x,x);
% plot(R_xx)
% plot(y)
% soundsc(y,fs)
