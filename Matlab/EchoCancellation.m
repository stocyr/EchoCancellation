%% ECHO Cancellation Project 
% Parameter
td = 600;       % Delay Time in [ms]
fs = 8000;      % Sampling frequency
a = 0.4;        % Gain of the Echo Signal

% Load Signal
[sound, fswav, nbit]= wavread('Lorem_ipsum.wav');
sig = sound(1:ceil(fswav/fs):end);  % Undersampling

% Create Echo
nshift = floor(td*10^-3*fs)
echo = [zeros(nshift,1)' a*sig']';
sig = [sig' zeros(nshift,1)']';
sig = sig + echo;

% Play signal
soundsc(sig, fs)
plot(1:length(sig), sig)