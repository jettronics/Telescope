clear all;

% The sampling frequency in Hz.
Fs = 10;

% Sampling time
T = 1/Fs;

% Buffer length
L = 3000;

% Create a time base
t = (0:L-1)*T;

v_o = zeros(2,L);
v_t = zeros(2,L);
v_c = zeros(2,L);

p_o = zeros(2,L);
p_t = zeros(2,L);


