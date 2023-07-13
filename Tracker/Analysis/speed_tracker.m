clear all;

% The sampling frequency in Hz.
Fs = 10;

% Sampling time
T = 1/Fs;

% Buffer length
L = 1000;

% Create a time base
t = (0:L-1)*T;

v_o = zeros(1,L);
v_t = zeros(1,L);
v_c = zeros(1,L);

p_t = zeros(1,L); % -> p_c

p_o = sin((6.28*Fs/500*t)+1.25);

T_c = 5;

for i=1:L
  v_c(i) = (p_o(i) - p_t(i))/T_c;
  if i > 1
    v_o(i) = ((p_o(i) - p_t(i)) - (p_o(i-1) - p_t(i-1)))/T;
  endif
end

figure(1)
clf
subplot(1,1,1)
plot(t, v_c, 'b')
hold
plot(t, v_o, 'g')
hold
%xlim([0 640])
%ylim([-2 2])

figure(2)
clf
subplot(1,1,1)
plot(t, p_o)
hold
