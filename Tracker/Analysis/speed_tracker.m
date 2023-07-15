clear all;

% The sampling frequency in Hz.
Fs = 10;

% Sampling time
T = 1/Fs;

% Buffer length
L = 1000;

% Create a time base
t = (0:L-1)*T;

v_o_in = zeros(1,L);
v_o_out = zeros(1,L);
v_o_mean = zeros(1,L);
v_t = zeros(1,L);

%p_t = zeros(1,L); % -> p_c
p_t = 0;
p_o_in = (10*sin((6.28*Fs/500*t)+1.25))+10;
p_o_out = zeros(1,L);

T_c = 5;

p_o_out(1) = p_o_in(1);

for i=1:L-1
  if i > 1
    % System behavior
    v_o_in(i) = (p_o_in(i) - p_o_in(i-1))/T;
    p_o_out(i) = ((v_o_in(i) - v_t(i)) * T) + p_o_out(i-1) - p_t;
    % System behavior end

    %v_t(i+1) = (p_o_out(i) - p_t)/T_c;
    v_o_out(i) = (p_o_out(i) - p_o_out(i-1) + p_t + (v_t(i)*T))/T;
    v_t(i+1) = v_o_out(i)+((p_o_out(i) - p_t)/T_c);
  endif
end

figure(1)
clf
subplot(2,1,1)
plot(t, p_o_in, 'b')
hold
plot(t, p_o_out, 'g')
hold
subplot(2,1,2)
plot(t, v_o_in, 'b')
hold
plot(t, v_o_out, 'g')
hold

%xlim([0 640])
%ylim([-2 2])



