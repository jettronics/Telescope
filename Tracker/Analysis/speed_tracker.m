clear all;

% The sampling frequency in Hz.
Fs = 10;

% Sampling time
T = 1/Fs;

% Buffer length
L = 640;

% Create a time base
t = (0:L-1)*T;

v_o = zeros(2,L);
v_t = zeros(2,L);
v_c = zeros(2,L);

p_o = zeros(2,L);
p_o_sim = zeros(2,L);
p_t = zeros(2,L);

for i=1:L
  p_o_sim(1,i) = i;
  p_o_sim(2,i) = 0.05 * (p_o_sim(1,i)-340)^2 + 0.5 * (p_o_sim(1,i)-340) + 100;
end

figure(1)
clf
subplot(1,1,1)
%plot(s_hat(:,1),s_hat(:,2))
plot(p_o_sim(1,:), p_o_sim(2,:))
xlim([0 640])
ylim([0 380])
