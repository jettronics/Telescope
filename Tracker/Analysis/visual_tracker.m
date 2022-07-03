clear all;

% The sampling frequency in Hz.
Fs = 10;

% Sampling time
T = 1/Fs;

% Buffer length
L = 3000;

% Create a time base
t = (0:L-1)*T;

vmax = 0.5;

A_noise = 400.0;
s_noise = A_noise*rand(2,L)-(A_noise/2)+A_noise/3*sin(6.28*Fs/5*t);
s_shake = zeros(2,L);

A = [ 1 0 T 0; 0 1 0 T; 0 0 0 0; 0 0 0 0];
%A_hat = [ 1 0 T 0; 0 1 0 T; 0 0 1 0; 0 0 0 1];
%D = [ 0 0 0 0; 0 0 0 0; 1/T 0 0 0; 0 1/T 0 0];
%D = [ 0 0 0 0; 0 0 0 0; 0 0 1/T 0; 0 0 0 1/T];
%D = [ 1/T 0; 0 1/T];
x1_hat = [ 0; 0; 0; 0]; 
x0_hat = [ 0; 0; 0; 0]; 
x1 = [ 0; 0; 0; 0]; 
x0 = [ 0; 0; 0; 0]; 
B = [ T 0; 0 T; 0 0; 0 0]; % Camera position change matrix
C = [ 0 0; 0 0; 1 0; 0 1]; % Object speed matrix
u = zeros(2,L);
w = zeros(2,L); % Object speed
w_hat = zeros(2,L);

H = [ 1 0 0 0; 0 1 0 0];
K = [ 0 0 1 0; 0 0 0 1];
F = [ 0.1 0; 0 0.1];

s_hat = zeros(2,L);
v_hat = zeros(2,L);
v_hat_mean = zeros(2,L);

s = zeros(2,L);

e_d = zeros(2,L);
E_ds = zeros(2,L);
o_noise = [1/60; 1/60];
o_shake = [1/30; 1/30];
lost = zeros(1,L);

ys = [ 0; 0];
kp = 0.2;
%Ti = T/13;
Ti = 0;
%Td = 0.6/T;
Td = 0.9;
xold = [ 0; 0];
uin = zeros(2,L);
up = zeros(2,L);
ui = zeros(2,L);
ud = zeros(2,L);

for i=1:L
    
  %PID controller
  if(i > 250)
    up(:,i) = (kp * (ys - s_hat(:,i-1)));
    ui(:,i) = (Ti * (ys - s_hat(:,i-1)))+ui(:,i-1);
    ud(:,i) = (Td * -v_hat_mean(:,i-1));
    uin(:,i) = up(:,i)+ui(:,i)+ud(:,i);
    if(abs(uin(1,i)) >= 900)
       u(1,i) = sign(uin(1,i))*1000;
    elseif(abs(uin(1,i)) >= 700)
       u(1,i) = sign(uin(1,i))*800;
    elseif(abs(uin(1,i)) >= 500)
       u(1,i) = sign(uin(1,i))*600;
    elseif(abs(uin(1,i)) >= 300)
       u(1,i) = sign(uin(1,i))*400;
    elseif(abs(uin(1,i)) >= 100)
       u(1,i) = sign(uin(1,i))*200;
    else
       u(1,i) = 0;
    endif
    if(abs(uin(2,i)) >= 900)
       u(2,i) = sign(uin(2,i))*1000;
    elseif(abs(uin(2,i)) >= 700)
       u(2,i) = sign(uin(2,i))*800;
    elseif(abs(uin(2,i)) >= 500)
       u(2,i) = sign(uin(2,i))*600;
    elseif(abs(uin(2,i)) >= 300)
       u(2,i) = sign(uin(2,i))*400;
    elseif(abs(uin(2,i)) >= 100)
       u(2,i) = sign(uin(2,i))*200;
    else
       u(2,i) = 0;
    endif
  else
    u(:,i) = uin(:,i);  
  endif 


  
##  if i <= 500
##    u(1,i) = 0; %1.2;
##    u(2,i) = 0; %4.5;
##  elseif (i > 500) && (i < 750)
##    u(1,i) = -100; %4.2;
##    u(2,i) = -100; %0.9;
##  elseif (i >= 750) && (i < 1900)
##    u(1,i) = -60; %4.2;
##    u(2,i) = -60; %0.9;
##  else
##    u(1,i) = 0; %-2;
##    u(2,i) = 0; %-3;
##  endif
  
  if i <= 500
    w(1,i) = 100;
    w(2,i) = 100;
  elseif (i > 500) && (i < 700)
    w(1,i) = 80; %4.2;
    w(2,i) = 10; %0.9;
  elseif (i > 1500) && (i < 1900)
    w(1,i) = 60;
    w(2,i) = 10;
  else
    w(1,i) = 10;
    w(2,i) = 10;
  endif
  
  if( (i > 720) && (i < 820) )
    lost(i) = 1;
  endif
  
  if( i > 101 )
    %w_hat(:,i) = ((1/(200*T)) * (s_hat(:,i-1)+s_hat(:,i-2) - (s_hat(:,i-100)+s_hat(:,i-101)))) - ...
    %              (0.5*(u(:,i-1)+u(:,i-20)));
    %w_hat(:,i) = ((1/(200*T)) * (s_hat(:,i-1)+s_hat(:,i-2) - (s_hat(:,i-100)+s_hat(:,i-101))))- ...
    %              (1/3*(u(:,i-1)+u(:,i-2)+u(:,i-3)));
    %w_hat(:,i) = ((1/(200*T)) * (s_hat(:,i-1)+s_hat(:,i-2) - (s_hat(:,i-100)+s_hat(:,i-101))))-u(:,i-1);
    w_hat(:,i) = ((1/(100*T)) * (s_hat(:,i-1) - (s_hat(:,i-100))))-u(:,i-1);
  endif
  
  x1_hat = A * x0_hat + C * (w_hat(:,i) + u(:,i));
  xold = H * x0_hat;
  
  x1 = A * x0 + C * (w(:,i) + u(:,i));
  x0 = x1;
  
  s_hat(:,i) = H * x1_hat;
  v_hat(:,i) = K * x1_hat;
  
##  if( i >= 50 )
##    v_sum = [0; 0];
##    for k=0:49
##      v_sum = v_hat(:,i-k)+v_sum;
##    end
##    v_hat_mean(:,i) = v_sum/50;
##  endif

  if( i == 50 )
    v_sum = [0; 0];
    for k=0:49
      v_sum = v_hat(:,i-k)+v_sum;
    end
    v_hat_mean(:,i) = v_sum/50;
  elseif( i > 50 )
    v_hat_mean(:,i) = v_hat(:,i)*2/50 + v_hat_mean(:,i-1)*48/50;
  endif
  
  if( (i > 910) && (i < 1000) )
    s_shake(:,i) = 6000*sin(6.28*Fs/15*i*T);
  endif

  if( lost(i) == 1 )
    % Lost sensor
    s(1,i) = s(1,i-1);
    s(2,i) = s(2,i-1);
    %s(1,i) = 0;
    %s(2,i) = 0;
  else
    s(1,i) = x1(1) + s_noise(1,i) + s_shake(1,i);
    s(2,i) = x1(2) + s_noise(2,i) + s_shake(2,i);
  endif
      
  e_d(:,i) = s(:,i) - s_hat(:,i);

  if i >= 21
    ds_sum = [0; 0];
    for k=0:19
      ds_sum = s(:,i-k)-s(:,i-k-1)+ds_sum;
    end
    E_ds(:,i) = abs(ds_sum/20);
  endif

  E_ds_nom_n = E_ds(:,i) .* o_noise;
  
  E_ds_nom_s = E_ds(:,i) .* o_shake;
  
  if( lost(i) == 1 )
    F(1,1) = 0;
    F(2,2) = 0;
  elseif( (E_ds_nom_s(1) > 1) || (E_ds_nom_s(2) > 1) )
    %F = min(E_ds(:,i) * o',0);
    if(E_ds_nom_s(1) > 1)
      F(1,1) = 0;
    endif
    if(E_ds_nom_s(2) > 1)
      F(2,2) = 0;
    endif
  else
    %F = min(E_ds(:,i) * o',1);
    F(1,1) = min(E_ds_nom_n(1),1);
    F(2,2) = min(E_ds_nom_n(2),1);
  endif

  F(1,2) = 0;
  F(2,1) = 0;

  x0_hat = x1_hat + H' * F * e_d(:,i);

  
end

figure(1)
clf
subplot(3,1,1)
%plot(s_hat(:,1),s_hat(:,2))
plot(t,s_hat(1,:))
%xlim([60 120])
%ylim([0 7000])
ylim([-2000 10000])
hold on
subplot(3,1,2)
%plot(s(:,1),s(:,2))
plot(t,s(1,:))
%plot(t,v_hat_mean(1,:));
%xlim([60 120])
%ylim([0 7000])
ylim([-2000 10000])
hold on
subplot(3,1,3)
%plot(e(:,1),e(:,2))
plot(t,v_hat_mean(1,:))
hold on
grid on

figure(2)
clf
subplot(4,1,1)
%plot(e_i(:,1),e_i(:,2))
plot(t,E_ds(1,:)*o_noise(1))
ylim([0 1])
hold on
subplot(4,1,2)
%plot(e_i(:,1),e_i(:,2))
plot(t,E_ds(1,:)*o_shake(1))
hold on
subplot(4,1,3)
%plot(e_i(:,1),e_i(:,2))
plot(t,w(1,:))
hold on
subplot(4,1,4)
%plot(e_i(:,1),e_i(:,2))
plot(t,uin(1,:))
hold on
grid on

figure(3);
clf
%mesh(s_hat(1,:),s_hat(2,:),t);
plot(s_hat(1,:),s_hat(2,:));
hold on
grid on
