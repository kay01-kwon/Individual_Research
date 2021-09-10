clear all;
clc;
close all;

%  System model example

% dxdt = A*x + B1*u + B2*d
% z = C*x + D1*u + D2*d
% x: State vector
% z: Output

A = [0 1;
    0 -10];

B1 = [0;1];

B2 = [0;1];

C = eye(2);

D1 = 0;

D2 = 0;

% Set Linear Matrix Inequality
setlmis([]);

% Set LMI variables: X(2 x 2), W = K(1 x 2)*X(2 x 2), gamma(1 x 1)
% lmivar info: lmivar(type,struct) 
% type = 1: symmetric matrix
% type = 2: m x n rectangular matrix
% type = 3: Specified matrix
[X nX sX] = lmivar(1,[2 1]);                     % 2 x 2 LMI variable
[gamma ngamma sgamma] = lmivar(1,[1 1]);         % 1 x 1 LMI variable
[W nW sW] = lmivar(2,[1 2]);                     % 1 x 2 LMI variable

% Construct LMI constraint to satisfy || G(s) ||_inf < gamma
% lmiterm info: lmiterm(termID, A, B)
% The bellow LMI matrix have to be symmetric negative definite.
% min gamma s.t. X > 0
% [ (A*X+X*A')+(B1*K+K'*B1')    B2              (C*X+D1*W)';
%   *                           -gamma*I        D2';
%   *                           *               -gamma*I];

lmiterm([-1 1 1 X],1,1)

lmiterm([2 1 1 X],A,1,'s')              % Partitioned matrix (1,1): A*X + X*A'
lmiterm([2 1 1 W],B1,1,'s')             % Partitioned matrix (1,1): B1*W + W'*B1'

lmiterm([2 1 2 0],B2);                  % Partitioned matrix (1,2): B2

lmiterm([2 1 3 X],1,C');                % Partitioned matrix (1,3): X*C'
lmiterm([2 1 3 -W],1,D1');               % Partitioned matrix (1,3): W'*D1'

lmiterm([2 2 2 gamma],-1,1);            % Partitioned matrix (2,2): -gamma*I

lmiterm([2 2 3 0],D2');                 % Partitioned matrix (2,3): D2'

lmiterm([2 3 3 gamma],-1,1);            % Partitioned matrix (3,3): -gamma*I


LMIs = getlmis;
n = decnbr(LMIs);

c = 5e-7*ones(n,1);
c(4) = 1;

[copt xopt] = mincx(LMIs,c); 
gamma_opt = dec2mat(LMIs,xopt,gamma);
X_opt = dec2mat(LMIs,xopt,X);
W_opt = dec2mat(LMIs,xopt,W);

gamma_opt
% X_opt
% W_opt
K_opt = W_opt*inv(X_opt)

% [tmin xmin] = feasp(LMIs);
% gamma_opt = dec2mat(LMIs,xmin,gamma);
% X_ = dec2mat(LMIs,xmin,X);
% W_ = dec2mat(LMIs,xmin,W);
% K_ = W_*inv(X_);
eigens = [-10+i;-10-i];
K_ = place(A,B1,eigens)

%%
t_span = 0:0.01:100;
x0 = [0;0];
x_ref = [1;0];

D = 5;
w = 1;

[t x_hinf] = ode45(@(t,x_hinf)motor_dynamics(x_hinf-x_ref,K_opt,D*cos(w*t)),t_span,x0);

[t_,x_] = ode45(@(t_,x_)motor_dynamics(x_-x_ref,-K_,D*cos(w*t_)),t_span,x0);
 
subplot(2,2,1)
plot(t,x_hinf(:,1),'LineWidth',2);
hold on;
grid on;
plot(t_,x_(:,1),'LineWidth',2)
plot(t,x_ref(1)*ones(length(t),1),'k');
legend('H_\infty','Feedback')

subplot(2,2,2)
plot(t,x_hinf(:,2),'LineWidth',2);
hold on;
grid on;
plot(t_,x_(:,2),'LineWidth',2)
plot(t,x_ref(2)*ones(length(t),1),'k');
legend('H_\infty','Feedback')
