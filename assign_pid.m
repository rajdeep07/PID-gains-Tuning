

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Digital Control of Automotive Powertrains - Spring 2019
%
% Model of I.C. engine dynamics for idle speed control.
% 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
% clear
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load engine geometric parameters and constant inputs

Vd = 2.4e-3;   % Displacement (m^3)
Z = 4;         % Number of Cylinders
Vm = 5.8e-3;   % Intake Manifold Volume (m^3)
J = 0.0789;    % Mass moment of inertia

p_amb = 1.0121*1e5;
T_amb = 302;
R=288;
gam = 1.35;

P0 = 26431;   % Initial MAP
N0 = 828;     % Initial RPM

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Model parameters (from steady-state calibration)

a = [1.69e-07,-1.136e-06,6.89e-06];  % Throttle
si = 0.812;   yi = 0.0633;           % Volumetric efficiency
P_IMEP = [0.0220,-1.1649];           % IMEP
par_sp = [-0.0017 -0.0277 1.36];     % Spark timing effect
par_fr = [7.4198e-7 -4.989e-4 11.3]; % Friction
par_IMEP0 = [1.2323e-4 2.1256];      % Base IMEP model

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Conversion to Crank Angle Domain

% Coefficients for crank-angle based model
Kthr = p_amb/sqrt(R*T_amb)*sqrt(gam)*sqrt((2/(gam+1))^((gam+1)/(gam-1)));
Kp1 = R*T_amb/Vm;
Kp2 = si*Vd/(4*pi*Vm);
Kp3 = yi*Vd/(4*pi*Vm);
KT = 1e5*Vd/(4*pi);
Kfr1 = (30/pi)^2 * par_fr(1);
Kfr2 = (30/pi) * par_fr(2);
Kfr3 = par_fr(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate Equilibrium Condition (p_0 T_0 w_0)

setp(1) = 9.81; % Throttle Position
setp(2) = -25;  % Spark Timing
setp(3) = 10;   % Load Torque
X0 = [26424 21.3765773202354 83.9019428270409]; % Equilibrium Conditions

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Linearization

% Coefficients for linearized model as shown in lecture
K1 = Kp1*Kthr*(2*a(1)*setp(1)+a(2));
K2 = Kp1*Kthr*(a(1)*setp(1)^2 + a(2)*setp(1) + a(3));
K3 = Kp2;
Kp = KT*par_IMEP0(1)*(par_sp(1)*setp(2)^2 + par_sp(2)*setp(2) + par_sp(3));    % Pressure multiplier
Kt = KT*(par_IMEP0(1)*X0(1) - par_IMEP0(2)) * (par_sp(1)*setp(2) + par_sp(2)); % Spark Timing multiplier
Kf = 2*Kfr1*X0(3)^2 + Kfr2*X0(3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Constants Calculation
Tau_d = pi/3;
Ndel = 3; 

CT1 = 1+ ((pi*K3)/6);
CT2 = ((pi*K3)/6)-1;
CT3 = (pi*K2)/(3*2*X0(3)^2);
CT4 = (pi*K1)/(2*X0(3)*3);
CT5 = ((pi*Kf)/(2*3*J*X0(3)^2))+1;
CT6 = ((pi*Kf)/(2*J*(X0(3)^2)*3))-1;
CT7 = pi/(3*2*J*X0(3));
CT8 = pi/(3*2*J*X0(3));

KP = 1;   
TI = 1; 
KI = KP/TI;
Td = 1;   
Kd = KP*Td;

