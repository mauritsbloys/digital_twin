%% WIS (combined plant + local control, cont)

% states of the plant 
%   y_1, water level pool 1
%   delta_1, pade approximation of the delay of pool 1
%   y_2,
%   delta_2,
%   y_3,
%   delta_3,


Ap = [0 62.269085474698 0 0 0 0     0;
      0 -92.3076923076923 0 0 0 0   0;
      0 0 0 180.5271392466 0 0      0;
      0 0 0 -171.428571428571 0 0   0;
      0 0 0 0 0 43.8788942518649    0;
      0 0 0 0 0 -80                 0;
      0 0 0 0 0 0                   0];  % disturbance

Bp = [-5.39665407447383 -5.39665407447383 0;
      16 0 0;
      0 -8.424599831508 -8.424599831508;
      0 16 0;
      0 0 -4.38788942518649;
      0 0 16;
      0 0 0]; % disturbance


% Do we need to output more states
Cp = [1 0 0 0 0 0   0;
      0 0 1 0 0 0   0;
      0 0 0 0 1 0   0];
      
Dp = 0*Cp*Bp;

% noise on pool 3 outflow which is not in the plant 
% results in a dropping water level
Wis.area3 = 0.2279; %m2
% disturbance 0.015 m^3/min
dist = 1 / Wis.area3; % m/min 
E = dist * [0; 0; 0; 0; 1; 0;   0]; 
% This will be an extra, small fluctuation about the steady state
% disturbance.

Ap_no_disturbance = Ap(1:end-1, 1:end-1);  % Remove the state
Bp_no_disturbance = Bp(1:end-1, :);
E_no_disturbance = E(1:end-1, :);

% Gabriel: is this necessary? I don;t want initial disturbance
Ap(5,end) = dist;  % Now in the A matrix.
% FIXED: must be the same as E (A(:,end) = E).

% WIS controller (P control only)

% TODO: do these states (created by Matlab) have a physical meaning?

Ac = [-10 0 0 0 0 0;
      1 0 0 0 0 0;
      0 0 -10 0 0 0;
      0 0 1 0 0 0;
      0 0 0 0 -10 0;
      0 0 0 0 1 0];

Bc = [1 0 0;
      0 0 0;
      0 1 0;
      0 0 0;
      0 0 1;
      0 0 0];
  
Cc = [-2.37153069171695 -0.237153069171695 0 0 0 0;
      0 0 -6.99590261662633 -0.699590261662633 0 0;
      0 0 0 0 -12.0648635755688 -1.20648635755688];

Dc = [0 0 0;0 0 0;0 0 0];

% Discretize the controller. Period = 1s, but model
% is in minutes, so use 1/60
contContr = ss(Ac, Bc, Cc, Dc);
discContr = c2d(contContr, 1/60, 'tustin');

Ac = discContr.A;
Bc = discContr.B;
Cc = discContr.C;
Dc = discContr.D;

% alternative, created exactly like on FF
Ac = [1.84615384615385 -0.846153846153846 0 0 0 0;1 0 0 0 0 0;0 0 1.84615384615385 -0.846153846153846 0 0;0 0 1 0 0 0;0 0 0 0 1.84615384615385 -0.846153846153846;0 0 0 0 1 0];
Bc = [-0.0790510230572318 0 0;0 0 0;0 -0.139918052332527 0;0 0 0;0 0 -0.402162119185627;0 0 0];
Cc = [0.426775147928994 -0.426005917159763 0 0 0 0;0 0 0.71129191321499 -0.710009861932939 0 0;0 0 0 0 0.426775147928994 -0.426005917159763];
Dc = [-0.0182577459022568 0 0;0 -0.053859481042104 0;0 0 -0.092883981758065];

% Put time scale to seconds (for conditioning)
Ap = Ap/60;
Bp = Bp/60;
E = E/60;

% Epoch length
h = 1;  % seconds now!


%% Dimensions

np = size(Ap,1);  % states of the plant
nc = size(Ac,1);  % states of the controller
pp = size(Cp,1);  % measured plant outputs
mp = size(Bp,2);  % number of control inputs
nw = size(E,2);   % number of disturbances
ppt = 3;  % For WIS: number of outputs to use in triggering


%% Assumption 4: Bound on disturbance
% negative disturbance => outflow
W_MAG = -0.015;
W_MAG = W_MAG*1000;  % Units to mm and the like

% Keep W_MAG at 100% because now there is only 'fluctuation'
%W_MAG = W_MAG/100;  % 1% fluctuation about real deal.

% Bound on noise
if exist('V_GLOBAL','var')
    V_EACH_ELEMENT = V_GLOBAL;
    disp('Using externally set noise value (V_GLOBAL)');
else
    V_EACH_ELEMENT = 0.1; % in mm
end

YFACTOR = 1.1;  % We never know the noise levels that precisely.
V = V_EACH_ELEMENT^2*YFACTOR^2*eye(pp)*pp;