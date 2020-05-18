function param = Pre_ContParam_TSR_NREL5MW
% Control parameters input file for TSR_Tracking_*.mdl
%
% These parameters are specific for the NREL 5MW (fixed bottom) controller
%
% Nikhar Abbas - May 2019

% Turbine and Environmental ParametersParameters
param.RotorRad      = 63;                % Rotor Radius, m.
param.GBRatio       = 97;                   % Gearbox Ratio, -.
param.GenEff        = .944;                 % Generator Efficiency, 1/%
param.RRSpeed       = 12.1 * pi/30;          % Rated Rotor Speed, rad/s.
param.rho           = 1.225;                % Air Density, kg/m^3.
param.J = 43702538.05700% 38677056.000 + 534.116*param.GBRatio^2;           % Total Rotor Inertia, kg-m^2.

% Filter Cornering Frequencies
param.filt_LPFtype       = 1;                   % GenSpeed LPF type. {1 = first order, 2 = second order}
param.filt_HSS           = 1.57 %4.519 * 1/3;    % Corner frequency in the recursive, second order low pass filter of the HSS Speed, Hz. -- Chosen to be 3/4 of the first drivetrain natural frequency
param.filt_WindSpeedEst  = 0.628319;            % Corner frequency for Wind Speed Filt, rad/s. -- (Currently arbitrary)
param.filt_GB            = 0.628319;           % Corner frequeny for Gain Bias Filter, rad/s. -- (Currently arbitrary)
param.filt_PS            = 1/5;            % Corner frequeny for Peak Shaving Filter, rad/s. -- (Currently arbitrary)
param.filt_Fl            = [0.2325*1.25 1]; %[(0.459+0.2325)/2 1];            % Corner frequeny for Floating Filter, rad/s. -- (Currently arbitrary)

% Variable Speed Torque Controller Parameters
param.VS_zeta = 0.7;                        % Damping constant, --
param.VS_om_n = 0.2; 1/(2*6.25);                 % Natural frequency, Hz. -- Time constant chosen to be on third the rotor frequency at rated. 

% Blade Pitch Controller Parameters
param.PC_zeta = 0.7;                         % Damping constant, --
param.PC_om_n = 0.6;                         % Natural frequency, Hz.

% Region 2.5 Gain Bias
param.VS_GainBias = 1.0;               % VS Controller Bias Gian for Region 2.5 smoothing, -.
param.PC_GainBias = .001;              % Pitch Controller Bias Gian for Region 2.5 smoothing, -.

% Wind Speed Estimator
param.WSE_v0    = 13;

% Peak Shaving
param.PS_switch = 0;                        % Peak shaving on/off, 1 = on, 0 = off
param.PS_Percent = 0.8;                     % Percent of peak shaving, 1/%

% Floating
param.Fl_switch = 1;        % Floating feedback on/off, 1 = on, 0 = off
param.Lt = 87.6             % Tower height

% Pitch Controller Setpoints
param.PC_MaxPit     = 90 * (pi/180);        % Maximum pitch setting in pitch controller, rad.
param.PC_MaxRat     = 10 * (pi/180);        % Maximum pitch  rate (in absolute value) in pitch  controller, rad/s.
param.PC_MinPit     = 0.0  * (pi/180);        % Minimum pitch setting in pitch controller, rad.
param.PC_RefSpd     = 122.9096;              % Desired (reference) HSS speed for pitch controller, rad/s.
param.PC_Vrated     = 11.4; 11.4;                 % Rated wind speed, m/s.
param.PC_Vmax       = 25;                   % Maximum wind speed, m/s.

% Variable Speed Torque Controller setpoints
param.VS_MaxRat     = 15000.0;                                             % Maximum torque rate (in absolute value) in torque controller, N-m/s.
param.VS_Rgn3MP     = 1 * pi/180;                                          % Minimum pitch angle at which the torque is computed as if we are in region 3 regardless of the generator speed, rad. -- chosen to be 1.0 degree above PC_MinPit
param.VS_RtGnSp     = 122.9096;                                            % Rated generator speed (HSS side), rad/s. 
param.VS_RtPwr      = 5000000.0/param.GenEff;                              % Rated generator generator power in Region 3, Watts. -- chosen to be 10MW divided by the electrical generator efficiency of 96%
param.VS_RatedTq    = param.VS_RtPwr/param.PC_RefSpd;                      % Rated generator torque in Region 3 (HSS side), N-m.
param.VS_MaxTq      = param.VS_RatedTq;                              % Maximum generator torque in Region 3 (HSS side), N-m.
param.VS_MinSpd     = 6.9*pi/30;                                           % Minimum rotor speed (rad/s)
param.VS_Vmin       = 3;                                                   % Minimum wind speed, m/s
param.VS_Rgn2K      = 2.065540;      % VS_Rgn2K

end