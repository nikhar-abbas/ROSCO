function [b,a] = filt_1lp_D(om,dt)
%%
% This function describes the discrete time transfer function for a first
% order low pass filter of the form: H(s) = om/(s+om)
%
% Inputs: om - cutoff frequency
%         dt - discrete timestep length
%
% Outputs: b - filter numerator coefficients
%          a - filter denominator coefficients  
%
% Nikhar Abbas - March 2019



%%
alph = exp(-om*dt);
K = (1 - alph)/2;

b = K * [1 1];
a = [1, -alph];

end