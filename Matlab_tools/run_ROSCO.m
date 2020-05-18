%% TSR Referenece Controller 
% Run the NREL 5MW Turbine simulation initialized with this script
%
% This implements a controller to track tip-speed-ratio in below rated
% operation, and optimal rotor speed in above rated operation. A formal
% writeup of the logic used will be presented at WindTech 2019. 

%% Setup Paths

% Fast Model 
ModDir = '/Users/nabbas/Documents/TurbineModels/NREL_5MW/5MW_OC4Semi_WSt_WavesWN';

% Controller path
ContPath = './';
addpath(ContPath)

%% Define Filenames
ModBase = '5MW_OC4Semi_WSt_WavesWN';
ElastoFile = 'NRELOffshrBsline5MW_OC4DeepCwindSemi_ElastoDyn.dat';
ServoFile = 'NRELOffshrBsline5MW_OC4DeepCwindSemi_ServoDyn.dat';
ModName = [ModBase, '.fst'];

FAST_InputFileName = [ModDir filesep ModName];
%% Simulation and controller setup
servoparams = {'PCMode', 'VSContrl', 'HSSBrMode', 'YCMode'};
Pre_FastMod([ModDir filesep ServoFile], servoparams, {4,4,4,4});
% Pre_FastMod([ModDir filesep ServoFile], servoparams, {5,5,0,0});
FastIn = Pre_SimSetup(ModDir, ModName, ElastoFile);
dt = FastIn.DT;
TMax = FastIn.TMax;         
ContParam = Pre_ContParam_TSR_NREL5MW;

%% Load CpSurface 
txtfile = '/Users/nabbas/Documents/TurbineModels/NREL_5MW/5MW_Baseline/Cp_Ct_Cq.NREL5MW.txt'
cpscan = Pre_LoadRotPerf(txtfile);
TSRvec = cpscan.TSR;
zind = find(cpscan.BlPitch == 0);
Cpvec = cpscan.Cpmat(:,zind)';
TSR_opt = TSRvec(Cpvec == max(Cpvec));

%% Find plant parameters
[Avec,Bbvec,GS,Beta_op,vv] = Pre_TSRtracking_GS(ContParam,cpscan);
ContParam.GS = GS;

% % Trim for BldPitch Controller (testing/debug)
% Bopind = find(Beta_op>0);
% Avec_BPC = Avec(Bopind(1):end);
% Bbvec_BPC = Bbvec(Bopind(1):end);
% Betaop_BPC = Beta_op(Bopind(1):end);
% vv_bpc = vv(Bopind(1):end);
%% Define Filter Parameters
% Defining filters this way is a bit unnecessary, but is done for
% consistency with the FORTRAN implementation of ROSCO
if ContParam.filt_LPFtype == 1
    [Filt.HSS.b,Filt.HSS.a] = filt_1lp_D(ContParam.filt_HSS, dt); % ContParam.HSSfilt_omn
elseif ContParam.filt_LPFtype == 2
    [Filt.HSS.b,Filt.HSS.a] = filt_2lp_D(ContParam.filt_HSS, 0.7, dt); % ContParam.HSSfilt_omn
end
[Filt.GBFilt.b,Filt.GBFilt.a] = filt_1lp_D(ContParam.filt_GB,dt); 
[Filt.Wind.b,Filt.Wind.a] = filt_1lp_D(ContParam.filt_WindSpeedEst,dt); 
[Filt.PeakShaving.b,Filt.PeakShaving.a] = filt_1lp_D(ContParam.filt_PS,dt); 
[Filt.Floating.b,Filt.Floating.a] = filt_2lp_D(ContParam.filt_Fl(1),ContParam.filt_Fl(2), dt); 

filt_Fl = tf(Filt.Floating.b,Filt.Floating.a,dt);
% Notch filter
filt_Fl2 = c2d(tf([1 0 0.459^2], [1 0.459/2 0.459^2]),dt,'Tustin');
[Filt.Fl2.b, Filt.Fl2.a] = tfdata(filt_Fl2);
Filt.Fl2.b = Filt.Fl2.b{:};
Filt.Fl2.a = Filt.Fl2.a{:};

% [Filt.Fl2.b,Filt.Fl2.a] = filt_2lp_D(ContParam.filt_Fl(1)*1, 0.7, dt); % ContParam.HSSfilt_omn

ContParam.Filt = Filt;
%% Load Outlist
% Might need to run this the first iteration because of some stupid bug
% where the OutList is empty
OutName = [ModBase, '.out'];
SFunc_OutfileName = [ModDir filesep OutName];
OutList = Post_LoadOutlist(SFunc_OutfileName); 

%% Run Simulation
sim('ROSCO.slx',[0,TMax]);

for i = 1:length(OutList)
    try
        simout.(OutList{i}) = FAST_Out.Data(:,i);
    catch
        warning(['Outlist Parameter ' OutList{i} ' was not loaded from the fast.out file.'])
    end
end
simout.VSparams_a = VS_params.signals.values(:,1);
simout.VSparams_rotspeederr = VS_params.signals.values(:,2);
simout.VSparams_Ki = VS_params.signals.values(:,3);
simout.VSparams_Kp = VS_params.signals.values(:,4);
simout.TSR = simout.RotSpeed./simout.Wind1VelX * ContParam.RotorRad * pi/30;
simout.VSparams_omopt = Om_opt.Data;

simout.PCparams_a = PC_params.signals.values(:,1);
simout.PCparams_rotspeederr = PC_params.signals.values(:,2);
simout.PCparams_Ki = PC_params.signals.values(:,3);
simout.PCparams_Kp = PC_params.signals.values(:,4);
simout.PCparams_B_ss = PC_params.signals.values(:,5);

simout.vhat = vhat.Data;
simout.ContParam = ContParam;
