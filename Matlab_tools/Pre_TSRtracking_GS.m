function [A,Bb,GS,Beta_op,vv] = Pre_TSRtracking_GS(ContParam,cpscan)
% This function finds the linearized plant parameters for a specific
% turbine. In below rated operation, the plant is linearized about the
% optimal tip-speed ratio. In above rated, the plant is linearized about
% the operating point at rated rotor speed, and depends on wind speed. The
% linearized model is defined by:
% omega = A(v) + Bt*tau_g + Bb*beta
% Note: Because Bt is unchanged in all operation, it is not calculated here
% specifically, and is calculated in the torque controller gain schedule
% that is a part of TSR_tracking.mdl. 
%
%
% Inputs: ContParam - Structure of control parameters found in
%                     ControlParameters_TSR.m
%         cpscan - Structure of Cp surface data found after running
%                  An_Cpscan.m
% Outputs: A - Vector of system matrices at varied wind speeds.
%          Bb - Vector of blade pitch input gain matrices at varied wind
%               speeds
%          GS - Gain Schedule structure. Contains all gain schedule info
%          Beta_op - Vectore of operational blade pitch angles at varied
%                    wind speeds, deg. 
%          vv - Wind speeds swept and linearized at, m/s. 
%
% Nikhar Abbas - May 2019

%% Load Turbine Parameters
J = ContParam.J;
rho = ContParam.rho;                            % Air Density (kg/m^3)
R = ContParam.RotorRad;                         % Rotor Radius (m)
Ar = pi*R^2; 
Ng = ContParam.GBRatio; 
RRspeed = ContParam.RRSpeed; 
Vmin = ContParam.VS_Vmin;
Vrat = ContParam.PC_Vrated;
Vmax = ContParam.PC_Vmax;
Lt = ContParam.Lt;
%% Load Cp data
TSRvec = cpscan.TSR;
Cpvec = cpscan.Cpmat(:,(cpscan.BlPitch == 0));
Betavec = cpscan.BlPitch .* pi/180;
Cpmat = cpscan.Cpmat;
Ctmat = cpscan.Ctmat;
Beta_del = Betavec(2) - Betavec(1);
TSR_del = TSRvec(2) - TSRvec(1);
%% Find Cp Operating Points
TSRr = RRspeed*R/Vrat;

% separate wind speeds by operation regions
vv_br = [Vmin:.5:Vrat]; 
vv_ar = [Vrat+.5:.5:Vmax];
vv = [vv_br vv_ar];

% separate TSRs by operation regions
TSR_br = ones(1,length(vv_br)) * TSRvec(Cpvec == max(Cpvec));
TSR_ar = RRspeed.*R./vv_ar;
TSRop = [TSR_br TSR_ar];

% separate Cp by operation regions
Cpr = interp1(TSRvec,Cpvec,TSR_ar(1));
Cp_op_br = ones(1,length(vv_br)) * max(Cpvec);
Cp_op_ar = Cpr.*(TSR_ar./(TSRr)).^3;

Cp_op = [Cp_op_br Cp_op_ar];


%% Find linearized state matrices
A = zeros(1,length(TSRop));
Bb = zeros(1,length(TSRop));

for toi = 1:length(TSRop)
    
tsr = TSRop(toi); % Operational TSR

% ---- Cp Operating conditions ----
CpTSR = zeros(1,length(Betavec));
for Bi = 1:length(Betavec)
    CpTSR(Bi) = interp1(TSRvec, Cpmat(:,Bi), tsr); % Vector of Cp values corresponding to operational TSR
end

%Saturate operational TSR
% Cp_op(toi) = max( min(Cp_op(toi), max(CpTSR)), min(CpTSR));               % might need to saturate

% Operational Beta to be linearized around
% Beta_op(toi) = interp1(CpTSR,Betavec,Cp_op(toi));
CpMi = find(CpTSR == max(CpTSR));
Beta_op(toi) = interp1(CpTSR(CpMi:end),Betavec(CpMi:end),Cp_op(toi));     % might need to interpolate "above maximum" for numerical stability

% Saturate TSR and Beta
% Beta_op(toi) = max(min(Beta_op(toi),dB(end)),dB(1));                      % might need to saturate
% tsr_sat(toi) = max(min(tsr,dTSR(end)),dTSR(1));
tsr_sat(toi) = tsr;

% Linearized operation points
[CpGrad_Beta,CpGrad_TSR] = gradient(Cpmat,1);

Cp(toi) = interp2(Betavec,TSRvec,Cpmat,Beta_op(toi),tsr);
dCpdB(toi) = interp2(Betavec,TSRvec,CpGrad_Beta,Beta_op(toi),tsr)./Beta_del;
dCpdTSR(toi) = interp2(Betavec,TSRvec,CpGrad_TSR,Beta_op(toi),tsr)./TSR_del;

%% Final derivatives and system "matrices"
dtdb(toi) = Ng/2*rho*Ar*R*(1/tsr_sat(toi))*dCpdB(toi)*vv(toi)^2;
dtdl = Ng/(2)*rho*Ar*R*vv(toi)^2*(1/tsr_sat(toi)^2)* (dCpdTSR(toi)*tsr_sat(toi) - Cp(toi)); % assume operating at optimal
dldo = R/vv(toi)/Ng;
dtdo = dtdl*dldo;

t_dtdl(toi) = dtdl;

A(toi) = dtdo/J;            % Plant pole
B_t = -Ng^2/J;              % Torque input gain 
Bb(toi) = dtdb(toi)/J;     % BldPitch input gain

%% Wind Disturbance Input gain
dldv = -tsr/vv(toi); 
dtdv(toi) = dtdl*dldv;
% dtdv2(toi) = (3/2) * rho * Ar * RRspeed * Cp(toi)/(tsr_sat(toi)) * vv(toi)^2;
dtdv2(toi) = (0.5 * rho * Ar * 1/RRspeed) * (dCpdTSR(toi)*dldv*vv(toi)^3 + Cp(toi)*3*vv(toi)^2);
B_v = dtdv/J;

end

%% Gain Schedule
% ----- Generator torque controller -----
% Plant Linearization Points
Avs = A(TSRop == TSR_br(1));
Bvs = B_t;
vv_vs = vv((TSRop == TSR_br(1)));

% % Linear fit for Avs w.r.t. v
% pAvs = polyfit(vv_vs,Avs,1);
% Avs_f = pAvs(1)*vv_vs + pAvs(2);


% Desired behavior
VS_zeta = ContParam.VS_zeta;
VS_om_n = ContParam.VS_om_n;

% % Torque Controller Gains, as a function of linearized v
Kp_vs = 1/Bvs * (2*VS_zeta*VS_om_n + Avs);
Ki_vs = VS_om_n^2/Bvs;

% % Linear fit for Kp_vs w.r.t. v
% pKp_vs = polyfit(vv_vs,Kp_vs,1);
% pKi_vs = Ki_vs;

% ------ Blade Pitch Controller ------
% Plant Linearization Points
Bop_pc = length(vv_br); % find(Beta_op>0 * pi/180);
Apc = A(Bop_pc(1)+1:end);
Bb_pc = Bb(Bop_pc(1)+1:end);
Betaop_pc = Beta_op(Bop_pc(1)+1:end);
vv_pc = vv(Bop_pc(1)+1:end);

% Desired behavior
PC_zeta = ContParam.PC_zeta;
PC_om_n = ContParam.PC_om_n;

% Linear fit for Apc w.r.t. v
pApc = polyfit(vv_pc,Apc,1);
pBb_pc = polyfit(vv_pc,Bb_pc,1);
Apc_f = pApc(1)*vv_pc + pApc(2);
Bb_pc_f = pBb_pc(1)*vv_pc + pBb_pc(2);

% % Blade Pitch Gains, as a function of linearized v or related beta
Kp_pc = 1./Bb_pc_f .* (2*PC_zeta*PC_om_n + Apc_f);
Ki_pc = PC_om_n^2./Bb_pc_f ;


% CpMi = find(CpTSR == max(CpTSR));
% Beta_op(toi) = interp1(CpTSR(CpMi:end),Betavec(CpMi:end),Cp_op(toi));     % might need to interpolate "above maximum" for numerical stability
% 

%% Peak Shaving

% Find unshaved rotor thrust coefficient and rotor thrust
Ct_op = interp2(Betavec, TSRvec, Ctmat, Beta_op, TSRop);
T = 0.5 * rho * Ar .*vv.^2 .*Ct_op;

% Define minimum blade pitch
Tmax = ContParam.PS_Percent * max(T);
Beta_min = ones(1,length(Beta_op)) * ContParam.PC_MinPit;

% Modify if peak shaving
Ct_max = zeros(1,length(Beta_min));
if ContParam.PS_switch
    for i = 1:length(Beta_min)
        Ct_tsr = interp2(Betavec, TSRvec, Ctmat, Betavec, TSRop(i)); % Find Ct for specific TSR
        % Define max Ct 
        Ct_max(i) = Tmax/(0.5 * rho * Ar * vv(i)^2);
        if T(i) > Tmax 
           Ct_op(i) = Ct_max(i);
        else 
%            Ct_max(i) = Tmax/(0.5 * rho * Ar * vv(i)^2);
           Ct_max(i) = min( max(Ct_tsr), Ct_max(i));
        end
        CtMi = find(Ct_tsr == min(Ct_tsr));
        % Find Beta corresponding to Ct_max
        Beta_min(i) = interp1(Ct_tsr(1:CtMi),Betavec(1:CtMi),Ct_max(i));
        if Beta_min(i) <= ContParam.PC_MinPit
            Beta_min(i) = ContParam.PC_MinPit;
        end
    end
end

Tshaved = 0.5 * rho * Ar .*vv.^2 .* Ct_op;

%% Floating
if ContParam.Fl_switch 
    % Tower pitching feedback
    Kpf_vec = (dtdv./dtdb)*Lt*Ng;
    Kpf_vec2 = (dtdv2./dtdb)*Lt*Ng*2*pi;
else
    Kpf_vec = zeros(1,length(dtdb));
end
    
    
%% ----- Save Gain Schedule -----
GS.Kp_vs = Kp_vs;                               % VS Controller Proportional Gain
GS.Kp_vs = Kp_vs(end) * ones(1,length(Kp_vs));                               % VS Controller Proportional Gain
GS.Ki_vs = Ki_vs*ones(1,length(Kp_vs));         % VS Controller Integral Gain
GS.Kp_pc = Kp_pc;                               % Pitch Controller Proportional Gain
GS.Ki_pc = Ki_pc;                               % Pitch Controller Integral Gain
GS.pA = polyfit(vv,A,1);                        % Linear fit polynomials for system pole - unused
GS.A = A;
GS.VS_vv = vv_vs;                               % Below rated wind speeds, (m/s)
GS.PC_vv = vv_pc;                               % Above rated wind speeds, (m/s)
GS.vv = vv;                                     % Full range of scheduled wind speeds, (m/s)
GS.PC_beta = Betaop_pc;                         % Above rated blade pitch angles corresponding to gains, (rad)
GS.Beta_min = Beta_min;                         % Minimum BldPitch angles for peak shaving, (rad)
% GS.Kpf = mean(Kpf_vec(find(vv==vv_ar(1)):end))
% GS.Kpf = Kpf_vec(end)
GS.Kpf = Kpf_vec2(vv==vv_ar(1));
% 
% GS.A = [-0.01597135 -0.01863324 -0.02129513 -0.02395702 -0.02661891 -0.02928081 -0.03194270 -0.03460459 -0.03726648 -0.03992837 -0.04259026 -0.04525216 -0.04791405 -0.05057594 -0.05323783 -0.05589972 -0.05856161 -0.05136706 -0.06083297 -0.07318141 -0.08698814 -0.10174996 -0.11701540 -0.13277020 -0.14916461 -0.16625567 -0.18314382 -0.20108255 -0.21861726 -0.23708646 -0.25523482 -0.27455940 -0.29291942 -0.31337978 -0.33196662 -0.35213321 -0.37322194 -0.39245925 -0.41381198 -0.43612755 -0.45572506 -0.47749086 -0.50133095 -0.53269989];
% GS.Kp_vs = -1028.52796000 * ones(1,length(Kp_vs));
% GS.Ki_vs = -185.790360000 * ones(1,length(Kp_vs));
% GS.Kp_pc = [-0.016030  -0.014103  -0.012524  -0.011207  -0.010091  -0.009134  -0.008304  -0.007577  -0.006936  -0.006365  -0.005854  -0.005394  -0.004978  -0.004599  -0.004254  -0.003937  -0.003645  -0.003376  -0.003126  -0.002895  -0.002679  -0.002477  -0.002289  -0.002112  -0.001946  -0.001790];                               % Pitch Controller Proportional Gain
% GS.Ki_pc = [-0.007252  -0.006532  -0.005942  -0.005450  -0.005033  -0.004675  -0.004365  -0.004093  -0.003854  -0.003640  -0.003450  -0.003278  -0.003122  -0.002981  -0.002851  -0.002733  -0.002624  -0.002523  -0.002430  -0.002344  -0.002263  -0.002188  -0.002117  -0.002051  -0.001989  -0.001931];                               % Pitch Controller Integral Gain
% GS.PC_beta = [0.091667  0.113820  0.132927  0.150163  0.165982  0.180787  0.194909  0.208461  0.221269  0.233885  0.245874  0.257736  0.269106  0.280469  0.291234  0.302266  0.312525  0.322919  0.333308  0.342993  0.352949  0.362942  0.372100  0.381515  0.391200  0.400190];
end