function FastIn = SimSetup(ModDir, ModName, ElastoName)
% Loads some desired simulink input values from .fst model file 
% Inputs:
%   ModDir  - .fst model directory
%   ModName - .fst model name
%   ElastoName - ElastoDyn file name
% Outputs:
%   setup   - structure containing desired fast model inputs 
%
% Nikhar Abbas
%
%% Open .fst model file
FAST_InputFileName = [ModDir filesep ModName];
fid = fopen(FAST_InputFileName);

% Define Parameters to load
params = {'TMax' 'DT'};

% Load data into output structure
i = 1;
while i <= length(params)
    fl = fgetl(fid);
    if fl == -1
        disp('Not all parameters loaded from .fst file')
        break
    end
    strcell = strsplit(fl);
    [Log, Ind] = ismember(params{i}, strcell);
    if Log == 1
        FastIn.(params{i}) = str2num(strcell{Ind - 1});
        i = i+1;
    end
end


%% Open ElastoDyn file
FAST_InputFileName = [ModDir filesep ElastoName];
fid = fopen(FAST_InputFileName);

% Define Parameters to load
params = {'BlPitch(1)', 'BlPitch(2)', 'BlPitch(3)', 'RotSpeed'};

% Load data into output structure
i = 1;
while i <= length(params)
    fl = fgetl(fid);
    if fl == -1
        disp('Not all parameters loaded from ElastoDyn file')
    break
    end
    strcell = strsplit(fl);
    [Log, Ind] = ismember(params{i}, strcell);
    if Log == 1
        if params{i}(1:7) == 'BlPitch'
            FastIn.([params{i}(1:7), params{i}(9)]) = str2double(strcell{Ind - 1});
            i = i+1;
        else
            FastIn.(params{i}) = str2double(strcell{Ind - 1});
            i = i+1;
        end
    end
end




end