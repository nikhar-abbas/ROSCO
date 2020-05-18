function Pre_FastMod(filename, params, values)
% This function is written to modify a defined Openfast input file quickly.
%
%
% Inputs: filename - the input file to be modified
%         params - a cell array of the file parameters to be modified
%         values - a cell array of final values corresponding to the 
%                  parameters indicated in the params input. Be sure to
%                  input numerical values as a double, not a string!
%
%
% Nikhar Abbas - February 2019


%% Check inputs for validity
if length(params) ~= length(values)
    error('Length of parameter input cell array must be the same as the length of the values input array')
end

if ~iscell(params) || ~iscell(values)
    error('Parameter and Value inputs must both be cell arrays')
end


%% Load File into cell array
fid = fopen(filename,'r');
if fid == -1
    error('Unable to load input file to Pre_FastMod')
end
li = 1;
tline = fgetl(fid);
while ischar(tline)
    A{li,1} = tline;
    tline = fgetl(fid);
    li = li+1;
end
fclose(fid);



%% Replace params
for pind = 1:length(params)
    if any(contains(A,params{pind}))
        if ischar(values{pind})
            prepstr = [values{pind},'$2',params{pind}];
            A = regexprep(A,['\"?(\w+)\"?(\s*)', params{pind}],prepstr);
        elseif isnumeric(values{pind})
            prepstr = [num2str(values{pind}),'$1',params{pind}];
            A = regexprep(A,['[-]?\d*[.]?\d*(\s+)', params{pind}],prepstr);
        end
    else
        display(['Unable to change ', params{pind}, ' in ', filename])
    end
end


%% Write to file
fid = fopen(filename,'w');
for li = 1:length(A)
    if li == length(A)
        fprintf(fid, '%s', A{li});
    else
        fprintf(fid, '%s\n', A{li});
    end
end
fclose(fid);


end