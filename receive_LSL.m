addpath('C:\users\tomme\MATLAB\liblsl-MATLAB');
addpath('C:\Users\tomme\MATLAB\liblsl\bin');
savepath;

lib = lsl_loadlib();
fprintf('test.\n');

result = {};
while isempty(result)
    result = lsl_resolve_byprop(lib,'name','BIS_EEG');
end

inlet = lsl_inlet(result{1});

fprintf('Receiving...\n');

while true
    [sample, ts] = inlet.pull_sample();
    eeg1 = sample(1);
    eeg2 = sample(2);

    fprintf('EEG1 = %.3f   EEG2 = %.3f   (t=%.3f)\n', eeg1, eeg2, ts);
end