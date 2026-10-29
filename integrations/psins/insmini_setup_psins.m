function insmini_setup_psins(root)
% Configure the ORIGINAL locally extracted PSINS; no vendor files are edited.
    if ~exist(fullfile(root,'base','glvs.m'),'file') || ...
       ~exist(fullfile(root,'base','ahrs','MahonyUpdate.m'),'file')
        error('INSmini:PSINS','Select the extracted PSINS directory containing base/.');
    end
    addpath(root); addpath(genpath(fullfile(root,'base')));
    if exist('OCTAVE_VERSION','builtin')
        try, pkg('load','signal'); catch, warning('INSmini:Signal','Octave signal is needed only for Mahony replay.'); end
    end
    glvs;
    global glv;
    glv.rootpath = root;
    glv.datapath = [fullfile(root,'data'),filesep];
end
