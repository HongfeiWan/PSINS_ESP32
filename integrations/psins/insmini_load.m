function [imu, meta, raw] = insmini_load(filename, Cbs, removeBias, allowSimulation)
% Read INSmini CSV v1. PSINS output: [dtheta(rad), dv(m/s), t(s)].
% Cbs maps sensor XYZ to your vehicle Right/Front/Up (column vectors).
% Default: identity mapping and RAW gyro; replay may request bias removal.
    if nargin < 2 || isempty(Cbs), Cbs = eye(3); end
    if nargin < 3, removeBias = false; end
    if nargin < 4, allowSimulation = false; end
    if ~isequal(size(Cbs),[3,3]) || any(~isfinite(Cbs(:))) || ...
       norm(Cbs*Cbs'-eye(3),'fro') > 1e-8 || abs(det(Cbs)-1) > 1e-8
        error('INSmini:Axes','Cbs must be a proper orthonormal rotation.');
    end
    fid = fopen(filename,'rt');
    if fid < 0, error('INSmini:File','Cannot open CSV.'); end
    guard = onCleanup(@() fclose(fid));
    meta = struct(); header = '';
    while true
        line = fgetl(fid);
        if ~ischar(line), error('INSmini:Header','Missing CSV header.'); end
        line = strtrim(line);
        if isempty(line), continue; end
        if line(1) ~= '#', header = line; break; end
        token = regexp(line,'^# ([a-z_]+)=(.*)$','tokens','once');
        if ~isempty(token), meta.(token{1}) = strtrim(token{2}); end
    end
    expected = 'seq,t_us,dt_us,flags,gx_rad_s,gy_rad_s,gz_rad_s,fx_m_s2,fy_m_s2,fz_m_s2';
    if isfield(meta,'data_mode') && strcmp(meta.data_mode,'simulation') && ~allowSimulation
        error('INSmini:Simulation','Simulated data: opt in with fourth argument true.');
    end
    if ~strcmp(header,expected), error('INSmini:Header','Unexpected column order.'); end
    required = {'protocol','gyro_unit','force_unit','time_unit','gyro_bias_rad_s','calibrated'};
    for k=1:numel(required)
        if ~isfield(meta,required{k}), error('INSmini:Header','Missing %s.',required{k}); end
    end
    if ~strcmp(meta.protocol,'insmini/1') || ~strcmp(meta.gyro_unit,'rad/s') || ...
       ~strcmp(meta.force_unit,'m/s2') || ~strcmp(meta.time_unit,'us')
        error('INSmini:Units','Unsupported protocol or units.');
    end
    data = textscan(fid,repmat('%f',1,10),'Delimiter',',','CommentStyle','#','CollectOutput',true,'ReturnOnError',false);
    raw = data{1};
    if isempty(raw) || size(raw,2) ~= 10 || any(~isfinite(raw(:)))
        error('INSmini:Data','Empty, incomplete or non-finite samples.');
    end
    if any(raw(:,1)~=fix(raw(:,1))) || any(raw(:,1)<1) || any(diff(raw(:,1))~=1) || any(raw(:,4)~=0)
        error('INSmini:Gap','Sequence discontinuity, hardware sample gap or sensor saturation.');
    end
    % Early development firmware had no saturation flag; reject its clipped data.
    if isfield(meta,'firmware') && strncmp(meta.firmware,'2.0.',4) && ...
       (any(any(abs(raw(:,5:7))>=32760*.00875*pi/180)) || any(any(abs(raw(:,8:10))>=32760*.000061*9.80665)))
        error('INSmini:Saturation','Clipped data from early +/-250 dps, +/-2 g firmware.');
    end
    if any(raw(:,2)~=fix(raw(:,2))) || any(raw(:,3)~=fix(raw(:,3))) || ...
       any(raw(:,3)<4000 | raw(:,3)>20000) || any(diff(raw(:,2))<=0) || ...
       any(abs(diff(raw(:,2))-raw(2:end,3))>1)
        error('INSmini:Time','Invalid or inconsistent sample timestamps.');
    end
    meta.biasSensor = sscanf(strrep(meta.gyro_bias_rad_s,',',' '),'%f');
    if numel(meta.biasSensor)~=3 || any(~isfinite(meta.biasSensor))
        error('INSmini:Bias','Invalid gyro bias metadata.');
    end
    meta.initialQ = [];
    if isfield(meta,'initial_q_sensor_to_level')
        meta.initialQ = sscanf(strrep(meta.initial_q_sensor_to_level,',',' '),'%f');
        if numel(meta.initialQ)~=4 || any(~isfinite(meta.initialQ)) || abs(norm(meta.initialQ)-1)>.001
            error('INSmini:Attitude','Invalid initial quaternion.');
        end
        meta.initialQ = meta.initialQ/norm(meta.initialQ);
    end
    meta.dt = raw(:,3)*1e-6;
    meta.medianTs = median(meta.dt);
    meta.timeOriginUs = raw(1,2)-raw(1,3);
    meta.Cbs = Cbs; meta.biasRemoved = logical(removeBias);
    rates = raw(:,5:7);
    if removeBias
        if ~strcmp(meta.calibrated,'1'), error('INSmini:Bias','No completed stationary calibration.'); end
        rates = bsxfun(@minus,rates,meta.biasSensor');
    end
    bodyRate = rates*Cbs'; bodyForce = raw(:,8:10)*Cbs';
    % Do not subtract gravity from accelerometer specific force.
    imu = [bsxfun(@times,bodyRate,meta.dt), bsxfun(@times,bodyForce,meta.dt), ...
           (raw(:,2)-meta.timeOriginUs)*1e-6];
end
