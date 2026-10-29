function [att, imu, meta] = insmini_replay(filename, Cbs, showPlot)
% Run ORIGINAL PSINS Mahony AHRS, using each sample's measured interval.
% att = [pitch, roll, relative_yaw, t] in radians, seconds (PSINS 3-1-2).
% A six-axis IMU has no absolute heading reference and no position solution.
    if nargin<2 || isempty(Cbs), Cbs=eye(3); end
    if nargin<3, showPlot=true; end
    if exist('MahonyUpdate','file')~=2
        error('INSmini:PSINS','Call insmini_setup_psins first.');
    end
    if exist('butter','file')~=2
        error('INSmini:Signal','PSINS MahonyInit needs MATLAB Signal Processing Toolbox or Octave signal.');
    end
    [imu,meta] = insmini_load(filename,Cbs,true);
    if ~isempty(meta.initialQ)
        q0 = m2qua(q2mat(meta.initialQ)*Cbs');
    else
        f = mean(bsxfun(@rdivide,imu(1:min(100,end),4:6),meta.dt(1:min(100,end))),1);
        q0 = a2qua([asin(max(-1,min(1,f(2)/norm(f)))); atan2(-f(1),f(3)); 0]);
    end
    ahrs = MahonyInit(4,q0,0);
    att = zeros(size(imu,1),4);
    for k=1:size(imu,1)
        ahrs = MahonyUpdate(ahrs,imu(k,1:6),[0;0;0],meta.dt(k));
        att(k,:) = [q2att(ahrs.q)',imu(k,7)];
    end
    if any(~isfinite(att(:))), error('INSmini:Replay','Non-finite AHRS result.'); end
    if showPlot
        figure; plot(att(:,4),att(:,1:3)*180/pi); grid on;
        xlabel('Time (s)'); ylabel('Attitude (deg)');
        legend('Pitch','Roll','Relative yaw'); title('INSmini / original PSINS Mahony AHRS');
    end
end
