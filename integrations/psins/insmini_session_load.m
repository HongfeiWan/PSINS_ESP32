function [imu, gpsPos, avp, meta] = insmini_session_load(folder, allowSimulation)
% Run tools/insmini_session.py on a closed TF session first.
% imu: PSINS [dtheta(3),dv(3),t]. gpsPos: [lat,lon,h,t], radians/metres.
% avp: recorded onboard PSINS [pitch,roll,yaw,VE,VN,VU,lat,lon,h,t].
    if nargin<2, allowSimulation=false; end
    [imu,meta]=insmini_load(fullfile(folder,'imu.csv'),eye(3),true,allowSimulation);
    sessionfile=fullfile(folder,'session.json');
    if ~exist(sessionfile,'file')
        matches=dir(fullfile(folder,'*_session.json'));
        if numel(matches)~=1, error('INSmini:Session','Put one session in each folder.'); end
        sessionfile=fullfile(folder,matches(1).name);
    end
    meta.session=jsondecode(fileread(sessionfile));
    gpsfile=fullfile(folder,'psins_gps_pos.txt');
    info=dir(gpsfile);
    if isempty(info)||info.bytes==0, gpsPos=zeros(0,4); else, gpsPos=load(gpsfile); end
    if ~isempty(gpsPos) && (size(gpsPos,2)~=4 || any(~isfinite(gpsPos(:))))
        error('INSmini:GNSS','Invalid position-only GNSS table.');
    end
    fid=fopen(fullfile(folder,'navigation.csv'),'rt');
    if fid<0, error('INSmini:File','Missing navigation.csv.'); end
    cleanup=onCleanup(@() fclose(fid));fgetl(fid);
    data=textscan(fid,repmat('%f',1,15),'Delimiter',',','CollectOutput',true,'ReturnOnError',false);
    nav=data{1};
    if isempty(nav), nav=zeros(0,15); end
    if size(nav,2)~=15 || any(~isfinite(nav(:))), error('INSmini:Navigation','Invalid navigation table.'); end
    meta.navigationStatus=nav(:,4);
    avp=[nav(:,11:13),nav(:,8:10),nav(:,5:7),(nav(:,2)-meta.timeOriginUs)*1e-6];
    meta.gnssVelocityNote='RMC supplies horizontal velocity only; no measured vertical velocity.';
end
