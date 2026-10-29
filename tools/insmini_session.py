"""Validate/export an INSmini INL3 session without MATLAB or third-party modules."""
import argparse,csv,json,math,pathlib,struct,zlib,collections
RECORD=struct.Struct('<IBBHIQq11dI')
def decode(folder,allow_simulation=False):
    folder=pathlib.Path(folder);prefix=''
    if not (folder/'session.json').exists():
        matches=list(folder.glob('*_session.json'))
        if len(matches)!=1:raise ValueError('Put exactly one session and its three downloaded files in this folder')
        prefix=matches[0].name[:-len('session.json')]
    meta=json.loads((folder/(prefix+'session.json')).read_text(encoding='utf8'))
    if meta.get('format')!='INL3' or meta.get('record_bytes')!=120:raise ValueError('Unsupported session format')
    mode=meta.get('mode');expected={'real':0,'simulation':1}.get(mode)
    if expected is None:raise ValueError('Unknown data source')
    if mode=='simulation' and not allow_simulation:raise ValueError('Simulation data: use --allow-simulation explicitly')
    raw=(folder/(prefix+'stream.inl')).read_bytes();records=[];counts=collections.Counter();errors=[]
    if len(raw)%120:errors.append('truncated record at end')
    for offset in range(0,len(raw)-119,120):
        b=raw[offset:offset+120];r=RECORD.unpack(b);magic,kind,source,flags,seq,mono,utc,*rest=r;values=rest[:-1]
        if magic!=0x334c4e49 or zlib.crc32(b[:-4])!=rest[-1]:raise ValueError(f'CRC or magic mismatch at byte {offset}')
        if source!=expected:raise ValueError(f'Mixed real/simulation sources at byte {offset}')
        if kind!=5 and not all(map(math.isfinite,values)):raise ValueError(f'Non-finite payload at byte {offset}')
        records.append((kind,flags,seq,mono,utc,values));counts[kind]+=1
    imu=[r for r in records if r[0]==1]
    for previous,r in zip(imu,imu[1:]):
        dt=r[3]-previous[3]
        if r[2]!=previous[2]+1 or dt<4000 or dt>20000:errors.append(f'IMU sequence/time gap at {r[2]}')
    encoded_dt=meta.get('imu_interval')=='flags>>2_us'
    if any((r[1]&3) if encoded_dt else r[1] for r in imu):errors.append('IMU hardware gap/saturation flags present')
    if encoded_dt:
        for previous,r in zip(imu,imu[1:]):
            if r[1]>>2!=r[3]-previous[3]:errors.append('Stored IMU interval differs from timestamp')
    closed_path=folder/(prefix+'closed.json');closed=json.loads(closed_path.read_text(encoding='utf8')) if closed_path.exists() else None
    if closed is None:errors.append('No closed.json: unclean stop/power loss')
    else:
        if closed.get('records')!=len(records) or closed.get('bytes')!=len(raw):errors.append('Closed record/byte count mismatch')
        if closed.get('drops') or not closed.get('write_ok'):errors.append('Recorder reports lost records/write error')
    report={'mode':mode,'records':len(records),'types':dict(counts),'imu_samples':len(imu),'duration_s':(imu[-1][3]-imu[0][3])*1e-6 if len(imu)>1 else 0,'utc_imu_samples':sum(r[4]>=0 for r in imu),'errors':list(dict.fromkeys(errors)),'complete':not errors}
    return meta,records,report
def export(folder,allow_simulation=False):
    folder=pathlib.Path(folder);meta,records,report=decode(folder,allow_simulation);imu=[r for r in records if r[0]==1]
    if not imu:raise ValueError('No IMU samples')
    # Strict continuous exports only. Preserve the raw INL file for partial-session recovery.
    if not report['complete']:raise ValueError('Incomplete session: '+ '; '.join(report['errors']))
    encoded_dt=meta.get('imu_interval')=='flags>>2_us'
    origin=imu[0][3];first_dt=(imu[0][1]>>2) if encoded_dt else ((imu[1][3]-origin) if len(imu)>1 else 10000);origin-=first_dt
    headers=['# INSmini CSV v1','# protocol=insmini/1','# firmware='+meta['firmware'],'# data_mode='+meta['mode'],'# gyro_unit=rad/s','# force_unit=m/s2','# time_unit=us','# calibrated=1','# gyro_bias_rad_s='+','.join(map(str,meta['gyro_bias_rad_s'])),'# initial_q_sensor_to_level='+','.join(map(str,imu[0][5][7:11])),'# calibration_id='+str(meta['calibration_id'])]
    with (folder/'imu.csv').open('w',encoding='utf8',newline='') as f:
        f.write('\n'.join(headers)+'\n');writer=csv.writer(f);writer.writerow(['seq','t_us','dt_us','flags','gx_rad_s','gy_rad_s','gz_rad_s','fx_m_s2','fy_m_s2','fz_m_s2'])
        previous=origin
        for _,flags,seq,t,utc,v in imu:writer.writerow([seq,t,t-previous,flags&3 if encoded_dt else flags,*v[:6]]);previous=t
    with (folder/'psins_imu.txt').open('w',encoding='ascii') as f:
        previous=origin
        for _,flags,seq,t,utc,v in imu:
            dt=(t-previous)*1e-6;f.write(' '.join(format(x,'.15g') for x in [*(x*dt for x in v[:6]),(t-origin)*1e-6])+'\n');previous=t
    specs={2:('gnss.csv',['lat_rad','lon_rad','height_ellipsoid_m','ve_m_s','vn_m_s','hdop','fix_quality','satellites','rx_start_us','rx_end_us','epoch_mono_us']),3:('navigation.csv',['lat_rad','lon_rad','height_m','ve','vn','vu','pitch_rad','roll_rad','yaw_rad','pos_sigma_m','vel_sigma_m_s']),4:('pps.csv',['clock_quality','drift_ppm']),6:('truth.csv',['lat_rad','lon_rad','height_m','ve','vn','vu'])}
    for kind,(name,columns) in specs.items():
        with (folder/name).open('w',encoding='utf8',newline='') as f:
            w=csv.writer(f);w.writerow(['seq','mono_us','utc_us','flags',*columns])
            for k,flags,seq,t,utc,v in records:
                if k==kind:w.writerow([seq,t,utc,flags,*v[:len(columns)]])
    # PSINS position-only gps Nx4; use the recorded PPS epoch, never UART arrival time.
    with (folder/'psins_gps_pos.txt').open('w',encoding='ascii') as f:
        for kind,flags,seq,t,utc,v in records:
            if kind==2 and flags&1 and v[10]>0:f.write(' '.join(format(x,'.15g') for x in [*v[:3],(v[10]-origin)*1e-6])+'\n')
    lines=[];pending=bytearray()
    for kind,flags,seq,t,utc,v in records:
        if kind!=5:continue
        pending+=struct.pack('<11d',*v)[:flags&255]
        if not flags&0x100:lines.append(pending.decode('ascii','replace'));pending.clear()
    (folder/'raw.nmea').write_text('\n'.join(lines)+'\n',encoding='ascii')
    (folder/'validation.json').write_text(json.dumps(report,indent=2),encoding='utf8');return report
if __name__=='__main__':
    a=argparse.ArgumentParser(description=__doc__);a.add_argument('folder');a.add_argument('--allow-simulation',action='store_true');args=a.parse_args();print(json.dumps(export(args.folder,args.allow_simulation),indent=2))
