import hashlib,json,math,pathlib,shutil,struct,subprocess,sys,tempfile,zlib

root=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'tools'))
from analyze_insmini_field import analyze,analyze_session
from insmini_session import RECORD

def record(kind,seq,mono,values=(),flags=0,utc=-1,source=0):
    values=tuple(values)+(0,)*(11-len(values))
    data=RECORD.pack(0x334c4e49,kind,source,flags,seq,mono,utc,*values,0)
    return data[:-4]+struct.pack('<I',zlib.crc32(data[:-4]))

def sentence(body,seq,mono,corrupt=False):
    checksum=0
    for char in body:checksum^=ord(char)
    data=('$'+body+'*'+format(checksum^int(corrupt),'02X')).encode('ascii');chunks=[]
    for start in range(0,len(data),88):
        chunk=data[start:start+88];flags=len(chunk)|(0x100 if start+88<len(data) else 0)
        chunks.append(record(5,seq,mono,struct.unpack('<11d',chunk.ljust(88,b'\0')),flags))
    return b''.join(chunks)

def fixture(folder,mode='real',valid_fix=True,navigation=True,pps=True,visible=True):
    folder.mkdir();source=int(mode=='simulation');data=b''
    for seq in range(1,11):data+=record(1,seq,1000000+seq*10000,(0,0,0,0,0,9.8,25,1,0,0,0),10000<<2,source=source)
    raw=[('GPTXT,01,01,01,ANTENNA OK',False),('GPTXT,01,01,01,'+'X'*100,False)]
    raw+=[('GPGSV,1,1,04,01,40,100,42,02,30,120,38,03,20,140,,04,10,160,0,1' if visible else 'GPGSV,1,1,00',False)]
    raw+=[('BDGSV,1,1,01,201,40,120,35,1' if visible else 'BDGSV,1,1,00',False),('GPGSV,1,1,99,99,90,360,99',True)]
    if valid_fix:raw+=[('GNGGA,120000.000,3000.00000,N,11400.00000,E,1,08,0.9,100.0,M,25.0,M,,',False),('GNRMC,120000.000,A,3000.00000,N,11400.00000,E,0.000,0.0,291026,,,A',False)]
    else:raw+=[('GNGGA,120000.000,,,,,0,00,25.5,,,,,,',False),('GNRMC,120000.000,V,,,,,,,,,,N',False)]
    for seq,(body,corrupt) in enumerate(raw,1):
        chunk=sentence(body,seq,1300000+seq*1000,corrupt)
        if source:
            pieces=[]
            for offset in range(0,len(chunk),120):
                b=bytearray(chunk[offset:offset+120]);b[5]=source;b[-4:]=struct.pack('<I',zlib.crc32(b[:-4]));pieces.append(bytes(b))
            chunk=b''.join(pieces)
        data+=chunk
    utc=1793275200000000
    if valid_fix:data+=record(2,1,1400000,(30*math.pi/180,114*math.pi/180,125,0,0,.9,1,8,1300000,1350000,1200000 if pps else 0),3,utc,source)
    if pps:
        for seq in range(1,4):data+=record(4,seq,1200000+seq*1000000,(2 if seq==3 else 0,0),0,utc+seq*1000000 if seq==3 else -1,source)
    if navigation:
        for seq,state in enumerate([1,1,2,3],1):data+=record(3,seq,4400000+seq*100000,(30*math.pi/180,114*math.pi/180,125,0,0,0,0,0,0,5,.5),state,utc,source)
    meta={'format':'INL3','record_bytes':120,'imu_interval':'flags>>2_us','firmware':'fixture','mode':mode,'gyro_bias_rad_s':[0,0,0],'calibration_id':1}
    (folder/'session.json').write_text(json.dumps(meta),encoding='utf8');(folder/'stream.inl').write_bytes(data);(folder/'closed.json').write_text(json.dumps({'records':len(data)//120,'bytes':len(data),'drops':0,'write_ok':True}),encoding='utf8')
    return data

with tempfile.TemporaryDirectory() as temp:
    p=pathlib.Path(temp);card=p/'card';card.mkdir();good=card/'good_REAL';fixture(good)
    before={f.relative_to(card).as_posix():hashlib.sha256(f.read_bytes()).hexdigest() for f in card.rglob('*') if f.is_file()}
    entry=analyze_session(good);assert entry['status']=='analyzed' and entry['integrity']['complete']
    radio=entry['reception'];assert radio['antenna_ok_observed'] and radio['raw_sentences']==7 and radio['checksum_errors']==1 and radio['fragment_errors']==0
    assert radio['gsv']['GPGSV']['max_satellites_in_view']==4 and radio['gsv']['GPGSV']['max_cn0_dbhz']==42
    assert radio['gsv']['BDGSV']['max_cn0_dbhz']==35 and radio['raw_coherent_position_epochs']==1
    assert entry['position']['valid_position_observed'] and entry['timing']['software_utc_lock_observed']
    assert entry['timing']['pps_interval_us_min']==entry['timing']['pps_interval_us_max']==1000000
    assert entry['psins']['state_counts']=={1:2,2:1,3:1} and entry['psins']['complete_navigation_evidence']
    assert not entry['timing']['physical_pps_accuracy_verified']
    assert before=={f.relative_to(card).as_posix():hashlib.sha256(f.read_bytes()).hexdigest() for f in card.rglob('*') if f.is_file()}
    signal_only=card/'signal_only_REAL';fixture(signal_only,valid_fix=False,navigation=False,pps=False)
    entry=analyze_session(signal_only);assert entry['reception']['nonzero_cn0_observed'] and not entry['position']['valid_position_observed'] and not entry['psins']['fused_output_observed']
    fix_only=card/'fix_only_REAL';fixture(fix_only,navigation=False,pps=False)
    entry=analyze_session(fix_only);assert entry['position']['valid_position_observed'] and not entry['timing']['software_utc_lock_observed'] and not entry['psins']['fused_output_observed']
    indoor=card/'indoor_REAL';fixture(indoor,valid_fix=False,navigation=False,pps=False,visible=False)
    entry=analyze_session(indoor);assert entry['reception']['antenna_ok_observed'] and not entry['reception']['gsv_satellites_reported'] and not entry['reception']['nonzero_cn0_observed']
    partial=card/'unclean_REAL';data=fixture(partial);(partial/'closed.json').unlink();(partial/'stream.inl').write_bytes(data[:-3])
    entry=analyze_session(partial);assert entry['status']=='analyzed' and not entry['integrity']['complete'] and not entry['psins']['complete_navigation_evidence']
    assert 'truncated record at end' in entry['integrity']['errors'] and any('closed.json' in error for error in entry['integrity']['errors'])
    damaged=card/'corrupt_REAL';data=bytearray(fixture(damaged));data[40]^=1;(damaged/'stream.inl').write_bytes(data)
    entry=analyze_session(damaged);assert entry['status']=='error' and 'CRC' in entry['error']
    bad_closed=card/'bad_closed_REAL';fixture(bad_closed);(bad_closed/'closed.json').write_text('[]',encoding='utf8')
    entry=analyze_session(bad_closed);assert entry['status']=='error'
    assert (bad_closed/'closed.json').read_text(encoding='utf8')=='[]'
    sim=card/'test_SIM';fixture(sim,mode='simulation');assert analyze_session(sim)['status']=='simulation_skipped'
    report=analyze(card,p/'reports');assert report['error_sessions']==2 and report['simulation_sessions_skipped']==1 and report['real_summary']['sessions_analyzed']==5
    assert report['real_summary']['psins_fused_output_observed'] and not report['real_summary']['cross_session_continuity_claimed']
    isolated=p/'isolated';isolated.mkdir();shutil.copytree(sim,isolated/'test_SIM')
    report=analyze(isolated,p/'sim-reports',True);assert report['simulation_summary']['psins_fused_output_observed'] and not report['real_summary']['psins_fused_output_observed']
    browser=p/'browser';shutil.copytree(good,browser)
    for name in ['session.json','stream.inl','closed.json']:(browser/name).rename(browser/('download_REAL_'+name))
    assert analyze_session(browser)['psins']['complete_navigation_evidence']
    cli=subprocess.run([sys.executable,'-B',str(root/'tools/analyze_insmini_field.py'),str(card),'--output',str(p/'cli-report')],capture_output=True,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0),timeout=20)
    assert cli.returncode==0,cli.stderr
    assert json.loads((p/'cli-report/field-report.json').read_text(encoding='utf8'))['simulation_sessions_skipped']==1
    assert 'ANTENNA OK仅表示天线电气检测' in (p/'cli-report/field-report.txt').read_text(encoding='utf8')
    try:analyze(card,card/'reports');raise AssertionError('Writing reports into source/card was accepted')
    except ValueError:pass
    try:analyze(card,p/'reports');raise AssertionError('Existing reports overwritten')
    except FileExistsError:pass

real=root/'work/nav3/antenna-live-20261028/153714Z/5bdaaf5_d8159282_REAL'
if real.exists():
    entry=analyze_session(real);assert entry['status']=='analyzed' and entry['integrity']['complete']
    assert entry['reception']['antenna_messages']['ANTENNA OK']==95
    assert not entry['reception']['gsv_satellites_reported'] and not entry['position']['valid_position_observed'] and not entry['timing']['pps_observed']
    print('PASS recorded indoor antenna session:',entry['integrity']['imu_samples'],'IMU samples, 95 ANTENNA OK, no GSV satellites/fix/PPS')
sim=root/'work/nav3/board-tests-v303/2eece100_eb7411f7_SIM'
if sim.exists():
    assert analyze_session(sim)['status']=='simulation_skipped'
    entry=analyze_session(sim,True);assert entry['psins']['fused_output_observed'] and entry['timing']['software_utc_lock_observed'] and entry['position']['valid_position_observed']
    print('PASS recorded simulator session remains separate:',entry['integrity']['imu_samples'],'IMU samples')
print('PASS field analyzer: electrical detection / GSV C/N0 / fix / PPS / UTC / actual PSINS states, source isolation, unclean prefix, CRC rejection, read-only source and no cross-session continuity claim')
