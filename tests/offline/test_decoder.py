import pathlib,sys,tempfile,json,struct,zlib
root=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'tools'))
from insmini_session import decode,export,RECORD
with tempfile.TemporaryDirectory() as temp:
    p=pathlib.Path(temp);meta={'format':'INL3','record_bytes':120,'imu_interval':'flags>>2_us','firmware':'3.0.0','mode':'simulation','gyro_bias_rad_s':[0,0,0],'calibration_id':1}
    (p/'session.json').write_text(json.dumps(meta));data=b''
    for seq in range(1,11):
        b=RECORD.pack(0x334c4e49,1,1,10000<<2,seq,1000000+seq*10000,-1,0,0,0,0,0,9.8,25,1,0,0,0,0)
        b=b[:-4]+struct.pack('<I',zlib.crc32(b[:-4]));data+=b
    (p/'stream.inl').write_bytes(data);(p/'closed.json').write_text(json.dumps({'records':10,'bytes':len(data),'drops':0,'write_ok':True}))
    try:decode(p);raise AssertionError('Simulation accepted without opt in')
    except ValueError as e:assert 'Simulation' in str(e)
    r=export(p,True);assert r['complete'] and r['imu_samples']==10
    imu=(p/'psins_imu.txt').read_text().splitlines();assert abs(float(imu[-1].split()[-1])-.1)<1e-12
    for name in ['session.json','stream.inl','closed.json']:(p/name).rename(p/('example_REAL_'+name))
    assert export(p,True)==r
    extra=p/'another_session.json';extra.write_text('{}')
    try:decode(p,True);raise AssertionError('Multiple downloaded sessions accepted')
    except ValueError as e:assert 'exactly one' in str(e)
    extra.unlink()
    for name in ['session.json','stream.inl','closed.json']:(p/('example_REAL_'+name)).rename(p/name)
    damaged=bytearray(data);damaged[40]^=1;(p/'stream.inl').write_bytes(damaged)
    try:decode(p,True);raise AssertionError('CRC corruption accepted')
    except ValueError as e:assert 'CRC' in str(e)
    (p/'stream.inl').write_bytes(data[:-1]);assert not decode(p,True)[2]['complete']
    (p/'stream.inl').write_bytes(data);(p/'closed.json').unlink();assert not decode(p,True)[2]['complete']
print('PASS decoder: source isolation, measured intervals, PSINS increments, browser filenames, ambiguous sessions, CRC corruption, truncation, unclean shutdown')
