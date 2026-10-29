"""Read TF sessions and report observed reception, timing and PSINS evidence."""
import argparse
import collections
import datetime
import json
import math
import pathlib
import struct

from insmini_session import decode


def session_folders(source):
    if (source/'session.json').exists() or list(source.glob('*_session.json')):
        return [source]
    return sorted({p.parent for pattern in ('session.json', '*_session.json') for p in source.rglob(pattern)})


def metadata(folder):
    path=folder/'session.json'
    if not path.exists():
        paths=list(folder.glob('*_session.json'))
        if len(paths)!=1:raise ValueError('Put exactly one session and its files in each folder')
        path=paths[0]
    value=json.loads(path.read_text(encoding='utf8'))
    if not isinstance(value,dict):raise ValueError('Session metadata must be an object')
    return value


def nmea_lines(records):
    pending=bytearray();first=None;fragment_errors=0;lines=[]
    for kind,flags,seq,mono,utc,values in records:
        if kind!=5:continue
        length=flags&255
        if not 0<length<=88:
            fragment_errors+=1;pending.clear();first=None;continue
        chunk=struct.pack('<11d',*values)[:length]
        if chunk.startswith(b'$') and pending:
            fragment_errors+=1;pending.clear();first=None
        if first is None:first=mono
        pending+=chunk
        if not flags&0x100:
            try:lines.append((first,pending.decode('ascii')))
            except UnicodeDecodeError:fragment_errors+=1
            pending.clear();first=None
    if pending:fragment_errors+=1
    return lines,fragment_errors


def number(text):
    value=float(text)
    if not math.isfinite(value):raise ValueError('Non-finite NMEA number')
    return value


def integer(text,low,high):
    value=number(text)
    if value!=int(value) or not low<=value<=high:raise ValueError('Invalid NMEA integer')
    return int(value)


def time_of_day(text):
    value=number(text);hour=int(value//10000);minute=int(value//100)%100;second=value-hour*10000-minute*100
    if value<0 or hour>23 or minute>59 or not 0<=second<60:raise ValueError('Invalid NMEA time')
    return round((hour*3600+minute*60+second)*1e6)


def coordinate(text,hemisphere,latitude):
    value=number(text);degrees=int(value//100);minutes=value-degrees*100;limit=90 if latitude else 180
    if value<0 or not 0<=minutes<60 or degrees>limit or (degrees==limit and minutes):raise ValueError('Invalid coordinate')
    if hemisphere not in (('N','S') if latitude else ('E','W')):raise ValueError('Invalid hemisphere')
    return (degrees+minutes/60)*(-1 if hemisphere in ('S','W') else 1)


def reception(records):
    lines,fragments=nmea_lines(records);counts=collections.Counter();antenna=collections.Counter();gsv={}
    bad_checksum=malformed=rmc_active=gga_positive=coherent=0;gga=rmc=None;last_epoch=None
    first_fix_utc=last_fix_utc=None
    for mono,line in lines:
        try:
            if not line.startswith('$') or line.count('*')!=1:raise ValueError('NMEA framing')
            body,checksum=line[1:].split('*');check=0
            for char in body:check^=ord(char)
            if len(checksum)!=2 or any(char not in '0123456789abcdefABCDEF' for char in checksum) or int(checksum,16)!=check:
                bad_checksum+=1;continue
            fields=body.split(',');tag=fields[0];kind=tag[-3:];counts[tag]+=1
            if kind=='TXT':
                text=','.join(fields[4:])
                if text.startswith('ANTENNA '):antenna[text]+=1
            elif kind=='GSV':
                integer(fields[1],1,99);integer(fields[2],1,int(fields[1]));visible=integer(fields[3],0,255)
                stat=gsv.setdefault(tag,{'reports':0,'max_satellites_in_view':0,'max_cn0_dbhz':None,'nonzero_cn0_samples':0,'satellite_ids_with_cn0':set()})
                stat['reports']+=1;stat['max_satellites_in_view']=max(stat['max_satellites_in_view'],visible)
                payload=fields[4:]
                if len(payload)%4==1:payload=payload[:-1] # NMEA 4.x optional signal ID.
                if len(payload)%4:raise ValueError('Incomplete GSV satellite group')
                for start in range(0,len(payload),4):
                    satellite,elevation,azimuth,cn0=payload[start:start+4]
                    if not satellite or not cn0:continue
                    integer(satellite,1,999);signal=number(cn0)
                    if not 0<=signal<=99:raise ValueError('Invalid GSV C/N0')
                    stat['max_cn0_dbhz']=signal if stat['max_cn0_dbhz'] is None else max(stat['max_cn0_dbhz'],signal)
                    if signal>0:stat['nonzero_cn0_samples']+=1;stat['satellite_ids_with_cn0'].add(satellite)
            elif kind=='GGA':
                gga=None
                if len(fields)<10:raise ValueError('Short GGA')
                quality=integer(fields[6],0,8)
                if quality:
                    gga_positive+=1
                    tod=time_of_day(fields[1]);lat=coordinate(fields[2],fields[3],True);lon=coordinate(fields[4],fields[5],False)
                    satellites=integer(fields[7],0,99);hdop=number(fields[8])
                    if satellites>=4 and 0<hdop<=10:gga=(tod,mono,lat,lon)
            elif kind=='RMC':
                rmc=None
                if len(fields)<10:raise ValueError('Short RMC')
                if fields[2]=='A':
                    rmc_active+=1;tod=time_of_day(fields[1]);lat=coordinate(fields[3],fields[4],True);lon=coordinate(fields[5],fields[6],False)
                    date=fields[9]
                    if len(date)!=6 or not date.isdigit():raise ValueError('Invalid RMC date')
                    day=datetime.date(2000+int(date[4:]),int(date[2:4]),int(date[:2]));speed=number(fields[7])
                    if not 0<=speed<=515:raise ValueError('Invalid RMC speed')
                    rmc=(tod,mono,lat,lon,day)
            if gga and rmc and abs(gga[0]-rmc[0])<=1000 and abs(gga[1]-rmc[1])<=1500000 and abs(gga[2]-rmc[2])<.0001 and abs(gga[3]-rmc[3])<.0001:
                epoch=(rmc[4],rmc[0])
                if epoch!=last_epoch:
                    last_epoch=epoch;coherent+=1
                    stamp=(datetime.datetime.combine(rmc[4],datetime.time(),datetime.timezone.utc)+datetime.timedelta(microseconds=rmc[0])).isoformat()
                    if first_fix_utc is None:first_fix_utc=stamp
                    last_fix_utc=stamp
        except (ValueError,IndexError,OverflowError):malformed+=1
    for stat in gsv.values():stat['satellite_ids_with_cn0']=sorted(stat['satellite_ids_with_cn0'],key=int)
    return {'raw_sentences':len(lines),'sentence_counts':dict(counts),'checksum_errors':bad_checksum,'malformed_sentences':malformed,'fragment_errors':fragments,'antenna_messages':dict(antenna),'antenna_ok_observed':antenna['ANTENNA OK']>0,'antenna_detection_note':'ANTENNA OK is electrical detection, not a radio reception pass.','gsv':gsv,'gsv_satellites_reported':any(s['max_satellites_in_view']>0 for s in gsv.values()),'nonzero_cn0_observed':any(s['nonzero_cn0_samples']>0 for s in gsv.values()),'gga_positive_quality_sentences':gga_positive,'rmc_active_status_sentences':rmc_active,'raw_coherent_position_epochs':coherent,'first_fix_utc':first_fix_utc,'last_fix_utc':last_fix_utc}


def analyze_session(folder,allow_simulation=False):
    entry={'folder':str(folder.resolve()),'session':folder.name}
    try:
        meta=metadata(folder);entry['source']=meta.get('mode')
        if meta.get('mode')=='simulation' and not allow_simulation:
            return {**entry,'status':'simulation_skipped','note':'模拟记录未参与真实外场结论；需要显式 --allow-simulation 才分析。'}
        meta,records,integrity=decode(folder,allow_simulation)
        radio=reception(records);gps=[r for r in records if r[0]==2];pps=[r for r in records if r[0]==4];nav=[r for r in records if r[0]==3]
        valid_gps=[r for r in gps if r[4]>=0 and abs(r[5][0])<=math.pi/2 and abs(r[5][1])<=math.pi and 0<r[5][5]<=10 and 1<=r[5][6]<=8 and r[5][6]==int(r[5][6]) and 4<=r[5][7]<=99 and r[5][7]==int(r[5][7])]
        intervals=[b[3]-a[3] for a,b in zip(pps,pps[1:])]
        states=collections.Counter(r[1] for r in nav);locked=any(r[5][0]==2 and r[4]>=0 for r in pps) or any(r[5][10]>0 for r in valid_gps)
        fixed=bool(valid_gps) or radio['raw_coherent_position_epochs']>0
        qualified=integrity['complete'] and integrity['imu_samples']>0 and fixed and locked and states[1]>0
        return {**entry,'status':'analyzed','firmware':meta.get('firmware'),'boot':meta.get('boot'),'heading_mode':meta.get('heading_mode','not recorded'),'heading_deg':meta.get('heading_deg'),'integrity':integrity,'reception':radio,'position':{'valid_position_observed':fixed,'firmware_coherent_fix_records':len(valid_gps),'invalid_fix_records':len(gps)-len(valid_gps)},'timing':{'pps_records':len(pps),'pps_observed':bool(pps),'software_utc_lock_observed':locked,'utc_imu_samples':integrity['utc_imu_samples'],'pps_interval_us_min':min(intervals) if intervals else None,'pps_interval_us_max':max(intervals) if intervals else None,'physical_pps_accuracy_verified':False},'psins':{'output_records':len(nav),'state_counts':dict(states),'fused_output_observed':states[1]>0,'coasting_output_observed':states[2]>0,'stale_or_error_output_observed':states[3]>0,'complete_navigation_evidence':qualified,'note':'没有导航输出时需核对初始朝向、有效定位和PPS；不能据此断言PSINS无法运行。'},'source_warning':'模拟证据，不能作为真实卫星接收或导航精度结果。' if meta['mode']=='simulation' else '真实记录观察结果；没有完成定位精度或物理PPS时序验收。'}
    except (OSError,ValueError,struct.error,TypeError,KeyError,AttributeError) as error:
        return {**entry,'status':'error','error':str(error),'note':'该段未用于接收、对时或导航通过结论；未修改源文件。'}


def summarize(sessions,source):
    selected=[s for s in sessions if s.get('source')==source and s['status']=='analyzed']
    return {'sessions_analyzed':len(selected),'complete_sessions':sum(s['integrity']['complete'] and s['integrity']['imu_samples']>0 for s in selected),'gsv_satellites_reported':any(s['reception']['gsv_satellites_reported'] for s in selected),'nonzero_cn0_observed':any(s['reception']['nonzero_cn0_observed'] for s in selected),'valid_position_observed':any(s['position']['valid_position_observed'] for s in selected),'pps_observed':any(s['timing']['pps_observed'] for s in selected),'software_utc_lock_observed':any(s['timing']['software_utc_lock_observed'] for s in selected),'psins_fused_output_observed':any(s['psins']['fused_output_observed'] for s in selected),'complete_navigation_evidence_sessions':sum(s['psins']['complete_navigation_evidence'] for s in selected),'cross_session_continuity_claimed':False}


def text_report(report):
    yes=lambda value:'曾观察到' if value else '未观察到'
    summary=report['real_summary'];lines=['INSmini TF 外场记录分析','',f"真实记录：已分析 {summary['sessions_analyzed']} 段；完整段 {summary['complete_sessions']} 段。",f"GSV卫星条目：{yes(summary['gsv_satellites_reported'])}；非零载噪比：{yes(summary['nonzero_cn0_observed'])}。",f"有效定位：{yes(summary['valid_position_observed'])}；PPS脉冲：{yes(summary['pps_observed'])}；软件UTC锁定：{yes(summary['software_utc_lock_observed'])}。",f"PSINS融合状态输出：{yes(summary['psins_fused_output_observed'])}；完整且包含定位、锁定、融合证据的记录 {summary['complete_navigation_evidence_sessions']} 段。",'','ANTENNA OK仅表示天线电气检测；GSV条目不等于有效定位，定位不等于对时，也不等于组合导航初始化。','没有导航输出时需核对初始朝向、有效定位和PPS，不能据此断言PSINS无法运行。','未正常结束的段只分析已通过CRC的完整记录前缀，不作为合格的连续导航数据；未修补或改写源文件。','各段分别检查，不宣称跨段连续；模拟记录与真实记录分开统计。','']
    for session in report['sessions']:
        lines.append(session['session']+'：'+session['status'])
        if session['status']=='error':lines.append('  错误：'+session['error']);continue
        if session['status']=='simulation_skipped':lines.append('  '+session['note']);continue
        r=session['reception'];integrity=session['integrity'];p=session['psins']
        lines.append(f"  来源 {session['source']}；IMU {integrity['imu_samples']} 样本；{integrity['duration_s']:.3f} 秒；完整性 {'通过' if integrity['complete'] and integrity['imu_samples'] else '不通过'}。")
        for error in integrity['errors']:lines.append('  完整性问题：'+error)
        for tag,stat in r['gsv'].items():lines.append(f"  {tag}：最多 {stat['max_satellites_in_view']} 颗卫星条目；最大载噪比 {stat['max_cn0_dbhz'] if stat['max_cn0_dbhz'] is not None else '无有效数值'} dB-Hz。")
        lines.append(f"  原始同历元定位 {r['raw_coherent_position_epochs']} 次，固件有效定位记录 {session['position']['firmware_coherent_fix_records']} 条，PPS {session['timing']['pps_records']} 条，导航状态计数 {p['state_counts']}。")
        if r['antenna_messages']:lines.append('  天线检测：'+json.dumps(r['antenna_messages'],ensure_ascii=False))
        if r['checksum_errors'] or r['malformed_sentences'] or r['fragment_errors']:lines.append(f"  原始报文问题：校验 {r['checksum_errors']}，格式 {r['malformed_sentences']}，片段 {r['fragment_errors']}。")
    lines+=['','本报告验证记录中出现过的状态，不代表已验证室外定位精度、物理PPS历元相位或长期可靠性。']
    return '\n'.join(lines)+'\n'


def analyze(source,output,allow_simulation=False):
    source=pathlib.Path(source).resolve(strict=True);output=pathlib.Path(output).resolve()
    if not source.is_dir():raise ValueError('Choose a session folder or the TF INSmini folder')
    if output==source or output.is_relative_to(source):raise ValueError('Output must be outside the input/card folder; source files are read-only')
    folders=session_folders(source)
    if not folders:raise ValueError('No session.json files found')
    targets=[output/'field-report.json',output/'field-report.txt']
    if any(p.exists() for p in targets):raise FileExistsError('Report files already exist; choose a new output folder')
    sessions=[analyze_session(folder,allow_simulation) for folder in folders]
    report={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'input':str(source),'allow_simulation':allow_simulation,'real_summary':summarize(sessions,'real'),'simulation_summary':summarize(sessions,'simulation'),'error_sessions':sum(s['status']=='error' for s in sessions),'simulation_sessions_skipped':sum(s['status']=='simulation_skipped' for s in sessions),'sessions':sessions}
    output.mkdir(parents=True,exist_ok=True)
    targets[0].write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8');targets[1].write_text(text_report(report),encoding='utf8')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('input');parser.add_argument('--output',required=True);parser.add_argument('--allow-simulation',action='store_true');args=parser.parse_args()
    report=analyze(args.input,args.output,args.allow_simulation)
    print(json.dumps({'real_summary':report['real_summary'],'error_sessions':report['error_sessions'],'simulation_sessions_skipped':report['simulation_sessions_skipped'],'output':str(pathlib.Path(args.output).resolve())},ensure_ascii=False,indent=2))
