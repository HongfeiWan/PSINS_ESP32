"""Retrieve and validate a TF session over the board's local HTTP interface."""
import argparse,datetime,hashlib,json,pathlib,re,time,urllib.request,urllib.parse,uuid
from insmini_session import export

def pull(address,output,session=None,stop_and_resume=False,allow_simulation=False):
    parsed=urllib.parse.urlsplit(address)
    if parsed.scheme not in ('http','https') or not parsed.hostname or parsed.username or parsed.password or parsed.path not in ('','/') or parsed.query or parsed.fragment:
        raise ValueError('Use a board base address such as http://192.168.8.231')
    base=address.rstrip('/');http=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def request(path,post=False):
        for attempt in range(4):
            try:return http.open(urllib.request.Request(base+path,data=b'' if post else None),timeout=30)
            except OSError:
                if attempt==3:raise
                time.sleep(.3)
    def state():
        with request('/api/telemetry') as r:j=json.load(r)
        if j.get('protocol')!='insmini/1':raise ValueError('Not an INSmini device')
        return j
    def wait(predicate):
        end=time.monotonic()+45
        while time.monotonic()<end:
            j=state()
            if predicate(j):return j
            time.sleep(.2)
        raise TimeoutError('Board did not finish the requested recording transition')
    first=state();sid=session or first['storage']['session']
    if not re.fullmatch(r'[A-Za-z0-9_]{1,63}',sid or ''):raise ValueError('No valid saved session; wait for stationary calibration first')
    if sid.endswith('_SIM') and not allow_simulation:raise ValueError('Simulation requires --allow-simulation')
    busy=first['storage']['enabled'] or first['storage']['active']
    if busy and not stop_and_resume:raise ValueError('Stop TF recording first or add --stop-and-resume')
    folder=pathlib.Path(output)/datetime.datetime.now().strftime('%Y%m%d-%H%M%S')/(sid+'_'+uuid.uuid4().hex[:6])
    folder.mkdir(parents=True,exist_ok=False);resume=busy and first['storage']['enabled'];failure=None;report=None
    try:
        if busy:
            with request('/api/record?enabled=0',True) as r:r.read()
            wait(lambda j:not j['storage']['active'])
        for name in ['session.json','stream.inl','closed.json']:
            print('Reading '+name,flush=True);part=folder/(name+'.part')
            for attempt in range(4):
                try:
                    with request('/api/log?session='+sid+'&file='+name) as r,part.open('wb') as f:
                        expected=int(r.headers['Content-Length']);size=0
                        while True:
                            chunk=r.read(65536)
                            if not chunk:break
                            size+=len(chunk);f.write(chunk)
                    if size!=expected:raise OSError('Incomplete HTTP file body')
                    part.rename(folder/name);break
                except OSError:
                    if attempt==3:raise
                    time.sleep(.3)
            if name=='session.json':
                meta=json.loads((folder/name).read_text(encoding='utf8'))
                if meta.get('mode')=='simulation' and not allow_simulation:raise ValueError('Simulation requires --allow-simulation')
        validation=export(folder,allow_simulation)
        report={'status':'passed','device':first['device'],'firmware':first['firmware'],'session':sid,'folder':str(folder.resolve()),'validation':validation,'files':[{'name':name,'bytes':(folder/name).stat().st_size,'sha256':hashlib.sha256((folder/name).read_bytes()).hexdigest()} for name in ['session.json','stream.inl','closed.json']]}
    except Exception as e:failure=e
    finally:
        if resume:
            try:
                with request('/api/record?enabled=1',True) as r:r.read()
                restored=wait(lambda j:j['storage']['active'])
                if report is not None:report['recording_resumed']=True;report['next_session']=restored['storage']['session']
            except Exception as e:
                if failure is None:failure=RuntimeError('Files retained, but recording could not resume: '+str(e))
        elif report is not None:report['recording_resumed']=False
    if failure:
        (folder/'transfer-error.txt').write_text(str(failure),encoding='utf8');raise failure
    (folder/'transfer.json').write_text(json.dumps(report,indent=2),encoding='utf8');return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('address');p.add_argument('output');p.add_argument('--session');p.add_argument('--stop-and-resume',action='store_true');p.add_argument('--allow-simulation',action='store_true');a=p.parse_args()
    print(json.dumps(pull(a.address,a.output,a.session,a.stop_and_resume,a.allow_simulation),ensure_ascii=False,indent=2))
