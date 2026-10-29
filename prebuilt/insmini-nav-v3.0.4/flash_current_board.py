import datetime, hashlib, json, pathlib, subprocess, sys
from serial.tools import list_ports

root = pathlib.Path(__file__).resolve().parent
out = root/'flash-logs';out.mkdir(exist_ok=True)
tag = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
manifest=json.loads((root/'MANIFEST.json').read_text(encoding='utf8'))
image=root/('rollback/firmware-3.0.3.bin' if '--rollback' in sys.argv else 'flash-images/firmware.bin')
partitions=root/'flash-images/partitions.bin';rollback=root/'rollback/firmware-3.0.3.bin'
for file in [image,partitions,rollback]:
 expected=next(x for x in manifest['files'] if x['path']==file.relative_to(root).as_posix())
 assert hashlib.sha256(file.read_bytes()).hexdigest()==expected['sha256']
assert 100000<image.stat().st_size<3342336
ports = [p for p in list_ports.comports() if p.vid==0x303a and p.pid==0x1001]
assert len(ports)==1, 'Expected exactly one ESP32 USB board'
base = [sys.executable,'-m','esptool','--chip','esp32s3','--port',ports[0].device,'--baud','460800']

def run(args, name):
    r = subprocess.run(base+args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=100,
                       creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    (out/(tag+'-'+name+'.log')).write_bytes(r.stdout)
    text = r.stdout.decode('utf-8','replace')
    if r.returncode: raise RuntimeError(text[-4000:])
    return text

identity = run(['--before','default-reset','--after','no-reset','read-mac'],'identity')
assert '80:45:6b:44:07:68' in identity.lower(), 'Unexpected board MAC; no write performed'
saved_partition = out/(tag+'-board-partitions.bin')
run(['--before','no-reset','--after','no-reset','read-flash','0x8000','3072',str(saved_partition)],'read-partitions')
assert saved_partition.read_bytes()==partitions.read_bytes(), 'Partition layout mismatch; no write performed'
log = run(['--before','no-reset','--after','hard-reset','write-flash','--flash-mode','dio','--flash-freq','80m','--flash-size','8MB','0x10000',str(image)],'flash')
assert 'Hash of data verified.' in log
manifest = {'timeUtc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'boardMac':'80:45:6b:44:07:68',
            'port':ports[0].device,'offset':'0x10000','imageSha256':hashlib.sha256(image.read_bytes()).hexdigest(),
            'imageBytes':image.stat().st_size,'deviceChecksumVerified':True,'partitionUnchanged':True,
            'rollback':str(rollback),'rollbackSha256':hashlib.sha256(rollback.read_bytes()).hexdigest()}
(out/(tag+'-flash.json')).write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print(log[-1000:]); print(json.dumps(manifest,indent=2))
