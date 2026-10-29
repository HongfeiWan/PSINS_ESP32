"""Stream real sensor samples from an INSmini LAN address, preserving gaps/errors.

python record_insmini.py http://192.168.8.231 output.csv --seconds 60
Only creates new files. No Wi-Fi password or third-party dependency is required.
"""
import argparse
import json
import math
import time
import urllib.error
import urllib.request
from pathlib import Path

COLUMNS = 'seq,t_us,dt_us,flags,gx_rad_s,gy_rad_s,gz_rad_s,fx_m_s2,fy_m_s2,fz_m_s2'


def get(base, since=None):
    suffix = '' if since is None else '?since=' + str(since)
    with urllib.request.urlopen(base + '/api/telemetry' + suffix, timeout=2.5) as r:
        j = json.load(r)
    if j.get('protocol') != 'insmini/1':
        raise ValueError('Unsupported device protocol')
    return j


def header(j):
    im = j['imu']
    fields = [('protocol', j['protocol']), ('firmware', j['firmware']), ('boot', j['boot']),
              ('data_mode', j.get('data_mode','real')),
              ('gyro_unit', 'rad/s'), ('force_unit', 'm/s2'), ('time_unit', 'us'),
              ('body_axes', 'sensor_X_right_Y_forward_Z_up'),
              ('gyro_bias_rad_s', ','.join(map(str, im['bias_rad_s']))),
              ('initial_q_sensor_to_level', ','.join(map(str, im['q']))),
              ('calibration_id', im['calibration_id']), ('calibrated', int(im['calibrated']))]
    return '# INSmini CSV v1\n' + ''.join(f'# {k}={v}\n' for k, v in fields) + COLUMNS + '\n'


def capture(base, filename, seconds, allow_simulation=False):
    base = base.rstrip('/')
    j = get(base)
    mode=j.get('data_mode','real')
    if mode not in ('real','simulation') or (mode=='simulation' and not allow_simulation):
        raise ValueError('Simulated data requires explicit --allow-simulation')
    if not (j['imu']['ready'] and j['imu']['selftest'] and j['imu']['calibrated'] and j['imu']['age_ms'] < 100):
        raise ValueError('IMU is not ready and calibrated; leave the board still for five seconds')
    if j['next'] != j['imu']['seq']:
        raise ValueError('Initial snapshot is not current')
    since, boot, cal = j['next'], j['boot'], j['imu']['calibration_id']
    previous_time, count = None, 0
    started = last_ok = time.monotonic()
    reason = 'duration complete'
    # Exclusive creation protects existing recordings. Partial data survives errors.
    with Path(filename).open('x', encoding='utf-8', newline='') as f:
        f.write(header(j)); f.flush()
        try:
            while time.monotonic() - started < seconds:
                tick = time.monotonic()
                try:
                    j = get(base, since)
                except (OSError, urllib.error.URLError, TimeoutError):
                    if time.monotonic() - last_ok > 8:
                        raise ValueError('Disconnected longer than the ring buffer recovery window')
                    time.sleep(.15)
                    continue
                if j.get('data_mode','real') != mode or j['boot'] != boot or not j['imu']['calibrated'] or j['imu']['calibration_id'] != cal:
                    raise ValueError('Device restarted or calibration changed')
                if j['dropped']:
                    raise ValueError('Network buffer overflow')
                for row in j['rows']:
                    if len(row) != 10 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in row):
                        raise ValueError('Invalid sensor row')
                    if row[0] != since+1 or row[3] != 0:
                        raise ValueError('Sample sequence, hardware gap or sensor saturation')
                    if not 4000 <= row[2] <= 20000 or (previous_time is not None and abs(row[1]-previous_time-row[2]) > 1):
                        raise ValueError('Invalid sample timing')
                    f.write(','.join(map(str, row))+'\n')
                    since, previous_time = row[0], row[1]
                    count += 1
                f.flush(); last_ok = time.monotonic()
                time.sleep(max(0, .1-(time.monotonic()-tick)))
        except KeyboardInterrupt:
            reason = 'user stopped'
        except Exception as e:
            reason = str(e)
            raise
        finally:
            # A trailing comment is accepted by the companion loader.
            f.write('# stopped_reason='+reason.replace('\n',' ')+'\n')
            print(json.dumps({'samples': count, 'seconds': round(time.monotonic()-started, 3),
                              'reason': reason, 'file': str(Path(filename).resolve())}, ensure_ascii=False))
    return count


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('url'); p.add_argument('output'); p.add_argument('--seconds', type=float, default=60)
    p.add_argument('--allow-simulation',action='store_true')
    a = p.parse_args()
    if not a.url.startswith('http://') or not math.isfinite(a.seconds) or a.seconds <= 0:
        p.error('Use an http:// LAN address and a positive duration')
    capture(a.url, a.output, a.seconds,a.allow_simulation)
