'use strict';
const R2D=180/Math.PI;
function qMatrix(q){const [w,x,y,z]=q;return [[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]];}
function transform(m,v){return m.map(r=>r[0]*v[0]+r[1]*v[1]+r[2]*v[2]);}
function slerp(a,b,t){let dot=a.reduce((s,v,i)=>s+v*b[i],0);if(dot<0){b=b.map(v=>-v);dot=-dot;}if(dot>.9995){const r=a.map((v,i)=>v+(b[i]-v)*t),n=Math.hypot(...r);return r.map(v=>v/n);}const theta=Math.acos(Math.max(-1,Math.min(1,dot))),sn=Math.sin(theta);return a.map((v,i)=>(Math.sin((1-t)*theta)*v+Math.sin(t*theta)*b[i])/sn);}
function csvText(record){
  const h=['# INSmini CSV v1','# protocol=insmini/1','# firmware='+record.meta.firmware,'# boot='+record.meta.boot,'# gyro_unit=rad/s','# force_unit=m/s2','# time_unit=us','# body_axes=sensor_X_right_Y_forward_Z_up','# gyro_bias_rad_s='+record.meta.imu.bias_rad_s.join(','),'# initial_q_sensor_to_level='+record.meta.imu.q.join(','),'# calibration_id='+record.meta.imu.calibration_id,'# calibrated='+Number(record.meta.imu.calibrated),'# stopped_reason='+record.reason.replace(/[\r\n]/g,' '),'seq,t_us,dt_us,flags,gx_rad_s,gy_rad_s,gz_rad_s,fx_m_s2,fy_m_s2,fz_m_s2'];
  h.splice(3,0,'# data_mode='+(record.meta.data_mode||'real'));
  return h.join('\n')+'\n'+record.rows.map(r=>r.join(',')).join('\n')+'\n';
}
function canDownload(storage){return !!storage&&!storage.enabled&&!storage.active;}
if(typeof module!=='undefined' && module.exports)module.exports={qMatrix,transform,slerp,csvText,canDownload};
else bootApp();

function bootApp(){
  const $=id=>document.getElementById(id),put=(id,v)=>$(id).textContent=v;
  let latest=null,boot='',since=null,lastReceived=0,record=null,connected=false,calPending=false,storageBusy=null,readingCard=false;
  const maximumRows=120000;
  let targetQ=[1,0,0,0],displayQ=targetQ.slice(),cameraYaw=-1.05,cameraElev=.58;
  const reduced=matchMedia('(prefers-reduced-motion: reduce)').matches;
  function stop(reason){if(record && record.active){record.active=false;record.reason=reason;put('record','开始记录');$('save').disabled=!record.rows.length;}}
  function recordStatus(){
    if(!record){put('recordInfo',latest?.imu.calibrated?'已就绪 · 建议先静置观察，再转动板子':'静止校准完成后即可开始');return;}
    const rows=record.rows,sec=rows.length?(rows.at(-1)[1]-rows[0][1])*1e-6:0;
    put('recordInfo',`${record.active?'正在记录':'记录已停止'} · ${rows.length.toLocaleString()} 个样本 · ${sec.toFixed(1)} 秒${record.active?'':' · '+record.reason}`);
  }
  function notice(text,mode=''){const n=$('notice');n.textContent=text;n.className='notice '+mode;}
  function apply(j){
    if(j.protocol!=='insmini/1' || !Array.isArray(j.rows) || !j.imu || !Array.isArray(j.imu.q))throw Error('不支持的数据格式');
    const oldBoot=boot,oldSince=since;
    if(oldBoot && (j.boot!==oldBoot || (oldSince!==null && j.next<oldSince))){stop('设备重启或序号重置，请保存这一段');since=null;}
    if(j.dropped>0)stop('网络缓冲已溢出，请保存已收到的连续段');
    if(record?.active && (j.imu.calibration_id!==record.meta.imu.calibration_id || !j.imu.calibrated))stop('校准发生变化，请保存这一段');
    if(record?.active && oldBoot===j.boot){
      for(const r of j.rows){
        if(r[0]<=record.startSeq)continue;
        const prev=record.rows.at(-1);
        if(r[0] !== (prev?prev[0]+1:record.startSeq+1) || r[3]!==0){stop('发现采样间断或量程超限，已保留前面的连续数据');break;}
        record.rows.push(r);
        if(record.rows.length>=maximumRows){stop('达到单段采集上限，请下载后开始下一段');break;}
      }
    }
    boot=j.boot;since=j.next;latest=j;lastReceived=performance.now();connected=true;calPending=false;
    const im=j.imu,nav=j.navigation,useNav=!!(nav?.heading_known && [1,2].includes(nav.status) && Array.isArray(nav.q));
    const fresh=im.ready && im.selftest && im.age_ms>=0 && im.age_ms<100;
    $('connectionDot').className='good';put('connection','板子已连接');put('ip',j.wifi.ip);put('device',j.device);put('firmware','INSmini '+j.firmware);
    $('imuDot').className=fresh?'good':'bad';put('imuHealth',fresh?'六轴数据正常':'IMU 数据异常');put('imuModel',`LSM6DS3TR-C · ID 0x${im.who.toString(16).toUpperCase()}`);
    put('rate',im.sample_hz.toFixed(1)+' Hz');put('samples',im.seq.toLocaleString());put('missed',im.missed.toLocaleString());put('age',im.age_ms.toFixed(1)+' ms');
    put('uptime',Math.floor(j.uptime_ms/60000)+' 分 '+Math.floor(j.uptime_ms/1000%60)+' 秒');put('rssi',j.wifi.rssi+' dBm');put('heap',(j.heap/1024).toFixed(0)+' KB');
    ['pitch','roll','yaw'].forEach((id,i)=>put(id,fresh?(useNav?(i===2?((360-nav.att_deg[2])%360):nav.att_deg[i]):im.att_deg[i]).toFixed(1):'—'));
    ['ax','ay','az'].forEach((id,i)=>put(id,fresh?im.force_m_s2[i].toFixed(3):'—'));
    ['gx','gy','gz'].forEach((id,i)=>put(id,fresh?((im.gyro_rad_s[i]-im.bias_rad_s[i])*R2D).toFixed(3):'—'));
    $('calProgress').value=im.calibrated?5:Math.min(5,im.still_s);put('calTitle',im.calibrated?'静止零偏校准完成':'请保持板子静止');
    put('calDetail',im.calibrated?'重置了相对航向的零点':`连续静止 ${Math.min(5,im.still_s).toFixed(1)} / 5.0 秒，移动会重新计时`);
    $('calibrate').disabled=!fresh || !!record?.active || j.data_mode==='simulation';
    $('record').disabled=!fresh || !im.calibrated || j.next!==im.seq || !!(record && !record.active && !record.saved && record.rows.length);
    const g=j.gnss,fix=g.fresh && g.fix>0 && g.rmc==='A';put('gnss',fix?`已定位 · ${g.satellites} 星`:g.valid?'尚未定位 · 等待天线/天空信号':'等待串口数据');put('pps',g.pps.toLocaleString());put('sd',j.sd_present?'已插入':'未插入');
    $('staleOverlay').hidden=fresh;
    if(fresh)targetQ=(useNav?nav.q:im.q).slice();
    put('yawLabel',useNav?'真北方向角':'相对航向');put('attSource',useNav?'三维模型使用 PSINS 组合导航姿态。航向由你设置的真北方向或直行航迹初始化；GNSS 中断时会继续漂移。':'三维模型使用六轴相对姿态，航向以本次静止校准为零点；未建立绝对朝向参考。');
    if(!fresh)notice('IMU 数据暂不可用。请检查板子供电与 USB 日志；当前不更新姿态。','bad');
    else if(!im.calibrated)notice('把板子放在稳定的位置，连续静止 5 秒。校准完成后即可查看相对姿态并记录数据。','warn');
    else if(j.imu_ranges?.saturated_samples>0)notice('本次启动出现过量程超限。请缓慢转动，并在放稳后重新校准；超限样本会中止记录。','warn');
    else if(im.missed>0)notice(`本次启动累计出现 ${im.missed} 个漏采估计。记录会检查每个样本，遇到缺口会停止。`,'warn');
    else if(j.storage && (j.storage.drops>0||j.storage.errors>0))notice('TF 记录出现丢弃或写入错误。请停止记录并检查该段文件，不能将它当成完整实验数据。','warn');
    else notice('六轴数据持续更新。转动板子观察模型；相对航向没有指南针参考，长时间会漂移。');
    if(j.data_mode==='simulation')notice('模拟测试正在运行 · 当前 IMU、定位和 PPS 均为软件生成，不能作为实测结果。','warn');
    put('dataMode',j.data_mode==='simulation'?'模拟数据 · 非实测':'真实传感器');
    const c=j.clock,n=j.navigation,l=j.storage;
    if(c){put('position',fix?`纬度 ${g.lat_deg.toFixed(7)}° · 经度 ${g.lon_deg.toFixed(7)}° · 椭球高 ${g.altitude_valid?g.height_m.toFixed(1)+' m':'待确认'}`:'等待有效 GNSS 定位');put('clockState',c.quality===2?`PPS 已锁定 · 漂移 ${c.drift_ppm.toFixed(1)} ppm · 串口延迟 ${c.latency_ms.toFixed(0)} ms`:c.quality===1?'PPS 短时中断 · 时钟保持':'UTC 尚未锁定 · 使用本机时间');}
    if(n){put('navState',['等待 GNSS 定位、PPS 对时和初始航向','PSINS 组合导航更新中','GNSS 失锁 · 惯性推算','导航过期或数据间断'][n.status]||'等待数据');put('navPosition',n.updates?`${n.lat_deg.toFixed(7)}°, ${n.lon_deg.toFixed(7)}° · 速度 E/N/U ${n.velocity_enu.map(v=>v.toFixed(2)).join(' / ')} m/s`:'—');}
    if(j.initial_heading){const h=j.initial_heading;put('savedHeading',h.saved_mode==='manual'?`离线启动方向：Y 轴相对真北顺时针 ${h.saved_degrees.toFixed(1)}°。定位前请保持这一朝向。`:h.saved_mode==='forward'?'离线启动方向：Y 轴沿前进方向，等待直行航迹对准。':'离线初始方向尚未保存；仍可完整记录搜星、定位和 PPS。');}
    if(l){put('tfState',l.active&&!l.enabled?'正在保存缓存，请稍等':l.active?`正在自主记录 · ${l.records.toLocaleString()} 条`:l.enabled?'等待插卡或静止校准':'已停止，可下载或断电');put('tfDetails',`${(l.bytes/1024).toFixed(1)} KB · 丢弃 ${l.drops} · 错误 ${l.errors}${l.error?' · '+l.error:''}`);put('tfToggle',l.enabled?'停止 TF 记录':'启动 TF 记录');$('tfToggle').disabled=l.active&&!l.enabled;}
    if(l && storageBusy!==!canDownload(l)){storageBusy=!canDownload(l);logLinks();put('storageNotice',storageBusy?'记录期间仅列出当前段；停止后刷新，可查看历史记录并下载。':'已停止。刷新可查看历史记录，缺少结束检查文件的记录可能未正常保存。');}
    recordStatus();
  }
  async function poll(){
    if(readingCard){setTimeout(poll,100);return;}
    const start=performance.now();
    try{const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),2500);let response;
      try{response=await fetch('/api/telemetry'+(since===null?'':'?since='+since),{cache:'no-store',signal:controller.signal});if(!response.ok)throw Error('HTTP '+response.status);const j=await response.json();if(!readingCard)apply(j);}finally{clearTimeout(timer);}
    }catch(e){if(!readingCard){connected=false;$('connectionDot').className='bad';put('connection','连接中断，正在重试');$('staleOverlay').hidden=false;$('calibrate').disabled=$('record').disabled=true;notice('暂时连不上板子。请检查供电和 Wi-Fi；恢复连接后会尝试补取缓冲样本。','bad');if(record?.active && performance.now()-lastReceived>8000)stop('连接中断超过缓冲时间，请保存这一段');recordStatus();}}
    setTimeout(poll,Math.max(25,100-(performance.now()-start)));
  }
  $('calibrate').addEventListener('click',async()=>{
    if(calPending || record?.active)return;calPending=true;$('calibrate').disabled=true;
    try{const r=await fetch('/api/calibrate',{method:'POST'});if(!r.ok)throw Error();notice('已请求重新校准，请把板子放稳，连续静止 5 秒。','warn');}
    catch{notice('校准请求未送达，请恢复连接后重试。','bad');calPending=false;}
  });
  $('record').addEventListener('click',()=>{
    if(record?.active){stop('手动停止');recordStatus();return;}
    if(!latest?.imu.calibrated || !connected || latest.next!==latest.imu.seq)return;
    if(record?.rows.length && !record.saved){notice('上一段数据还未下载，请先点击“下载 CSV”。','warn');return;}
    record={active:true,rows:[],startSeq:latest.next,meta:JSON.parse(JSON.stringify(latest)),reason:'手动停止',saved:false};$('save').disabled=true;put('record','停止记录');$('calibrate').disabled=true;recordStatus();
  });
  $('save').addEventListener('click',()=>{
    if(!record?.rows.length || record.active)return;
    const a=document.createElement('a'),url=URL.createObjectURL(new Blob([csvText(record)],{type:'text/csv;charset=utf-8'}));
    a.href=url;a.download='insmini_'+new Date().toISOString().replace(/[:.]/g,'-')+'.csv';a.click();record.saved=true;setTimeout(()=>URL.revokeObjectURL(url),10000);recordStatus();
  });
  async function post(url){const r=await fetch(url,{method:'POST'});if(!r.ok)throw Error(await r.text());return r;}
  function cardRead(active){
    readingCard=active;$('refreshSessions').disabled=active;
    if(active){stop('正在读取 TF，电脑端采集已停止');recordStatus();$('record').disabled=$('calibrate').disabled=$('tfToggle').disabled=true;put('connection','正在读取 TF');put('staleOverlay','正在读取 TF · 保留最后一次姿态');$('staleOverlay').hidden=false;}
    else{since=null;put('staleOverlay','连接中断 · 保留最后一次姿态');}
  }
  function downloadLog(){
    if(readingCard||!canDownload(latest?.storage))return false;cardRead(true);
    put('storageNotice','请在浏览器中确认文件下载完成，再点击 ');const resume=document.createElement('button');resume.textContent='恢复实时显示';resume.className='quiet';resume.addEventListener('click',()=>{cardRead(false);put('storageNotice','实时显示已恢复。同一段的三个文件放入同一文件夹后导入。');});$('storageNotice').append(resume);return true;
  }
  function logLinks(){const id=$('sessions').value,box=$('logLinks');box.replaceChildren();if(!id)return;for(const file of ['session.json','stream.inl','closed.json']){const a=document.createElement('a');a.textContent=file==='stream.inl'?'下载记录':file==='session.json'?'记录说明':'结束检查';if(canDownload(latest?.storage)){a.href='/api/log?session='+encodeURIComponent(id)+'&file='+file;a.addEventListener('click',e=>{if(!downloadLog())e.preventDefault();});}else{a.title='请先停止 TF 记录并等待保存完成';a.setAttribute('aria-disabled','true');}box.append(a);}}
  async function sessions(){if(readingCard)return;cardRead(true);try{const r=await fetch('/api/storage'),j=await r.json(),select=$('sessions'),old=select.value;select.replaceChildren();for(const id of j.sessions.slice().reverse()){const option=document.createElement('option');option.value=option.textContent=id;select.append(option);}if(j.sessions.includes(old))select.value=old;logLinks();put('storageNotice',canDownload(latest?.storage)?'已停止。缺少结束检查文件的记录可能未正常保存。':'记录期间仅列出当前段；停止后刷新，可查看历史记录并下载。');}catch{put('storageNotice','记录列表读取失败，请稍后重试。');}finally{cardRead(false);}}
  $('refreshSessions').addEventListener('click',sessions);$('sessions').addEventListener('change',logLinks);
  $('tfToggle').addEventListener('click',async()=>{if(!latest?.storage)return;$('tfToggle').disabled=true;try{await post('/api/record?enabled='+(latest.storage.enabled?'0':'1'));put('storageNotice','请求已送达，正在刷新状态。');}catch(e){put('storageNotice',e.message);}});
  async function heading(mode){try{let url='/api/navigation?mode='+mode;if(mode==='manual'){const v=$('heading').value;if(v===''||!Number.isFinite(+v)||+v<0||+v>=360)throw Error('请输入 0 至 359.9 度的真北方向角。');url+='&heading='+encodeURIComponent(v);}if($('persistHeading').checked)url+='&persist=1';await post(url);put('headingResult','方向已设置。等待有效 GNSS/PPS；定位前请保持所设朝向。');}catch(e){put('headingResult',e.message);}}
  $('setHeading').addEventListener('click',()=>heading('manual'));$('trackHeading').addEventListener('click',()=>heading('forward'));$('clearHeading').addEventListener('click',()=>heading('clear'));
  sessions();
  window.addEventListener('beforeunload',e=>{if(record?.rows.length && !record.saved){e.preventDefault();e.returnValue='';}});

  const canvas=$('scene'),ctx=canvas.getContext('2d');let width=0,height=0,drag=null;
  const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],unit=a=>{const n=Math.hypot(...a);return a.map(v=>v/n);};
  new ResizeObserver(()=>{const d=Math.min(2,devicePixelRatio||1);width=canvas.clientWidth;height=canvas.clientHeight;canvas.width=width*d;canvas.height=height*d;ctx.setTransform(d,0,0,d,0,0);}).observe(canvas);
  canvas.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);});
  canvas.addEventListener('pointermove',e=>{if(drag){cameraYaw+=(e.clientX-drag[0])*.008;cameraElev=Math.max(.12,Math.min(1.4,cameraElev+(e.clientY-drag[1])*.006));drag=[e.clientX,e.clientY];}});
  canvas.addEventListener('pointerup',()=>drag=null);canvas.addEventListener('pointercancel',()=>drag=null);$('viewReset').addEventListener('click',()=>{cameraYaw=-1.05;cameraElev=.58;});
  const boxes=[
    [[0,0,0],[1.3,1.44,.055],[31,127,119]],
    [[-.21,.31,.11],[.64,.69,.17],[180,190,193]],
    [[-.21,.66,.09],[.64,.15,.13],[31,43,52]],
    [[.39,.18,.075],[.24,.27,.08],[57,66,71]],
    [[.29,-.32,.09],[.36,.38,.12],[170,180,184]],
    [[-.28,-.73,.075],[.34,.22,.14],[170,181,190]],
    [[.5,-.53,.045],[.1,.1,.03],[174,125,62]],
    [[-.51,-.19,.04],[.07,.1,.025],[190,160,81]],
    [[-.51,-.36,.04],[.07,.1,.025],[190,160,81]],
    [[-.51,-.53,.04],[.07,.1,.025],[190,160,81]]
  ];
  function frame(){
    if(width && height){
      displayQ=reduced?targetQ.slice():slerp(displayQ,targetQ,.24);
      const m=qMatrix(displayQ),eye=[4*Math.cos(cameraElev)*Math.cos(cameraYaw),4*Math.cos(cameraElev)*Math.sin(cameraYaw),4*Math.sin(cameraElev)];
      const right=unit(cross(unit(eye),[0,0,1])).map(v=>-v),up=unit(cross(unit(eye),right));
      function project(v){const depth=4-dot(v,unit(eye)),scale=Math.min(width*.88,height*1.18)/Math.max(1,depth);return [width*.5+dot(v,right)*scale,height*.49-dot(v,up)*scale,depth];}
      function line(a,b,color,w=1){const p=project(a),q=project(b);ctx.beginPath();ctx.moveTo(p[0],p[1]);ctx.lineTo(q[0],q[1]);ctx.strokeStyle=color;ctx.lineWidth=w;ctx.stroke();return [p,q];}
      ctx.clearRect(0,0,width,height);
      for(let i=-6;i<=6;i++){let k=i*.24;line([-1.44,k,-.45],[1.44,k,-.45],i===0?'#c5d4df':'#e7edf2');line([k,-1.44,-.45],[k,1.44,-.45],i===0?'#c5d4df':'#e7edf2');}
      const faces=[];
      for(const [c,size,color] of boxes){
        const vertices=[];for(let z=-1;z<=1;z+=2)for(let y=-1;y<=1;y+=2)for(let x=-1;x<=1;x+=2)vertices.push(transform(m,[c[0]+x*size[0]/2,c[1]+y*size[1]/2,c[2]+z*size[2]/2]));
        for(const ids of [[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]]){
          const v=ids.map(i=>vertices[i]),normal=unit(cross(v[1].map((x,i)=>x-v[0][i]),v[2].map((x,i)=>x-v[0][i]))),shade=.68+.3*Math.abs(dot(normal,unit([-.4,-.6,1])));
          faces.push({points:v.map(project),depth:v.reduce((s,p)=>s+project(p)[2],0)/4,color:`rgb(${color.map(v=>Math.round(v*shade)).join(',')})`});
        }
      }
      faces.sort((a,b)=>b.depth-a.depth);
      for(const f of faces){ctx.beginPath();f.points.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.closePath();ctx.fillStyle=f.color;ctx.fill();ctx.strokeStyle='rgba(20,42,53,.16)';ctx.lineWidth=.7;ctx.stroke();}
      for(const [vec,label,color] of [[[1.02,0,.02],'X','#c65c4b'],[[0,1.07,.02],'Y','#168577'],[[0,0,1.0],'Z','#246bce']]){
        const [,p]=line(transform(m,[0,0,.1]),transform(m,vec),color,2);ctx.fillStyle=color;ctx.beginPath();ctx.arc(p[0],p[1],3,0,Math.PI*2);ctx.fill();ctx.font='600 14px Segoe UI';ctx.fillText(label,p[0]+8,p[1]+3);
      }
    }
    requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);poll();
}
