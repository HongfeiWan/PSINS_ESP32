const assert=require('node:assert/strict');
const {qMatrix,transform,slerp,csvText,canDownload}=require('../../firmware/insmini-nav/web/app.js');
const close=(a,b)=>assert(Math.abs(a-b)<1e-10);
assert(!canDownload(null));assert(!canDownload({enabled:true,active:true}));assert(!canDownload({enabled:true,active:false}));assert(!canDownload({enabled:false,active:true}));assert(canDownload({enabled:false,active:false}));
const q=[Math.SQRT1_2,0,0,Math.SQRT1_2];
const v=transform(qMatrix(q),[1,0,0]);close(v[0],0);close(v[1],1);close(v[2],0);
const half=slerp([1,0,0,0],q,.5);close(half[0],Math.cos(Math.PI/8));close(half[3],Math.sin(Math.PI/8));
const neg=slerp(q,q.map(x=>-x),.5);neg.forEach((v,i)=>close(v,q[i]));
const csv=csvText({meta:{firmware:'test',boot:'test',imu:{q:[1,0,0,0],bias_rad_s:[.01,.02,.03],calibrated:true,calibration_id:1}},reason:'manual',rows:[[1,10000,10000,0,0,0,0,0,0,9.8]]});
assert(csv.includes('# initial_q_sensor_to_level=1,0,0,0\n'));
assert(csv.includes('# gyro_unit=rad/s\n'));assert(csv.endsWith('1,10000,10000,0,0,0,0,0,0,9.8\n'));
console.log('PASS: quaternion model axes, shortest-path interpolation, CSV units and initial attitude');

const simulated=csvText({meta:{firmware:"3.0.0",boot:"test",data_mode:"simulation",imu:{q:[1,0,0,0],bias_rad_s:[0,0,0],calibrated:true,calibration_id:1}},reason:"simulation",rows:[]});assert(simulated.includes("# data_mode=simulation\n"));
