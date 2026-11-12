// Headless three.js renderer (SwiftShader WebGL). Usage: node render_gl.js job.json
// job.json: {bin, meta, out, size:[w,h], views:[{label,elev,azim}], title, axes, edges, bg}
// meta: {parts:[{name,color,offset,count,edgeOffset?}]}; bin = float32 triangle soup (9 floats/tri)
const fs = require('fs'), path = require('path'), http = require('http');
const { chromium } = require('playwright');
const job = JSON.parse(fs.readFileSync(process.argv[2]));
const here = __dirname;
const html = `<!doctype html><html><body style="margin:0"><script type="importmap">{"imports":{"three":"/three/build/three.module.js"}}</script>
<script type="module">
import * as THREE from 'three';
window.run = async function(job, buf){
  const [W,H]=job.size, views=job.views, n=views.length;
  const cols=n<=1?1:(n<=3?n:Math.ceil(Math.sqrt(n))), rows=Math.ceil(n/cols);
  const cw=Math.floor(W/cols), ch=Math.floor(H/rows);
  const ren=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});
  ren.setPixelRatio(1); ren.setSize(cw,ch,false);
  const out=document.createElement('canvas'); out.width=W; out.height=H;
  const g=out.getContext('2d'); g.fillStyle=job.bg; g.fillRect(0,0,W,H);
  const scene=new THREE.Scene(); scene.background=new THREE.Color(job.bg);
  const f=new Float32Array(buf); const grp=new THREE.Group(); scene.add(grp);
  const box=new THREE.Box3();
  for(const p of job.parts){
    const pos=f.subarray(p.offset, p.offset+p.count*9);
    const geo=new THREE.BufferGeometry(); geo.setAttribute('position',new THREE.BufferAttribute(pos,3));
    geo.computeVertexNormals();
    const mat=new THREE.MeshStandardMaterial({color:p.color,roughness:0.55,metalness:0.05,side:THREE.DoubleSide,
      polygonOffset:true,polygonOffsetFactor:1,polygonOffsetUnits:1});
    const m=new THREE.Mesh(geo,mat); grp.add(m); geo.computeBoundingBox(); box.union(geo.boundingBox);
    if(job.edges){ const e=new THREE.LineSegments(new THREE.EdgesGeometry(geo,35),new THREE.LineBasicMaterial({color:p.edgeColor,transparent:true,opacity:0.55})); grp.add(e); }
  }
  const c=box.getCenter(new THREE.Vector3()), size=box.getSize(new THREE.Vector3());
  const R=Math.max(size.length()/2,1e-3);
  scene.add(new THREE.HemisphereLight(0xffffff,0x8890a0,1.1));
  const L=new THREE.DirectionalLight(0xffffff,2.2); scene.add(L); scene.add(L.target);
  const L2=new THREE.DirectionalLight(0xdde6ff,0.8); scene.add(L2); scene.add(L2.target);
  if(job.axes){ const A=Math.max(size.x,size.y,size.z)*0.25+5;
    for(const [d,col] of [[[1,0,0],0xe03030],[[0,1,0],0x30a030],[[0,0,1],0x3060e0]]){
      const geo=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0,0,0),new THREE.Vector3(...d).multiplyScalar(A)]);
      scene.add(new THREE.Line(geo,new THREE.LineBasicMaterial({color:col}))); } }
  const aspect=cw/ch;
  views.forEach((v,i)=>{
    const e=v.elev*Math.PI/180, a=v.azim*Math.PI/180;
    const dir=new THREE.Vector3(Math.sin(a)*Math.cos(e),-Math.cos(a)*Math.cos(e),Math.sin(e));
    const cam=new THREE.OrthographicCamera(-1,1,1,-1,0.1,R*20);
    cam.position.copy(c).addScaledVector(dir,R*5);
    cam.up.set(0,0,1); if(Math.abs(Math.sin(e))>0.999) cam.up.set(0,Math.sin(e)>0?1:-1,0);
    cam.lookAt(c);
    // fit: project bbox corners
    cam.updateMatrixWorld(); const inv=cam.matrixWorldInverse; let mx=0,my=0;
    for(const x of [box.min.x,box.max.x])for(const y of [box.min.y,box.max.y])for(const z of [box.min.z,box.max.z]){
      const q=new THREE.Vector3(x,y,z).applyMatrix4(inv); mx=Math.max(mx,Math.abs(q.x)); my=Math.max(my,Math.abs(q.y)); }
    const hh=Math.max(my, mx/aspect)*1.12; cam.left=-hh*aspect; cam.right=hh*aspect; cam.top=hh; cam.bottom=-hh; cam.updateProjectionMatrix();
    // key light fixed relative to camera (upper left)
    const right=new THREE.Vector3().crossVectors(cam.up,dir).normalize(), up2=new THREE.Vector3().crossVectors(dir,right).normalize();
    L.position.copy(c).addScaledVector(dir,R*3).addScaledVector(right,-R*2).addScaledVector(up2,R*2.5); L.target.position.copy(c);
    L2.position.copy(c).addScaledVector(dir,R*2).addScaledVector(right,R*3).addScaledVector(up2,-R); L2.target.position.copy(c);
    ren.render(scene,cam);
    const x0=(i%cols)*cw, y0=Math.floor(i/cols)*ch;
    g.drawImage(ren.domElement,x0,y0);
    g.strokeStyle='#c8ccd4'; g.lineWidth=1; g.strokeRect(x0+.5,y0+.5,cw-1,ch-1);
    g.fillStyle='#444'; g.font='bold 16px sans-serif'; g.fillText(v.label,x0+10,y0+22);
    if(job.axes||true){ // scale bar (mm)
      const ppm=ch/(2*hh); let L10=Math.pow(10,Math.floor(Math.log10(hh*0.6))); if(L10*ppm<30)L10*=5;
      const bx=x0+cw-20-L10*ppm, by=y0+ch-18; g.strokeStyle='#333'; g.lineWidth=2; g.beginPath();
      g.moveTo(bx,by);g.lineTo(bx+L10*ppm,by);g.moveTo(bx,by-4);g.lineTo(bx,by+4);g.moveTo(bx+L10*ppm,by-4);g.lineTo(bx+L10*ppm,by+4);g.stroke();
      g.font='13px sans-serif'; g.fillStyle='#333'; g.fillText(L10+' mm',bx,by-8); }
  });
  if(job.title){ g.fillStyle='#111'; g.font='bold 22px sans-serif'; g.textAlign='right'; g.fillText(job.title,W-14,28); g.textAlign='left'; }
  // legend
  if(job.legend){ g.font='13px sans-serif'; let y=H-12-14*job.parts.length; for(const p of job.parts.slice(0,40)){
    g.fillStyle=p.color; g.fillRect(8,y,10,10); g.fillStyle='#222'; g.fillText(p.name,24,y+9); y+=14; } }
  return out.toDataURL('image/png');
};
</script></body></html>`;
const bin = fs.readFileSync(job.bin);
const srv = http.createServer((req, res) => {
  if (req.url === '/') { res.setHeader('content-type', 'text/html'); return res.end(html); }
  if (req.url === '/bin') { return res.end(bin); }
  if (req.url.startsWith('/three/')) {
    const p = path.join(here, 'node_modules', req.url.replace(/\.\./g, ''));
    res.setHeader('content-type', 'text/javascript'); return res.end(fs.readFileSync(p));
  }
  res.statusCode = 404; res.end();
}).listen(0, '127.0.0.1', async () => {
  const port = srv.address().port;
  const b = await chromium.launch({ args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
  try {
    const pg = await b.newPage({ viewport: { width: 800, height: 600 } });
    pg.on('pageerror', e => { console.error('PAGEERROR', e.message); });
    await pg.goto(`http://127.0.0.1:${port}/`);
    await pg.waitForFunction(() => window.run);
    const url = await pg.evaluate(async ([j, port]) => {
      const buf = await (await fetch('/bin')).arrayBuffer(); return window.run(j, buf);
    }, [job, port]);
    fs.writeFileSync(job.out, Buffer.from(url.split(',')[1], 'base64'));
  } catch (e) { console.error('ERR', e.message); process.exitCode = 1; }
  await b.close(); srv.close();
});
