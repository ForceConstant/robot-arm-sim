import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
const {chromium}=await import(process.env.PLAYWRIGHT_MODULE||'/home/node/.openclaw/workspace/dockhand/node_modules/playwright/index.mjs');
const base=process.env.TEST_URL||'http://127.0.0.1:18098';
const browser=await chromium.launch({headless:true,executablePath:process.env.BROWSER_PATH||'/home/node/.cache/ms-playwright/chromium-1243/chrome-linux64/chrome',args:['--no-sandbox','--disable-dev-shm-usage']});
const page=await browser.newPage({viewport:{width:1440,height:1100}});
const errors=[];page.on('pageerror',e=>errors.push(e.message));
const status=async()=>await (await page.request.get(base+'/api/status')).json();
async function until(fn,message,timeout=45000){const end=Date.now()+timeout;while(Date.now()<end){const s=await status();if(await fn(s))return s;await new Promise(r=>setTimeout(r,150))}throw Error(message)}
try{
 await page.goto(base);
 await until(s=>s.ready,'Simulator did not initialize');
 await page.waitForFunction(()=>document.querySelector('#main-camera').naturalWidth>0);
 assert.equal(await page.locator('#main-camera').evaluate(e=>e.naturalWidth),800);
 for(const camera of ['overview','top','white_wrist','black_wrist']){const r=await page.request.get(base+'/api/frame/'+camera+'.jpg');assert.equal(r.status(),200);const bytes=await r.body();assert.equal(bytes[0],255);assert.equal(bytes[1],216)}
 await page.locator('#stop').click();await until(s=>s.phase==='stopped','Stop not reflected');
 await page.locator('#mode').selectOption('opening');await page.locator('#observations').selectOption('all');await page.locator('#apply').click();
 await until(s=>s.phase==='ready'&&s.observations==='all','Configuration/reset failed');
 assert.ok((await (await page.request.get(base+'/api/observations')).json()).cameras.top);
 await page.locator('#observations').selectOption('wrists');await page.locator('#apply').click();
 await until(s=>s.phase==='ready'&&s.observations==='wrists','Wrist-only configuration failed');
 const sensors=await (await page.request.get(base+'/api/observations')).json();
 assert.deepEqual(Object.keys(sensors.cameras).sort(),['black_wrist','white_wrist']);assert.equal('fen' in sensors,false);
 await page.locator('#start').click();await until(s=>s.running&&s.phase==='moving','Start did not run');
 await page.locator('#pause').click();await until(s=>s.phase==='paused'&&!s.running,'Pause did not freeze');
 await new Promise(r=>setTimeout(r,300));const paused=await status();await new Promise(r=>setTimeout(r,300));assert.equal((await status()).sim_time,paused.sim_time);
 await page.locator('#start').click();await until(s=>s.running&&s.sim_time>paused.sim_time,'Resume reset or failed to advance');
 await page.locator('#stop').click();await until(s=>s.phase==='stopped','Stop did not cancel');
 await page.locator('#reset').click();await until(s=>s.phase==='ready'&&s.ply===0,'Reset did not restore scene');
 await page.locator('#camera').selectOption('top');await page.waitForFunction(()=>document.querySelector('#main-camera').src.endsWith('/top')&&document.querySelector('#main-camera').naturalWidth>0);
 await page.locator('#camera').selectOption('overview');
 await page.waitForTimeout(1600); // cached MJPEG frames still need the next boundary
 await page.screenshot({path:process.env.SCREENSHOT_PATH||'/tmp/robot-arm-lab-desktop.png',fullPage:true});
 const csrf=await page.request.post(base+'/api/control',{data:{action:'start'}});assert.equal(csrf.status(),403);
 await page.setViewportSize({width:390,height:844});await page.screenshot({path:'/tmp/robot-arm-lab-mobile.png',fullPage:true});
 assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
 assert.deepEqual(errors,[]);
 const final=await status();await fs.writeFile(process.env.RECEIPT_PATH||'/tmp/robot-arm-browser-check.json',JSON.stringify({base,version:final.version,renderer:final.renderer,phase:final.phase,checks:['4 real JPEG views','start','pause freezes clock','resume advances clock','stop','reset','sensor selection','CSRF','mobile layout'],errors},null,2));
 console.log('Browser checks passed:',base,'renderer:',final.renderer);
}finally{await browser.close()}
