#include "web_ui.h"
#include <WebServer.h>
#include <WiFi.h>
#include "app.h"
#include "config.h"

namespace {
WebServer g_server(80);

const char kPage[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>PathPilot S3</title><style>
body{font-family:system-ui,sans-serif;background:#10141a;color:#e8e8e8;max-width:760px;margin:auto;padding:14px}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:8px 0}
.card{background:#1a212b;border-radius:10px;padding:12px;margin:10px 0}
button{background:#2b6cb0;color:#fff;border:0;border-radius:8px;padding:10px 16px;font-size:15px}
button.stop{background:#c53030}input{background:#0d1117;color:#eee;border:1px solid #333;border-radius:6px;padding:8px;width:70px}
#st{font-size:26px;font-weight:700}.s{width:44px;height:44px;border-radius:6px;background:#333}
.step{display:inline-block;padding:4px 9px;margin:2px;border-radius:5px;background:#2a323d}
.cur{background:#d69e2e;color:#000}.done{opacity:.35}canvas{width:100%;background:#1a212b;border-radius:10px}
small{opacity:.7}</style></head><body><h2>PathPilot S3</h2>
<div class="card"><div id="st">--</div><small id="info"></small><div class="row" id="sens"></div><div id="route"></div></div>
<div class="row"><button onclick="cmd('start')">Start</button><button class="stop" onclick="cmd('stop')">Stop</button>
<button onclick="cmd('cal')">Calibrate</button></div>
<div class="card row"><input id="r" style="width:170px" placeholder="route, e.g. SLRUF"><button onclick="setRoute()">Set route</button></div>
<div class="card row">kp<input id="kp">ki<input id="ki">kd<input id="kd">speed<input id="sp"><button onclick="tune()">Apply</button></div>
<canvas id="map" width="700" height="300"></canvas>
<script>
const $=i=>document.getElementById(i),q=u=>fetch(u).then(r=>r.json());let trail=[],init=false;
for(let i=0;i<5;i++)$('sens').innerHTML+='<div class="s" id="s'+i+'"></div>';
function cmd(c){q('/api/cmd?c='+c).then(()=>{if(c=='start')trail=[]})}
function setRoute(){q('/api/route?r='+encodeURIComponent($('r').value)).then(d=>{if(!d.ok)alert(d.error)})}
function tune(){q(`/api/tune?kp=${$('kp').value}&ki=${$('ki').value}&kd=${$('kd').value}&speed=${$('sp').value}`)}
function draw(){const c=$('map'),g=c.getContext('2d');g.clearRect(0,0,c.width,c.height);if(trail.length<2)return;
const xs=trail.map(p=>p[0]),ys=trail.map(p=>p[1]),x0=Math.min(...xs),y0=Math.min(...ys);
const k=Math.min((c.width-40)/Math.max(Math.max(...xs)-x0,.2),(c.height-40)/Math.max(Math.max(...ys)-y0,.2));
g.strokeStyle='#63b3ed';g.lineWidth=2;g.beginPath();
trail.forEach((p,i)=>{const X=20+(p[0]-x0)*k,Y=c.height-20-(p[1]-y0)*k;i?g.lineTo(X,Y):g.moveTo(X,Y)});g.stroke()}
async function tick(){try{const d=await q('/api/status');
$('st').textContent=d.state+(d.calibrating?' (calibrating)':'');
$('info').textContent=`pos ${d.pos.toFixed(2)} · obstacle ${d.obstacle<0?'none':d.obstacle.toFixed(2)+' m'} · `+
`dist ${d.dist.toFixed(2)} m · junctions ${d.crossings} · loop ${d.loop_us} us`+(d.calibrated?'':' · NOT CALIBRATED');
d.level.forEach((v,i)=>$('s'+i).style.background=`rgb(${v*255|0},${v*190|0},40)`);
$('route').innerHTML=[...d.route].map((ch,i)=>`<span class="step ${i<d.route_idx?'done':''} ${i==d.route_idx&&d.running?'cur':''}">${ch}</span>`).join('');
if(!init){$('kp').value=d.kp;$('ki').value=d.ki;$('kd').value=d.kd;$('sp').value=d.speed;$('r').value=d.route;init=true}
const l=trail[trail.length-1];if(!l||Math.hypot(l[0]-d.x,l[1]-d.y)>.01){trail.push([d.x,d.y]);if(trail.length>800)trail.shift();draw()}
}catch(e){$('st').textContent='offline'}}
setInterval(tick,250);tick();
</script></body></html>)HTML";

void reply(int code, const String& json) { g_server.send(code, "application/json", json); }
void ok() { reply(200, "{\"ok\":true}"); }
void fail(const char* msg) { reply(400, String("{\"ok\":false,\"error\":\"") + msg + "\"}"); }

void handle_status() {
  Telemetry t = app_telemetry();
  String j;
  j.reserve(512);
  j += "{";
  auto num = [&](const char* k, float v, int p) { j += '"'; j += k; j += "\":"; j += String(v, p); j += ','; };
  auto flag = [&](const char* k, bool v) { j += '"'; j += k; j += "\":"; j += v ? "true," : "false,"; };
  j += "\"state\":\"";
  j += Navigator::state_name(t.state);
  j += "\",\"route\":\"";
  j += t.route;
  j += "\",";
  flag("running", t.running);
  flag("calibrating", t.calibrating);
  flag("calibrated", t.calibrated);
  flag("on_line", t.line.on_line);
  num("pos", t.line.position, 3);
  num("obstacle", t.obstacle_m, 3);
  num("dist", t.distance_m, 3);
  num("heading", t.heading_deg, 1);
  num("x", t.x_m, 3);
  num("y", t.y_m, 3);
  num("ml", t.motor_l, 2);
  num("mr", t.motor_r, 2);
  num("kp", t.kp, 3);
  num("ki", t.ki, 3);
  num("kd", t.kd, 3);
  num("speed", t.speed, 2);
  j += "\"crossings\":" + String(t.crossings) + ",\"route_idx\":" + String(t.route_idx);
  j += ",\"loop_us\":" + String(t.loop_us) + ",\"level\":[";
  for (size_t i = 0; i < kLineSensors; ++i) j += String(t.line.level[i], 2) + (i + 1 < kLineSensors ? "," : "");
  j += "],\"raw\":[";
  for (size_t i = 0; i < kLineSensors; ++i) j += String(t.raw[i]) + (i + 1 < kLineSensors ? "," : "");
  j += "]}";
  reply(200, j);
}

void handle_cmd() {
  String c = g_server.arg("c");
  Command cmd;
  if (c == "start") cmd.type = CmdType::Start;
  else if (c == "stop") cmd.type = CmdType::Stop;
  else if (c == "cal") cmd.type = CmdType::Calibrate;
  else return fail("unknown command");
  app_send(cmd) ? ok() : fail("queue full");
}

void handle_route() {
  String r = g_server.arg("r");
  Route check;
  if (!check.parse(r.c_str())) return fail("route must use L S R U F, max 32 steps");
  Command cmd;
  cmd.type = CmdType::SetRoute;
  strncpy(cmd.text, r.c_str(), sizeof(cmd.text) - 1);
  app_send(cmd) ? ok() : fail("queue full");
}

void handle_tune() {
  Telemetry t = app_telemetry();
  auto get = [](const char* k, float def) { return g_server.hasArg(k) ? g_server.arg(k).toFloat() : def; };
  Command pid;
  pid.type = CmdType::SetPid;
  pid.a = get("kp", t.kp);
  pid.b = get("ki", t.ki);
  pid.c = get("kd", t.kd);
  Command speed;
  speed.type = CmdType::SetSpeed;
  speed.a = get("speed", t.speed);
  (app_send(pid) && app_send(speed)) ? ok() : fail("queue full");
}
}  // namespace

void web_ui_begin() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  g_server.on("/", HTTP_GET, [] { g_server.send(200, "text/html", kPage); });
  g_server.on("/api/status", HTTP_GET, handle_status);
  g_server.on("/api/cmd", HTTP_GET, handle_cmd);
  g_server.on("/api/route", HTTP_GET, handle_route);
  g_server.on("/api/tune", HTTP_GET, handle_tune);
  g_server.onNotFound([] { fail("not found"); });
  g_server.begin();
}

void web_ui_loop() { g_server.handleClient(); }
