// ############################################################
// #  화재 감지 + 인원 탐지 시스템  /  [ LD2450 버전 ]
// #
// #  사용 센서 : HLK-LD2450 (24GHz mmWave)
// #  연결 방식 : ESP32에 UART로 직접 연결  (라즈베리파이 필요 없음)
// #  최대 인원 : 3명  (LD2450 하드웨어 한계)
// #  높이(Z)   : 측정 불가 (2D 평면만)
// #  설치 방식 : 벽면 부착 권장, 최대 추적거리 6m
// #
// #  LD2450 배선
// #    LD2450 TX  -> ESP32 GPIO16
// #    LD2450 RX  -> ESP32 GPIO17
// #    LD2450 VCC -> 5V
// #    LD2450 GND -> GND
// ############################################################

#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>
                                                                
// ==========================================================
// Wi-Fi 정보
// ==========================================================    
const char* ssid = "YOUR_WIFI_SSID";                                      // 여기에 와이파이 이름 입력 지금은 제 핫스팜 이름이여서 변경 하시고 사용하세요 
const char* password = "YOUR_WIFI_PASSWORD";                              // 여기에 와이파이 비밀번호 입력 여기도 제 핫스팟 비밀번호여서 변경 하시고 사용하세요
                                                                // 그리고 시리얼 모니터에 정상작동하면 웹사이트 주소나와요 142.97이런식으로 https://142.97 이런식으로
                                                                // 웹사이트 들어가면 됩니다
                                                                // ld2450 센서는 일단 천장 중앙에 붙여서 한 번 테스트 해보는게 좋을 것 같아요. 옆 벽에 붙이면 약간  
                                                                // 2d형태니깐 천장에 붙여서 확인 해봐야 할 것 같고 그랬을 때도 부정확하면 부품 스펙 올리는게 맞는거 
                                                                // 같긴해요 일단 저의 사용할 수 있는 금액은 579,680원 정도 남았어요.
                                                                // ld2450은 따로 코딩 안 넣고 전원만 연결하면 hlkrader tool어플로 저번처럼 확인 할 수 있고 
                                                                // 어플에 확인 된 인원 수를 웹사이트에 뛰울려면 코딩 넣어서 인지한 사람 인원을 esp32에 보내고 그걸 
                                                                // 웹사이트에 뛰우는 코딩 넣으면 됩니다.
                                                                // 그래도 안되면 인원감지는 따로 시현해도 된다 했으니깐 산업용을 사용해야 할 수도 있을것 같아요                                       
// ==========================================================
// DHT22 설정
// ==========================================================
#define DHTPIN 4                                                // 온도 센서는 esp32 4번에 연결
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// ==========================================================
// MQ-2 가스 센서
// ==========================================================
#define GAS_PIN 34                                              // 가스 센서는 esp32 34번에 연결

// ==========================================================
// IR 센서 2개
// IR1 -> GPIO27
// IR2 -> GPIO26
// ==========================================================
#define IR1_PIN 27                                              // 이게 출입용 ir센서 esp32 27번에 연결 
#define IR2_PIN 26                                              // 이게 외출용 ir센서 esp32 26번에 연결

// ==========================================================
// 화재 판단 기준
// ==========================================================
float TEMP_LIMIT = 33.0;                                        // 저번에 온도 해보니깐 생각보다 많이 올라가서 쫌 더 높게 수정해도 될꺼 같아요
int GAS_LIMIT = 120;                                            // 가스도 한 번 보시고 바꾸셔도 될꺼 같습니다 5000으로 하고싶다 그러면 600만 지우고 5000하면됩니다
                                                                // 그 이외에 코딩은 크게 변경할꺼는 없습니다. 
// ==========================================================
// 온도 관련 변수
// ==========================================================
float tempSum = 0;
int tempCount = 0;
float avgTemp = 0;
float currentTemp = 0;

// ==========================================================
// 현재 측정값
// ==========================================================
int currentGas = 0;
bool fire = false;

// ==========================================================
// 사람 수 카운트 변수
// ==========================================================
int peopleCount = 0;

// ==========================================================
// LD2450 mmWave 레이더 (ESP32 UART2 직결)
// LD2450 TX -> ESP32 GPIO16 (RX2)
// LD2450 RX -> ESP32 GPIO17 (TX2)
// LD2450 VCC -> 5V,  GND -> GND
// ==========================================================
#define LD_RX_PIN 16                                            // ESP32가 받는 핀 (LD2450의 TX와 연결)
#define LD_TX_PIN 17                                            // ESP32가 보내는 핀 (LD2450의 RX와 연결)
#define LD_BAUD 256000                                          // LD2450 고정 통신속도 (변경하지 마세요)

#define MAX_TARGETS 3                                           // LD2450 하드웨어 한계: 최대 3명
#define LD_FRAME_SIZE 30                                        // 한 프레임 = 헤더4 + 타겟3x8 + 꼬리2
#define LD_RX_BUFFER 2048                                       // LD2450 수신 버퍼 크기 (기본 256바이트 -> 2048바이트로 확대)
#define LD_MAX_READ 240                                         // loop 1회당 최대로 읽는 바이트 수 (프레임 8개 분량)

// ----------------------------------------------------------
// LD2450 진단 모드 : 원인 확인이 끝나면 false 로 바꾸세요
// 켜두면 시리얼 모니터에 2초마다 [LD2450] 진단 한 줄이 찍힙니다
// ----------------------------------------------------------
#define LD_DEBUG false
unsigned long dbgBytes = 0;                                     // 2초 동안 LD2450에서 받은 바이트 수
unsigned long dbgFramesOk = 0;                                  // 2초 동안 정상적으로 해석한 프레임 수
unsigned long dbgFramesBad = 0;                                 // 2초 동안 꼬리(55 CC)가 안 맞아 버린 프레임 수
unsigned long dbgMaxLoop = 0;                                   // 2초 동안 loop 1바퀴에 걸린 최대 시간(ms)
unsigned long dbgLastPrint = 0;
uint8_t dbgFirst[32];                                           // 처음 받은 32바이트 (통신속도 확인용)
int dbgFirstLen = 0;
bool dbgFirstShown = false;

float targetX[MAX_TARGETS];                                     // 좌우 위치 (m)
float targetY[MAX_TARGETS];                                     // 전방 거리 (m)
float targetZ[MAX_TARGETS];                                     // LD2450은 높이 측정 불가 -> 항상 0
int targetCount = 0;

// ==========================================================
// 구역(방) 관리
// 이 ESP32가 담당하는 방을 MY_ROOM 으로 지정하세요.
//   0 = 510호  (현재 구역은 510호 1개)
// 다른 방의 ESP32는 이 메인 ESP32의 /report 로 자기 데이터를 보냅니다.
// ==========================================================
#define ROOM_COUNT 1
#define MY_ROOM 0                                               // 이 보드가 담당하는 방 번호

// 데모 모드 : 구역을 여러 개 둘 때, 아직 설치 안 된 방을 샘플값으로 채우는 기능입니다.
// 현재는 510호 1개만 쓰므로 꺼둡니다.
#define DEMO_MODE false

#define ROOM_TIMEOUT 10000                                      // 10초간 보고 없으면 연결 끊김 처리

const char* roomNames[ROOM_COUNT] = {
"정보공학관 510호"
};

struct RoomData {
float temp;
int gas;
int count;
int radarCount;
bool fire;
bool demo;
unsigned long lastSeen;
float tx[MAX_TARGETS];
float ty[MAX_TARGETS];
float tz[MAX_TARGETS];
};
RoomData rooms[ROOM_COUNT];

bool roomOnline(int i) {
if (rooms[i].demo) return true;
if (i == MY_ROOM) return true;
return (rooms[i].lastSeen > 0) && (millis() - rooms[i].lastSeen < ROOM_TIMEOUT);
}

// 내 방 센서값을 rooms 배열에 반영 (loop에서 호출)
void syncMyRoom() {
rooms[MY_ROOM].temp = avgTemp;
rooms[MY_ROOM].gas = currentGas;
rooms[MY_ROOM].count = peopleCount;
rooms[MY_ROOM].radarCount = targetCount;
rooms[MY_ROOM].fire = fire;
rooms[MY_ROOM].lastSeen = millis();
for (int i = 0; i < MAX_TARGETS; i++) {
rooms[MY_ROOM].tx[i] = targetX[i];
rooms[MY_ROOM].ty[i] = targetY[i];
rooms[MY_ROOM].tz[i] = targetZ[i];
}
}

// 데모용 샘플 데이터 (실제 배치 시 DEMO_MODE를 false로)
void initDemoRooms() {
if (!DEMO_MODE) return;
for (int i = 0; i < ROOM_COUNT; i++) {
if (i == MY_ROOM) continue;
rooms[i].temp = 24.0;
rooms[i].gas = 300;
rooms[i].count = 2;
rooms[i].radarCount = 2;
rooms[i].fire = false;
rooms[i].demo = true;
rooms[i].lastSeen = millis();
for (int t = 0; t < rooms[i].radarCount && t < MAX_TARGETS; t++) {
rooms[i].tx[t] = -2.0 + t * 1.4;
rooms[i].ty[t] = 2.5 + t * 1.1;
rooms[i].tz[t] = 0.4;
}
}
}


uint8_t ldBuf[LD_FRAME_SIZE];
int ldIdx = 0;
unsigned long lastRadarUpdate = 0;
const unsigned long RADAR_TIMEOUT = 2000;                       // 2초간 데이터 없으면 인원 0으로 처리

// 상태 머신
int sequenceState = 0;

unsigned long sequenceStart = 0;
const unsigned long SEQUENCE_TIMEOUT = 2000;

unsigned long lastCountTime = 0;
const unsigned long COUNT_COOLDOWN = 800;

bool prevIr1 = false;
bool prevIr2 = false;

unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL = 1000;

// ==========================================================
// 웹 서버
// ==========================================================
WebServer server(80);

// ==========================================================
// 메인 웹페이지
// ==========================================================
void handleRoot() {
String html = R"rawliteral(
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">
<title>화재 감지 시스템</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
body{
font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;
background:#0A0E14;color:#E5E7EB;padding:14px;min-height:100vh;
display:flex;flex-direction:column;transition:background .3s;
}
body.alert{background:#1A0808}
.wrap{max-width:520px;margin:0 auto;width:100%;flex:1;display:flex;flex-direction:column}
.top{display:flex;align-items:center;justify-content:space-between;margin-bottom:14px}
.brand{display:flex;align-items:center;gap:8px;font-size:16px;font-weight:600}
.dot{width:9px;height:9px;border-radius:50%;background:#22C55E;flex-shrink:0}
body.alert .dot{background:#EF4444}
.badge{font-size:11px;letter-spacing:.5px;padding:4px 9px;border-radius:20px;color:#4ADE80;background:rgba(34,197,94,.12)}
body.alert .badge{color:#FCA5A5;background:rgba(239,68,68,.18)}
.status{border-radius:14px;padding:18px;text-align:center;margin-bottom:12px;
background:rgba(34,197,94,.08);border:1px solid rgba(34,197,94,.35);transition:.3s}
body.alert .status{background:rgba(239,68,68,.15);border-color:#EF4444}
.status .big{font-size:26px;font-weight:600;color:#4ADE80}
body.alert .status .big{color:#F87171}
.status .sub{font-size:13px;color:#6B7280;margin-top:4px}
body.alert .status .sub{color:#FCA5A5}
.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:9px;margin-bottom:12px}
.card{background:#141922;border-radius:12px;padding:13px;transition:background .3s}
body.alert .card{background:#2A1010}
.card .label{font-size:15px;font-weight:500;color:#9CA3AF;margin-bottom:5px}
body.alert .card .label{color:#D1D5DB}
.card .val{font-size:21px;font-weight:600}
.card .unit{font-size:13px;font-weight:400}
.c-temp,.c-gas{color:#38BDF8}
.c-ppl{color:#E5E7EB}
.c-radar{color:#A78BFA}
.val.over{color:#F87171 !important}                       /* 설정 범위 초과한 항목만 빨간색 */

.log-panel{flex:none;min-height:0;padding:13px}
.log-head{margin-bottom:9px}
.log-tag{font-size:10.5px;font-weight:500;color:#A78BFA;background:rgba(167,139,250,.12);
border:1px solid rgba(167,139,250,.35);border-radius:20px;padding:2px 7px;margin-left:6px;white-space:nowrap}
.log-sub{font-size:11px;color:#4B5563;margin-top:3px}
.log-box{
height:150px;overflow-y:auto;background:#0D1219;border-radius:8px;
padding:8px 12px;font-family:'SFMono-Regular',Consolas,monospace;
scroll-behavior:smooth;
}
.log-line{
font-size:12.5px;color:#6B7280;padding:5px 0;border-bottom:1px solid #1A202C;
display:flex;justify-content:space-between;opacity:0;
animation:logIn .3s ease forwards;
}
.log-line:last-child{border-bottom:none}
.log-line .lc{color:#A78BFA;font-weight:600}
.log-line.latest{color:#D1D5DB}
.log-line.latest .lc{color:#C4B5FD;font-size:13.5px}
@keyframes logIn{from{opacity:0;transform:translateY(4px)}to{opacity:1;transform:translateY(0)}}
body.alert .log-box{background:#170A0A}
body.alert .log-line{border-bottom-color:#2A1414}
body.alert .log-line .lc{color:#F87171}

/* ===== 방 목록 ===== */
.sect{font-size:13px;color:#6B7280;margin:4px 2px 8px}
.room{display:flex;align-items:center;gap:11px;background:#141922;border-radius:12px;
padding:14px 13px;margin-bottom:8px;cursor:pointer;border:1px solid transparent;transition:.15s}
.room:active{background:#1B2230}
.room.fire{background:rgba(239,68,68,.12);border-color:#EF4444;animation:flash 1s infinite}
.room .rdot{width:10px;height:10px;border-radius:50%;background:#22C55E;flex-shrink:0}
.room.fire .rdot{background:#EF4444}
.room.off .rdot{background:#4B5563}
.room .rinfo{flex:1;min-width:0}
.room .rname{font-size:16px;font-weight:600}
.room .rmeta{font-size:12px;color:#6B7280;margin-top:3px}
.room.fire .rmeta{color:#FCA5A5}
.room .rstate{font-size:13px;font-weight:600;color:#4ADE80;flex-shrink:0}
.room.fire .rstate{color:#F87171}
.room.off .rstate{color:#6B7280}
.room .arw{color:#4B5563;font-size:17px;flex-shrink:0}
.tag{font-size:10px;color:#6B7280;border:1px solid #374151;border-radius:4px;padding:1px 4px;margin-left:5px}

/* ===== 상세 화면 ===== */
.back{display:inline-flex;align-items:center;gap:6px;background:#141922;border:none;
color:#9CA3AF;font-size:14px;padding:8px 13px;border-radius:9px;cursor:pointer;font-family:inherit}
.back:active{background:#1B2230}
.panel{background:#141922;border-radius:12px;padding:13px;transition:background .3s;
flex:1;display:flex;flex-direction:column;min-height:220px}
body.alert .panel{background:#2A1010}
.panel .label{font-size:15px;font-weight:500;color:#9CA3AF;margin-bottom:9px}
#radar{width:100%;flex:1;display:block;border-radius:8px;min-height:180px}
.foot{text-align:center;font-size:11px;color:#4B5563;margin-top:12px;flex-shrink:0}
@keyframes flash{0%,100%{opacity:1}50%{opacity:.45}}
body.alert .status{animation:flash 1s infinite}
.hide{display:none !important}
</style>
</head>
<body>
<div class="wrap">

<!-- ================= 첫 화면 : 방 목록 ================= -->
<div id="viewList">
<div class="top">
<div class="brand"><span class="dot"></span>동의대 화재 감지 시스템</div>
<span class="badge" id="badgeAll">LIVE</span>
</div>

<div class="status">
<div class="big" id="allStatus">정상</div>
<div class="sub" id="allSub">전체 구역 이상 없음</div>
</div>

<div class="grid">
<div class="card"><div class="label">총 재실 인원</div><div class="val c-ppl"><span id="allCount">0</span><span class="unit">명</span></div></div>
<div class="card"><div class="label">총 감지 인원</div><div class="val c-radar"><span id="allRadar">0</span><span class="unit">명</span></div></div>
</div>

<div class="sect">구역 선택</div>
<div id="roomList"></div>
<div class="foot">1초마다 자동 갱신</div>
</div>

<!-- ================= 상세 화면 : 개별 방 ================= -->
<div id="viewDetail" class="hide">
<div class="top">
<button class="back" onclick="showList()">&#8592; 목록</button>
<span class="badge" id="badgeRoom">LIVE</span>
</div>

<div class="sect" id="roomTitle" style="font-size:17px;font-weight:600;color:#E5E7EB;margin-bottom:10px"></div>

<div class="status">
<div class="big" id="statusText">정상</div>
<div class="sub" id="statusSub">이상 없음</div>
</div>

<div class="grid">
<div class="card"><div class="label">온도</div><div class="val c-temp" id="tempVal"><span id="temp">0.0</span><span class="unit">&deg;C</span></div></div>
<div class="card"><div class="label">가연성 가스 농도</div><div class="val c-gas" id="gasVal"><span id="gas">0</span><span class="unit">ppm</span></div></div>
<div class="card"><div class="label">재실 인원</div><div class="val c-ppl"><span id="count">0</span><span class="unit">명</span></div></div>
<div class="card"><div class="label">인체 감지 인원</div><div class="val c-radar"><span id="radarCount">0</span><span class="unit">명</span></div></div>
</div>

<div class="panel" style="margin-bottom:12px">
<div class="label">실내 인원 위치</div>
<canvas id="radar" width="600" height="360"></canvas>
</div>

<div class="panel log-panel">
<div class="log-head">
<div class="label" style="margin-bottom:0">실시간 감지 로그 <span class="log-tag">인체 감지 기준</span></div>
<div class="log-sub">5초마다 자동 기록 · 최신 기록이 항상 유지됩니다</div>
</div>
<div id="logBox" class="log-box"></div>
</div>
<div class="foot">1초마다 자동 갱신</div>
</div>

</div>

<script>
var curRoom = -1;   // -1 이면 목록 화면

function showList(){
curRoom = -1;
document.getElementById('viewList').classList.remove('hide');
document.getElementById('viewDetail').classList.add('hide');
update();
}

function showRoom(i){
curRoom = i;
document.getElementById('viewList').classList.add('hide');
document.getElementById('viewDetail').classList.remove('hide');
update();
}

// ================= 목록 화면 갱신 =================
function updateList(){
fetch('/rooms').then(r=>r.json()).then(d=>{
document.body.classList.toggle('alert', d.fire);
document.getElementById('allStatus').innerHTML = d.fire ? '화재 발생' : '정상';
document.getElementById('allSub').innerHTML = d.fire
  ? (d.fireNames + ' &mdash; 즉시 대피')
  : '전체 구역 이상 없음';
document.getElementById('badgeAll').innerHTML = d.fire ? 'ALERT' : 'LIVE';
document.getElementById('allCount').innerHTML = d.totalCount;
document.getElementById('allRadar').innerHTML = d.totalRadar;

var h = '';
for (var i=0;i<d.rooms.length;i++){
var r = d.rooms[i];
var cls = 'room' + (r.fire ? ' fire' : '') + (r.online ? '' : ' off');
var state = r.online ? (r.fire ? '화재' : '정상') : '연결 끊김';
var meta = r.online
  ? ('재실 ' + r.count + '명 &middot; 인체감지 ' + r.radarCount + '명')
  : '데이터 수신 없음';
var tag = r.demo ? '<span class="tag">데모</span>' : '';
h += '<div class="'+cls+'" onclick="showRoom('+i+')">'
   + '<span class="rdot"></span>'
   + '<div class="rinfo"><div class="rname">'+r.name+tag+'</div>'
   + '<div class="rmeta">'+meta+'</div></div>'
   + '<span class="rstate">'+state+'</span>'
   + '<span class="arw">&#8250;</span></div>';
}
document.getElementById('roomList').innerHTML = h;
});
}

// ================= 상세 화면 갱신 =================
function updateDetail(){
fetch('/data?room='+curRoom).then(r=>r.json()).then(data=>{
document.getElementById('roomTitle').innerHTML = data.name;
document.getElementById('temp').innerHTML = data.temp.toFixed(1);
document.getElementById('gas').innerHTML = data.gas;
document.getElementById('count').innerHTML = data.count;
document.getElementById('radarCount').innerHTML = data.radarCount;

// 설정 범위를 넘은 항목만 각각 빨간색으로 표시
document.getElementById('tempVal').classList.toggle('over', data.tempOver);
document.getElementById('gasVal').classList.toggle('over', data.gasOver);

if (data.fire) {
document.body.classList.add('alert');
document.getElementById('statusText').innerHTML = '화재 발생';
document.getElementById('statusSub').innerHTML = '실내 인원 ' + data.radarCount + '명 &mdash; 즉시 대피';
document.getElementById('badgeRoom').innerHTML = 'ALERT';
} else {
document.body.classList.remove('alert');
document.getElementById('statusText').innerHTML = data.online ? '정상' : '연결 끊김';
document.getElementById('statusSub').innerHTML = data.online ? '이상 없음' : '데이터 수신 없음';
document.getElementById('badgeRoom').innerHTML = data.online ? 'LIVE' : 'OFF';
}
drawRadar(data.targets);
});
}

function update(){ if(curRoom<0) updateList(); else updateDetail(); }
update();
setInterval(update, 1000);

// ==========================================================
// 실시간 감지 로그 (인체 감지 기준) — 5초마다 기록, 자동 스크롤
// 건물 붕괴 등으로 센서가 끊기기 직전까지도 마지막 인원수가
// 로그에 남도록, 화면 값을 덮어쓰지 않고 누적 로그로 관리합니다.
// ==========================================================
const MAX_LOG_LINES = 40; // 무한정 쌓이지 않도록 오래된 줄은 제거

function formatTimestamp(d){
const p2 = n => String(n).padStart(2,'0');
return `${d.getFullYear()}.${p2(d.getMonth()+1)}.${p2(d.getDate())} `
     + `${p2(d.getHours())}시 ${p2(d.getMinutes())}분 ${p2(d.getSeconds())}초`;
}

function appendLog(count){
const box = document.getElementById('logBox');
if (!box) return;

const prev = box.querySelector('.log-line.latest');
if (prev) prev.classList.remove('latest');

const line = document.createElement('div');
line.className = 'log-line latest';
line.innerHTML = `<span>${formatTimestamp(new Date())}</span><span class="lc">${count}명</span>`;
box.appendChild(line);

while (box.children.length > MAX_LOG_LINES) box.removeChild(box.firstChild);
box.scrollTop = box.scrollHeight; // 항상 최신 줄이 보이도록 자동 스크롤
}

function logTick(){
if (curRoom < 0) return; // 목록 화면에서는 기록하지 않음
fetch('/data?room='+curRoom).then(r=>r.json()).then(data=>{
appendLog(data.radarCount);
});
}
setInterval(logTick, 5000);

// ==========================================================
// 레이더 Top-Down 시각화
// ==========================================================
const canvas = document.getElementById('radar');
const ctx = canvas.getContext('2d');
const RANGE_M = 8; // 화면에 표시할 최대 거리(미터). 필요시 조정하세요.

function fitCanvas() {
const dpr = window.devicePixelRatio || 1;
const r = canvas.getBoundingClientRect();
if (r.width && r.height) {
canvas.width = Math.round(r.width * dpr);
canvas.height = Math.round(r.height * dpr);
}
}

function drawRadar(targets) {
fitCanvas();
const w = canvas.width, h = canvas.height;
const alert = document.body.classList.contains('alert');
ctx.clearRect(0, 0, w, h);

// 부채꼴 그리드 (센서를 중심으로 한 거리 표시)
ctx.strokeStyle = '#FFFFFF';                              // 레이더 범위 선 : 하얀색 (정상·화재 공통)
ctx.lineWidth = 1.5;
for (let r = 1; r <= 3; r++) {
ctx.beginPath();
ctx.arc(w / 2, h, (h * r) / 3, Math.PI, 2 * Math.PI);
ctx.stroke();
}

// 방사형 기준선
[-60, 0, 60].forEach(deg => {
const rad = (deg - 90) * Math.PI / 180;
ctx.beginPath();
ctx.moveTo(w / 2, h);
ctx.lineTo(w / 2 + Math.cos(rad) * h, h + Math.sin(rad) * h);
ctx.stroke();
});

// 센서 위치 (하단 중앙)
ctx.fillStyle = alert ? '#EF4444' : '#22C55E';
ctx.beginPath();
ctx.arc(w / 2, h - 6, 8, 0, 2 * Math.PI);
ctx.fill();

// 감지된 사람 (X=좌우, Y=전방거리, Z=높이 -> 점 크기로 표현)
targets.forEach(t => {
const px = w / 2 + (t.x / RANGE_M) * (w / 2);
const py = h - (t.y / RANGE_M) * h;
const size = (w / 55) + Math.max(0, t.z) * 3;

ctx.fillStyle = 'rgba(167,139,250,0.25)';
ctx.beginPath();
ctx.arc(px, py, size * 1.9, 0, 2 * Math.PI);
ctx.fill();

ctx.fillStyle = '#A78BFA';
ctx.beginPath();
ctx.arc(px, py, size, 0, 2 * Math.PI);
ctx.fill();
});
}

window.addEventListener('resize', update);
</script>
</body>
</html>
)rawliteral";

server.send(200, "text/html; charset=UTF-8", html);
}

// ==========================================================
// JSON 데이터 전송
// ==========================================================
// ==========================================================
// 전체 구역 요약 (첫 화면용)
// ==========================================================
void handleRooms() {
bool anyFire = false;
int totalCount = 0, totalRadar = 0;
String fireNames = "";

for (int i = 0; i < ROOM_COUNT; i++) {
bool on = roomOnline(i);
if (!on) continue;
totalCount += rooms[i].count;
totalRadar += rooms[i].radarCount;
if (rooms[i].fire) {
anyFire = true;
if (fireNames.length() > 0) fireNames += ", ";
fireNames += roomNames[i];
}
}

String json = "{";
json += "\"fire\":"; json += (anyFire ? "true" : "false"); json += ",";
json += "\"fireNames\":\"" + fireNames + "\",";
json += "\"totalCount\":" + String(totalCount) + ",";
json += "\"totalRadar\":" + String(totalRadar) + ",";
json += "\"rooms\":[";
for (int i = 0; i < ROOM_COUNT; i++) {
if (i > 0) json += ",";
bool on = roomOnline(i);
json += "{\"name\":\"" + String(roomNames[i]) + "\",";
json += "\"fire\":"; json += (rooms[i].fire && on ? "true" : "false"); json += ",";
json += "\"online\":"; json += (on ? "true" : "false"); json += ",";
json += "\"demo\":"; json += (rooms[i].demo ? "true" : "false"); json += ",";
json += "\"count\":" + String(rooms[i].count) + ",";
json += "\"radarCount\":" + String(rooms[i].radarCount) + "}";
}
json += "]}";
server.send(200, "application/json", json);
}

// ==========================================================
// 개별 구역 상세 (상세 화면용)
// ==========================================================
void handleData() {
int r = MY_ROOM;
if (server.hasArg("room")) r = server.arg("room").toInt();
if (r < 0 || r >= ROOM_COUNT) r = MY_ROOM;

bool on = roomOnline(r);

String json = "{";
json += "\"name\":\"" + String(roomNames[r]) + "\",";
json += "\"temp\":" + String(rooms[r].temp, 1) + ",";
json += "\"gas\":" + String(rooms[r].gas) + ",";
json += "\"count\":" + String(rooms[r].count) + ",";
json += "\"radarCount\":" + String(rooms[r].radarCount) + ",";
json += "\"online\":"; json += (on ? "true" : "false"); json += ",";
json += "\"fire\":"; json += (rooms[r].fire && on ? "true" : "false"); json += ",";
json += "\"tempOver\":"; json += (on && rooms[r].temp >= TEMP_LIMIT ? "true" : "false"); json += ",";   // 온도 기준 초과 여부
json += "\"gasOver\":"; json += (on && rooms[r].gas >= GAS_LIMIT ? "true" : "false"); json += ",";      // 가연성 가스 농도 기준 초과 여부
json += "\"targets\":[";
int n = rooms[r].radarCount;
if (n > MAX_TARGETS) n = MAX_TARGETS;
for (int i = 0; i < n; i++) {
if (i > 0) json += ",";
json += "{\"x\":" + String(rooms[r].tx[i], 2) +
",\"y\":" + String(rooms[r].ty[i], 2) +
",\"z\":" + String(rooms[r].tz[i], 2) + "}";
}
json += "]}";
server.send(200, "application/json", json);
}

// ==========================================================
// 다른 방 ESP32가 자기 데이터를 보내는 곳
// 예) POST /report  room=1&temp=23.4&gas=300&count=2&radarCount=2&fire=0
//     &points=1.2,3.4,0.5;-0.8,2.1,0.3
// ==========================================================
void handleReport() {
if (!server.hasArg("room")) {
server.send(400, "text/plain", "Missing room");
return;
}
int r = server.arg("room").toInt();
if (r < 0 || r >= ROOM_COUNT) {
server.send(400, "text/plain", "Bad room");
return;
}

if (server.hasArg("temp")) rooms[r].temp = server.arg("temp").toFloat();
if (server.hasArg("gas")) rooms[r].gas = server.arg("gas").toInt();
if (server.hasArg("count")) rooms[r].count = server.arg("count").toInt();
if (server.hasArg("fire")) rooms[r].fire = (server.arg("fire").toInt() != 0);

if (server.hasArg("points")) {
String data = server.arg("points");
int n = 0, start = 0;
while (start < (int)data.length() && n < MAX_TARGETS) {
int sep = data.indexOf(';', start);
String chunk = (sep == -1) ? data.substring(start) : data.substring(start, sep);
int c1 = chunk.indexOf(',');
int c2 = chunk.indexOf(',', c1 + 1);
if (c1 > 0 && c2 > c1) {
rooms[r].tx[n] = chunk.substring(0, c1).toFloat();
rooms[r].ty[n] = chunk.substring(c1 + 1, c2).toFloat();
rooms[r].tz[n] = chunk.substring(c2 + 1).toFloat();
n++;
}
if (sep == -1) break;
start = sep + 1;
}
rooms[r].radarCount = n;
} else if (server.hasArg("radarCount")) {
rooms[r].radarCount = server.arg("radarCount").toInt();
}

rooms[r].demo = false;
rooms[r].lastSeen = millis();
server.send(200, "text/plain", "OK");
}

// ==========================================================
// LD2450 신호값 해석
// 최상위 비트가 1이면 양수, 0이면 음수 (LD2450 고유 방식)
// ==========================================================
int16_t ldDecode(uint8_t lo, uint8_t hi) {
int16_t v = (int16_t)((((uint16_t)hi << 8) | lo) & 0x7FFF);
return (hi & 0x80) ? v : -v;
}

// ==========================================================
// LD2450 데이터 읽기 (loop에서 계속 호출)
// 프레임 형식: AA FF 03 00 [타겟1 8바이트][타겟2][타겟3] 55 CC
// ==========================================================
void readRadar() {
int readCount = 0;                                              // 이번 loop에서 읽은 바이트 수
while (Serial2.available() && readCount < LD_MAX_READ) {        // 한 번에 최대 LD_MAX_READ 바이트만 읽고 빠져나감
readCount++;                                                    // -> 웹서버(handleClient)가 멈추지 않음
uint8_t b = Serial2.read();
dbgBytes++;
if (dbgFirstLen < 32) dbgFirst[dbgFirstLen++] = b;

// 헤더 4바이트를 순서대로 찾아서 프레임 시작점 맞추기
if (ldIdx == 0 && b != 0xAA) continue;
if (ldIdx == 1 && b != 0xFF) { ldIdx = 0; continue; }
if (ldIdx == 2 && b != 0x03) { ldIdx = 0; continue; }
if (ldIdx == 3 && b != 0x00) { ldIdx = 0; continue; }

ldBuf[ldIdx++] = b;

if (ldIdx < LD_FRAME_SIZE) continue;
ldIdx = 0;

// 꼬리(55 CC)가 안 맞으면 깨진 프레임이므로 버림
if (ldBuf[28] != 0x55 || ldBuf[29] != 0xCC) { dbgFramesBad++; continue; }
dbgFramesOk++;

// 타겟 3개 해석
int found = 0;
for (int i = 0; i < MAX_TARGETS; i++) {
int o = 4 + i * 8;
int16_t x = ldDecode(ldBuf[o],     ldBuf[o + 1]);
int16_t y = ldDecode(ldBuf[o + 2], ldBuf[o + 3]);

// 좌표가 둘 다 0이면 그 자리에는 사람이 없음
if (x == 0 && y == 0) continue;

targetX[found] = x / 1000.0;                                // mm -> m 변환
targetY[found] = y / 1000.0;
targetZ[found] = 0;                                         // LD2450은 높이 정보 없음
found++;
}

targetCount = found;
lastRadarUpdate = millis();
}

// 일정 시간 데이터가 안 오면 인원 0으로 초기화
if (millis() - lastRadarUpdate > RADAR_TIMEOUT) {
targetCount = 0;
}
}

// ==========================================================
// LD2450 진단 출력 (LD_DEBUG 가 true 일 때만, 2초마다 1줄)
// ==========================================================
void ldDebugPrint() {
if (!LD_DEBUG) return;

// 처음 받은 32바이트를 한 번만 16진수로 출력 -> AA FF 03 00 이 보이면 통신속도 정상
if (!dbgFirstShown && dbgFirstLen >= 32) {
dbgFirstShown = true;
Serial.print("[LD2450] 처음 받은 32바이트: ");
for (int i = 0; i < dbgFirstLen; i++) {
if (dbgFirst[i] < 0x10) Serial.print("0");
Serial.print(dbgFirst[i], HEX);
Serial.print(" ");
}
Serial.println();
}

if (millis() - dbgLastPrint < 2000) return;
dbgLastPrint = millis();

Serial.print("[LD2450] 수신 ");
Serial.print(dbgBytes);
Serial.print("B | 정상프레임 ");
Serial.print(dbgFramesOk);
Serial.print(" | 깨진프레임 ");
Serial.print(dbgFramesBad);
Serial.print(" | 인원 ");
Serial.print(targetCount);
Serial.print(" | loop최대 ");
Serial.print(dbgMaxLoop);
Serial.println("ms");

dbgBytes = 0; dbgFramesOk = 0; dbgFramesBad = 0; dbgMaxLoop = 0;
}

// ==========================================================
// IR 센서 인원 카운트
// ==========================================================
void updateCounter() {

bool ir1 = (digitalRead(IR1_PIN) == LOW);
bool ir2 = (digitalRead(IR2_PIN) == LOW);

if (millis() - lastCountTime < COUNT_COOLDOWN) {
prevIr1 = ir1;
prevIr2 = ir2;
return;
}

if (sequenceState != 0 &&
millis() - sequenceStart > SEQUENCE_TIMEOUT) {
sequenceState = 0;
}

bool ir1Triggered = (ir1 && !prevIr1);
bool ir2Triggered = (ir2 && !prevIr2);

prevIr1 = ir1;
prevIr2 = ir2;

if (sequenceState == 0) {
if (ir1Triggered && !ir2) {
sequenceState = 1;
sequenceStart = millis();
}
else if (ir2Triggered && !ir1) {
sequenceState = 2;
sequenceStart = millis();
}
}
else if (sequenceState == 1) {
if (ir2Triggered) {
peopleCount++;
lastCountTime = millis();
sequenceState = 0;
}
}
else if (sequenceState == 2) {
if (ir1Triggered) {
if (peopleCount > 0) peopleCount--;
lastCountTime = millis();
sequenceState = 0;
}
}
}

// ==========================================================
// 온도 및 가스 센서 측정
// ==========================================================
void readSensors() {

float t = dht.readTemperature();

if (!isnan(t)) {
if (t >= 10 && t <= 80) {
currentTemp = t;
tempSum += t;
tempCount++;
}
}

if (tempCount >= 5) {
avgTemp = tempSum / 5.0;
tempSum = 0;
tempCount = 0;
}

currentGas = analogRead(GAS_PIN);

// =========================
// 🔥 여기만 OR로 수정됨
// =========================
fire = false;
if (avgTemp != 0 &&
   (avgTemp >= TEMP_LIMIT || currentGas >= GAS_LIMIT)) {
    fire = true;
}

// 시리얼 출력
Serial.println("\r평균 온도: ");
Serial.println(avgTemp, 1);
Serial.println(" °C | 가스 값: ");
Serial.println(currentGas);
Serial.println(" | 인원 수: ");
Serial.println(peopleCount);

if (fire) {
Serial.println(" 🔥 화재 발생!");
} else {
Serial.println(" 🟢 정상");
}
}

// ==========================================================
// setup
// ==========================================================
void setup() {
Serial.begin(115200);

dht.begin();

// LD2450 시리얼 시작 (UART2)
Serial2.setRxBufferSize(LD_RX_BUFFER);                          // 웹페이지 전송 중에도 LD2450 데이터가 넘치지 않도록 버퍼 확대 (반드시 begin 앞에)
Serial2.begin(LD_BAUD, SERIAL_8N1, LD_RX_PIN, LD_TX_PIN);
Serial.println("LD2450 시작");
analogReadResolution(12);

pinMode(IR1_PIN, INPUT);
pinMode(IR2_PIN, INPUT);

WiFi.begin(ssid, password);

Serial.print("Wi-Fi 연결 중");
while (WiFi.status() != WL_CONNECTED) {
delay(500);
Serial.print(".");
}

Serial.println("\nWi-Fi 연결 완료!");
Serial.println(WiFi.localIP());

server.on("/", handleRoot);
server.on("/data", handleData);
server.on("/rooms", handleRooms);
server.on("/report", HTTP_POST, handleReport);
initDemoRooms();
server.begin();
}

// ==========================================================
// loop
// ==========================================================
void loop() {
unsigned long loopStart = millis();                             // 진단용: loop 1바퀴 시간 측정 시작
updateCounter();
readRadar();
syncMyRoom();
server.handleClient();

if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
lastSensorRead = millis();
readSensors();
}

unsigned long loopTime = millis() - loopStart;                  // 진단용: 이번 바퀴에 걸린 시간
if (loopTime > dbgMaxLoop) dbgMaxLoop = loopTime;
ldDebugPrint();

delay(5);
}