import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const DIR_LABELS = {
  'STATIC': 'ĐỨNG YÊN (STATIC)',
  '+X': 'ĐANG SANG PHẢI (+X)',
  '-X': 'ĐANG SANG TRÁI (-X)',
  '+Y': 'ĐANG TIẾN TỚI (+Y)',
  '-Y': 'ĐANG LÙI LẠI (-Y)',
  '+Z': 'ĐANG ĐI LÊN (+Z)',
  '-Z': 'ĐANG ĐI XUỐNG (-Z)',
  'ROT': 'ĐANG XOAY TẠI CHỖ (ROT)',
};

const el = {
  sceneContainer: document.getElementById('sceneContainer'),
  btnConnect: document.getElementById('btnConnect'),
  btnMock: document.getElementById('btnMock'),
  mockScenario: document.getElementById('mockScenario'),
  btnCal: document.getElementById('btnCal'),
  btnReset: document.getElementById('btnReset'),
  connDot: document.getElementById('connDot'),
  connStatus: document.getElementById('connStatus'),
  dirBadge: document.getElementById('dirBadge'),
  valRoll: document.getElementById('valRoll'),
  valPitch: document.getElementById('valPitch'),
  valYaw: document.getElementById('valYaw'),
  valAx: document.getElementById('valAx'),
  valAy: document.getElementById('valAy'),
  valAz: document.getElementById('valAz'),
  valVx: document.getElementById('valVx'),
  valVy: document.getElementById('valVy'),
  valVz: document.getElementById('valVz'),
  accChart: document.getElementById('accChart'),
  serialLog: document.getElementById('serialLog'),
};

// ---------------- 3D Scene Setup ----------------
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(
  50,
  el.sceneContainer.clientWidth / el.sceneContainer.clientHeight,
  0.05,
  50
);
camera.position.set(2.4, 1.9, 2.8);

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(el.sceneContainer.clientWidth, el.sceneContainer.clientHeight);
el.sceneContainer.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 0.25, 0);

scene.add(new THREE.AmbientLight(0xffffff, 0.7));
const dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
dirLight.position.set(3, 5, 4);
scene.add(dirLight);

const grid = new THREE.GridHelper(6, 16, 0x334155, 0x1e293b);
scene.add(grid);

// World axis tripod (IMU +X=Right, +Y=Forward(-Z_three), +Z=Up(+Y_three))
function addAxisArrow(dirVec, color, length = 1.1) {
  const arrow = new THREE.ArrowHelper(dirVec.clone().normalize(), new THREE.Vector3(0, 0.01, 0), length, color, 0.12, 0.06);
  scene.add(arrow);
}
addAxisArrow(new THREE.Vector3(1, 0, 0), 0xf87171);  // +X Right
addAxisArrow(new THREE.Vector3(0, 0, -1), 0x4ade80); // +Y Forward
addAxisArrow(new THREE.Vector3(0, 1, 0), 0x60a5fa);  // +Z Up

// Procedural ESP32-S3 + MPU6050 3D Device Model
const deviceGroup = new THREE.Group();
const pcbGeo = new THREE.BoxGeometry(0.55, 0.04, 1.05);
const pcbMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.45, metalness: 0.2 });
const pcbMesh = new THREE.Mesh(pcbGeo, pcbMat);
deviceGroup.add(pcbMesh);

const shieldGeo = new THREE.BoxGeometry(0.34, 0.06, 0.36);
const shieldMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, roughness: 0.25, metalness: 0.85 });
const shieldMesh = new THREE.Mesh(shieldGeo, shieldMat);
shieldMesh.position.set(0, 0.05, -0.22);
deviceGroup.add(shieldMesh);

const mpuGeo = new THREE.BoxGeometry(0.24, 0.035, 0.32);
const mpuMat = new THREE.MeshStandardMaterial({ color: 0x2563eb, roughness: 0.4, metalness: 0.3 });
const mpuMesh = new THREE.Mesh(mpuGeo, mpuMat);
mpuMesh.position.set(0, 0.045, 0.24);
deviceGroup.add(mpuMesh);

// Nose cone pointing Forward (+Y_imu = -Z_three)
const noseGeo = new THREE.ConeGeometry(0.11, 0.24, 16);
const noseMat = new THREE.MeshStandardMaterial({ color: 0x22d3ee, emissive: 0x0e7490, emissiveIntensity: 0.4 });
const noseMesh = new THREE.Mesh(noseGeo, noseMat);
noseMesh.rotation.x = -Math.PI / 2;
noseMesh.position.set(0, 0.04, -0.62);
deviceGroup.add(noseMesh);

const devicePivot = new THREE.Group();
devicePivot.position.set(0, 0.5, 0);
devicePivot.add(deviceGroup);
scene.add(devicePivot);

// Dynamic world-frame motion vector arrow
const motionArrow = new THREE.ArrowHelper(
  new THREE.Vector3(0, 0, -1),
  devicePivot.position,
  0.001,
  0x22d3ee,
  0.18,
  0.09
);
motionArrow.visible = false;
scene.add(motionArrow);

// Motion trail
const MAX_TRAIL = 80;
const trailPoints = [];
const trailGeo = new THREE.BufferGeometry();
const trailPositions = new Float32Array(MAX_TRAIL * 3);
trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
const trailLine = new THREE.Line(
  trailGeo,
  new THREE.LineBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.7 })
);
scene.add(trailLine);

// ---------------- Chart State ----------------
const MAX_HIST = 140;
const hist = { ax: [], ay: [], az: [] };
const ctx = el.accChart.getContext('2d');

function drawChart() {
  const w = el.accChart.width;
  const h = el.accChart.height;
  ctx.clearRect(0, 0, w, h);

  ctx.strokeStyle = '#1e293b';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, h / 2);
  ctx.lineTo(w, h / 2);
  ctx.stroke();

  const series = [
    { data: hist.ax, color: '#f87171' },
    { data: hist.ay, color: '#4ade80' },
    { data: hist.az, color: '#60a5fa' },
  ];
  const scaleY = (h * 0.42) / 10.0; // +/- 10 m/s^2 full range

  for (const s of series) {
    if (s.data.length < 2) continue;
    ctx.strokeStyle = s.color;
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    for (let i = 0; i < s.data.length; i++) {
      const x = (i / (MAX_HIST - 1)) * w;
      const y = h / 2 - Math.max(-10, Math.min(10, s.data[i])) * scaleY;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
}

// ---------------- Telemetry Application ----------------
const targetQuat = new THREE.Quaternion();
const targetPos = new THREE.Vector3(0, 0.5, 0);
const motionVec = new THREE.Vector3();

function logLine(msg) {
  const lines = el.serialLog.textContent ? el.serialLog.textContent.split('\n') : [];
  lines.push(msg);
  if (lines.length > 40) lines.shift();
  el.serialLog.textContent = lines.join('\n');
  el.serialLog.scrollTop = el.serialLog.scrollHeight;
}

function applyTelemetry(d) {
  // Exact right-handed basis mapping: IMU(X=Right, Y=Fwd, Z=Up) -> Three(X=Right, Y=Up, Z=-Fwd)
  targetQuat.set(d.qx, d.qz, -d.qy, d.qw).normalize();

  const posScale = 3.2; // Amplify small hand displacements for 3D clarity
  targetPos.set(d.px * posScale, 0.5 + d.pz * posScale, -d.py * posScale);

  // Use velocity if active, otherwise linear acceleration for the motion direction arrow
  const vx = d.vx || 0;
  const vy = d.vy || 0;
  const vz = d.vz || 0;
  const vMag = Math.hypot(vx, vy, vz);
  if (vMag > 0.03) {
    motionVec.set(vx, vz, -vy);
  } else {
    motionVec.set(d.ax, d.az, -d.ay);
  }

  const arrowMag = motionVec.length();
  if (arrowMag > 0.05 && d.dir !== 'STATIC' && d.dir !== 'ROT') {
    motionArrow.visible = true;
    motionArrow.position.copy(devicePivot.position);
    motionArrow.setDirection(motionVec.clone().normalize());
    const len = Math.min(1.6, 0.35 + arrowMag * 1.1);
    motionArrow.setLength(len, 0.16, 0.08);
  } else {
    motionArrow.visible = false;
  }

  // Update HUD & numeric metrics
  const dirKey = d.dir || 'STATIC';
  el.dirBadge.textContent = DIR_LABELS[dirKey] || dirKey;
  el.dirBadge.className = 'dir-badge ' + (
    dirKey === 'STATIC' ? 'dir-static' : (dirKey === 'ROT' ? 'dir-rot' : 'dir-active')
  );

  el.valRoll.textContent = `${Number(d.roll).toFixed(1)}°`;
  el.valPitch.textContent = `${Number(d.pitch).toFixed(1)}°`;
  el.valYaw.textContent = `${Number(d.yaw).toFixed(1)}°`;

  el.valAx.textContent = Number(d.ax).toFixed(2);
  el.valAy.textContent = Number(d.ay).toFixed(2);
  el.valAz.textContent = Number(d.az).toFixed(2);

  el.valVx.textContent = Number(vx).toFixed(2);
  el.valVy.textContent = Number(vy).toFixed(2);
  el.valVz.textContent = Number(vz).toFixed(2);

  hist.ax.push(d.ax);
  hist.ay.push(d.ay);
  hist.az.push(d.az);
  if (hist.ax.length > MAX_HIST) {
    hist.ax.shift();
    hist.ay.shift();
    hist.az.shift();
  }
  drawChart();
}

// ---------------- Web Serial Connection ----------------
let serialPort = null;
let serialReader = null;
let serialWriter = null;
let keepReading = false;

async function sendSerialCmd(cmd) {
  if (!serialWriter) return;
  const data = new TextEncoder().encode(cmd + '\n');
  await serialWriter.write(data);
  logLine(`> ${cmd}`);
}

async function disconnectSerial() {
  keepReading = false;
  if (serialReader) {
    try { await serialReader.cancel(); } catch {}
    serialReader = null;
  }
  if (serialWriter) {
    try { serialWriter.releaseLock(); } catch {}
    serialWriter = null;
  }
  if (serialPort) {
    try { await serialPort.close(); } catch {}
    serialPort = null;
  }
  el.btnConnect.textContent = 'Kết nối USB Serial';
  el.btnCal.disabled = true;
  if (!mockRunning) {
    el.connDot.className = 'dot';
    el.connStatus.textContent = 'Đã ngắt kết nối';
  }
}

async function connectSerial() {
  if (serialPort) {
    await disconnectSerial();
    return;
  }
  if (!('serial' in navigator)) {
    alert('Trình duyệt không hỗ trợ Web Serial API. Vui lòng dùng Chrome hoặc Edge.');
    return;
  }
  try {
    if (mockRunning) stopMock();
    serialPort = await navigator.serial.requestPort();
    await serialPort.open({ baudRate: 115200 });

    serialWriter = serialPort.writable.getWriter();
    keepReading = true;
    el.btnConnect.textContent = 'Ngắt kết nối';
    el.btnCal.disabled = false;
    el.connDot.className = 'dot connected';
    el.connStatus.textContent = 'Đã kết nối ESP32-S3 (115200 baud)';
    logLine('[Connected to USB Serial @ 115200]');

    const decoder = new TextDecoderStream();
    serialPort.readable.pipeTo(decoder.writable);
    serialReader = decoder.readable.getReader();

    let buffer = '';
    while (keepReading) {
      const { value, done } = await serialReader.read();
      if (done) break;
      buffer += value;
      const lines = buffer.split('\n');
      buffer = lines.pop() || '';
      for (const line of lines) {
        const trimmed = line.trim();
        if (!trimmed) continue;
        if (trimmed.startsWith('{')) {
          try {
            const obj = JSON.parse(trimmed);
            if (obj.evt) {
              logLine(`[ESP32] Event: ${obj.evt}`);
            } else if ('qw' in obj) {
              applyTelemetry(obj);
            }
          } catch {
            // Ignore partial frame
          }
        } else {
          logLine(trimmed);
        }
      }
    }
  } catch (err) {
    logLine(`[Serial Error] ${err.message}`);
    await disconnectSerial();
  }
}

// ---------------- Mock Simulator Mode ----------------
let mockRunning = false;
let mockTimer = null;
let mockT = 0;

function eulerToQuat(rollDeg, pitchDeg, yawDeg) {
  const r = THREE.MathUtils.degToRad(rollDeg) * 0.5;
  const p = THREE.MathUtils.degToRad(pitchDeg) * 0.5;
  const y = THREE.MathUtils.degToRad(yawDeg) * 0.5;
  const cr = Math.cos(r), sr = Math.sin(r);
  const cp = Math.cos(p), sp = Math.sin(p);
  const cy = Math.cos(y), sy = Math.sin(y);
  return {
    qw: cr * cp * cy + sr * sp * sy,
    qx: sr * cp * cy - cr * sp * sy,
    qy: cr * sp * cy + sr * cp * sy,
    qz: cr * cp * sy - sr * sp * cy,
  };
}

function stepMock() {
  mockT += 0.02; // 50 Hz
  const mode = el.mockScenario.value;
  let roll = 0, pitch = 0, yaw = 0;
  let px = 0, py = 0, pz = 0;
  let vx = 0, vy = 0, vz = 0;
  let ax = 0, ay = 0, az = 0;
  let dir = 'STATIC';

  const w = 2.4;
  if (mode === 'lr' || (mode === 'cycle' && (mockT % 12) < 3)) {
    const t = mode === 'cycle' ? (mockT % 3) : mockT;
    px = 0.25 * Math.sin(w * t);
    vx = 0.25 * w * Math.cos(w * t);
    ax = -0.25 * w * w * Math.sin(w * t);
    roll = -10 * Math.sin(w * t);
    dir = vx >= 0.05 ? '+X' : (vx <= -0.05 ? '-X' : 'STATIC');
  } else if (mode === 'fb' || (mode === 'cycle' && (mockT % 12) < 6)) {
    const t = mode === 'cycle' ? (mockT % 3) : mockT;
    py = 0.25 * Math.sin(w * t);
    vy = 0.25 * w * Math.cos(w * t);
    ay = -0.25 * w * w * Math.sin(w * t);
    pitch = 10 * Math.sin(w * t);
    dir = vy >= 0.05 ? '+Y' : (vy <= -0.05 ? '-Y' : 'STATIC');
  } else if (mode === 'ud' || (mode === 'cycle' && (mockT % 12) < 9)) {
    const t = mode === 'cycle' ? (mockT % 3) : mockT;
    pz = 0.20 * Math.sin(w * t);
    vz = 0.20 * w * Math.cos(w * t);
    az = -0.20 * w * w * Math.sin(w * t);
    dir = vz >= 0.05 ? '+Z' : (vz <= -0.05 ? '-Z' : 'STATIC');
  } else if (mode === 'tilted') {
    roll = 35;
    pitch = -12;
    px = 0.20 * Math.sin(w * mockT);
    py = 0.20 * Math.cos(w * mockT);
    vx = 0.20 * w * Math.cos(w * mockT);
    vy = -0.20 * w * Math.sin(w * mockT);
    ax = -0.20 * w * w * Math.sin(w * mockT);
    ay = -0.20 * w * w * Math.cos(w * mockT);
    dir = Math.abs(vx) >= Math.abs(vy) ? (vx >= 0 ? '+X' : '-X') : (vy >= 0 ? '+Y' : '-Y');
  } else {
    roll = 28 * Math.sin(1.8 * mockT);
    pitch = 22 * Math.cos(1.5 * mockT);
    yaw = 45 * Math.sin(0.9 * mockT);
    dir = 'ROT';
  }

  const q = eulerToQuat(roll, pitch, yaw);
  applyTelemetry({ ...q, ax, ay, az, vx, vy, vz, px, py, pz, roll, pitch, yaw, dir });
}

function startMock() {
  if (serialPort) disconnectSerial();
  mockRunning = true;
  mockT = 0;
  el.btnMock.classList.add('active');
  el.btnMock.textContent = 'Tắt Giả lập';
  el.connDot.className = 'dot mock';
  el.connStatus.textContent = 'Đang chạy Chế độ Giả lập (Mock 50 Hz)';
  logLine('[Mock Mode started]');
  mockTimer = setInterval(stepMock, 20);
}

function stopMock() {
  mockRunning = false;
  clearInterval(mockTimer);
  mockTimer = null;
  el.btnMock.classList.remove('active');
  el.btnMock.textContent = 'Bật Giả lập (Mock)';
  if (!serialPort) {
    el.connDot.className = 'dot';
    el.connStatus.textContent = 'Chưa kết nối — Hãy cắm USB hoặc bật Giả lập';
  }
}

el.btnConnect.addEventListener('click', connectSerial);
el.btnMock.addEventListener('click', () => (mockRunning ? stopMock() : startMock()));
el.btnCal.addEventListener('click', () => sendSerialCmd('CAL'));
el.btnReset.addEventListener('click', () => {
  trailPoints.length = 0;
  if (serialWriter) sendSerialCmd('RESET');
  else if (mockRunning) mockT = 0;
});

window.addEventListener('resize', () => {
  const w = el.sceneContainer.clientWidth;
  const h = el.sceneContainer.clientHeight;
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
});

// ---------------- Render Loop ----------------
function animate() {
  requestAnimationFrame(animate);

  devicePivot.quaternion.slerp(targetQuat, 0.35);
  devicePivot.position.lerp(targetPos, 0.35);

  trailPoints.push(devicePivot.position.clone());
  if (trailPoints.length > MAX_TRAIL) trailPoints.shift();
  for (let i = 0; i < trailPoints.length; i++) {
    trailPositions[i * 3] = trailPoints[i].x;
    trailPositions[i * 3 + 1] = trailPoints[i].y;
    trailPositions[i * 3 + 2] = trailPoints[i].z;
  }
  trailGeo.setDrawRange(0, trailPoints.length);
  trailGeo.attributes.position.needsUpdate = true;

  controls.update();
  renderer.render(scene, camera);
}
animate();
