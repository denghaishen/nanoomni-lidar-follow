const canvas = document.getElementById('lidarCanvas');
const context = canvas.getContext('2d');
const connection = document.getElementById('connection');
const followStatus = document.getElementById('followStatus');
const motionStatus = document.getElementById('motionStatus');
const batteryValue = document.getElementById('batteryValue');
const targetDistance = document.getElementById('targetDistance');
const commandValue = document.getElementById('commandValue');
const radiusValue = document.getElementById('radiusValue');
const message = document.getElementById('message');
const distanceSlider = document.getElementById('distanceSlider');
const distanceValue = document.getElementById('distanceValue');
const systemStatus = document.getElementById('systemStatus');
const systemStartButton = document.getElementById('systemStartButton');
const systemStopButton = document.getElementById('systemStopButton');
const followStartButton = document.getElementById('startButton');
const followStopButton = document.getElementById('stopButton');
document.getElementById('pageAddress').textContent = location.origin;

const view = { front: 3.5, back: 1.0, side: 2.0 };
let socket;
let points = [];
let trackedTarget = null;
let selection = null;
let dragging = false;
let distanceEditing = false;
let systemRunning = false;
let drawPending = false;
let limits = { min_radius: 0.10, max_radius: 0.80 };

function resizeCanvas() {
  const ratio = Math.min(window.devicePixelRatio || 1, 1.5);
  const rectangle = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.round(rectangle.width * ratio));
  canvas.height = Math.max(1, Math.round(rectangle.height * ratio));
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  draw();
}

function dimensions() {
  const rectangle = canvas.getBoundingClientRect();
  const scale = Math.min(
    rectangle.width / (view.side * 2),
    rectangle.height / (view.front + view.back)
  );
  return {
    width: rectangle.width,
    height: rectangle.height,
    scale,
    originX: rectangle.width / 2,
    originY: rectangle.height - view.back * scale
  };
}

function toPixel(point, d = dimensions()) {
  return {
    x: d.originX - point.y * d.scale,
    y: d.originY - point.x * d.scale
  };
}

function toRobot(clientX, clientY) {
  const rectangle = canvas.getBoundingClientRect();
  const d = dimensions();
  const pixelX = clientX - rectangle.left;
  const pixelY = clientY - rectangle.top;
  return {
    x: (d.originY - pixelY) / d.scale,
    y: (d.originX - pixelX) / d.scale
  };
}

function connect() {
  const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
  socket = new WebSocket(`${protocol}//${location.host}/ws`);
  connection.textContent = '连接中';
  connection.className = 'badge offline';

  socket.addEventListener('open', () => {
    connection.textContent = '已连接';
    connection.className = 'badge online';
    setMessage('按住目标点云中心并拖动，黄色圆覆盖目标后松开。');
  });
  socket.addEventListener('close', () => {
    connection.textContent = '已断开';
    connection.className = 'badge offline';
    setTimeout(connect, 1500);
  });
  socket.addEventListener('message', event => {
    const data = JSON.parse(event.data);
    if (data.type === 'scan_data') {
      points = data.points || [];
      trackedTarget = data.target || trackedTarget;
      limits = data.selection_limits || limits;
      if (data.distance_limits) {
        distanceSlider.min = data.distance_limits.min;
        distanceSlider.max = data.distance_limits.max;
      }
      if (typeof data.distance === 'number' && !distanceEditing) {
        distanceSlider.value = data.distance;
        distanceValue.textContent = `${data.distance.toFixed(2)} m`;
      }
      const status = data.status || {};
      followStatus.textContent = status.follow || 'UNKNOWN';
      motionStatus.textContent = status.motion || 'UNKNOWN';
      if (data.battery && typeof data.battery.voltage === 'number') {
        const current = typeof data.battery.current === 'number'
          ? ` · ${data.battery.current.toFixed(2)} A`
          : '';
        batteryValue.textContent =
          `${data.battery.voltage.toFixed(2)} V${current}`;
      }
      if (trackedTarget) {
        const distance = Math.hypot(trackedTarget.x, trackedTarget.y);
        targetDistance.textContent = `${distance.toFixed(2)} m`;
      } else {
        targetDistance.textContent = '-- m';
      }
      if (data.command && typeof data.command.linear_x === 'number') {
        const vx = data.command.linear_x;
        const label = vx > 0.01 ? '前进' : (vx < -0.01 ? '后退' : '停止');
        commandValue.textContent = `${label} ${vx.toFixed(2)} m/s`;
      }
      scheduleDraw();
    } else if (data.type === 'selection') {
      selection = { x: data.x, y: data.y, radius: data.radius };
      radiusValue.textContent = `${data.radius.toFixed(2)} m`;
      setMessage('区域已发送；绿色圆点出现后表示目标已锁定。');
    } else if (data.type === 'distance') {
      distanceValue.textContent = `${data.distance.toFixed(2)} m`;
      setMessage(`保持距离已设为 ${data.distance.toFixed(2)} m`);
    } else if (data.type === 'system_state') {
      updateSystemState(data);
    } else if (data.type === 'error') {
      setMessage(data.message, true);
    }
  });
}

function send(command) {
  if (!socket || socket.readyState !== WebSocket.OPEN) {
    setMessage('网页尚未连接到机器人。', true);
    return;
  }
  socket.send(JSON.stringify(command));
}

function setMessage(text, error = false) {
  message.textContent = text;
  message.className = error ? 'message error' : 'message';
}

function updateSystemState(data) {
  const labels = {
    STOPPED: '已停止',
    RUNNING: '运行中',
    STOPPING: '正在停止',
    FAILED: '启动失败',
    UNAVAILABLE: '不可用'
  };
  const state = data.state || 'UNAVAILABLE';
  systemStatus.textContent = labels[state] || state;
  systemStatus.dataset.state = state;
  systemStartButton.disabled = !data.enabled || !['STOPPED', 'FAILED'].includes(state);
  systemStopButton.disabled = !data.enabled || !['RUNNING', 'STOPPING'].includes(state);
  const running = state === 'RUNNING';
  systemRunning = running;
  if (!running) {
    batteryValue.textContent = '-- V';
    targetDistance.textContent = '-- m';
    commandValue.textContent = '0.00 m/s';
  }
  followStartButton.disabled = !running;
  followStopButton.disabled = !running;
  distanceSlider.disabled = !running;
  if (state === 'FAILED' && data.error) {
    setMessage(data.error, true);
  }
}

function beginSelection(event) {
  if (!systemRunning) {
    setMessage('请先启动机器人系统。', true);
    return;
  }
  canvas.setPointerCapture(event.pointerId);
  const center = toRobot(event.clientX, event.clientY);
  selection = { x: center.x, y: center.y, radius: 0.30 };
  dragging = true;
  scheduleDraw();
}

function updateSelection(event) {
  if (!dragging || !selection) return;
  const edge = toRobot(event.clientX, event.clientY);
  const radius = Math.hypot(edge.x - selection.x, edge.y - selection.y);
  selection.radius = Math.max(limits.min_radius, Math.min(radius, limits.max_radius));
  radiusValue.textContent = `${selection.radius.toFixed(2)} m`;
  scheduleDraw();
}

function finishSelection(event) {
  if (!dragging || !selection) return;
  updateSelection(event);
  dragging = false;
  send({
    type: 'select_region',
    x: selection.x,
    y: selection.y,
    radius: selection.radius
  });
}

canvas.addEventListener('pointerdown', beginSelection);
canvas.addEventListener('pointermove', updateSelection);
canvas.addEventListener('pointerup', finishSelection);
canvas.addEventListener('pointercancel', () => { dragging = false; });
systemStartButton.addEventListener('click', () => {
  systemStartButton.disabled = true;
  setMessage('正在启动机器人系统，请等待雷达点出现。');
  send({ type: 'system_start' });
});
systemStopButton.addEventListener('click', () => {
  systemStopButton.disabled = true;
  setMessage('正在安全停止机器人系统。');
  send({ type: 'system_stop' });
});
followStopButton.addEventListener('click', () => send({ type: 'stop' }));
followStartButton.addEventListener('click', () => send({ type: 'start' }));
distanceSlider.addEventListener('input', () => {
  distanceEditing = true;
  distanceValue.textContent = `${Number(distanceSlider.value).toFixed(2)} m`;
});
distanceSlider.addEventListener('change', () => {
  const value = Number(distanceSlider.value);
  distanceValue.textContent = `${value.toFixed(2)} m`;
  send({ type: 'set_distance', distance: value });
  distanceEditing = false;
});
window.addEventListener('resize', resizeCanvas);
document.addEventListener('visibilitychange', () => {
  if (!document.hidden) scheduleDraw();
});

function scheduleDraw() {
  if (drawPending || document.hidden) return;
  drawPending = true;
  requestAnimationFrame(() => {
    drawPending = false;
    draw();
  });
}

function draw() {
  const d = dimensions();
  context.clearRect(0, 0, d.width, d.height);
  context.fillStyle = '#050a0f';
  context.fillRect(0, 0, d.width, d.height);
  drawGrid(d);
  drawPoints(d);
  if (selection) drawCircle(selection, '#ffd84d', 2, d);
  if (trackedTarget) drawCircle({ ...trackedTarget, radius: 0.30 }, '#3ce890', 3, d);
  drawRobot(d);
}

function drawGrid(d) {
  context.strokeStyle = 'rgba(130,160,185,.18)';
  context.lineWidth = 1;
  for (let distance = 0.5; distance <= 3.5; distance += 0.5) {
    const center = { x: d.originX, y: d.originY };
    context.beginPath();
    context.arc(center.x, center.y, distance * d.scale, 0, Math.PI * 2);
    context.stroke();
  }
  context.strokeStyle = 'rgba(130,160,185,.28)';
  context.beginPath();
  context.moveTo(0, d.originY);
  context.lineTo(d.width, d.originY);
  context.moveTo(d.originX, 0);
  context.lineTo(d.originX, d.height);
  context.stroke();
}

function drawPoints(d) {
  context.fillStyle = '#ff4d58';
  context.beginPath();
  for (const point of points) {
    const pixel = toPixel(point, d);
    context.moveTo(pixel.x + 3, pixel.y);
    context.arc(pixel.x, pixel.y, 3, 0, Math.PI * 2);
  }
  context.fill();
}

function drawCircle(region, color, lineWidth, d) {
  const center = toPixel(region, d);
  context.strokeStyle = color;
  context.lineWidth = lineWidth;
  context.beginPath();
  context.arc(center.x, center.y, region.radius * d.scale, 0, Math.PI * 2);
  context.stroke();
  context.beginPath();
  context.moveTo(center.x - 7, center.y);
  context.lineTo(center.x + 7, center.y);
  context.moveTo(center.x, center.y - 7);
  context.lineTo(center.x, center.y + 7);
  context.stroke();
}

function drawRobot(d) {
  context.fillStyle = '#2ad6e9';
  context.beginPath();
  context.arc(d.originX, d.originY, 9, 0, Math.PI * 2);
  context.fill();
  context.strokeStyle = '#2ad6e9';
  context.lineWidth = 3;
  context.beginPath();
  context.moveTo(d.originX, d.originY);
  context.lineTo(d.originX, d.originY - 28);
  context.stroke();
}

resizeCanvas();
connect();
