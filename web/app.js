// =====================================================================
//  CONFIGURATION — adjust topic names / endpoints here
// =====================================================================
const CFG = {
    topics: {
        map: '/map',
        plan: '/plan',
        odom: '/odom',
        tf: '/tf',
        tfStatic: '/tf_static',
        joint: '/joint_states',
        cmd: '/velocity_controller/commands',
        cmdVel: '/cmd_vel',
        joy: '/joy',
        camera: '/camera/image_raw'
    },
    mapFrame: 'map',
    baseFrames: ['base_link', 'base_footprint'],
    cmdOrder: ['fr', 'br', 'fl', 'bl'],
    jointOverrides: {},
    modeUrl: '/system/mode',
    modeTimeoutMs: 40000,
    modePollMs: 1000,
    modeIdlePollMs: 5000,
    chartWindowSec: 30,
    joyRateHz: 20
};

const MODE_LABELS = { slam: 'SLAM Only', slam_nav: 'SLAM + Nav', slam_update: 'SLAM Update + Nav', nav: 'Nav Only', manual: 'Manual' };

// =====================================================================
//  Notifications
// =====================================================================
const NOTIFY_TTL = { ERROR: 12000, WARNING: 8000, INFO: 5000 };

function notify(level, message) {
    level = (level || 'INFO').toUpperCase();
    const stack = document.getElementById('notify-stack');
    for (const el of stack.children) {
        if (el.dataset.key === level + message) { resetNoticeTimer(el, level); return; }
    }
    const el = document.createElement('div');
    el.className = 'notice ' + level.toLowerCase();
    el.dataset.key = level + message;
    el.setAttribute('role', level === 'ERROR' ? 'alert' : 'status');
    el.innerHTML = '<div class="notice-hd"><span class="notice-lvl">' + level +
        '</span><button class="notice-x" aria-label="Dismiss">&times;</button></div><div class="notice-msg"></div>';
    el.querySelector('.notice-msg').textContent = message;
    el.querySelector('.notice-x').onclick = () => dismissNotice(el);
    stack.appendChild(el);
    while (stack.children.length > 5) stack.removeChild(stack.firstChild);
    resetNoticeTimer(el, level);
}
function resetNoticeTimer(el, level) {
    clearTimeout(el._t);
    el._t = setTimeout(() => dismissNotice(el), NOTIFY_TTL[level] || 5000);
}
function dismissNotice(el) {
    clearTimeout(el._t);
    el.classList.add('leaving');
    setTimeout(() => el.remove(), 220);
}

// =====================================================================
//  Theme
// =====================================================================
const theme = {};
function readTheme() {
    const cs = getComputedStyle(document.body);
    ['--map-bg', '--map-free', '--map-occ', '--map-grid', '--series-a', '--series-b', '--border', '--muted', '--text', '--success', '--primary', '--panel']
        .forEach(k => theme[k] = cs.getPropertyValue(k).trim());
}
function applyTheme(isLight) {
    document.body.classList.toggle('light-theme', isLight);
    readTheme();
    mapImageDirty = true; needsDraw = true;
}
function toggleTheme() {
    const isLight = !document.body.classList.contains('light-theme');
    applyTheme(isLight);
    try { localStorage.setItem('milusions-theme', isLight ? 'light' : 'dark'); } catch (e) {}
}

// =====================================================================
//  State & Watchdogs
// =====================================================================
let mapImageDirty = false, needsDraw = true;
let latestMap = null;
let latestPlan = []; 
let navStatus = 'Idle'; 
let distanceRemaining = 0;
const mapCanvasOff = document.createElement('canvas');
const tfTree = {};
let odom = null;
let robot = null;
let currentCmdVel = { linear: 0, angular: 0 };

let activeNavBtn = null;
let navInitTimeout = null;

function setNavLoading(btn) {
    if (activeNavBtn) {
        activeNavBtn.classList.remove('loading');
        activeNavBtn.disabled = false;
    }
    activeNavBtn = btn;
    if (activeNavBtn) {
        activeNavBtn.classList.add('loading');
        activeNavBtn.disabled = true;
    }
}

function clearNavLoading() {
    if (navInitTimeout) {
        clearTimeout(navInitTimeout);
        navInitTimeout = null;
    }
    if (activeNavBtn) {
        activeNavBtn.classList.remove('loading');
        activeNavBtn.disabled = false;
        activeNavBtn = null;
    }
}

function startNavWatchdog() {
    if (navInitTimeout) clearTimeout(navInitTimeout);
    navInitTimeout = setTimeout(async () => {
        clearNavLoading();
        navStatus = 'Aborted';
        latestPlan = [];
        try {
            await postJSON('/abort');
        } catch (e) {}
        notify('ERROR', 'something is wrong with the navigation server.');
    }, 20000);
}

const wheelIds = ['fl', 'fr', 'bl', 'br'];
const series = {};
wheelIds.forEach(id => series[id] = { a: [], c: [] });

// =====================================================================
//  ROS connection
// =====================================================================
// ---- Rover IP (stored in localStorage as "rover_ip") ----
const ROVER_IP_KEY = 'rover_ip';
function normalizeRoverIp(v) {
    return (v || '').trim().replace(/^[a-z]+:\/\//i, '').replace(/[\/?#].*$/, '').replace(/:\d+$/, '');
}
function isValidRoverHost(h) {
    return /^(\d{1,3}(\.\d{1,3}){3}|[a-z0-9]([a-z0-9.-]*[a-z0-9])?)$/i.test(h);
}
function getRoverIp() {
    try { return normalizeRoverIp(localStorage.getItem(ROVER_IP_KEY)); } catch (e) { return ''; }
}
function saveRoverIpAndReload(v) {
    const ip = normalizeRoverIp(v);
    if (!isValidRoverHost(ip)) return false;
    try { localStorage.setItem(ROVER_IP_KEY, ip); } catch (e) {}
    location.reload();
    return true;
}
const ROVER_IP = getRoverIp();
const ROVER_HOST = ROVER_IP;   // fallback only while the IP dialog is showing

const ROS_URL = 'ws://' + ROVER_HOST + ':9090';
const ros = new ROSLIB.Ros({ url: ROS_URL });
const statusEl = document.getElementById('ros-status');
let rosState = 'init';
let subs = [];
let joyTopic = null;

ros.on('connection', () => {
    statusEl.textContent = 'ROS Connected';
    statusEl.classList.add('connected');
    const overlay = document.getElementById('connection-overlay');
    if (overlay) overlay.style.display = 'none';
    if (rosState !== 'up') notify('INFO', 'Connected to rosbridge.');
    rosState = 'up';
    subscribeAll();
});
ros.on('error', () => {
    statusEl.textContent = 'Connection Error';
    statusEl.classList.remove('connected');
});
ros.on('close', () => {
    statusEl.textContent = 'Disconnected';
    statusEl.classList.remove('connected');
    const overlay = document.getElementById('connection-overlay');
    if (overlay) overlay.style.display = 'flex';
    if (rosState === 'up') notify('WARNING', 'Rosbridge connection lost. Retrying…');
    else if (rosState === 'init') notify('ERROR', 'Unable to connect to rosbridge at ' + ROS_URL + '.');
    rosState = 'down';
    setTimeout(() => { try { ros.connect(ROS_URL); } catch (e) {} }, 5000);
});

function strip(f) { return (f || '').replace(/^\//, ''); }
function quatYaw(q) {
    return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

function subscribeAll() {
    subs.forEach(s => { try { s.unsubscribe(); } catch (e) {} });
    subs = [];
    const T = CFG.topics;

    const mk = (name, type, cb) => {
        const t = new ROSLIB.Topic({ ros, name, messageType: type });
        t.subscribe(cb); subs.push(t); return t;
    };

    mk(T.map, 'nav_msgs/OccupancyGrid', (m) => {
        latestMap = {
            w: m.info.width, h: m.info.height, res: m.info.resolution,
            ox: m.info.origin.position.x, oy: m.info.origin.position.y,
            yaw: quatYaw(m.info.origin.orientation), data: m.data
        };
        mapImageDirty = true; needsDraw = true;
    });

    mk(T.plan, 'nav_msgs/Path', (m) => {
        if (m && m.poses) {
            latestPlan = m.poses.map(p => ({
                x: p.pose.position.x,
                y: p.pose.position.y,
                yaw: quatYaw(p.pose.orientation)
            }));
            if (latestPlan.length > 0) {
                if (navStatus === 'Idle') {
                    navStatus = 'Navigating';
                }
                clearNavLoading();
            }
            needsDraw = true;
        }
    });

    mk(T.odom, 'nav_msgs/Odometry', (m) => {
        const v = m.twist.twist.linear;
        odom = {
            x: m.pose.pose.position.x, y: m.pose.pose.position.y,
            yaw: quatYaw(m.pose.pose.orientation), speed: Math.hypot(v.x, v.y)
        };
        needsDraw = true;
    });

    mk(T.cmdVel, 'geometry_msgs/Twist', (m) => {
        currentCmdVel = { linear: m.linear.x, angular: m.angular.z };
        needsDraw = true;
    });

    const onTf = (m) => {
        m.transforms.forEach(t => {
            tfTree[strip(t.child_frame_id)] = {
                parent: strip(t.header.frame_id),
                x: t.transform.translation.x, y: t.transform.translation.y,
                yaw: quatYaw(t.transform.rotation)
            };
        });
        needsDraw = true;
    };
    mk(T.tf, 'tf2_msgs/TFMessage', onTf);
    mk(T.tfStatic, 'tf2_msgs/TFMessage', onTf);

    mk(T.joint, 'sensor_msgs/JointState', onJointState);

    const cam = new ROSLIB.Topic({ ros, name: T.camera, messageType: 'sensor_msgs/Image', throttle_rate: 100, queue_length: 1 });
    cam.subscribe(onCameraImage); subs.push(cam);

    ros.getTopicType(T.cmd, (type) => {
        if (!type) return;
        mk(T.cmd, type, onCommand);
    }, () => notify('WARNING', 'Command topic ' + T.cmd + ' not found; wheel command traces will be empty.'));

    joyTopic = new ROSLIB.Topic({ ros, name: T.joy, messageType: 'sensor_msgs/Joy' });
}

function resolveRobotPose() {
    for (const base of CFG.baseFrames) {
        if (!tfTree[base]) continue;
        let cur = base, acc = { x: 0, y: 0, yaw: 0 }, depth = 0;
        while (cur !== CFG.mapFrame && depth++ < 12) {
            const t = tfTree[cur];
            if (!t) { acc = null; break; }
            const c = Math.cos(t.yaw), s = Math.sin(t.yaw);
            acc = { x: t.x + c * acc.x - s * acc.y, y: t.y + s * acc.x + c * acc.y, yaw: t.yaw + acc.yaw };
            cur = t.parent;
        }
        if (acc && cur === CFG.mapFrame) return { x: acc.x, y: acc.y, yaw: acc.yaw, src: 'map' };
    }
    if (odom) return { x: odom.x, y: odom.y, yaw: odom.yaw, src: 'odom' };
    return null;
}

function classifyJoint(name) {
    if (CFG.jointOverrides[name]) return CFG.jointOverrides[name];
    const n = name.toLowerCase();
    if (/(^|[_\-])(fl|lf)([_\-]|$)|front[_\-]?left|left[_\-]?front/.test(n)) return 'fl';
    if (/(^|[_\-])(fr|rf)([_\-]|$)|front[_\-]?right|right[_\-]?front/.test(n)) return 'fr';
    if (/(^|[_\-])(bl|lb|rl|lr)([_\-]|$)|(rear|back)[_\-]?left|left[_\-]?(rear|back)/.test(n)) return 'bl';
    if (/(^|[_\-])(br|rb|rr)([_\-]|$)|(rear|back)[_\-]?right|right[_\-]?(rear|back)/.test(n)) return 'br';
    return null;
}

function formatSigned(v, digits = 2) {
    const n = Number(v);
    if (!Number.isFinite(n)) return '—';
    if (Math.abs(n) < 0.0005) return '0.' + '0'.repeat(digits);
    return (n > 0 ? '+' : '') + n.toFixed(digits);
}

function pushPoint(arr, v) {
    const n = Number(v);
    if (!Number.isFinite(n)) return;
    const now = performance.now() / 1000;
    arr.push({ t: now, v: n });
    const cutoff = now - CFG.chartWindowSec - 5;
    while (arr.length && arr[0].t < cutoff) arr.shift();
}
function onJointState(m) {
    if (!m.velocity || !m.name) return;
    m.name.forEach((n, i) => {
        const id = classifyJoint(n);
        if (id && typeof m.velocity[i] === 'number') pushPoint(series[id].a, m.velocity[i]);
    });
}
function onCommand(m) {
    if (!m.data) return;
    CFG.cmdOrder.forEach((id, i) => { if (typeof m.data[i] === 'number') pushPoint(series[id].c, m.data[i]); });
}

const camCanvas = document.getElementById('cam-canvas');
let lastCamFrame = 0;
function onCameraImage(m) {
    const w = m.width, h = m.height;
    if (!w || !h) return;
    const bin = atob(m.data), step = m.step || 0;
    const enc = (m.encoding || '').toLowerCase();
    const ch = { rgb8: 3, bgr8: 3, rgba8: 4, bgra8: 4, mono8: 1, '8uc1': 1, '8uc3': 3 }[enc];
    if (!ch) {
        if (!onCameraImage.warned) { onCameraImage.warned = true; notify('WARNING', 'Unsupported camera encoding: ' + m.encoding + '.'); }
        return;
    }
    if (camCanvas.width !== w || camCanvas.height !== h) { camCanvas.width = w; camCanvas.height = h; }
    const ctx = camCanvas.getContext('2d'), img = ctx.createImageData(w, h), d = img.data;
    const stride = step || w * ch, swap = (enc === 'bgr8' || enc === 'bgra8');
    for (let y = 0; y < h; y++) {
        let si = y * stride, di = y * w * 4;
        for (let x = 0; x < w; x++, si += ch, di += 4) {
            if (ch === 1) { d[di] = d[di + 1] = d[di + 2] = bin.charCodeAt(si); }
            else {
                const r = bin.charCodeAt(si), g = bin.charCodeAt(si + 1), b = bin.charCodeAt(si + 2);
                d[di] = swap ? b : r; d[di + 1] = g; d[di + 2] = swap ? r : b;
            }
            d[di + 3] = 255;
        }
    }
    ctx.putImageData(img, 0, 0);
    lastCamFrame = performance.now();
    document.getElementById('cam-empty').style.display = 'none';
    document.getElementById('cam-meta').textContent = '· ' + w + '×' + h;
}

const chartCanvases = {};
wheelIds.forEach(id => chartCanvases[id] = document.getElementById('chart-' + id));

function fitCanvas(canvas) {
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
        canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { ctx, w, h };
}

function drawChart(id) {
    const { ctx, w, h } = fitCanvas(chartCanvases[id]);
    if (!w || !h) return;
    ctx.clearRect(0, 0, w, h);
    const now = performance.now() / 1000, win = CFG.chartWindowSec, t0 = now - win;
    const L = 46, R = 10, T = 10, B = 20;
    const pw = w - L - R, ph = h - T - B;
    const A = series[id].a.filter(p => p.t >= t0 - 1), C = series[id].c;

    let lo = Infinity, hi = -Infinity;
    const scan = p => { if (p.t >= t0 - 1) { lo = Math.min(lo, p.v); hi = Math.max(hi, p.v); } };
    A.forEach(scan); C.forEach(scan);
    if (!isFinite(lo)) { lo = -1; hi = 1; }
    if (hi - lo < 0.2) { const mid = (hi + lo) / 2; lo = mid - 0.1; hi = mid + 0.1; }
    const pad = (hi - lo) * 0.1; lo -= pad; hi += pad;

    const X = t => L + ((t - t0) / win) * pw;
    const Y = v => T + (1 - (v - lo) / (hi - lo)) * ph;

    ctx.font = '400 10px Roboto, sans-serif';
    ctx.lineWidth = 1;
    ctx.strokeStyle = theme['--border']; ctx.fillStyle = theme['--muted'];
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    for (let i = 0; i <= 4; i++) {
        const v = lo + (hi - lo) * (i / 4), y = Math.round(Y(v)) + 0.5;
        ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(w - R, y); ctx.stroke();
        ctx.fillText(v.toFixed(2), L - 6, y);
    }
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    for (let s = 0; s <= win; s += 10) {
        const x = Math.round(X(t0 + s)) + 0.5;
        ctx.beginPath(); ctx.moveTo(x, T); ctx.lineTo(x, T + ph); ctx.stroke();
        ctx.fillText(s === win ? 'now' : '-' + (win - s) + 's', x, T + ph + 4);
    }
    if (lo < 0 && hi > 0) {
        ctx.strokeStyle = theme['--muted']; ctx.globalAlpha = 0.6;
        const y0 = Math.round(Y(0)) + 0.5;
        ctx.beginPath(); ctx.moveTo(L, y0); ctx.lineTo(w - R, y0); ctx.stroke(); ctx.globalAlpha = 1;
    }

    ctx.save();
    ctx.beginPath(); ctx.rect(L, T, pw, ph); ctx.clip();
    if (C.length) {
        ctx.strokeStyle = theme['--series-b']; ctx.lineWidth = 1.6; ctx.setLineDash([5, 3]);
        ctx.beginPath();
        let started = false, prevY = 0;
        C.forEach(p => {
            const x = X(p.t), y = Y(p.v);
            if (!started) { ctx.moveTo(x, y); started = true; } else { ctx.lineTo(x, prevY); ctx.lineTo(x, y); }
            prevY = y;
        });
        ctx.lineTo(X(now), prevY); ctx.stroke(); ctx.setLineDash([]);
    }
    if (A.length) {
        ctx.strokeStyle = theme['--series-a']; ctx.lineWidth = 1.6;
        ctx.beginPath();
        A.forEach((p, i) => { const x = X(p.t), y = Y(p.v); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
        ctx.stroke();
    }
    ctx.restore();

    const la = A.length ? A[A.length - 1].v : null, lc = C.length ? C[C.length - 1].v : null;
    document.getElementById('lg-' + id + '-a').textContent = formatSigned(la);
    document.getElementById('lg-' + id + '-c').textContent = formatSigned(lc);
}
setInterval(() => wheelIds.forEach(drawChart), 100);

// =====================================================================
//  SLAM map canvas & Interaction
// =====================================================================
const mapWrap = document.getElementById('map-wrapper');
const mapCanvas = document.getElementById('map-canvas');
const view = { vx: 0, vy: 0, s: 30 };
let follow = false, viewFitted = false;
let clickMode = 'single';
let singleTarget = null;
let waypointsData = [];
let activeGoal = null, panState = null;

const mapSize = () => ({ W: mapCanvas.clientWidth, H: mapCanvas.clientHeight });
function w2s(x, y) { const { W, H } = mapSize(); return [W / 2 - (y - view.vy) * view.s, H / 2 - (x - view.vx) * view.s]; }
function s2w(sx, sy) { const { W, H } = mapSize(); return { x: view.vx - (sy - H / 2) / view.s, y: view.vy - (sx - W / 2) / view.s }; }

function fitMapView() {
    if (!latestMap) return;
    const m = latestMap, c = Math.cos(m.yaw), s = Math.sin(m.yaw);
    const cs = [[0, 0], [m.w, 0], [0, m.h], [m.w, m.h]].map(([i, j]) =>
        [m.ox + m.res * (i * c - j * s), m.oy + m.res * (i * s + j * c)]);
    const xs = cs.map(p => p[0]), ys = cs.map(p => p[1]);
    const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
    const { W, H } = mapSize();
    view.vx = (minX + maxX) / 2; view.vy = (minY + maxY) / 2;
    view.s = Math.max(2, Math.min(W / (maxY - minY || 1), H / (maxX - minX || 1)) * 0.94);
    follow = false; document.getElementById('follow-btn').classList.remove('on');
    viewFitted = true; needsDraw = true;
}
function toggleFollow() {
    follow = !follow;
    document.getElementById('follow-btn').classList.toggle('on', follow);
    needsDraw = true;
}

function rebuildMapImage() {
    const m = latestMap; if (!m) return;
    mapCanvasOff.width = m.w; mapCanvasOff.height = m.h;
    const ctx = mapCanvasOff.getContext('2d');
    const img = ctx.createImageData(m.w, m.h), d = img.data;
    const parse = (hex) => {
        const c = document.createElement('canvas').getContext('2d'); c.fillStyle = hex; c.fillRect(0, 0, 1, 1);
        return c.getImageData(0, 0, 1, 1).data;
    };
    const free = parse(theme['--map-free']), occ = parse(theme['--map-occ']);
    for (let j = 0; j < m.h; j++) {
        for (let i = 0; i < m.w; i++) {
            const v = m.data[j * m.w + i], k = (j * m.w + i) * 4;
            if (v < 0) { d[k + 3] = 0; }
            else {
                const t = Math.min(v, 100) / 100;
                d[k] = free[0] + (occ[0] - free[0]) * t;
                d[k + 1] = free[1] + (occ[1] - free[1]) * t;
                d[k + 2] = free[2] + (occ[2] - free[2]) * t;
                d[k + 3] = 255;
            }
        }
    }
    ctx.putImageData(img, 0, 0);
    document.getElementById('map-empty').style.display = 'none';
    document.getElementById('map-meta').textContent =
        '· ' + m.w + '×' + m.h + ' · ' + m.res.toFixed(3) + ' m/cell';
}

function drawArrow(ctx, x, y, yawRad, color, label, size) {
    const p = w2s(x, y), q = w2s(x + Math.cos(yawRad), y + Math.sin(yawRad));
    const ang = Math.atan2(q[1] - p[1], q[0] - p[0]);
    ctx.save();
    ctx.translate(p[0], p[1]); ctx.rotate(ang);
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(size, 0); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(size + 9, 0); ctx.lineTo(size - 2, -6); ctx.lineTo(size - 2, 6); ctx.closePath(); ctx.fill();
    ctx.restore();
    ctx.fillStyle = color; ctx.beginPath(); ctx.arc(p[0], p[1], 5, 0, 7); ctx.fill();
    ctx.strokeStyle = theme['--panel']; ctx.lineWidth = 1.5; ctx.stroke();
    if (label) {
        ctx.font = '700 11px Roboto, sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
        ctx.fillStyle = color; ctx.fillText(label, p[0], p[1] - 9);
    }
}

function drawPathWithArrows(ctx, points, color) {
    if (!points || points.length < 2) return;
    
    ctx.strokeStyle = color;
    ctx.lineWidth = 3;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    points.forEach((pt, idx) => {
        const sPt = w2s(pt.x, pt.y);
        if (idx === 0) ctx.moveTo(sPt[0], sPt[1]);
        else ctx.lineTo(sPt[0], sPt[1]);
    });
    ctx.stroke();
    ctx.setLineDash([]);

    const spacingPx = 45;
    let accumulatedDist = 0;
    
    for (let i = 0; i < points.length - 1; i++) {
        const p1 = w2s(points[i].x, points[i].y);
        const p2 = w2s(points[i+1].x, points[i+1].y);
        const segLen = Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
        if (segLen === 0) continue;

        const angle = Math.atan2(p2[1] - p1[1], p2[0] - p1[0]);
        let currentDist = spacingPx - (accumulatedDist % spacingPx);

        while (currentDist < segLen) {
            const t = currentDist / segLen;
            const cx = p1[0] + (p2[0] - p1[0]) * t;
            const cy = p1[1] + (p2[1] - p1[1]) * t;

            ctx.save();
            ctx.translate(cx, cy);
            ctx.rotate(angle);
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.moveTo(-5, -4);
            ctx.lineTo(4, 0);
            ctx.lineTo(-5, 4);
            ctx.closePath();
            ctx.fill();
            ctx.restore();

            currentDist += spacingPx;
        }
        accumulatedDist += segLen;
    }
}

function drawMap() {
    const dpr = window.devicePixelRatio || 1;
    const W = mapCanvas.clientWidth, H = mapCanvas.clientHeight;
    if (!W || !H) return;
    if (mapCanvas.width !== Math.round(W * dpr) || mapCanvas.height !== Math.round(H * dpr)) {
        mapCanvas.width = Math.round(W * dpr); mapCanvas.height = Math.round(H * dpr);
    }
    const ctx = mapCanvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = theme['--map-bg']; ctx.fillRect(0, 0, W, H);

    if (follow && robot) { view.vx = robot.x; view.vy = robot.y; }

    if (latestMap) {
        const m = latestMap, c = Math.cos(m.yaw), s = Math.sin(m.yaw), k = m.res * view.s;
        ctx.save();
        ctx.imageSmoothingEnabled = false;
        ctx.transform(-k * s, -k * c, -k * c, k * s,
            W / 2 - (m.oy - view.vy) * view.s, H / 2 - (m.ox - view.vx) * view.s);
        ctx.drawImage(mapCanvasOff, 0, 0);
        ctx.restore();
    }

    const step = view.s >= 6 ? 1 : (view.s >= 1.5 ? 5 : 10);
    document.getElementById('map-grid-label').textContent = 'Grid ' + step + ' m';
    const a = s2w(0, 0), b = s2w(W, H);
    const xLo = Math.min(a.x, b.x), xHi = Math.max(a.x, b.x), yLo = Math.min(a.y, b.y), yHi = Math.max(a.y, b.y);
    ctx.strokeStyle = theme['--map-grid']; ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = Math.ceil(xLo / step) * step; x <= xHi; x += step) {
        const sy = Math.round(w2s(x, 0)[1]) + 0.5; ctx.moveTo(0, sy); ctx.lineTo(W, sy);
    }
    for (let y = Math.ceil(yLo / step) * step; y <= yHi; y += step) {
        const sx = Math.round(w2s(0, y)[0]) + 0.5; ctx.moveTo(sx, 0); ctx.lineTo(sx, H);
    }
    ctx.stroke();

    const o = w2s(0, 0), ax = w2s(1, 0), ay = w2s(0, 1);
    ctx.lineWidth = 2;
    ctx.strokeStyle = '#e5484d'; ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(ax[0], ax[1]); ctx.stroke();
    ctx.strokeStyle = '#30a46c'; ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(ay[0], ay[1]); ctx.stroke();

    if (latestPlan && latestPlan.length > 0) {
        drawPathWithArrows(ctx, latestPlan, theme['--primary']);
        const finalPt = latestPlan[latestPlan.length - 1];
        drawArrow(ctx, finalPt.x, finalPt.y, finalPt.yaw, theme['--danger'], 'GOAL', 28);
    }

    const WPCOL = '#E8A317', TCOL = '#0077ff';
    if (waypointsData.length > 1) {
        ctx.strokeStyle = WPCOL; ctx.lineWidth = 1.2; ctx.setLineDash([4, 4]); ctx.beginPath();
        waypointsData.forEach((wp, i) => { const p = w2s(wp.x, wp.y); i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]); });
        ctx.stroke(); ctx.setLineDash([]);
    }
    waypointsData.forEach((wp, i) => drawArrow(ctx, wp.x, wp.y, wp.yaw * Math.PI / 180, WPCOL, String(i + 1), 26));
    if (singleTarget) drawArrow(ctx, singleTarget.x, singleTarget.y, singleTarget.yaw * Math.PI / 180, TCOL, 'T', 26);
    if (activeGoal) drawArrow(ctx, activeGoal.x, activeGoal.y, activeGoal.yaw * Math.PI / 180,
        clickMode === 'single' ? TCOL : WPCOL, clickMode === 'single' ? 'T' : String(waypointsData.length + 1), 26);

    if (robot) {
        const p = w2s(robot.x, robot.y), q = w2s(robot.x + Math.cos(robot.yaw), robot.y + Math.sin(robot.yaw));
        const ang = Math.atan2(q[1] - p[1], q[0] - p[0]);
        const r = Math.max(8, Math.min(16, 0.3 * view.s));
        const col = theme['--success'];
        
        ctx.fillStyle = col; ctx.globalAlpha = 0.18;
        ctx.beginPath(); ctx.arc(p[0], p[1], r * 2, 0, 7); ctx.fill(); ctx.globalAlpha = 1;
        
        ctx.save(); ctx.translate(p[0], p[1]); ctx.rotate(ang);
        ctx.fillStyle = col; ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.moveTo(r * 1.6, 0); ctx.lineTo(-r * 0.9, -r * 0.95); ctx.lineTo(-r * 0.4, 0); ctx.lineTo(-r * 0.9, r * 0.95); ctx.closePath();
        ctx.fill(); ctx.stroke();
        
        if (Math.abs(currentCmdVel.linear) > 0.02) {
            const linLen = currentCmdVel.linear * 35;
            const dir = Math.sign(currentCmdVel.linear);
            ctx.strokeStyle = '#38bdf8';
            ctx.fillStyle = '#38bdf8';
            ctx.lineWidth = 2.5;
            ctx.beginPath();
            ctx.moveTo(r * 1.8, 0);
            ctx.lineTo(r * 1.8 + linLen, 0);
            ctx.stroke();
            
            const headX = r * 1.8 + linLen;
            ctx.beginPath();
            ctx.moveTo(headX + dir * 6, 0);
            ctx.lineTo(headX, -5);
            ctx.lineTo(headX, 5);
            ctx.fill();
        }

        if (Math.abs(currentCmdVel.angular) > 0.05) {
            const arcR = r * 2.5;
            ctx.strokeStyle = '#facc15';
            ctx.fillStyle = '#facc15';
            ctx.lineWidth = 2.5;
            
            const dir = Math.sign(currentCmdVel.angular);
            const sweepAng = Math.min(Math.max(Math.abs(currentCmdVel.angular) * 0.5, 0.3), Math.PI / 1.2); 
            
            ctx.beginPath();
            ctx.arc(0, 0, arcR, 0, -dir * sweepAng, dir > 0);
            ctx.stroke();
            
            const endAngle = -dir * sweepAng;
            const ax = arcR * Math.cos(endAngle);
            const ay = arcR * Math.sin(endAngle);
            
            ctx.save();
            ctx.translate(ax, ay);
            ctx.rotate(endAngle - (dir * Math.PI / 2));
            ctx.beginPath();
            ctx.moveTo(0, 0);
            ctx.lineTo(-5, -3);
            ctx.lineTo(-5, 3);
            ctx.closePath();
            ctx.fill();
            ctx.restore();
        }

        ctx.restore();
    }
}

function evtPos(evt) { const r = mapCanvas.getBoundingClientRect(); return [evt.clientX - r.left, evt.clientY - r.top]; }

let poseActionPopup = null;

function openPoseActionPopup(pose, evt) {
    const popup = document.getElementById('map-pose-popup');
    const poseText = document.getElementById('map-pose-popup-pose');
    if (!popup || !pose) return;

    poseActionPopup = { x: pose.x, y: pose.y, yaw: pose.yaw || 0 };

    poseText.textContent =
        'X ' + pose.x.toFixed(2) +
        ' | Y ' + pose.y.toFixed(2) +
        ' | Yaw ' + (pose.yaw || 0).toFixed(1) + '°';

    const margin = 10;
    const pw = 230;
    const ph = 175;
    let left = evt.clientX + margin;
    let top = evt.clientY + margin;

    if (left + pw > window.innerWidth - margin) left = evt.clientX - pw - margin;
    if (top + ph > window.innerHeight - margin) top = evt.clientY - ph - margin;

    popup.style.left = Math.max(margin, left) + 'px';
    popup.style.top = Math.max(margin, top) + 'px';
    popup.classList.add('show');
}

function closePoseActionPopup() {
    const popup = document.getElementById('map-pose-popup');
    if (popup) popup.classList.remove('show');
    poseActionPopup = null;
}

async function popupNavigateToPoint() {
    if (!poseActionPopup) return;
    const p = { ...poseActionPopup };
    closePoseActionPopup();

    singleTarget = p;
    updateUIInputs(p.x, p.y, p.yaw);
    needsDraw = true;

    await sendGoalPose();
}

async function popupSetInitialCheckpoint() {
    if (!poseActionPopup) return;
    const p = { ...poseActionPopup };
    closePoseActionPopup();

    singleTarget = p;
    updateUIInputs(p.x, p.y, p.yaw);
    needsDraw = true;

    await sendInitialPose();
}

function popupAddWaypoint() {
    if (!poseActionPopup) return;
    const p = { ...poseActionPopup };
    closePoseActionPopup();

    waypointsData.push(p);
    updateWaypointsUI();
    setClickMode('waypoint');
    document.querySelectorAll('input[name="click_mode"]').forEach(i => i.checked = (i.value === 'waypoint'));
    needsDraw = true;
    notify('INFO', 'Waypoint ' + waypointsData.length + ' added at (' +
        p.x.toFixed(2) + ', ' + p.y.toFixed(2) + ').');
}

document.addEventListener('pointerdown', (evt) => {
    const popup = document.getElementById('map-pose-popup');
    if (popup && popup.classList.contains('show') && !popup.contains(evt.target) && !mapWrap.contains(evt.target)) {
        closePoseActionPopup();
    }
});

mapWrap.addEventListener('contextmenu', e => e.preventDefault());
mapWrap.addEventListener('pointerdown', (evt) => {
    if (evt.target.closest('#map-pose-popup')) return;
    mapWrap.setPointerCapture(evt.pointerId);
    const [sx, sy] = evtPos(evt);
    if (evt.button === 1 || evt.button === 2 || evt.shiftKey) {
        panState = { sx, sy, vx: view.vx, vy: view.vy };
        follow = false; document.getElementById('follow-btn').classList.remove('on');
        return;
    }
    const w = s2w(sx, sy);
    activeGoal = { x: w.x, y: w.y, yaw: 0 };
    needsDraw = true;
});
mapWrap.addEventListener('pointermove', (evt) => {
    if (evt.target.closest('#map-pose-popup')) return;
    const [sx, sy] = evtPos(evt);
    const w = s2w(sx, sy);
    document.getElementById('map-cursor').textContent = 'x ' + w.x.toFixed(2) + '  y ' + w.y.toFixed(2) + ' m';
    if (panState) {
        view.vy = panState.vy + (sx - panState.sx) / view.s;
        view.vx = panState.vx + (sy - panState.sy) / view.s;
        needsDraw = true;
    } else if (activeGoal) {
        const dx = w.x - activeGoal.x, dy = w.y - activeGoal.y;
        if (Math.abs(dx) > 0.05 || Math.abs(dy) > 0.05) activeGoal.yaw = Math.atan2(dy, dx) * 180 / Math.PI;
        needsDraw = true;
    }
});
const endPointer = (evt) => {
    if (evt.target.closest('#map-pose-popup')) return;
    if (panState) { panState = null; return; }
    if (!activeGoal) return;
    const g = activeGoal; activeGoal = null;

    if (clickMode === 'single') {
        singleTarget = g;
        updateUIInputs(g.x, g.y, g.yaw);
    } else {
        waypointsData.push(g);
        updateWaypointsUI();
    }

    needsDraw = true;
    openPoseActionPopup(g, evt);
};
mapWrap.addEventListener('pointerup', endPointer);

document.addEventListener('keydown', (evt) => {
    if (evt.key === 'Escape') closePoseActionPopup();
});
mapWrap.addEventListener('pointercancel', () => { panState = null; activeGoal = null; needsDraw = true; });
mapWrap.addEventListener('wheel', (evt) => {
    evt.preventDefault();
    const [sx, sy] = evtPos(evt), { W, H } = mapSize();
    const w = s2w(sx, sy);
    view.s = Math.max(1, Math.min(500, view.s * (evt.deltaY < 0 ? 1.12 : 1 / 1.12)));
    view.vy = w.y + (sx - W / 2) / view.s;
    view.vx = w.x + (sy - H / 2) / view.s;
    needsDraw = true;
}, { passive: false });
new ResizeObserver(() => { needsDraw = true; }).observe(mapWrap);

let lastStatus = 0;
function frame(ts) {
    robot = resolveRobotPose();
    if (mapImageDirty && latestMap) {
        rebuildMapImage(); mapImageDirty = false;
        if (!viewFitted) fitMapView();
        needsDraw = true;
    }
    if (follow || needsDraw) { drawMap(); needsDraw = false; }
    if (ts - lastStatus > 200) { lastStatus = ts; updateStatus(); }
    requestAnimationFrame(frame);
}

function calculateDistanceRemaining() {
    if (!latestPlan || latestPlan.length < 2) return 0;
    let dist = 0;
    for (let i = 0; i < latestPlan.length - 1; i++) {
        dist += Math.hypot(latestPlan[i+1].x - latestPlan[i].x, latestPlan[i+1].y - latestPlan[i].y);
    }
    return dist;
}

function updateStatus() {
    if (lastCamFrame && performance.now() - lastCamFrame > 3000) {
        const e = document.getElementById('cam-empty');
        e.textContent = 'No signal from /camera/image_raw'; e.style.display = 'flex';
    }
    const spd = odom ? odom.speed : 0;
    document.getElementById('stat-speed').textContent = spd.toFixed(2);

    distanceRemaining = calculateDistanceRemaining();
    document.getElementById('stat-distance').textContent = latestPlan.length > 0 ? distanceRemaining.toFixed(2) : '—';

    if (navStatus === 'Navigating' && latestPlan.length > 0 && distanceRemaining < 0.25) {
        navStatus = 'Reached';
        notify('INFO', 'Navigation destination reached successfully.');
        latestPlan = [];
    }

    const statusEl = document.getElementById('stat-nav-status');
    statusEl.textContent = navStatus;
    if (navStatus === 'Reached') statusEl.style.color = 'var(--success)';
    else if (navStatus === 'Aborted') statusEl.style.color = 'var(--danger)';
    else if (navStatus === 'Navigating') statusEl.style.color = 'var(--primary)';
    else statusEl.style.color = 'var(--muted)';

    if (robot) {
        let deg = robot.yaw * 180 / Math.PI;
        deg = ((deg + 180) % 360 + 360) % 360 - 180;
        document.getElementById('stat-heading').textContent = deg.toFixed(1);
        document.getElementById('hdg-arrow').style.transform = 'rotate(' + (-deg) + 'deg)';
        document.getElementById('stat-location').innerHTML =
            'X ' + robot.x.toFixed(2) + ' &nbsp; Y ' + robot.y.toFixed(2);
    }
}

function setClickMode(mode) {
    clickMode = mode;
    document.getElementById('single-actions').style.display = mode === 'single' ? 'flex' : 'none';
    document.getElementById('waypoint-actions').style.display = mode === 'waypoint' ? 'flex' : 'none';
    document.getElementById('waypoints-container').style.display = mode === 'waypoint' ? 'block' : 'none';
    document.querySelectorAll('input[name="click_mode"]').forEach(i => i.checked = (i.value === mode));
}
function updateUIInputs(x, y, yaw) {
    document.getElementById('target-x').value = x.toFixed(2);
    document.getElementById('target-y').value = y.toFixed(2);
    document.getElementById('target-yaw').value = yaw.toFixed(2);
    document.getElementById('lbl-x').textContent = x.toFixed(2);
    document.getElementById('lbl-y').textContent = y.toFixed(2);
    document.getElementById('lbl-yaw').textContent = yaw.toFixed(2);
}
function updateSingleFromInputs() {
    const x = parseFloat(document.getElementById('target-x').value) || 0;
    const y = parseFloat(document.getElementById('target-y').value) || 0;
    const yaw = parseFloat(document.getElementById('target-yaw').value) || 0;
    document.getElementById('lbl-x').textContent = x.toFixed(2);
    document.getElementById('lbl-y').textContent = y.toFixed(2);
    document.getElementById('lbl-yaw').textContent = yaw.toFixed(2);
    singleTarget = { x, y, yaw }; needsDraw = true;
}
function updateWaypointsUI() {
    const container = document.getElementById('waypoints-container');
    container.innerHTML = waypointsData.length === 0 ? '<em>No waypoints added.</em>' : '';
    waypointsData.forEach((wp, i) => {
        const div = document.createElement('div');
        div.innerHTML = '<strong>WP ' + (i + 1) + ':</strong> X:' + wp.x.toFixed(2) + ' Y:' + wp.y.toFixed(2) + ' Yaw:' + wp.yaw.toFixed(0) + '&deg;';
        container.appendChild(div);
    });
}
function clearMarkers() {
    closePoseActionPopup();
    singleTarget = null; waypointsData = []; activeGoal = null; latestPlan = [];
    navStatus = 'Idle';
    updateWaypointsUI(); needsDraw = true;
}

function yawDegToQuaternion(yawDeg) {
    const r = (yawDeg * Math.PI) / 180;
    return { yaw_z: Math.sin(r / 2), yaw_w: Math.cos(r / 2) };
}
async function postJSON(url, body) {
    const r = await fetch(url, {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: body === undefined ? undefined : JSON.stringify(body)
    });
    if (!r.ok) throw new Error('HTTP ' + r.status);
    return r;
}
async function withBusy(btn, fn) {
    if (btn) { if (btn.classList.contains('loading')) return; btn.classList.add('loading'); btn.disabled = true; }
    try { return await fn(); }
    finally { if (btn) { btn.classList.remove('loading'); btn.disabled = false; } }
}
function run(btn, fn) { return withBusy(btn, fn); }

function readTarget() {
    return {
        x: parseFloat(document.getElementById('target-x').value),
        y: parseFloat(document.getElementById('target-y').value),
        yawDeg: parseFloat(document.getElementById('target-yaw').value) || 0
    };
}
async function sendGoalPose() {
    const btn = document.getElementById('nav-pose-btn');
    setNavLoading(btn);
    startNavWatchdog();
    const t = readTarget();
    try {
        await postJSON('/navigate_to_pose', { x: t.x, y: t.y, yaw_deg: t.yawDeg });
        navStatus = 'Navigating';
        notify('INFO', 'Navigation goal sent to (' + t.x.toFixed(2) + ', ' + t.y.toFixed(2) + ').');
    } catch (e) {
        clearNavLoading();
        notify('ERROR', 'Failed to send navigation goal: ' + e.message);
    }
}
async function sendInitialPose() {
    const t = readTarget();
    try {
        await postJSON('/set_initial_pose', { x: t.x, y: t.y, yaw_deg: t.yawDeg });
        notify('INFO', 'Initial pose set to (' + t.x.toFixed(2) + ', ' + t.y.toFixed(2) + ').');
    } catch (e) { notify('ERROR', 'Failed to set initial pose: ' + e.message); }
}
async function sendWaypoints() {
    if (waypointsData.length === 0) return notify('WARNING', 'No waypoints added. Click the map to add some.');
    const btn = document.getElementById('follow-wp-btn');
    setNavLoading(btn);
    startNavWatchdog();
    const waypoints = waypointsData.map(wp => ({ x: wp.x, y: wp.y, yaw_deg: wp.yaw }));
    try {
        await postJSON('/follow_waypoints', { waypoints });
        navStatus = 'Navigating';
        notify('INFO', 'Following ' + waypoints.length + ' waypoint(s).');
    } catch (e) {
        clearNavLoading();
        notify('ERROR', 'Failed to send waypoints: ' + e.message);
    }
}
async function sendAbort() {
    try {
        await postJSON('/abort');
        navStatus = 'Aborted';
        latestPlan = [];
        clearNavLoading();
        notify('WARNING', 'Mission abort requested.');
    }
    catch (e) { notify('ERROR', 'Failed to abort mission: ' + e.message); }
}
async function saveCurrentMap() {
    const mapName = document.getElementById('map-name-input').value;
    if (!mapName) return notify('WARNING', 'Please enter a map name to save.');
    try { await postJSON('/system/save_map', { map_name: mapName }); notify('INFO', 'Saving "' + mapName + '" map + SLAM continuation data…'); }
    catch (e) { notify('ERROR', 'Failed to save map: ' + e.message); }
}

let currentMode = null, pending = null, modeSelectTouched = false, modeTimer = null, polling = false;

function normalizeMode(raw) {
    if (raw === undefined || raw === null) return null;
    const s = String(raw).toLowerCase().replace(/[^a-z]+/g, '_').replace(/^_|_$/g, '');
    if (['slam_update', 'slam_update_nav', 'continued_mapping', 'lifelong_mapping', 'lifelong'].includes(s)) return 'slam_update';
    if (['slam_nav', 'slam_with_nav', 'slam_navigation', 'slam_and_nav', 'slam_nav2'].includes(s)) return 'slam_nav';
    if (['slam', 'slam_only', 'mapping', 'mapping_only'].includes(s)) return 'slam';
    if (['nav', 'nav_only', 'navigation', 'navigation_only', 'amcl', 'localization'].includes(s)) return 'nav';
    if (['manual', 'teleop', 'joystick'].includes(s)) return 'manual';
    return null;
}

async function fetchMode() {
    const ctl = new AbortController();
    const to = setTimeout(() => ctl.abort(), 4000);
    try {
        const r = await fetch(CFG.modeUrl, { cache: 'no-store', signal: ctl.signal });
        if (!r.ok) return undefined;
        const text = await r.text();
        let raw = text;
        try {
            const j = JSON.parse(text);
            raw = (j && typeof j === 'object') ? (j.mode ?? j.current_mode ?? j.system_mode) : j;
        } catch (e) {}
        return normalizeMode(raw);
    } catch (e) { return undefined; }
    finally { clearTimeout(to); }
}

function renderMode() {
    const chip = document.getElementById('mode-chip'), txt = document.getElementById('stat-mode');
    const btn = document.getElementById('deploy-btn');
    chip.classList.remove('known', 'pending');
    if (pending) {
        chip.classList.add('pending');
        txt.textContent = 'Switching to ' + MODE_LABELS[pending.target] + '…';
    } else if (currentMode) {
        chip.classList.add('known');
        txt.textContent = MODE_LABELS[currentMode];
    } else {
        txt.textContent = 'Unknown';
    }
    btn.disabled = !!pending;
    btn.classList.toggle('loading', !!pending);
}

function setCurrentMode(m) {
    currentMode = m;
    if (m && !modeSelectTouched && !pending) document.getElementById('sys-mode-select').value = m;
    renderMode();
}

function clearPending() {
    if (pending) clearTimeout(pending.timer);
    pending = null; renderMode();
    scheduleModePoll();
}

async function pollMode() {
    if (polling) return;
    polling = true;
    try {
        const m = await fetchMode();
        if (m !== undefined) {
            const prev = currentMode;
            setCurrentMode(m);
            if (pending && m === pending.target) {
                const label = MODE_LABELS[m]; clearPending();
                notify('INFO', 'Mode changed to ' + label + '.');
            } else if (!pending && prev && m && prev !== m) {
                notify('INFO', 'Mode is now ' + MODE_LABELS[m] + '.');
            }
        }
    } finally { polling = false; }
}

function scheduleModePoll() {
    clearTimeout(modeTimer);
    modeTimer = setTimeout(async () => { await pollMode(); scheduleModePoll(); },
        pending ? CFG.modePollMs : CFG.modeIdlePollMs);
}

async function applySystemMode() {
    if (pending) return;
    const mode = document.getElementById('sys-mode-select').value;
    const mapName = document.getElementById('map-name-input').value || 'small_warehouse';
    if (mode === currentMode) return notify('INFO', 'Already in ' + MODE_LABELS[mode] + ' mode.');

    const p = { target: mode, timer: null };
    p.timer = setTimeout(async () => {
        if (pending !== p) return;
        const m = await fetchMode();
        if (pending !== p) return;
        if (m !== undefined) setCurrentMode(m);
        const seconds = CFG.modeTimeoutMs / 1000;
        if (m === p.target) { clearPending(); notify('INFO', 'Mode changed to ' + MODE_LABELS[mode] + '.'); }
        else {
            clearPending();
            notify('ERROR', 'Mode change to ' + MODE_LABELS[mode] + ' timed out after ' + seconds + ' s. Current mode: ' +
                (currentMode ? MODE_LABELS[currentMode] : 'unknown') + '.');
        }
    }, CFG.modeTimeoutMs);
    pending = p; renderMode();
    notify('INFO', 'Switching to ' + MODE_LABELS[mode] + '… waiting up to ' + (CFG.modeTimeoutMs / 1000) + ' s for confirmation.');
    scheduleModePoll();

    try { await postJSON(CFG.modeUrl, { mode, map_name: mapName }); }
    catch (e) {
        if (pending === p) { clearPending(); notify('ERROR', 'Mode request failed: ' + e.message); }
    }
}

const joyPad = document.getElementById('joy-pad'), joyKnob = document.getElementById('joy-knob');
const joy = { x: 0, y: 0, active: false, timer: null };

function publishJoy() {
    if (!joyTopic || rosState !== 'up') return;
    const ms = Date.now();
    joyTopic.publish(new ROSLIB.Message({
        header: { stamp: { sec: Math.floor(ms / 1000), nanosec: (ms % 1000) * 1e6 }, frame_id: 'joy' },
        axes: [-joy.x, -joy.y, 0, 0, 0, 0, 0, 0],
        buttons: new Array(12).fill(0)
    }));
}
function joyShow() {
    joyKnob.style.transform = 'translate(' + (joy.x * joyR()) + 'px,' + (joy.y * joyR()) + 'px)';
    document.getElementById('joy-x').textContent = (-joy.x).toFixed(2);
    document.getElementById('joy-y').textContent = (-joy.y).toFixed(2);
}
function joyR() { return joyPad.clientWidth / 2 - 26; }
function joyMove(evt) {
    const r = joyPad.getBoundingClientRect(), R = joyR();
    let dx = evt.clientX - (r.left + r.width / 2), dy = evt.clientY - (r.top + r.height / 2);
    const mag = Math.hypot(dx, dy);
    if (mag > R) { dx *= R / mag; dy *= R / mag; }
    joy.x = dx / R; joy.y = dy / R; joyShow();
}
joyPad.addEventListener('pointerdown', (e) => {
    joyPad.setPointerCapture(e.pointerId); joyPad.classList.add('active');
    joy.active = true; joyMove(e);
    clearInterval(joy.timer);
    joy.timer = setInterval(publishJoy, 1000 / CFG.joyRateHz);
    publishJoy();
});
joyPad.addEventListener('pointermove', (e) => { if (joy.active) joyMove(e); });
const joyEnd = () => {
    if (!joy.active) return;
    joy.active = false; joyPad.classList.remove('active');
    clearInterval(joy.timer);
    joy.x = 0; joy.y = 0; joyShow();
    publishJoy(); setTimeout(publishJoy, 50); setTimeout(publishJoy, 100);
};
joyPad.addEventListener('pointerup', joyEnd);
joyPad.addEventListener('pointercancel', joyEnd);

(function init() {
    let saved = null;
    try { saved = localStorage.getItem('milusions-theme'); } catch (e) {}
    applyTheme(saved !== 'dark');
    renderMode();
    pollMode().then(scheduleModePoll);
    requestAnimationFrame(frame);
})();

// ---- Web Audio API - Emo Sound FX ----
let audioCtx = null;

function initAudio() {
    if (!audioCtx) {
        audioCtx = new (window.AudioContext || window.webkitAudioContext)();
    }
    if (audioCtx.state === 'suspended') {
        audioCtx.resume();
    }
}

function playStartupSound() {
    try {
        initAudio();
        const now = audioCtx.currentTime;
        const osc = audioCtx.createOscillator();
        const gain = audioCtx.createGain();
        osc.type = 'sine';
        osc.frequency.setValueAtTime(440, now);
        osc.frequency.exponentialRampToValueAtTime(880, now + 0.15);
        gain.gain.setValueAtTime(0.45, now);
        gain.gain.exponentialRampToValueAtTime(0.001, now + 0.35);
        osc.connect(gain);
        gain.connect(audioCtx.destination);
        osc.start(now);
        osc.stop(now + 0.35);
    } catch (e) { console.log(e); }
}

function playListenSound() {
    try {
        initAudio();
        const now = audioCtx.currentTime;
        const osc = audioCtx.createOscillator();
        const gain = audioCtx.createGain();
        osc.type = 'sine';
        osc.frequency.setValueAtTime(587.33, now);
        osc.frequency.setValueAtTime(880, now + 0.08);
        gain.gain.setValueAtTime(0.12, now);
        gain.gain.exponentialRampToValueAtTime(0.001, now + 0.2);
        osc.connect(gain);
        gain.connect(audioCtx.destination);
        osc.start(now);
        osc.stop(now + 0.2);
    } catch (e) { console.log(e); }
}

function playProcessingSound() {
    try {
        initAudio();
        const now = audioCtx.currentTime;
        const osc = audioCtx.createOscillator();
        const gain = audioCtx.createGain();
        osc.type = 'triangle';
        osc.frequency.setValueAtTime(320, now);
        osc.frequency.linearRampToValueAtTime(160, now + 0.1);
        gain.gain.setValueAtTime(0.1, now);
        gain.gain.exponentialRampToValueAtTime(0.001, now + 0.12);
        osc.connect(gain);
        gain.connect(audioCtx.destination);
        osc.start(now);
        osc.stop(now + 0.12);
    } catch (e) { console.log(e); }
}

const SpeechRecognitionImpl = window.SpeechRecognition || window.webkitSpeechRecognition;
let recognition = null;
let inactivityTimer = null;
const LISTEN_TIMEOUT_MS = 8000;      // nobody says anything -> go to sleep
let listenDeadline = 0, heardSpeech = false;
let currentCallId = null; 

const modal = document.getElementById('voice-modal');
const robotFace = document.getElementById('robot-face');
const captionStatus = document.getElementById('caption-status');
const captionText = document.getElementById('caption-text');
const historyListEl = document.getElementById('history-list');

let agentState = 'idle';
let thinkingTimer = null;
let thinkingStageIndex = 0;
window.currentUtterance = null;

const thinkingPanel = document.getElementById('thinking-panel');
const thinkingStageText = document.getElementById('thinking-stage-text');

const THINKING_STAGES = ['Analyzing prompt', 'Thinking...', 'Accessing rover', 'Finalizing'];

function startThinkingIndicator() {
    stopThinkingIndicator();
    if (!thinkingPanel) return;
    thinkingPanel.classList.add('active');
    thinkingStageIndex = 0;
    const update = () => {
        if (thinkingStageText) thinkingStageText.textContent = THINKING_STAGES[Math.min(thinkingStageIndex, THINKING_STAGES.length - 1)];
        thinkingStageIndex++;
        if (thinkingStageIndex >= THINKING_STAGES.length && thinkingTimer) { clearInterval(thinkingTimer); thinkingTimer = null; }
    };
    update();
    thinkingTimer = setInterval(update, 1600);
}

function stopThinkingIndicator() {
    if (thinkingTimer) { clearInterval(thinkingTimer); thinkingTimer = null; }
    if (thinkingPanel) thinkingPanel.classList.remove('active');
}

function appendHistory(sender, text) {
    const item = document.createElement('div');
    item.className = `history-item ${sender}`;
    
    const label = document.createElement('div');
    label.className = 'history-label';
    label.textContent = sender === 'you' ? 'you:' : 'agent:';
    
    const content = document.createElement('div');
    if (typeof marked !== 'undefined') {
        content.innerHTML = marked.parse(text);
    } else {
        content.textContent = text;
    }
    
    item.appendChild(label);
    item.appendChild(content);
    historyListEl.appendChild(item);
    historyListEl.scrollTop = historyListEl.scrollHeight;
}

const SPEECH_LANG = (navigator.language || '').toLowerCase().startsWith('en') ? navigator.language : 'en-US';
const SILENCE_COMMIT_MS = 1200;   // no new words for this long after speaking -> send what we have
const MAX_UTTERANCE_MS = 30000;   // hard cap on one listening session
let lastText = '', lastResultAt = 0, listenStartedAt = 0, committed = false, silenceTimer = null;

function clearListenTimers() {
    if (inactivityTimer) { clearTimeout(inactivityTimer); inactivityTimer = null; }
    if (silenceTimer) { clearTimeout(silenceTimer); silenceTimer = null; }
}

function startRecognition(retry = true) {
    if (!recognition) return;
    stopWake();
    try { recognition.start(); }
    catch (e) {
        if (retry) setTimeout(() => {
            if (modal.classList.contains('active') && (agentState === 'listening' || agentState === 'waking')) startRecognition(false);
        }, 250);
    }
}

function commitUtterance(text) {
    text = (text || '').replace(/\s+/g, ' ').trim();
    if (committed || agentState !== 'listening') return;
    if (text.length < 2) return;
    committed = true;
    clearListenTimers();
    captionText.textContent = text;
    appendHistory('you', text);
    processVoiceCommand(text);
}

// pick the most confident alternative of a result
function bestAlt(res) {
    let best = res[0];
    for (let j = 1; j < res.length; j++) if (res[j].confidence > best.confidence) best = res[j];
    return best.transcript;
}

if (SpeechRecognitionImpl) {
    recognition = new SpeechRecognitionImpl();
    recognition.continuous = false;
    recognition.interimResults = true;
    recognition.maxAlternatives = 3;
    recognition.lang = SPEECH_LANG;

    recognition.onstart = () => {
        const restarting = !heardSpeech && listenDeadline && Date.now() < listenDeadline;
        agentState = 'listening';
        committed = false;
        if (!restarting) {
            heardSpeech = false; lastText = '';
            listenStartedAt = Date.now();
            listenDeadline = Date.now() + LISTEN_TIMEOUT_MS;
            setRobotMood('listening', 'Listening', 'Speak your command...');
            playListenSound();
        }
        clearListenTimers();
        inactivityTimer = setTimeout(() => {
            if (agentState === 'listening' && !heardSpeech) goToSleep();
        }, Math.max(0, listenDeadline - Date.now()));
    };

    recognition.onerror = (e) => {
        if (e.error === 'aborted' || e.error === 'no-speech') return;   // onend decides what to do
        if (agentState !== 'listening') return;
        if (heardSpeech && lastText && !committed) { commitUtterance(lastText); return; }
        if (e.error === 'not-allowed' || e.error === 'service-not-allowed' || e.error === 'audio-capture') {
            setRobotMood('listening', 'Error', `Microphone problem: ${e.error}`);
            setTimeout(closeVoiceModal, 3000);
        }
        // network / other errors: onend restarts until the listen deadline, then sleeps
    };

    recognition.onresult = (event) => {
        if (agentState !== 'listening' || committed) return;
        let full = '', allFinal = true;
        for (let i = 0; i < event.results.length; i++) {
            full += bestAlt(event.results[i]) + ' ';
            if (!event.results[i].isFinal) allFinal = false;
        }
        full = full.replace(/\s+/g, ' ').trim();
        if (!full) return;

        heardSpeech = true; lastText = full; lastResultAt = Date.now();
        clearListenTimers();
        captionText.textContent = full;

        if (allFinal) commitUtterance(full);
        else silenceTimer = setTimeout(() => commitUtterance(lastText), SILENCE_COMMIT_MS);   // silence detected -> send
    };

    recognition.onend = () => {
        if (agentState !== 'listening' || committed) return;
        if (heardSpeech && lastText) { commitUtterance(lastText); return; }   // never leave words hanging
        if (Date.now() < listenDeadline - 300) startRecognition();
        else goToSleep();
    };

    // Watchdog: nothing may leave the assistant stuck in "listening"
    setInterval(() => {
        if (agentState !== 'listening' || committed) return;
        const now = Date.now();
        if (heardSpeech && lastText && now - lastResultAt > SILENCE_COMMIT_MS + 700) commitUtterance(lastText);
        else if (!heardSpeech && listenDeadline && now > listenDeadline + 600) goToSleep();
        else if (listenStartedAt && now - listenStartedAt > MAX_UTTERANCE_MS) {
            if (lastText) commitUtterance(lastText); else goToSleep();
        }
    }, 400);
} else {
    console.warn('Speech Recognition not supported in this browser.');
}

function setRobotMood(className, statusMsg, captionMsg) {
    robotFace.className = 'robot-face ' + className;
    if (statusMsg !== null) captionStatus.textContent = statusMsg;
    if (captionMsg !== null) {
        captionText.innerHTML = (typeof marked !== 'undefined' && captionMsg.length > 20) ? marked.parse(captionMsg) : captionMsg;
    }
}

function openVoiceModal() {
    if (!recognition) return alert("Voice not supported on this browser.");
    if (modal.classList.contains('active') && agentState === 'asleep') return wakeFromSleep();
    stopWake();
    modal.classList.add('active');
    playStartupSound();

    const bubble = document.getElementById('speech-bubble');
    const greeting = 'Hello! How can I help?';
    if (bubble) {
        bubble.textContent = greeting;
        bubble.classList.add('show');
        setTimeout(() => bubble.classList.remove('show'), 3200);
    }

    if (!currentCallId) {
        currentCallId = String(Math.floor(Math.random() * 900) + 100);
    }

    agentState = 'speaking';
    setRobotMood('speaking', 'Helio', greeting);

    if ('speechSynthesis' in window) {
        window.speechSynthesis.cancel();
        const greetUtterance = new SpeechSynthesisUtterance(greeting);
        greetUtterance.lang = 'en-US';
        greetUtterance.rate = 1.0;
        let listenStarted = false, greetSpoke = false;
        const startListening = () => {
            if (listenStarted) return;
            listenStarted = true;
            if (modal.classList.contains('active') && agentState === 'speaking') {
                agentState = 'listening';
                startRecognition();
            }
        };
        greetUtterance.onstart = () => { greetSpoke = true; };
        greetUtterance.onend = startListening;
        greetUtterance.onerror = startListening;
        setTimeout(() => { if (!greetSpoke) startListening(); }, 1500);
        window.currentUtterance = greetUtterance;
        window.currentSpokenText = greeting;
        window.speechSynthesis.speak(greetUtterance);
        syncWake();   // wake word can interrupt the greeting
    } else {
        agentState = 'listening';
        startRecognition();
    }
}

function closeVoiceModal() {
    agentState = 'idle';
    listenDeadline = 0;
    clearListenTimers();
    modal.classList.remove('active');
    
    currentCallId = null;
    historyListEl.innerHTML = '';
    const bubble = document.getElementById('speech-bubble');
    if (bubble) bubble.classList.remove('show');

    if (recognition) {
        try { recognition.stop(); } catch(e){}
    }
    if ('speechSynthesis' in window) window.speechSynthesis.cancel();
    stopThinkingIndicator();
    const ui = document.getElementById('user-input');
    if (ui) ui.value = '';
    syncWake();
}

async function sendManualText() {
    const input = document.getElementById('user-input');
    const btn = document.getElementById('manual-send-btn');
    const text = (input && input.value ? input.value : '').trim();

    if (!text) {
        notify('WARNING', 'Enter a command before pressing SEND.');
        if (input) input.focus();
        return;
    }

    if (!modal.classList.contains('active')) {
        openVoiceModal();
    }

    if (!currentCallId) {
        currentCallId = String(Math.floor(Math.random() * 900) + 100);
    }

    appendHistory('you', text);
    if (input) input.value = '';

    if (btn) {
        btn.disabled = true;
        btn.textContent = 'SENDING…';
    }

    try {
        await processVoiceCommand(text);
    } finally {
        if (btn) {
            btn.disabled = false;
            btn.textContent = 'SEND';
        }
    }
}

async function processVoiceCommand(text) {
    agentState = 'thinking';
    listenDeadline = 0;
    clearListenTimers();
    try { recognition.stop(); } catch(e){}

    setRobotMood('thinking', 'Processing', text);
    startThinkingIndicator();
    playProcessingSound();

    try {
        const response = await fetch('http://' + ROVER_HOST + ':8001/agentic/waregv', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                call_id: currentCallId,
                message: `MAIN PROMPT: ${text}\nUSER INPUT FIELD: ${(document.getElementById('user-input').value || '').trim()}`
            })
        });

        if (!response.ok) throw new Error(`HTTP Error: ${response.status}`);

        const data = await response.json();
        stopThinkingIndicator();
        const agentResponseText = data.output || "I didn't receive a valid response.";
        
        appendHistory('agent', agentResponseText);
        speakAndLoop(agentResponseText);

    } catch (err) {
        stopThinkingIndicator();
        const friendlyMsg = "Sorry, I am having trouble connecting. Try again later.";
        appendHistory('agent', friendlyMsg);
        speakThenClose(friendlyMsg);
    }
}

function speakThenClose(text) {
    agentState = 'speaking';
    setRobotMood('speaking', 'Connection Error', text);

    if ('speechSynthesis' in window) {
        window.speechSynthesis.cancel();
        const utterance = new SpeechSynthesisUtterance(text);
        utterance.lang = 'en-US';
        utterance.rate = 1.0;
        utterance.onend = () => setTimeout(closeVoiceModal, 600);
        window.currentUtterance = utterance;
        window.currentSpokenText = text;
        window.speechSynthesis.speak(utterance);
    } else {
        setTimeout(closeVoiceModal, 3000);
    }
}

function speakAndLoop(text) {
    agentState = 'speaking';
    setRobotMood('speaking', 'Response', text);

    if ('speechSynthesis' in window) {
        window.speechSynthesis.cancel();
        const utterance = new SpeechSynthesisUtterance(text);
        utterance.lang = 'en-US';
        utterance.rate = 1.0;
        
        utterance.onend = () => {
            // ignore end events from utterances that were cancelled / interrupted
            if (window.currentUtterance !== utterance || agentState !== 'speaking') return;
            if (modal.classList.contains('active')) {
                agentState = 'listening';
                setRobotMood('listening', 'Listening', 'Listening for next command...');
                startRecognition();
            }
        };

        window.currentUtterance = utterance;
        window.currentSpokenText = text;
        window.speechSynthesis.speak(utterance);
        syncWake();   // say the wake word to interrupt
    } else {
        setTimeout(closeVoiceModal, 4000); 
    }
}

const manualInput = document.getElementById('user-input');
if (manualInput) {
    manualInput.addEventListener('keydown', (event) => {
        if (event.key === 'Enter' && !event.shiftKey) {
            event.preventDefault();
            sendManualText();
        }
    });
}

const WAKE_OPTIONS = {
    rover: ['rover', 'rova', 'rovar', 'ro ver'],
    helio: ['helio', 'heleo', 'healio', 'hilio', 'hello'],
    computer: ['computer'],
    car: ['car'],
    robot: ['robot']
};
let wakeWord = 'rover';
try { const w = localStorage.getItem('milusions-wake-word'); if (w && WAKE_OPTIONS[w]) wakeWord = w; } catch (e) {}
let WAKE_RE = null;
function buildWakeRe() { WAKE_RE = new RegExp('\\b(' + WAKE_OPTIONS[wakeWord].join('|') + ')\\b', 'i'); }
buildWakeRe();
let wakeRec = null, wakeRunning = false, wakeWanted = false, wakeBlocked = false;
let wakeEnabled = true;
try { wakeEnabled = localStorage.getItem('milusions-wake') !== 'off'; } catch (e) {}

function updateWakeBtn() {
    const btn = document.getElementById('wake-btn');
    if (!btn) return;
    const on = wakeEnabled && !wakeBlocked;
    btn.classList.toggle('on', on); btn.classList.toggle('off', !on);
    document.getElementById('wake-label').textContent = wakeWord;
    btn.title = 'Wake word "' + wakeWord + '": ' + (wakeBlocked ? 'blocked (microphone)' : (wakeEnabled ? 'on' : 'off'));
    renderWakeMenu();
}

function renderWakeMenu() {
    const m = document.getElementById('wake-menu');
    if (!m) return;
    m.innerHTML = '<div class="wm-title">Wake word</div>';
    Object.keys(WAKE_OPTIONS).forEach(w => {
        const b = document.createElement('button');
        b.setAttribute('role', 'menuitemradio');
        b.setAttribute('aria-checked', w === wakeWord ? 'true' : 'false');
        if (w === wakeWord) b.classList.add('sel');
        b.textContent = w;
        b.onclick = () => { setWakeWord(w); closeWakeMenu(); };
        m.appendChild(b);
    });
    m.appendChild(document.createElement('hr'));
    const t = document.createElement('button');
    t.className = 'wm-toggle' + (wakeEnabled ? ' on' : '');
    t.dataset.state = wakeEnabled ? 'ON' : 'OFF';
    t.textContent = 'Listen for wake word';
    t.onclick = () => { toggleWake(); };
    m.appendChild(t);
}

function setWakeWord(w) {
    if (!WAKE_OPTIONS[w] || w === wakeWord) return;
    wakeWord = w; buildWakeRe();
    try { localStorage.setItem('milusions-wake-word', w); } catch (e) {}
    updateWakeBtn();
    if (typeof modal !== 'undefined' && modal.classList.contains('active') && agentState === 'asleep') {
        captionText.textContent = 'Say "' + wakeWord + '" to wake me up';
    }
    notify('INFO', 'Wake word set to "' + wakeWord + '".');
}

function toggleWakeMenu(ev) {
    ev.stopPropagation();
    const m = document.getElementById('wake-menu'), btn = document.getElementById('wake-btn');
    const open = m.hidden;
    m.hidden = !open;
    btn.setAttribute('aria-expanded', open ? 'true' : 'false');
}
function closeWakeMenu() {
    const m = document.getElementById('wake-menu');
    if (m) m.hidden = true;
    const btn = document.getElementById('wake-btn');
    if (btn) btn.setAttribute('aria-expanded', 'false');
}
document.addEventListener('click', (e) => { if (!e.target.closest('.wake-menu-wrap')) closeWakeMenu(); });
document.addEventListener('keydown', (e) => { if (e.key === 'Escape') closeWakeMenu(); });

function syncWake() {
    const popupOpen = modal.classList.contains('active');
    const should = wakeEnabled && !wakeBlocked && SpeechRecognitionImpl && (!popupOpen || agentState === 'asleep' || agentState === 'speaking');
    if (should) startWake(); else stopWake();
}
function startWake() {
    wakeWanted = true;
    if (wakeRunning || !wakeRec) return;
    try { wakeRec.start(); } catch (e) {}
}
function stopWake() {
    wakeWanted = false;
    if (wakeRunning && wakeRec) { try { wakeRec.abort(); } catch (e) {} }
}
function toggleWake() {
    wakeEnabled = !wakeEnabled;
    if (wakeEnabled) wakeBlocked = false;
    try { localStorage.setItem('milusions-wake', wakeEnabled ? 'on' : 'off'); } catch (e) {}
    updateWakeBtn(); syncWake();
    notify('INFO', 'Wake word "' + wakeWord + '" ' + (wakeEnabled ? 'enabled.' : 'disabled.'));
}

if (SpeechRecognitionImpl) {
    wakeRec = new SpeechRecognitionImpl();
    wakeRec.continuous = true;
    wakeRec.interimResults = true;
    wakeRec.maxAlternatives = 5;
    wakeRec.lang = SPEECH_LANG;
    wakeRec.onstart = () => { wakeRunning = true; };
    wakeRec.onend = () => {
        wakeRunning = false;
        if (wakeWanted) setTimeout(() => { if (wakeWanted && !wakeRunning) { try { wakeRec.start(); } catch (e) {} } }, 400);
    };
    wakeRec.onerror = (e) => {
        if (e.error === 'not-allowed' || e.error === 'service-not-allowed') {
            wakeBlocked = true; wakeWanted = false; updateWakeBtn();
            notify('WARNING', 'Microphone access is blocked, so the wake word is disabled.');
        }
    };
    wakeRec.onresult = (ev) => {
        for (let i = ev.resultIndex; i < ev.results.length; i++) {
            const res = ev.results[i];
            for (let j = 0; j < res.length; j++) {
                const t = res[j].transcript;
                if (!WAKE_RE.test(t)) continue;
                // while Helio is talking, ignore the mic hearing Helio's own voice
                if (agentState === 'speaking' && isEchoOfSpeech(t)) continue;
                onWakeWord(); return;
            }
        }
    };
}

function isEchoOfSpeech(t) {
    const words = (t.toLowerCase().match(/[a-z']+/g) || []);
    if (words.length < 4) return false;
    const spoken = new Set((String(window.currentSpokenText || '').toLowerCase().match(/[a-z']+/g) || []));
    const hit = words.filter(w => spoken.has(w)).length;
    return hit / words.length >= 0.85;
}

function onWakeWord() {
    stopWake();
    if (modal.classList.contains('active')) {
        window.currentUtterance = null;   // so the cancelled utterance's onend does nothing
        if ('speechSynthesis' in window) window.speechSynthesis.cancel();
        stopThinkingIndicator();
        if (agentState === 'asleep') {
            wakeFromSleep();
        } else if (agentState === 'speaking' || agentState === 'thinking') {
            agentState = 'listening';
            listenDeadline = 0; heardSpeech = false; lastText = '';
            setRobotMood('listening', 'Listening', 'Interrupted. Listening...');
            playStartupSound();
            setTimeout(() => { if (agentState === 'listening') startRecognition(); }, 220);
        }
    } else {
        openVoiceModal();
    }
}

function goToSleep() {
    if (!modal.classList.contains('active') || agentState === 'asleep') return;
    agentState = 'asleep';
    listenDeadline = 0;
    clearListenTimers();
    try { recognition.stop(); } catch (e) {}
    if ('speechSynthesis' in window) window.speechSynthesis.cancel();
    setRobotMood('sleeping', 'Sleeping', 'Say "' + wakeWord + '" to wake me up');
    syncWake();
}

function wakeFromSleep() {
    stopWake();
    agentState = 'waking';
    setRobotMood('startled', 'Awake', 'Yes?');
    playStartupSound();
    const bubble = document.getElementById('speech-bubble');
    if (bubble) {
        bubble.textContent = 'Yes?';
        bubble.classList.add('show');
        setTimeout(() => bubble.classList.remove('show'), 2200);
    }
    setTimeout(() => {
        if (modal.classList.contains('active') && agentState === 'waking') {
            try { recognition.start(); } catch (e) { console.log(e); }
        }
    }, 750);
}

updateWakeBtn();
syncWake();

function stopTalking() {
    if ('speechSynthesis' in window) {
        window.speechSynthesis.cancel();
    }
    window.currentUtterance = null;
    stopThinkingIndicator();
    if (agentState === 'speaking' || agentState === 'thinking') {
        agentState = 'listening';
        listenDeadline = 0; heardSpeech = false; lastText = '';
        setRobotMood('listening', 'Listening', 'Stopped talking. Listening...');
        setTimeout(() => { if (agentState === 'listening') startRecognition(); }, 150);
    }
    notify('INFO', 'Announcer stopped.');
}