// Panda MJK Dashboard — Direct Joint & Gripper Control via rosbridge websocket
// Connects to ws://localhost:9090 by default

const ARM_JOINTS = ["fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4", "fer_joint5", "fer_joint6", "fer_joint7"];
const MONITOR_JOINTS = [...ARM_JOINTS, "finger_joint1", "finger_joint2"];

// franka_description's robots/fer/joint_limits.yaml position limits (rad).
const JOINT_LIMITS = {
  fer_joint1: [-2.8973, 2.8973],
  fer_joint2: [-1.7628, 1.7628],
  fer_joint3: [-2.8973, 2.8973],
  fer_joint4: [-3.0718, -0.0698],
  fer_joint5: [-2.8973, 2.8973],
  fer_joint6: [-0.0175, 3.7525],
  fer_joint7: [-2.8973, 2.8973],
};

const HOME_POSE = [0, -0.785398, 0, -2.356194, 0, 1.570796, 0.785398];
const ZERO_POSE = [0, 0, 0, 0, 0, 0, 0];

let ros = null;
let jointStateTopic = null;
let commandTopic = null;
let effortTopic = null;
let gripperAction = null;
let latestJointPositions = null; // ordered by ARM_JOINTS
let latestJointVelocities = null; // ordered by ARM_JOINTS
let impedanceInterval = null;
let lastJsStamp = 0;
let jsRateEma = null;
let liveSendLastAt = 0;
let lastDebugLogStamp = 0;

// ---- Generic Topic Monitoring & Live Plotting State ----
let allTopicsList = [];
let activeTopicSub = null;
let activeTopicName = null;
let activeTopicType = null;
let plotTopicSub = null;
let plotTopicName = null;
let plotTopicType = null;
let uplotInst = null;
let plotData = [[], []]; // [x], [y]
let plotStartTime = 0;

const $ = (id) => document.getElementById(id);

function log(msg, cls = "") {
  const el = $("log");
  if (!el) return;
  const time = new Date().toLocaleTimeString();
  const line = document.createElement("div");
  if (cls) line.className = cls;
  line.textContent = `[${time}] ${msg}`;
  el.appendChild(line);
  el.scrollTop = el.scrollHeight;
}

// ---- Connection handling -----------------------------------------------

function connect() {
  const host = $("wsHost").value.trim() || "localhost";
  const port = $("wsPort").value.trim() || "9090";
  const url = `ws://${host}:${port}`;

  if (ros) {
    try { ros.close(); } catch (e) {}
  }

  setConnStatus("connecting", `Connecting to ${url}...`);
  ros = new ROSLIB.Ros({ url });

  ros.on("connection", () => {
    setConnStatus("connected", `Connected to ${url}`);
    log(`Connected to ${url}`, "ok");
    subscribeTopics();
  });

  ros.on("error", (err) => {
    setConnStatus("disconnected", "Connection error");
    log(`WebSocket error: ${JSON.stringify(err)}`, "err");
  });

  ros.on("close", () => {
    setConnStatus("disconnected", "Disconnected");
    log("Disconnected from rosbridge", "warn");
    stopImpedance();
    stopMonitoringTopic();
    stopPlotting();
  });
}

function setConnStatus(state, text) {
  const dot = $("statusDot");
  dot.className = `status-dot ${state}`;
  $("statusText").textContent = text;
}

function subscribeTopics() {
  // Joint states subscriber
  jointStateTopic = new ROSLIB.Topic({
    ros,
    name: "/joint_states",
    messageType: "sensor_msgs/msg/JointState",
  });
  jointStateTopic.subscribe(onJointState);

  // Forward position controller command publisher
  commandTopic = new ROSLIB.Topic({
    ros,
    name: "/forward_position_controller/commands",
    messageType: "std_msgs/msg/Float64MultiArray",
  });

  // Forward effort controller command publisher (for impedance control)
  effortTopic = new ROSLIB.Topic({
    ros,
    name: "/forward_effort_controller/commands",
    messageType: "std_msgs/msg/Float64MultiArray",
  });

  // Gripper Action Client
  gripperAction = new ROSLIB.ActionClient({
    ros,
    serverName: "/panda_gripper_controller/gripper_cmd",
    actionName: "control_msgs/action/GripperCommand",
  });
}

// ---- Joint state monitor ---------------------------------------------

function onJointState(msg) {
  const now = performance.now();
  if (lastJsStamp) {
    const instHz = 1000 / (now - lastJsStamp);
    jsRateEma = jsRateEma === null ? instHz : jsRateEma * 0.9 + instHz * 0.1;
    $("jsHz").textContent = `${jsRateEma.toFixed(0)} Hz`;
  }
  lastJsStamp = now;

  // Debug logic validating message structure
  if (now - lastDebugLogStamp > 2000) {
    let hasError = false;
    if (msg.name.length !== msg.position.length) {
      if (msg.position.length > 0) {
        log(`MISMATCH ERROR: name.length=${msg.name.length} != position.length=${msg.position.length}`, "err");
        hasError = true;
      }
    }
    
    msg.position.forEach((pos, i) => {
      if (Number.isNaN(pos)) {
        log(`NaN ERROR: position is NaN for joint: ${msg.name[i]}`, "err");
        hasError = true;
      }
    });

    if (hasError) {
      lastDebugLogStamp = now;
    }
  }

  const nameToIdx = {};
  msg.name.forEach((n, i) => (nameToIdx[n] = i));

  const rows = MONITOR_JOINTS.map((name) => {
    const i = nameToIdx[name];
    const pos = i !== undefined ? msg.position[i] : null;
    const vel = i !== undefined && msg.velocity ? msg.velocity[i] : null;
    return { name, pos, vel };
  });

  const armRows = rows.slice(0, ARM_JOINTS.length);
  if (armRows.every((r) => r.pos !== null && !Number.isNaN(r.pos))) {
    latestJointPositions = armRows.map((r) => r.pos);
  }
  if (armRows.every((r) => r.vel !== null && !Number.isNaN(r.vel))) {
    latestJointVelocities = armRows.map((r) => r.vel);
  }

  const body = $("jointStateBody");
  body.innerHTML = rows
    .map((r) => {
      const deg = r.pos !== null ? ((r.pos * 180) / Math.PI).toFixed(2) : "--";
      const rad = r.pos !== null ? r.pos.toFixed(4) : "--";
      const vel = r.vel !== null ? r.vel.toFixed(4) : "--";
      return `<tr><td>${r.name}</td><td>${deg}</td><td>${rad}</td><td>${vel}</td></tr>`;
    })
    .join("");
}

// ---- Joint control -----------------------------------------------------

function buildSliders() {
  const container = $("jointSliders");
  container.innerHTML = ARM_JOINTS.map((name) => {
    const [lo, hi] = JOINT_LIMITS[name];
    return `
      <div class="joint-row">
        <label for="slider-${name}">${name}</label>
        <input type="range" id="slider-${name}" min="${lo}" max="${hi}" step="0.001" value="0">
        <span class="val" id="val-${name}">0.000</span>
      </div>`;
  }).join("");

  ARM_JOINTS.forEach((name) => {
    const slider = $(`slider-${name}`);
    slider.addEventListener("input", () => {
      $(`val-${name}`).textContent = Number(slider.value).toFixed(3);
      if ($("liveSend").checked) {
        const now = performance.now();
        if (now - liveSendLastAt > 20) {
          liveSendLastAt = now;
          sendJointCommand();
        }
      }
    });
  });
}

function getSliderValues() {
  return ARM_JOINTS.map((name) => Number($(`slider-${name}`).value));
}

function setSliderValues(values) {
  ARM_JOINTS.forEach((name, i) => {
    const val = values[i];
    $(`slider-${name}`).value = val;
    $(`val-${name}`).textContent = Number(val).toFixed(3);
  });
}

function sendJointCommand() {
  if (!commandTopic) {
    log("Not connected to rosbridge", "err");
    return;
  }
  const data = getSliderValues();
  const msg = new ROSLIB.Message({
    layout: { dim: [], data_offset: 0 },
    data,
  });
  commandTopic.publish(msg);
  log(`Sent joint command: [${data.map((x) => x.toFixed(3)).join(", ")}]`, "ok");
}

function syncToCurrent() {
  if (!latestJointPositions) {
    log("No joint state received yet to sync from", "warn");
    return;
  }
  setSliderValues(latestJointPositions);
  log("Synced sliders to current joint positions", "ok");
}

function setHomePose() {
  setSliderValues(HOME_POSE);
  log("Loaded HOME pose into sliders", "ok");
}

function setZeroPose() {
  setSliderValues(ZERO_POSE);
  log("Loaded ZERO pose into sliders", "ok");
}

// ---- Joint Impedance Control -------------------------------------------

function toggleImpedance() {
  if (impedanceInterval) {
    stopImpedance();
  } else {
    startImpedance();
  }
}

function startImpedance() {
  if (!effortTopic) {
    log("Not connected to rosbridge", "err");
    return;
  }
  syncToCurrent(); // Sync sliders to current position so the robot doesn't violently snap
  impedanceInterval = setInterval(impedanceLoop, 20); // 50 Hz
  $("impedanceToggleBtn").textContent = "Stop Impedance Control";
  $("impedanceToggleBtn").className = "danger";
  log("Started joint impedance control loop (50 Hz)", "ok");
}

function stopImpedance() {
  if (impedanceInterval) {
    clearInterval(impedanceInterval);
    impedanceInterval = null;
    $("impedanceToggleBtn").textContent = "Start Impedance Control";
    $("impedanceToggleBtn").className = "";
    log("Stopped joint impedance control", "warn");
    if (effortTopic) {
      effortTopic.publish(new ROSLIB.Message({ layout: { dim: [], data_offset: 0 }, data: [0, 0, 0, 0, 0, 0, 0] }));
    }
  }
}

function impedanceLoop() {
  if (!effortTopic || !latestJointPositions || !latestJointVelocities) return;
  if (latestJointPositions.some(p => p === null || Number.isNaN(p))) return;
  if (latestJointVelocities.some(v => v === null || Number.isNaN(v))) return;

  const target = getSliderValues();
  const K_base = Number($("impedanceK").value);
  const D_base = Number($("impedanceD").value);
  
  // Gain scaling for wrist/smaller joints
  const K_gains = [K_base, K_base, K_base, K_base, K_base / 2, K_base / 4, K_base / 10];
  const D_gains = [D_base, D_base, D_base, D_base, D_base / 2, D_base / 4, D_base / 10];
  
  const torques = target.map((t, i) => {
    const err = t - latestJointPositions[i];
    return K_gains[i] * err - D_gains[i] * latestJointVelocities[i];
  });
  
  if (torques.some(t => t === null || Number.isNaN(t) || !isFinite(t))) {
    log("Refusing to send invalid or NaN torque commands", "err");
    stopImpedance();
    return;
  }
  
  effortTopic.publish(
    new ROSLIB.Message({ layout: { dim: [], data_offset: 0 }, data: torques })
  );
}

// ---- Gripper Control ---------------------------------------------------

function sendGripperCommand(position, maxEffort = 40.0) {
  if (!gripperAction) {
    log("Not connected to rosbridge", "err");
    return;
  }
  const goal = new ROSLIB.Goal({
    actionClient: gripperAction,
    goalMessage: {
      command: {
        position: position,
        max_effort: maxEffort,
      },
    },
  });
  goal.send();
  log(`Sent gripper goal: position=${position.toFixed(3)}m, max_effort=${maxEffort}N`, "ok");
}

// ---- Topic Explorer ----------------------------------------------------

function fetchTopics() {
  if (!ros) { log("Not connected", "err"); return; }
  ros.getTopics((result) => {
    allTopicsList = result.topics.map((t, i) => ({ name: t, type: result.types[i] }));
    allTopicsList.sort((a, b) => a.name.localeCompare(b.name));
    
    // Update the plotting dropdown
    const select = $("plotTopicSelect");
    select.innerHTML = "";
    allTopicsList.forEach(t => {
      const opt = document.createElement("option");
      opt.value = t.name; opt.textContent = t.name;
      opt.dataset.type = t.type;
      select.appendChild(opt);
    });

    if (allTopicsList.length > 0) {
      fetchTopicFieldsForPlot(); // Auto-fetch for the default selected topic
    }

    renderTopicList();
    log(`Fetched ${allTopicsList.length} topics`, "ok");
  });
}

function renderTopicList() {
  const query = $("topicSearch").value.toLowerCase();
  const list = $("topicList");
  list.innerHTML = "";
  
  allTopicsList.filter(t => t.name.toLowerCase().includes(query)).forEach(t => {
    const item = document.createElement("div");
    item.className = "topic-item";
    
    const info = document.createElement("div");
    info.innerHTML = `<span class="topic-name">${t.name}</span><span class="topic-type">${t.type}</span>`;
    
    const btn = document.createElement("button");
    btn.className = "monitor-btn";
    btn.textContent = "Monitor";
    btn.onclick = () => startMonitoringTopic(t.name, t.type);
    
    item.appendChild(info);
    item.appendChild(btn);
    list.appendChild(item);
  });
}

function startMonitoringTopic(topicName, topicType) {
  stopMonitoringTopic();
  
  $("activeTopicName").textContent = topicName;
  $("topicDataViewer").style.display = "block";
  $("activeTopicData").textContent = "Waiting for data...";
  
  activeTopicName = topicName;
  activeTopicType = topicType;
  
  activeTopicSub = new ROSLIB.Topic({
    ros: ros,
    name: topicName,
    messageType: topicType
  });
  
  activeTopicSub.subscribe((msg) => {
    // Only update every 100ms max to prevent UI freezing on high freq topics
    if (Date.now() - (activeTopicSub.lastUpdate || 0) > 100) {
      $("activeTopicData").textContent = JSON.stringify(msg, null, 2);
      activeTopicSub.lastUpdate = Date.now();
    }
  });
  log(`Started monitoring ${topicName}`, "ok");
}

function stopMonitoringTopic() {
  if (activeTopicSub) {
    activeTopicSub.unsubscribe();
    activeTopicSub = null;
  }
  $("topicDataViewer").style.display = "none";
}

// ---- Live Plotter ------------------------------------------------------

function resolvePath(obj, path) {
  return path.split(/[\.\[\]\'\"]/).filter(p => p).reduce((o, p) => o ? o[p] : undefined, obj);
}

function extractNumberPaths(obj, prefix = '') {
  let paths = [];
  for (let key in obj) {
    if (obj.hasOwnProperty(key)) {
      let val = obj[key];
      let newPrefix = prefix ? `${prefix}.${key}` : key;
      if (Array.isArray(val)) {
        if (val.length > 0 && typeof val[0] === 'number') {
           for (let i=0; i<val.length; i++) {
               paths.push(`${newPrefix}[${i}]`);
           }
        } else if (val.length > 0 && typeof val[0] === 'object') {
           for (let i=0; i<val.length; i++) {
               paths = paths.concat(extractNumberPaths(val[i], `${newPrefix}[${i}]`));
           }
        }
      } else if (val !== null && typeof val === 'object') {
        paths = paths.concat(extractNumberPaths(val, newPrefix));
      } else if (typeof val === 'number') {
        paths.push(newPrefix);
      }
    }
  }
  return paths;
}

function fetchTopicFieldsForPlot() {
  const select = $("plotTopicSelect");
  if (!select.options[select.selectedIndex]) return;
  const topicName = select.value;
  const topicType = select.options[select.selectedIndex].dataset.type;
  
  const datalist = $("plotFieldsDatalist");
  datalist.innerHTML = "";
  $("plotFieldInput").placeholder = "Loading fields...";

  const tempSub = new ROSLIB.Topic({
    ros: ros,
    name: topicName,
    messageType: topicType
  });

  // Subscribe once to get the message structure
  tempSub.subscribe((msg) => {
    tempSub.unsubscribe();
    const paths = extractNumberPaths(msg);
    datalist.innerHTML = paths.map(p => `<option value="${p}">`).join("");
    $("plotFieldInput").placeholder = paths.length > 0 ? "Select or type a field" : "No numeric fields found";
    log(`Discovered ${paths.length} numeric fields in ${topicName}`, "ok");
  });
}

function initUPlot() {
  if (uplotInst) {
    uplotInst.destroy();
  }
  const container = $("plotContainer");
  container.innerHTML = "";
  
  const opts = {
    title: "",
    id: "livePlot",
    class: "live-plot",
    width: container.clientWidth || 400,
    height: 250,
    axes: [
      { grid: { show: true, stroke: "rgba(255,255,255,0.1)" } },
      { grid: { show: true, stroke: "rgba(255,255,255,0.1)" } }
    ],
    series: [
      {},
      {
        stroke: "#5b8cff",
        fill: "rgba(91, 140, 255, 0.1)",
        width: 2
      }
    ]
  };
  plotData = [[], []];
  uplotInst = new uPlot(opts, plotData, container);
}

function startPlotting() {
  const select = $("plotTopicSelect");
  if (!select.options[select.selectedIndex]) return;
  const topicName = select.value;
  const topicType = select.options[select.selectedIndex].dataset.type;
  const fieldPath = $("plotFieldInput").value.trim();
  
  if (!fieldPath) {
    log("Please enter a data field to plot", "err");
    return;
  }

  stopPlotting();
  initUPlot();
  plotStartTime = performance.now() / 1000;
  
  plotTopicSub = new ROSLIB.Topic({
    ros: ros,
    name: topicName,
    messageType: topicType
  });
  
  let lastDraw = performance.now();
  
  plotTopicSub.subscribe((msg) => {
    const val = resolvePath(msg, fieldPath);
    if (typeof val === "number") {
      const now = performance.now();
      const t = now / 1000 - plotStartTime;
      
      plotData[0].push(t);
      plotData[1].push(val);
      
      // keep last 500 points
      if (plotData[0].length > 500) {
        plotData[0].shift();
        plotData[1].shift();
      }
      
      if (now - lastDraw > 30) { // cap at ~30 FPS
        uplotInst.setData(plotData);
        lastDraw = now;
      }
    }
  });
  log(`Started plotting ${fieldPath} from ${topicName}`, "ok");
}

function stopPlotting() {
  if (plotTopicSub) {
    plotTopicSub.unsubscribe();
    plotTopicSub = null;
    log("Stopped plotting", "warn");
  }
}

// ---- DOM Wireup --------------------------------------------------------

window.addEventListener("DOMContentLoaded", () => {
  buildSliders();
  setSliderValues(HOME_POSE);

  $("connectBtn").addEventListener("click", connect);
  $("sendBtn").addEventListener("click", sendJointCommand);
  $("syncBtn").addEventListener("click", syncToCurrent);
  $("homeBtn").addEventListener("click", setHomePose);
  $("zeroBtn").addEventListener("click", setZeroPose);

  $("impedanceToggleBtn").addEventListener("click", toggleImpedance);
  $("impedanceK").addEventListener("input", () => {
    $("impedanceKVal").textContent = $("impedanceK").value;
  });
  $("impedanceD").addEventListener("input", () => {
    $("impedanceDVal").textContent = Number($("impedanceD").value).toFixed(1);
  });

  $("gripperOpenBtn").addEventListener("click", () => {
    $("gripperWidthSlider").value = "0.08";
    $("gripperWidthVal").textContent = "0.080";
    sendGripperCommand(0.08);
  });

  $("gripperCloseBtn").addEventListener("click", () => {
    $("gripperWidthSlider").value = "0.0";
    $("gripperWidthVal").textContent = "0.000";
    sendGripperCommand(0.0);
  });

  $("gripperWidthSlider").addEventListener("input", () => {
    $("gripperWidthVal").textContent = Number($("gripperWidthSlider").value).toFixed(3);
  });

  $("gripperSendWidthBtn").addEventListener("click", () => {
    const w = Number($("gripperWidthSlider").value);
    sendGripperCommand(w);
  });

  $("clearLogBtn").addEventListener("click", () => {
    $("log").innerHTML = "";
  });

  $("refreshTopicsBtn").addEventListener("click", fetchTopics);
  $("topicSearch").addEventListener("input", renderTopicList);
  $("closeTopicViewerBtn").addEventListener("click", stopMonitoringTopic);
  $("plotTopicSelect").addEventListener("change", fetchTopicFieldsForPlot);
  
  $("startPlotBtn").addEventListener("click", startPlotting);
  $("stopPlotBtn").addEventListener("click", stopPlotting);

  window.addEventListener("resize", () => {
    if (uplotInst) {
      const container = $("plotContainer");
      uplotInst.setSize({ width: container.clientWidth || 400, height: 250 });
    }
  });

  // Auto-connect on load
  connect();
});
