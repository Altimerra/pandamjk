// Panda MJK Dashboard — connects directly to the rosbridge websocket
// (panda_control.launch.py brings this up on :9090) using roslibjs.
// No backend of its own; see run_dashboard.py for the static file server.

const ARM_JOINTS = ["fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4", "fer_joint5", "fer_joint6", "fer_joint7"];
const MONITOR_JOINTS = [...ARM_JOINTS, "finger_joint1"];

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

// Standard Franka "ready" pose, widely used as a safe home configuration.
const HOME_POSE = [0, -0.785398, 0, -2.356194, 0, 1.570796, 0.785398];

// Best-effort mapping of moveit_servo's StatusCode enum. The exact ordering
// has shifted slightly across releases, so the raw code is always shown too.
const SERVO_STATUS = {
  0: ["NO_WARNING", "ok"],
  1: ["DECELERATE_FOR_SINGULARITY", "warn"],
  2: ["HALT_FOR_SINGULARITY", "halt"],
  3: ["DECELERATE_FOR_COLLISION", "warn"],
  4: ["HALT_FOR_COLLISION", "halt"],
  5: ["DECELERATE_FOR_LEADING_SINGULARITY", "warn"],
  6: ["JOINT_BOUND", "warn"],
  7: ["INVALID", "halt"],
};

let ros = null;
let jointStateTopic = null;
let servoStatusTopic = null;
let commandTopic = null;
let twistTopic = null;
let effortTopic = null;
let latestJointPositions = null; // ordered by ARM_JOINTS
let latestJointVelocities = null; // ordered by ARM_JOINTS
let impedanceInterval = null;
let lastJsStamp = 0;
let jsRateEma = null;
let jogInterval = null;
let liveSendLastAt = 0;
let lastDebugLogStamp = 0;

const $ = (id) => document.getElementById(id);

function log(msg, cls) {
  const el = $("log");
  const line = document.createElement("div");
  if (cls) line.className = cls;
  const t = new Date().toLocaleTimeString();
  line.textContent = `[${t}] ${msg}`;
  el.appendChild(line);
  el.scrollTop = el.scrollHeight;
}

function setStatus(state, text) {
  const dot = $("statusDot");
  dot.className = `status-dot ${state}`;
  $("statusText").textContent = text;
}

// ---- Connection -----------------------------------------------------

function connect() {
  if (ros) {
    ros.close();
    ros = null;
  }

  const host = $("wsHost").value.trim() || "localhost";
  const port = $("wsPort").value.trim() || "9090";
  const url = `ws://${host}:${port}`;

  setStatus("connecting", `Connecting to ${url}...`);
  ros = new ROSLIB.Ros({ url });

  ros.on("connection", () => {
    setStatus("connected", `Connected to ${url}`);
    log(`Connected to ${url}`, "ok");
    $("connectBtn").textContent = "Disconnect";
    subscribeAll();
  });

  ros.on("error", (err) => {
    setStatus("disconnected", "Connection error");
    log(`Connection error: ${err}`, "err");
  });

  ros.on("close", () => {
    setStatus("disconnected", "Disconnected");
    log("Disconnected");
    $("connectBtn").textContent = "Connect";
  });
}

function disconnect() {
  if (ros) {
    ros.close();
  }
}

function subscribeAll() {
  jointStateTopic = new ROSLIB.Topic({
    ros,
    name: "/joint_states",
    messageType: "sensor_msgs/msg/JointState",
  });
  jointStateTopic.subscribe(onJointState);

  servoStatusTopic = new ROSLIB.Topic({
    ros,
    name: "/servo_node/status",
    messageType: "std_msgs/msg/Int8",
  });
  servoStatusTopic.subscribe(onServoStatus);

  commandTopic = new ROSLIB.Topic({
    ros,
    name: "/forward_position_controller/commands",
    messageType: "std_msgs/msg/Float64MultiArray",
  });

  twistTopic = new ROSLIB.Topic({
    ros,
    name: "/servo_node/delta_twist_cmds",
    messageType: "geometry_msgs/msg/TwistStamped",
  });

  effortTopic = new ROSLIB.Topic({
    ros,
    name: "/forward_effort_controller/commands",
    messageType: "std_msgs/msg/Float64MultiArray",
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

  // Debug logic mirroring robot_state_publisher's validation
  if (now - lastDebugLogStamp > 2000) {
    let hasError = false;
    
    // Check array size mismatch
    if (msg.name.length !== msg.position.length) {
      if (msg.position.length > 0) {
        log(`MISMATCH ERROR: name.length=${msg.name.length} != position.length=${msg.position.length}`, "err");
        log(`name array: ${JSON.stringify(msg.name)}`, "err");
        log(`position array: ${JSON.stringify(msg.position)}`, "err");
        log(`velocity array: ${JSON.stringify(msg.velocity)}`, "err");
        log(`effort array: ${JSON.stringify(msg.effort)}`, "err");
        log(`header frame_id: "${msg.header ? msg.header.frame_id : ''}"`, "err");
        hasError = true;
      }
    }
    
    // Check for NaNs in position
    msg.position.forEach((pos, i) => {
      if (Number.isNaN(pos)) {
        log(`NaN ERROR: position is NaN for joint: ${msg.name[i]}`, "err");
        hasError = true;
      }
    });

    // Check for NaNs in velocity and effort (warnings)
    if (msg.velocity) {
      msg.velocity.forEach((vel, i) => {
        if (Number.isNaN(vel)) {
          log(`NaN WARNING: velocity is NaN for joint: ${msg.name[i]}`, "warn");
          hasError = true;
        }
      });
    }
    if (msg.effort) {
      msg.effort.forEach((eff, i) => {
        if (Number.isNaN(eff)) {
          log(`NaN WARNING: effort is NaN for joint: ${msg.name[i]}`, "warn");
          hasError = true;
        }
      });
    }
    
    if (hasError) {
      lastDebugLogStamp = now; // Throttle error logging to avoid flooding the DOM
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
  if (armRows.every((r) => r.pos !== null)) {
    latestJointPositions = armRows.map((r) => r.pos);
  }
  if (armRows.every((r) => r.vel !== null)) {
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

function onServoStatus(msg) {
  const code = msg.data;
  const [label, cls] = SERVO_STATUS[code] || ["UNKNOWN", "warn"];
  const el = $("servoStatus");
  el.textContent = `${label} (code ${code})`;
  el.className = `servo-status ${cls}`;
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
        if (now - liveSendLastAt > 100) {
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
    $(`slider-${name}`).value = values[i];
    $(`val-${name}`).textContent = Number(values[i]).toFixed(3);
  });
}

function sendJointCommand() {
  if (!commandTopic) {
    log("Not connected — cannot send joint command", "err");
    return;
  }
  const data = getSliderValues();
  commandTopic.publish(
    new ROSLIB.Message({ layout: { dim: [], data_offset: 0 }, data })
  );
  log(`Sent joint command: [${data.map((v) => v.toFixed(3)).join(", ")}]`);
}

function syncToCurrent() {
  if (!latestJointPositions) {
    log("No joint state received yet", "err");
    return;
  }
  setSliderValues(latestJointPositions);
}

function goHome() {
  setSliderValues(HOME_POSE);
  sendJointCommand();
}

// ---- Cartesian jog -------------------------------------------------------

function publishTwist(linear, angular) {
  if (!twistTopic) return;
  twistTopic.publish(
    new ROSLIB.Message({
      header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "fer_link0" },
      twist: { linear, angular },
    })
  );
}

function zeroTwist() {
  publishTwist({ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 });
}

function startJog(axis, comp, sign) {
  if (!twistTopic) {
    log("Not connected — cannot jog", "err");
    return;
  }
  stopJog();
  const scale = Number($("jogScale").value) * sign;
  const linear = { x: 0, y: 0, z: 0 };
  const angular = { x: 0, y: 0, z: 0 };
  const send = () => {
    if (axis === "linear") linear[comp] = scale;
    else angular[comp] = scale;
    publishTwist(linear, angular);
  };
  send();
  jogInterval = setInterval(send, 50); // 20 Hz, well under servo's 100ms timeout
}

function stopJog() {
  if (jogInterval) {
    clearInterval(jogInterval);
    jogInterval = null;
    zeroTwist();
  }
}

// ---- Impedance Control ---------------------------------------------------

function toggleImpedance() {
  if (impedanceInterval) {
    clearInterval(impedanceInterval);
    impedanceInterval = null;
    $("impedanceToggleBtn").textContent = "Start Impedance Control";
    log("Impedance control stopped.");
    if (effortTopic) {
      effortTopic.publish(new ROSLIB.Message({ layout: { dim: [], data_offset: 0 }, data: [0, 0, 0, 0, 0, 0, 0] }));
    }
  } else {
    $("impedanceToggleBtn").textContent = "Stop Impedance Control";
    log("Impedance control started.");
    impedanceInterval = setInterval(impedanceLoop, 20); // 50 Hz
  }
}

function impedanceLoop() {
  if (!effortTopic || !latestJointPositions || !latestJointVelocities) return;
  const target = getSliderValues();
  const K_base = Number($("impedanceK").value);
  const D_base = Number($("impedanceD").value);
  
  // Lower gains for smaller joints
  const K_gains = [K_base, K_base, K_base, K_base, K_base / 2, K_base / 4, K_base / 10];
  const D_gains = [D_base, D_base, D_base, D_base, D_base / 2, D_base / 4, D_base / 10];
  
  const torques = target.map((t, i) => {
    const err = t - latestJointPositions[i];
    return K_gains[i] * err - D_gains[i] * latestJointVelocities[i];
  });
  
  effortTopic.publish(
    new ROSLIB.Message({ layout: { dim: [], data_offset: 0 }, data: torques })
  );
}

// ---- Gripper (best-effort, sim only) -------------------------------------

function sendGripperCommand(position, maxEffort) {
  if (!ros) {
    log("Not connected — cannot command gripper", "err");
    return;
  }
  const client = new ROSLIB.ActionClient({
    ros,
    serverName: "/panda_gripper_controller/gripper_cmd",
    actionName: "control_msgs/action/GripperCommand",
  });
  const goal = new ROSLIB.Goal({
    actionClient: client,
    goalMessage: { command: { position, max_effort: maxEffort } },
  });
  goal.on("result", (result) => log(`Gripper goal result: ${JSON.stringify(result)}`));
  goal.on("timeout", () => log("Gripper goal timed out", "err"));
  goal.send();
  log(`Sent gripper goal: position=${position}, max_effort=${maxEffort}`);
}

// ---- Wiring ---------------------------------------------------------------

function init() {
  buildSliders();

  $("connectBtn").addEventListener("click", () => {
    if (ros && ros.isConnected) disconnect();
    else connect();
  });

  $("syncBtn").addEventListener("click", syncToCurrent);
  $("homeBtn").addEventListener("click", goHome);
  $("sendBtn").addEventListener("click", sendJointCommand);

  $("jogScale").addEventListener("input", () => {
    $("jogScaleVal").textContent = Number($("jogScale").value).toFixed(2);
  });
  $("jogStopBtn").addEventListener("click", stopJog);

  document.querySelectorAll(".jog-buttons button").forEach((btn) => {
    const axis = btn.dataset.axis;
    const comp = btn.dataset.comp;
    const sign = Number(btn.dataset.sign);
    btn.addEventListener("pointerdown", () => startJog(axis, comp, sign));
    btn.addEventListener("pointerup", stopJog);
    btn.addEventListener("pointerleave", stopJog);
    btn.addEventListener("pointercancel", stopJog);
  });

  $("gripperOpenBtn").addEventListener("click", () => sendGripperCommand(0.04, 20));
  $("gripperCloseBtn").addEventListener("click", () => sendGripperCommand(0.0, 40));

  $("impedanceToggleBtn").addEventListener("click", toggleImpedance);
  $("impedanceK").addEventListener("input", () => {
    $("impedanceKVal").textContent = Number($("impedanceK").value).toFixed(0);
  });
  $("impedanceD").addEventListener("input", () => {
    $("impedanceDVal").textContent = Number($("impedanceD").value).toFixed(1);
  });

  window.addEventListener("beforeunload", () => {
    if (ros) ros.close();
  });
}

init();
