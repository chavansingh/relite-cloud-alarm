const fs = require("fs");
const path = require("path");
const express = require("express");
const mqtt = require("mqtt");

const CONFIG_PATH = path.join(__dirname, "config.json");
const PORT = Number(process.env.PORT || 3000);

function readConfig() {
  const raw = fs.readFileSync(CONFIG_PATH, "utf8");
  const parsed = JSON.parse(raw);

  return {
    timezone: parsed.timezone || "Asia/Kolkata",
    brokerUrl: parsed.brokerUrl || "mqtt://broker.hivemq.com:1883",
    topicPrefix: parsed.topicPrefix || "relite",
    defaultTarget: normalizeTarget(parsed.defaultTarget || "all"),
    deviceOfflineMs: Number.isFinite(Number(parsed.deviceOfflineMs)) ? Number(parsed.deviceOfflineMs) : 35000,
    retry: {
      onCount: clampNumber(parsed.retry && parsed.retry.onCount, 1, 20, 3),
      offCount: clampNumber(parsed.retry && parsed.retry.offCount, 1, 20, 3),
      gapMinutes: clampNumber(parsed.retry && parsed.retry.gapMinutes, 1, 60, 5)
    },
    alarms: Array.isArray(parsed.alarms) ? parsed.alarms : []
  };
}

function clampNumber(value, min, max, fallback) {
  const n = Number(value);
  if (!Number.isFinite(n)) {
    return fallback;
  }
  return Math.max(min, Math.min(max, n));
}

function normalizeTarget(target) {
  if (!target) {
    return "all";
  }
  const text = String(target).trim();
  if (!text || text.toLowerCase() === "all") {
    return "all";
  }
  const n = Number(text);
  if (!Number.isFinite(n) || n < 1 || n > 9999) {
    return "all";
  }
  return String(Math.floor(n)).padStart(4, "0");
}

function nowParts(timeZone) {
  const dtf = new Intl.DateTimeFormat("en-CA", {
    timeZone,
    year: "numeric",
    month: "2-digit",
    day: "2-digit",
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hourCycle: "h23"
  });

  const parts = dtf.formatToParts(new Date());
  const bag = {};
  for (const part of parts) {
    if (part.type !== "literal") {
      bag[part.type] = part.value;
    }
  }

  return {
    dayKey: `${bag.year}-${bag.month}-${bag.day}`,
    hour: Number(bag.hour),
    minute: Number(bag.minute),
    second: Number(bag.second)
  };
}

function parseHHMM(text) {
  if (typeof text !== "string") {
    return null;
  }
  const m = text.trim().match(/^(\d{1,2}):(\d{1,2})$/);
  if (!m) {
    return null;
  }
  const hh = Number(m[1]);
  const mm = Number(m[2]);
  if (!Number.isInteger(hh) || !Number.isInteger(mm) || hh < 0 || hh > 23 || mm < 0 || mm > 59) {
    return null;
  }
  return { hh, mm };
}

const config = readConfig();

const state = {
  mqttConnected: false,
  onlineMap: new Map(),
  manualLockByTarget: new Map(),
  alarmLastTrigger: new Map(),
  jobs: new Map()
};

const clientId = `relite_cloud_alarm_${Math.random().toString(16).slice(2, 10)}`;
const mqttClient = mqtt.connect(config.brokerUrl, {
  clientId,
  clean: true,
  reconnectPeriod: 5000,
  keepalive: 60
});

mqttClient.on("connect", () => {
  state.mqttConnected = true;
  log("MQTT connected");
  mqttClient.subscribe(`${config.topicPrefix}/motor/+/status`, { qos: 1 });
  mqttClient.subscribe(`${config.topicPrefix}/motor/+/telemetry`, { qos: 1 });
  mqttClient.subscribe(`${config.topicPrefix}/sensors/all`, { qos: 1 });
});

mqttClient.on("close", () => {
  state.mqttConnected = false;
  log("MQTT disconnected");
});

mqttClient.on("error", (err) => {
  log(`MQTT error: ${err.message}`);
});

mqttClient.on("message", (topic, payloadBuf) => {
  const payloadText = payloadBuf.toString();

  const topicMatch = topic.match(/\/motor\/(\d+)\/(status|telemetry)$/i);
  if (topicMatch) {
    const id4 = String(Number(topicMatch[1])).padStart(4, "0");
    markOnline(id4);
    return;
  }

  if (topic === `${config.topicPrefix}/sensors/all`) {
    try {
      const payload = JSON.parse(payloadText);
      const moduleId = Number(payload && (payload.module_id || payload.module || payload.motor_id));
      if (Number.isInteger(moduleId) && moduleId >= 1 && moduleId <= 9999) {
        markOnline(String(moduleId).padStart(4, "0"));
      }
    } catch (err) {
      // ignore parse errors from mixed payloads
    }
  }
});

function markOnline(target) {
  state.onlineMap.set(target, Date.now());
}

function isOnline(target) {
  const now = Date.now();
  if (target === "all") {
    for (const ts of state.onlineMap.values()) {
      if (now - ts <= config.deviceOfflineMs) {
        return true;
      }
    }
    return false;
  }

  const ts = state.onlineMap.get(target);
  return Boolean(ts && now - ts <= config.deviceOfflineMs);
}

function topicForTarget(target) {
  if (target === "all") {
    return `${config.topicPrefix}/motor/all/control`;
  }
  return `${config.topicPrefix}/motor/${target}/control`;
}

function enqueueAlarmJob(target, command, sourceLabel) {
  const retryCount = command === "OFF" ? config.retry.offCount : config.retry.onCount;
  const gapMs = config.retry.gapMinutes * 60 * 1000;
  const now = Date.now();

  if (command === "OFF" && isManualLockActive(target)) {
    log(`OFF blocked by manual lock for target ${target}`);
    return;
  }

  const id = `${target}_${command}_${sourceLabel.replace(/\s+/g, "_")}_${now}`;
  state.jobs.set(id, {
    id,
    target,
    topic: topicForTarget(target),
    command,
    sourceLabel,
    retryCount,
    gapMs,
    attemptsSent: 0,
    nextAttemptAt: now,
    expiresAt: now + retryCount * gapMs
  });

  log(`Queued ${command} job for ${target} (${sourceLabel}) with ${retryCount} tries`);
}

function processJobs() {
  const now = Date.now();

  for (const [jobId, job] of state.jobs.entries()) {
    if (job.attemptsSent >= job.retryCount || now > job.expiresAt) {
      state.jobs.delete(jobId);
      continue;
    }

    if (now < job.nextAttemptAt) {
      continue;
    }

    if (!state.mqttConnected) {
      continue;
    }

    if (!isOnline(job.target)) {
      continue;
    }

    if (job.command === "OFF" && isManualLockActive(job.target)) {
      log(`Skipped OFF for ${job.target} due to manual lock`);
      state.jobs.delete(jobId);
      continue;
    }

    const attemptNo = job.attemptsSent + 1;
    mqttClient.publish(job.topic, job.command, { qos: 1, retain: false }, (err) => {
      if (err) {
        log(`Publish failed ${job.command} ${job.target} attempt ${attemptNo}: ${err.message}`);
        return;
      }
      log(`Published ${job.command} to ${job.target} attempt ${attemptNo}/${job.retryCount}`);
    });

    job.attemptsSent += 1;
    job.nextAttemptAt = now + job.gapMs;

    if (job.attemptsSent >= job.retryCount) {
      state.jobs.delete(jobId);
    }
  }
}

function isManualLockActive(target) {
  if (state.manualLockByTarget.get("all") === true) {
    return true;
  }
  if (target === "all") {
    for (const [key, value] of state.manualLockByTarget.entries()) {
      if (key !== "all" && value === true) {
        return true;
      }
    }
    return false;
  }
  return state.manualLockByTarget.get(target) === true;
}

function setManualLock(target, active) {
  state.manualLockByTarget.set(target, !!active);
  if (active) {
    cancelQueuedOffJobs(target);
  }
}

function cancelQueuedOffJobs(target) {
  for (const [jobId, job] of state.jobs.entries()) {
    if (job.command !== "OFF") {
      continue;
    }
    if (target === "all" || job.target === target || job.target === "all") {
      state.jobs.delete(jobId);
    }
  }
}

function checkAlarms() {
  const now = nowParts(config.timezone);

  config.alarms.forEach((alarm, index) => {
    if (!alarm || alarm.enabled !== true) {
      return;
    }

    const target = normalizeTarget(alarm.target || config.defaultTarget);
    const on = parseHHMM(alarm.on);
    const off = parseHHMM(alarm.off);

    if (on && on.hh === now.hour && on.mm === now.minute) {
      const key = `${now.dayKey}|${index}|ON|${on.hh}:${on.mm}|${target}`;
      if (state.alarmLastTrigger.get(`ON_${index}_${target}`) !== key) {
        state.alarmLastTrigger.set(`ON_${index}_${target}`, key);
        enqueueAlarmJob(target, "ON", `alarm ${index + 1} ON`);
      }
    }

    if (off && off.hh === now.hour && off.mm === now.minute) {
      const key = `${now.dayKey}|${index}|OFF|${off.hh}:${off.mm}|${target}`;
      if (state.alarmLastTrigger.get(`OFF_${index}_${target}`) !== key) {
        state.alarmLastTrigger.set(`OFF_${index}_${target}`, key);
        enqueueAlarmJob(target, "OFF", `alarm ${index + 1} OFF`);
      }
    }
  });
}

function publishManualCommand(target, command, done) {
  if (!state.mqttConnected) {
    done(new Error("MQTT disconnected"));
    return;
  }

  const normalizedTarget = normalizeTarget(target || config.defaultTarget);
  const topic = topicForTarget(normalizedTarget);

  if (command === "ON") {
    setManualLock(normalizedTarget, true);
  }
  if (command === "OFF") {
    setManualLock(normalizedTarget, false);
  }

  mqttClient.publish(topic, command, { qos: 1, retain: false }, (err) => {
    if (err) {
      done(err);
      return;
    }
    log(`Manual ${command} published to ${normalizedTarget}`);
    done(null, { topic, target: normalizedTarget, command });
  });
}

function log(text) {
  const stamp = new Date().toISOString();
  console.log(`[${stamp}] ${text}`);
}

setInterval(checkAlarms, 15000);
setInterval(processJobs, 5000);

const app = express();
app.use(express.json());

app.get("/health", (_req, res) => {
  res.json({
    ok: true,
    mqttConnected: state.mqttConnected,
    jobsQueued: state.jobs.size,
    timezone: config.timezone,
    now: new Date().toISOString()
  });
});

app.get("/api/state", (_req, res) => {
  const online = {};
  const now = Date.now();

  for (const [k, ts] of state.onlineMap.entries()) {
    online[k] = {
      lastSeenMsAgo: now - ts,
      online: now - ts <= config.deviceOfflineMs
    };
  }

  const manualLocks = {};
  for (const [k, v] of state.manualLockByTarget.entries()) {
    manualLocks[k] = v;
  }

  res.json({
    mqttConnected: state.mqttConnected,
    config,
    online,
    manualLocks,
    jobs: Array.from(state.jobs.values())
  });
});

app.post("/api/manual", (req, res) => {
  const command = String(req.body && req.body.command || "").trim().toUpperCase();
  const target = req.body && req.body.target ? String(req.body.target) : config.defaultTarget;

  if (command !== "ON" && command !== "OFF") {
    res.status(400).json({ ok: false, error: "command must be ON or OFF" });
    return;
  }

  publishManualCommand(target, command, (err, data) => {
    if (err) {
      res.status(500).json({ ok: false, error: err.message });
      return;
    }
    res.json({ ok: true, ...data });
  });
});

app.post("/api/manual-lock", (req, res) => {
  const target = normalizeTarget(req.body && req.body.target ? String(req.body.target) : "all");
  const active = Boolean(req.body && req.body.active);
  setManualLock(target, active);
  res.json({ ok: true, target, active });
});

app.listen(PORT, () => {
  log(`Cloud alarm service running on port ${PORT}`);
});
