# Relite Cloud Alarm Service

Always-on scheduler for ESP32 motors via MQTT. This runs in cloud 24x7, so laptop/dashboard page does not need to stay open.

## What it does

- Sends scheduled ON/OFF commands at configured times
- Retries commands (example: every 5 minutes, 3 times)
- Waits if device is offline, sends when it comes online (within retry window)
- Supports manual ON lock: scheduled OFF is blocked until manual OFF

## Files

- `config.json` -> all schedule and retry settings
- `index.js` -> scheduler + MQTT + API

## Configure

Edit `config.json` (or set Render env vars):

- `timezone`: `Asia/Kolkata`
- `brokerUrl`: your MQTT broker
- `topicPrefix`: `relite` or `relite2` (must match your firmware)
- `defaultTarget`: `0025` or `all`
- `retry.onCount`, `retry.offCount`, `retry.gapMinutes`
- `alarms[]` with `target`, `on`, `off`, `enabled`

Example alarm:

```json
{
  "enabled": true,
  "target": "0025",
  "on": "06:00",
  "off": "07:00"
}
```

## Run locally

```powershell
cd "c:\Users\DELL\Documents\Arduino\vkesh code\cloud-alarm-service"
npm install
npm start
```

Health check:

- `http://localhost:3000/health`

## API endpoints

### Health

- `GET /health`

### View runtime state

- `GET /api/state`

### Manual ON/OFF command

- `POST /api/manual`

Body:

```json
{
  "target": "0025",
  "command": "ON"
}
```

or

```json
{
  "target": "0025",
  "command": "OFF"
}
```

### Set manual lock directly

- `POST /api/manual-lock`

Body:

```json
{
  "target": "0025",
  "active": true
}
```

## Deploy on Render (free/low-cost)

1. Push `cloud-alarm-service` folder to a GitHub repo
2. In Render: New -> Web Service -> connect repo
3. Root directory: `cloud-alarm-service`
4. Build command: `npm install`
5. Start command: `npm start`
6. Deploy

Render sets `PORT` automatically.

Current deployment mode (separate and relite-only):

- Active now: `TOPIC_PREFIX=relite` for:
  - `dashboard/Dashboard_Relite.html`
  - `ESP32_Adhana_Motor_0001/ESP32_Adhana_Motor_0001.ino`
- Paused for now: `relite2`

`render.yaml` is currently configured for relite-only service.

Important for overnight run (laptop OFF):

- This repo is currently set to `plan: free` in `render.yaml` to avoid charges.
- Free tier may sleep and cold-start, so exact alarm time can be delayed.
- For strict on-time cloud alarms, use paid always-on plan.

Optional Render env vars:

- `TIMEZONE` (example: `Asia/Kolkata`)
- `MQTT_BROKER_URL` (example: `mqtt://broker.hivemq.com:1883`)
- `TOPIC_PREFIX` (`relite` or `relite2`)
- `DEFAULT_TARGET` (`all` or `0025`)
- `DEVICE_OFFLINE_MS` (example: `35000`)
- `RETRY_ON_COUNT`, `RETRY_OFF_COUNT`, `RETRY_GAP_MINUTES`
- `ALARMS_JSON` (JSON array string to override `config.json` alarms)

## Deploy on Railway

1. Create new project from GitHub repo
2. Set root to `cloud-alarm-service`
3. Deploy (auto detects Node)

## Notes

- This service must stay running 24x7
- If cloud service is down, schedules pause
- Use one service instance only to avoid duplicate commands
