// Step 3: the only place a trajectory is sent to the robot. Pick a saved
// trajectory, robot and point count (the same choices as the robot preview),
// a port, and start / pause / stop the run. The live visual of the robot's
// state isn't built yet; the main area just says "No robot connected".
let run = null;          // last /api/run status
let pollTimer = 0;
const ACTIVE = new Set(["connecting", "running", "paused", "stopping"]);

async function init() {
  const q = new URLSearchParams(location.search);
  try {
    await fillRobotSelect($("robot"), q.get("robot"));
    const list = await api("GET", "/api/trajectories");
    fillTrajectorySelect($("trajectory"), list, q.get("t"));
    if (!list.length) showError("No saved trajectories yet. Create one in the Trajectory generator.", "runError");
  } catch (e) { showError("Could not reach the server.", "runError"); }
  if (q.get("points") !== null) $("points").value = q.get("points");
  await loadPorts();
  pollRun();
}

async function loadPorts() {
  try {
    const prev = $("port").value;
    const ports = await api("GET", "/api/ports");
    $("port").innerHTML = ports.map(p =>
      `<option value="${p.device}">${p.device === "simulated" ? p.description : `${p.device} — ${p.description}`}</option>`).join("");
    if (ports.some(p => p.device === prev)) $("port").value = prev;
  } catch (e) { showError(e.message, "runError"); }
  renderRun();
}
$("refreshPorts").onclick = loadPorts;

// Polls quickly while a run is active, slowly otherwise (a run may be started
// or finish elsewhere).
async function pollRun() {
  clearTimeout(pollTimer);
  try { run = await api("GET", "/api/run"); } catch { run = null; }
  renderRun();
  pollTimer = setTimeout(pollRun, run && ACTIVE.has(run.state) ? 300 : 2000);
}

function renderRun() {
  const s = run;
  const active = !!s && ACTIVE.has(s.state);
  const showBox = !!s && s.state !== "idle";
  $("runBox").hidden = !showBox;
  if (showBox) {
    const what = s.meta.kind === "home" ? "Returning home" : `${s.meta.name} on ${s.meta.robot}`;
    $("runState").textContent = s.state === "done" && s.meta.kind === "home" ? "At home" : s.state;
    $("runCount").textContent = `${what} · ${s.index.toLocaleString()} / ${s.total.toLocaleString()}`;
    $("runBar").style.width = s.total ? `${(100 * s.index / s.total).toFixed(1)}%` : "0";
  }
  if (s?.state === "error") showError(s.error, "runError");

  const notHome = s && !s.at_home && !active;
  $("homeBox").hidden = !notHome;
  if (notHome) $("homeText").textContent =
    `The pen isn't at home (the last ${s.position_robot} run ${s.state === "done" ? "ended at the drawing's last point" : "didn't finish"}). ` +
    `A new drawing assumes it starts at home.`;

  $("runStart").hidden = active;
  $("runStart").disabled = notHome || !$("port").value || !$("trajectory").value;
  $("runPause").hidden = !active || s.state === "connecting" || s.state === "stopping";
  $("runPause").textContent = s?.state === "paused" ? "Resume" : "Pause";
  $("runStop").hidden = !active;
  $("runStop").disabled = s?.state === "stopping";
  for (const id of ["port", "baud", "refreshPorts", "runHome", "markHome", "robot", "trajectory", "points"])
    $(id).disabled = active;
}

async function runAction(path, body) {
  try { run = await api("POST", path, body); showError("", "runError"); }
  catch (e) { showError(e.message, "runError"); }
  pollRun();
}
const portBody = () => ({ port: $("port").value, baud: Number($("baud").value) || 9600 });
$("runStart").onclick = () => runAction("/api/run", {
  trajectory: $("trajectory").value, robot: $("robot").value,
  points: $("points").value ? Number($("points").value) : null, ...portBody() });
$("runPause").onclick = () => runAction(run?.state === "paused" ? "/api/run/resume" : "/api/run/pause");
$("runStop").onclick = () => runAction("/api/run/stop");
$("runHome").onclick = () => runAction("/api/run/home", portBody());
$("markHome").onclick = () => runAction("/api/run/mark-home");

init();
