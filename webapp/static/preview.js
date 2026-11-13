// Step 2: pick a saved trajectory (or import a points file) and a robot. The
// server fits the drawing to that robot's board and computes the
// steps1/steps2/interval commands; this page animates them with the moving
// linkage. Nothing is sent to a robot from here (that's the Live page).
let result = null;      // last /plan response being previewed
let planSeq = 0;        // ignore responses to superseded plan requests
// Playback runs on a continuous clock (ms of robot time). Within each move the
// pen and linkage are interpolated, so every frame shows a distinct position.
let anim = { playing: false, t: 0, raf: 0, last: 0 };
let play = null;        // per-preview timeline, built by buildTimeline()

// ---------------------------------------------------------------- inputs
async function init() {
  try {
    await fillRobotSelect($("robot"));
    fillTrajectories(await api("GET", "/api/trajectories"), new URLSearchParams(location.search).get("t"));
  } catch (e) { showError("Could not reach the server."); }
}

function fillTrajectories(list, selectId) {
  fillTrajectorySelect($("trajectory"), list, selectId);
  $("deleteTraj").disabled = !list.length;
  if (!list.length) {
    result = null; stop(); setStats([]);
    for (const id of ["player", "download"]) $(id).hidden = true;
    $("stage").innerHTML = `<p class="empty">No saved trajectories yet. Create one in the
      <a href="/">Trajectory generator</a>, or import a points file.</p>`;
    return;
  }
  history.replaceState(null, "", `?t=${$("trajectory").value}`);
  refresh();
}

// Import a points file; it is saved as a trajectory like a generated one.
$("importBtn").onclick = () => $("importFile").click();
$("importFile").onchange = async () => {
  const f = $("importFile").files[0];
  $("importFile").value = "";
  if (!f) return;
  const fd = new FormData();
  fd.append("file", f);
  try {
    const meta = await api("POST", "/api/trajectories/import", fd);
    fillTrajectories(await api("GET", "/api/trajectories"), meta.id);
  } catch (e) { showError(e.message); }
};

$("trajectory").onchange = () => {
  history.replaceState(null, "", `?t=${$("trajectory").value}`);
  refresh();
};
$("robot").onchange = () => refresh();
$("points").onchange = () => refresh();
$("deleteTraj").onclick = async () => {
  const id = $("trajectory").value, name = $("trajectory").selectedOptions[0]?.textContent;
  if (!id || !window.confirm(`Delete saved trajectory “${name}”?`)) return;
  try {
    await api("DELETE", `/api/trajectories/${id}`);
    history.replaceState(null, "", location.pathname);
    fillTrajectories(await api("GET", "/api/trajectories"));
  } catch (e) { showError(e.message); }
};

async function refresh() {
  const id = $("trajectory").value;
  if (!id) return;
  const body = { robot: $("robot").value };
  if ($("points").value) body.points = Number($("points").value);
  const seq = ++planSeq;
  document.body.style.cursor = "progress";
  try {
    const r = await api("POST", `/api/trajectories/${id}/plan`, body);
    if (seq !== planSeq) return;
    showError("");
    result = r;
    renderPreview();
  } catch (e) {
    if (seq === planSeq) showError(e.message);
  } finally {
    if (seq === planSeq) document.body.style.cursor = "";
  }
}

// ---------------------------------------------------------------- preview
// View covering the drawable board plus the robot's mechanism (bases, reach).
function boardBounds(board) {
  let [x0, y0, x1, y1] = [0, 0, board.width, board.height];
  const rg = board.robot;
  const xs = [...rg.bounds.filter((_, i) => i % 2 === 0), ...rg.bases.map(b => b[0])];
  const ys = [...rg.bounds.filter((_, i) => i % 2 === 1), ...rg.bases.map(b => b[1])];
  return [Math.min(x0, ...xs), Math.min(y0, ...ys), Math.max(x1, ...xs), Math.max(y1, ...ys)];
}

function renderPreview() {
  stop();
  const r = result, b = r.board;
  const { svg, g, sw } = makeView(boardBounds(b));
  if (!$("showRobot").checked) svg.classList.add("no-robot");
  dashedRect(g, [0, 0, b.width, b.height], sw);

  let prev = b.start;
  for (const piece of r.pieces) { connector(g, prev, piece[0], sw); prev = piece[piece.length - 1]; }

  el("polyline", { points: ptsAttr(r.path), fill: "none", stroke: "var(--path)", "stroke-opacity": 0.25,
    "stroke-width": sw * 1.5, "stroke-linejoin": "round", "stroke-linecap": "round" }, g);
  for (const piece of r.pieces) addArrows(g, piece, sw, "var(--muted)", Math.max(b.width, b.height) / 8);
  buildTimeline();
  // The drawn part is the whole route revealed by a dash: one attribute per frame.
  anim.drawn = el("polyline", { points: ptsAttr(play.pts), fill: "none", stroke: "var(--path)",
    "stroke-width": sw * 1.8, "stroke-linejoin": "round", "stroke-linecap": "round",
    pathLength: play.length || 1, "stroke-dasharray": `${play.length || 1} ${play.length || 1}` }, g);
  anim.robot = makeRobot(g, b.robot, sw);
  anim.pen = el("circle", { r: sw * 4, fill: "var(--accent)", visibility: "hidden" }, g);
  for (const i of r.unreachable)
    el("circle", { cx: r.path[i][0], cy: r.path[i][1], r: sw * 3, fill: "var(--danger)" }, g);
  addHome(svg, b.start, sw);
  $("stage").replaceChildren(svg);

  const t = r.trajectory, n = r.path.length;
  setStats([
    ["Trajectory", t.name],
    ["Order", MODE_LABELS[t.mode] || t.mode],
    ["Robot", `${r.robot} (${b.width} × ${b.height} board)`],
    ["Pieces", r.pieces.length],
    ["Travel between pieces", travelLength(b.start, r.pieces).toFixed(2)],
    ["Points", `${n.toLocaleString()} of ${r.total_points.toLocaleString()}`],
    ["Est. draw time", r.rows.length ? fmtDuration(r.duration_ms) : "—"],
  ]);
  if (r.unreachable.length)
    showError(`${r.unreachable.length} point(s) are outside ${r.robot}'s reach (red dots). No commands were generated.`);

  const hasRows = r.rows.length > 0;
  $("player").hidden = !hasRows;
  $("download").hidden = !hasRows;
  $("scrub").max = Math.max(1, Math.round(play.duration));
  setTime(0);
}

// Timeline for the current preview. Point k of `pts` is home (k = 0) or the
// target of move k; the pen reaches it at cumT[k] ms, having drawn cumLen[k].
// Linkage keyframes: keyM[j] moves done -> keyPose[j] (home first, then the
// server's poses, which are sampled every pose_stride moves).
function buildTimeline() {
  const r = result, pts = [r.board.start, ...r.path], n = r.rows.length;
  const cumT = new Float64Array(pts.length), cumLen = new Float64Array(pts.length);
  for (let k = 1; k < pts.length; k++) {
    cumT[k] = cumT[k - 1] + (k <= n ? r.rows[k - 1][6] : 0);
    cumLen[k] = cumLen[k - 1] + dist(pts[k - 1], pts[k]);
  }
  const stride = r.pose_stride || 1, keyM = [0], keyPose = [r.board.robot.home];
  (r.poses || []).forEach((pose, j) => { keyM.push(Math.min(j * stride, n - 1) + 1); keyPose.push(pose); });
  play = { pts, cumT, cumLen, keyM, keyPose, duration: cumT[pts.length - 1], length: cumLen[pts.length - 1] };
}

// Largest k with a[k] <= x (a ascending).
function floorIndex(a, x) {
  let lo = 0, hi = a.length - 1;
  while (lo < hi) { const mid = (lo + hi + 1) >> 1; if (a[mid] <= x) lo = mid; else hi = mid - 1; }
  return lo;
}
const lerp = (a, b, f) => a + (b - a) * f;

// Linkage after `m` moves (fractional), interpolated between keyframes.
function poseAt(m) {
  const { keyM, keyPose } = play, j = floorIndex(keyM, m);
  const a = keyPose[j], b = keyPose[Math.min(j + 1, keyPose.length - 1)];
  if (!a || !b || a === b) return a || b;
  const f = Math.min(1, (m - keyM[j]) / (keyM[j + 1] - keyM[j]));
  return a.map((v, i) => lerp(v, b[i], f));
}

function setTime(t) {
  const { pts, cumT, cumLen } = play, last = pts.length - 1;
  t = Math.max(0, Math.min(t, play.duration));
  anim.t = t;
  // Move k (1-based) runs from cumT[k-1] to cumT[k]; zero-length moves are instant.
  const k = Math.max(1, Math.min(last, floorIndex(cumT, t) + 1));
  const span = cumT[k] - cumT[k - 1], f = t >= play.duration ? 1 : span > 0 ? Math.min(1, (t - cumT[k - 1]) / span) : 1;
  const done = t <= 0 ? 0 : (k - 1) + f;

  const x = lerp(pts[k - 1][0], pts[k][0], done ? f : 0), y = lerp(pts[k - 1][1], pts[k][1], done ? f : 0);
  const drawn = done ? lerp(cumLen[k - 1], cumLen[k], f) : 0;
  anim.drawn.setAttribute("stroke-dashoffset", (play.length || 1) - drawn);
  anim.drawn.setAttribute("visibility", drawn > 0 ? "visible" : "hidden");
  anim.pen.setAttribute("cx", x); anim.pen.setAttribute("cy", y);
  anim.pen.setAttribute("visibility", done ? "visible" : "hidden");

  const pose = poseAt(done);
  if (pose && done) { pose[0] = x; pose[1] = y; }   // keep the linkage's tip on the pen
  anim.robot.set(pose);
  $("scrub").value = Math.round(t);
}
function tick(now) {
  if (!anim.playing) return;
  setTime(anim.t + (now - anim.last) * Number($("speed").value));
  anim.last = now;
  if (anim.t >= play.duration) return stop();
  anim.raf = requestAnimationFrame(tick);
}
function startPlayback() {
  if (anim.t >= play.duration) setTime(0);
  anim.playing = true; anim.last = performance.now();
  $("play").textContent = "Pause";
  anim.raf = requestAnimationFrame(tick);
}
function stop() { anim.playing = false; cancelAnimationFrame(anim.raf); $("play").textContent = "Play"; }
$("play").onclick = () => anim.playing ? stop() : startPlayback();
$("scrub").oninput = () => { stop(); setTime(Number($("scrub").value)); };
$("showRobot").onchange = () => {
  const svg = $("stage").querySelector("svg");
  if (svg) svg.classList.toggle("no-robot", !$("showRobot").checked);
};

// Tab-separated export of the computed commands (one row per move).
$("download").onclick = () => {
  const r = result;
  const lines = ["x\ty\ttheta1\ttheta2\tsteps1\tsteps2\tinterval",
    ...r.rows.map(row => row.map((v, i) => i < 4 ? v.toFixed(3) : v).join("\t"))];
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([lines.join("\n") + "\n"], { type: "text/plain" }));
  a.download = `${r.robot}_${r.trajectory.name.replace(/[^\w.-]+/g, "_")}_${r.path.length}.txt`;
  a.click();
  URL.revokeObjectURL(a.href);
};

init();
