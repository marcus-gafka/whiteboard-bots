// Step 1: load a DXF, order its pieces (auto or manual) in the drawing's own
// units, and save the result as a robot-independent trajectory.
const app = {
  chosenFile: null,
  drawing: null,        // response of POST /api/drawings
  mode: "auto",         // "auto" | "manual"
  autoPieces: null,     // ordered pieces for the auto tab
  manual: null,         // { pool:Set, placed, complete, can_undo, pen, cuttable:Set }
  pieces: new Map(),    // piece id -> display points (cached across manual actions)
  roots: new Map(),     // piece id -> original geometry id (for colour)
  tool: "place",
  busy: false,
};
let editor = null;      // { svg, g, sw, hoverLayer } for the current manual render

// ---------------------------------------------------------------- inputs
async function init() {
  try {
    const samples = await api("GET", "/api/samples");
    $("sample").innerHTML += samples.map(s => `<option>${s}</option>`).join("");
  } catch (e) { showError("Could not reach the server."); }
}
function setFile(f) {
  app.chosenFile = f;
  $("dropName").textContent = f ? f.name : "Drop a .dxf here";
  if (f) $("sample").value = "";
  updateLoad();
}
function updateLoad() { $("load").disabled = app.busy || !(app.chosenFile || $("sample").value); }

$("drop").onclick = () => $("file").click();
$("drop").onkeydown = (e) => { if (e.key === "Enter" || e.key === " ") { e.preventDefault(); $("file").click(); } };
$("file").onchange = () => setFile($("file").files[0] || null);
$("drop").ondragover = (e) => { e.preventDefault(); $("drop").classList.add("over"); };
$("drop").ondragleave = () => $("drop").classList.remove("over");
$("drop").ondrop = (e) => {
  e.preventDefault(); $("drop").classList.remove("over");
  if (e.dataTransfer.files[0]) setFile(e.dataTransfer.files[0]);
};
$("sample").onchange = () => {
  if ($("sample").value) { app.chosenFile = null; $("file").value = ""; $("dropName").textContent = "Drop a .dxf here"; }
  updateLoad();
};

async function busy(fn) {
  if (app.busy) return;
  app.busy = true; updateLoad(); document.body.style.cursor = "progress";
  try { showError(""); await fn(); }
  catch (e) { showError(e.message); }
  finally { app.busy = false; updateLoad(); updateSave(); document.body.style.cursor = ""; }
}

$("load").onclick = () => busy(async () => {
  const fd = new FormData();
  if (app.chosenFile) fd.append("file", app.chosenFile); else fd.append("sample", $("sample").value);
  $("load").textContent = "Loading…";
  try { app.drawing = await api("POST", "/api/drawings", fd); }
  finally { $("load").textContent = "Load drawing"; }
  app.autoPieces = null; app.manual = null; app.pieces.clear(); app.roots.clear();
  $("tabs").hidden = false; $("savePanel").hidden = false; $("saved").hidden = true;
  await showMode(app.mode);
});

// ---------------------------------------------------------------- modes
$("tabs").onclick = (e) => {
  const mode = e.target.closest("button")?.dataset.mode;
  if (mode && mode !== app.mode && app.drawing) busy(() => showMode(mode));
};

function selectTab(mode) {
  app.mode = mode;
  for (const b of $("tabs").querySelectorAll("button")) b.setAttribute("aria-selected", b.dataset.mode === mode);
  $("saveName").placeholder = `${app.drawing.source} (${mode})`;
}

async function showMode(mode) {
  selectTab(mode);
  if (mode === "auto") {
    if (!app.autoPieces) app.autoPieces = (await api("POST", `/api/drawings/${app.drawing.drawing}/order`, { mode })).pieces;
    renderAuto();
  } else {
    if (!app.manual) applyManual(await api("GET", `/api/drawings/${app.drawing.drawing}/manual`));
    renderEditor();
  }
}

function drawingStats(extra) {
  const d = app.drawing, [x0, y0, x1, y1] = d.bounds;
  setStats([
    ["Drawing", d.source],
    ["Size (DXF units)", `${fmtNum(x1 - x0)} × ${fmtNum(y1 - y0)}`],
    ["Entities", entitySummary(d.counts)],
    ["Points", d.total_points.toLocaleString()],
    ...extra,
  ]);
  const skipped = Object.entries(d.counts.skipped || {});
  if (skipped.length)
    showError(`Not drawn (unsupported in this DXF): ${skipped.map(([k, n]) => `${n} ${k}`).join(", ")}.`);
}

const arrowEvery = ([x0, y0, x1, y1]) => Math.max(x1 - x0, y1 - y0) / 8;

function renderAuto() {
  const d = app.drawing, pieces = app.autoPieces;
  const { svg, g, sw } = makeView(d.bounds);
  dashedRect(g, d.bounds, sw);
  let prev = d.start;
  for (const p of pieces) { connector(g, prev, p[0], sw); prev = p[p.length - 1]; }
  for (const p of pieces) {
    el("polyline", { points: ptsAttr(p), fill: "none", stroke: "var(--path)", "stroke-width": sw * 1.8,
      "stroke-linejoin": "round", "stroke-linecap": "round" }, g);
    addArrows(g, p, sw, "var(--accent)", arrowEvery(d.bounds));
  }
  addHome(svg, d.start, sw);
  $("stage").replaceChildren(svg);
  editor = null;
  $("toolbar").hidden = true; $("lgPool").hidden = true; $("autoBar").hidden = false;
  drawingStats([
    ["Order", "Auto (nearest neighbour)"],
    ["Pieces", pieces.length],
    ["Pen-down travel", fmtNum(travelLength(d.start, pieces))],
  ]);
}

// ---------------------------------------------------------------- manual editor
function applyManual(s) {
  for (const [id, pts] of Object.entries(s.pieces)) app.pieces.set(Number(id), pts);
  for (const [id, root] of Object.entries(s.roots)) app.roots.set(Number(id), root);
  app.manual = { ...s, pool: new Set(s.pool), cuttable: new Set(s.cuttable) };
}
function oriented(id, reverse) {
  const pts = app.pieces.get(id);
  return reverse ? [...pts].reverse() : pts;
}
const colorOf = (id) => pieceColor(id, app.roots);

function renderEditor() {
  const m = app.manual, d = app.drawing;
  const { svg, g, sw } = makeView(d.bounds);
  dashedRect(g, d.bounds, sw);
  svg.classList.add("editor");
  svg.dataset.tool = app.tool;
  const many = app.pieces.size > 300;   // skip per-piece decorations on very dense drawings

  // Placed pieces + travel connectors.
  const placedPieces = m.placed.map(([id, rev]) => oriented(id, rev));
  let prev = d.start;
  placedPieces.forEach((pts, i) => {
    connector(g, prev, pts[0], sw);
    el("polyline", { points: ptsAttr(pts), fill: "none", stroke: "var(--path)", "stroke-width": sw * 1.8,
      "stroke-linejoin": "round", "stroke-linecap": "round", "pointer-events": "none" }, g);
    addArrows(g, pts, sw, "var(--accent)", arrowEvery(d.bounds));
    if (!many) addText(svg, pts[0], i + 1, sw, { "font-size": sw * 9, "font-weight": 700,
      fill: "var(--muted)", dx: sw * 3, dy: -sw * 3 });
    prev = pts[pts.length - 1];
  });

  // Pool pieces: visible stroke + a wide invisible hit target.
  const pool = [...m.pool];
  for (const id of pool) {
    const pts = app.pieces.get(id), color = colorOf(id);
    el("polyline", { points: ptsAttr(pts), fill: "none", stroke: color, "stroke-opacity": 0.6,
      "stroke-width": sw * 1.6, "stroke-linejoin": "round", "stroke-linecap": "round",
      "pointer-events": "none", "data-vis": id }, g);
    if (!many) for (const end of [pts[0], pts[pts.length - 1]])
      el("circle", { cx: end[0], cy: end[1], r: sw * 1.6, fill: "var(--board)", stroke: color,
        "stroke-width": sw * 0.8, "pointer-events": "none" }, g);
  }
  m.placed.forEach(([id], i) => {
    const hit = el("polyline", { points: ptsAttr(placedPieces[i]), class: "hit placed", "stroke-width": sw * 9 }, g);
    hit.addEventListener("pointermove", () => hoverPlaced(id));
    hit.addEventListener("pointerleave", () => clearHover());
    hit.addEventListener("click", () => manualAction({ action: "flip", piece: id }));
  });
  for (const id of pool) {
    const hit = el("polyline", { points: ptsAttr(app.pieces.get(id)), class: "hit pool" +
      (m.cuttable.has(id) ? "" : " nocut"), "stroke-width": sw * 9, "data-id": id }, g);
    hit.addEventListener("pointermove", (e) => hover(id, e));
    hit.addEventListener("pointerleave", () => clearHover());
    hit.addEventListener("click", (e) => clickPiece(id, e));
  }

  const hoverLayer = el("g", { "pointer-events": "none" }, g);
  el("circle", { cx: m.pen[0], cy: m.pen[1], r: sw * 3.5, fill: "var(--accent)", "pointer-events": "none" }, g);
  addHome(svg, d.start, sw);
  $("stage").replaceChildren(svg);
  editor = { svg, g, sw, hoverLayer };

  const placedCount = m.placed.length, total = placedCount + pool.length;
  $("progress").textContent = `${placedCount} / ${total} placed`;
  $("undo").disabled = !m.can_undo;
  $("autofinish").disabled = m.complete;
  $("reset").disabled = !m.can_undo && placedCount === 0;
  setToolButtons();
  $("toolbar").hidden = false; $("lgPool").hidden = false; $("autoBar").hidden = true;
  drawingStats([
    ["Order", "Manual"],
    ["Pieces", `${total}${total !== d.piece_count ? ` (${d.piece_count} before cuts)` : ""}`],
    ["Placed", `${placedCount} of ${total}`],
    ["Pen-down travel so far", fmtNum(travelLength(d.start, placedPieces))],
  ]);
  updateSave();
}

function toBoard(e) {
  const p = new DOMPoint(e.clientX, e.clientY).matrixTransform(editor.g.getScreenCTM().inverse());
  return [p.x, p.y];
}
function nearestIndex(pts, q) {
  let best = 0, bd = Infinity;
  pts.forEach((p, i) => { const dd = dist(p, q); if (dd < bd) { bd = dd; best = i; } });
  return best;
}
// Start from whichever end of the piece the pointer is nearer to.
function placementFor(id, q) {
  const pts = app.pieces.get(id);
  return dist(q, pts[pts.length - 1]) < dist(q, pts[0]);
}

function clearHover() {
  if (!editor) return;
  editor.hoverLayer.replaceChildren();
  editor.g.querySelectorAll("[data-vis].hl").forEach(n => {
    n.classList.remove("hl"); n.setAttribute("stroke-opacity", 0.6);
  });
}
function hover(id, e) {
  clearHover();
  const { hoverLayer: L, sw } = editor, q = toBoard(e), pts = app.pieces.get(id);
  const vis = editor.g.querySelector(`[data-vis="${id}"]`);
  vis.classList.add("hl"); vis.setAttribute("stroke-opacity", 1);
  el("polyline", { points: ptsAttr(pts), fill: "none", stroke: colorOf(id), "stroke-width": sw * 3,
    "stroke-linejoin": "round", "stroke-linecap": "round" }, L);

  if (app.tool === "place") {
    const rev = placementFor(id, q), start = rev ? pts[pts.length - 1] : pts[0];
    connector(L, app.manual.pen, start, sw);
    el("circle", { cx: start[0], cy: start[1], r: sw * 3, fill: colorOf(id), stroke: "var(--board)",
      "stroke-width": sw }, L);
    addArrows(L, rev ? [...pts].reverse() : pts, sw, "var(--accent)", arrowEvery(app.drawing.bounds));
  } else if (app.manual.cuttable.has(id)) {
    const p = pts[nearestIndex(pts, q)];
    const s = sw * 4;
    for (const [dx, dy] of [[1, 1], [1, -1]])
      el("line", { x1: p[0] - s * dx, y1: p[1] - s * dy, x2: p[0] + s * dx, y2: p[1] + s * dy,
        stroke: "var(--danger)", "stroke-width": sw * 1.5, "stroke-linecap": "round" }, L);
  }
}

// Flip tool: preview the reversed direction of a placed piece.
function hoverPlaced(id) {
  clearHover();
  const { hoverLayer: L, sw } = editor;
  const rev = app.manual.placed.find(([pid]) => pid === id)[1];
  const flipped = oriented(id, !rev);
  el("polyline", { points: ptsAttr(flipped), fill: "none", stroke: "var(--accent)", "stroke-width": sw * 3,
    "stroke-opacity": 0.35, "stroke-linejoin": "round", "stroke-linecap": "round" }, L);
  addArrows(L, flipped, sw, "var(--danger)", arrowEvery(app.drawing.bounds));
}

function manualAction(body) {
  return busy(async () => {
    applyManual(await api("POST", `/api/drawings/${app.drawing.drawing}/manual`, body));
    renderEditor();
  });
}
function clickPiece(id, e) {
  const q = toBoard(e);
  if (app.tool === "place") manualAction({ action: "place", piece: id, reverse: placementFor(id, q) });
  else if (app.manual.cuttable.has(id)) manualAction({ action: "cut", piece: id, point: q });
  else showError("That piece is too short to cut further.");
}

function setToolButtons() {
  for (const [id, tool] of [["toolPlace", "place"], ["toolCut", "cut"], ["toolFlip", "flip"]])
    $(id).setAttribute("aria-pressed", app.tool === tool);
}
function setTool(tool) {
  app.tool = tool;
  if (editor) { editor.svg.dataset.tool = tool; clearHover(); }
  setToolButtons();
}
$("toolPlace").onclick = () => setTool("place");
$("toolCut").onclick = () => setTool("cut");
$("toolFlip").onclick = () => setTool("flip");
// The auto order is exactly what Auto-finish produces from an empty manual order.
$("editAuto").onclick = () => busy(async () => {
  const url = `/api/drawings/${app.drawing.drawing}/manual`;
  await api("POST", url, { action: "reset" });
  applyManual(await api("POST", url, { action: "autofinish" }));
  selectTab("manual");
  setTool("flip");
  renderEditor();
});
$("undo").onclick = () => manualAction({ action: "undo" });
$("autofinish").onclick = () => manualAction({ action: "autofinish" });
$("reset").onclick = () => manualAction({ action: "reset" });

document.addEventListener("keydown", (e) => {
  if (app.mode !== "manual" || !app.manual) return;
  if (e.target.closest("input, select, textarea")) return;
  if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "z") {
    e.preventDefault(); if (app.manual.can_undo) manualAction({ action: "undo" });
  } else if (!e.ctrlKey && !e.metaKey && !e.altKey) {
    const k = e.key.toLowerCase();
    if (k === "p" || k === "escape") setTool("place");
    else if (k === "c") setTool("cut");
    else if (k === "f") setTool("flip");
  }
});

// ---------------------------------------------------------------- save
function updateSave() {
  const ready = app.drawing && (app.mode === "auto" || app.manual?.complete);
  $("save").disabled = app.busy || !ready;
  $("save").title = ready ? "" : "Place every piece first.";
}
$("save").onclick = () => busy(async () => {
  const meta = await api("POST", "/api/trajectories",
    { drawing: app.drawing.drawing, mode: app.mode, name: $("saveName").value });
  const box = $("saved");
  box.replaceChildren();
  const msg = document.createElement("span");
  msg.textContent = `Saved “${meta.name}” (${meta.piece_count} pieces).`;
  const link = document.createElement("a");
  link.href = `/preview?t=${meta.id}`;
  link.textContent = "Open in robot preview →";
  box.append(msg, link);
  box.hidden = false;
});

init();
