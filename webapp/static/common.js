// Helpers shared by the generator and robot preview pages.
const $ = (id) => document.getElementById(id);
const SVGNS = "http://www.w3.org/2000/svg";

function el(tag, attrs = {}, parent) {
  const e = document.createElementNS(SVGNS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(e);
  return e;
}
const ptsAttr = (pts) => pts.map(p => p.join(",")).join(" ");
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);
function showError(msg, id = "error") { $(id).textContent = msg || ""; $(id).hidden = !msg; }
function fmtDuration(ms) {
  const s = Math.round(ms / 1000), m = Math.floor(s / 60);
  return m ? `${m} min ${s % 60} s` : `${s} s`;
}
const fmtNum = (v) => Number(v).toLocaleString(undefined, { maximumFractionDigits: 2 });
function entitySummary(c) {
  const parts = [["lines", "line"], ["circles", "circle"], ["arcs", "arc"], ["polylines", "polyline"],
                 ["splines", "spline"], ["ellipses", "ellipse"]]
    .filter(([k]) => c[k]).map(([k, one]) => `${c[k]} ${c[k] === 1 ? one : k}`);
  return parts.join(", ") || "—";
}
function setStats(rows) {
  $("stats").innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join("");
  $("stats").hidden = !rows.length;
}
function travelLength(start, orderedPieces) {
  let total = 0, prev = start;
  for (const p of orderedPieces) { total += dist(prev, p[0]); prev = p[p.length - 1]; }
  return total;
}

async function api(method, url, body) {
  const opts = { method };
  if (body instanceof FormData) opts.body = body;
  else if (body !== undefined) {
    opts.body = JSON.stringify(body);
    opts.headers = { "Content-Type": "application/json" };
  }
  const res = await fetch(url, opts);
  const data = await res.json().catch(() => ({}));
  if (!res.ok) {
    const d = data.detail;
    throw new Error(typeof d === "string" ? d : Array.isArray(d) ? d.map(x => x.msg).join("; ") : "Request failed.");
  }
  return data;
}

// Saved trajectories (robot preview and live pages).
const MODE_LABELS = { auto: "Auto (nearest neighbour)", manual: "Manual", imported: "Imported points" };
function fillTrajectorySelect(sel, list, selectId) {
  sel.replaceChildren(...list.map(t => {
    const o = document.createElement("option");
    o.value = t.id;
    o.textContent = `${t.name} — ${new Date(t.created * 1000).toLocaleString()}`;
    return o;
  }));
  if (selectId && list.some(t => t.id === selectId)) sel.value = selectId;
}
async function fillRobotSelect(sel, selectName) {
  const robots = await api("GET", "/api/robots");
  sel.innerHTML = robots.map(r => `<option>${r.name}</option>`).join("");
  if (robots.some(r => r.name === selectName)) sel.value = selectName;
}

// ---------------------------------------------------------------- drawing
// A y-up view of `bounds` [x0, y0, x1, y1] (any units). Content goes in `g`,
// which is flipped; text must go on `svg` at (x, -y) so it isn't mirrored.
function makeView(bounds) {
  const [x0, y0, x1, y1] = bounds;
  const span = Math.max(x1 - x0, y1 - y0) || 1, pad = span * 0.04;
  const svg = el("svg", { viewBox: `${x0 - pad} ${-y1 - pad} ${x1 - x0 + 2 * pad} ${y1 - y0 + 2 * pad}` });
  const g = el("g", { transform: "scale(1 -1)" }, svg);
  return { svg, g, sw: span / 400 };
}
function dashedRect(g, [x0, y0, x1, y1], sw) {
  el("rect", { x: x0, y: y0, width: x1 - x0, height: y1 - y0, fill: "var(--board)", stroke: "var(--board-edge)",
    "stroke-width": sw, "stroke-dasharray": `${sw * 4} ${sw * 3}` }, g);
}
function addText(svg, [x, y], text, sw, attrs = {}) {
  const t = el("text", { x, y: -y, "font-size": sw * 14, "pointer-events": "none", fill: "var(--text)", ...attrs }, svg);
  t.textContent = text;
  return t;
}
function addHome(svg, xy, sw) {
  addText(svg, xy, "★", sw, { "text-anchor": "middle", "dominant-baseline": "central" });
}
function connector(g, a, b, sw) {
  return el("line", { x1: a[0], y1: a[1], x2: b[0], y2: b[1], stroke: "var(--connector)",
    "stroke-width": sw * 1.2, "stroke-dasharray": `${sw * 3} ${sw * 2}`, "pointer-events": "none" }, g);
}
// Chevrons spaced along a polyline, pointing the way it is drawn. `every` is
// the target distance between chevrons in view units (at least one is drawn).
function addArrows(g, pts, sw, color, every) {
  const cum = [0];
  for (let i = 1; i < pts.length; i++) cum.push(cum[i - 1] + dist(pts[i - 1], pts[i]));
  const L = cum[cum.length - 1];
  if (!(L > 0)) return;
  const count = Math.max(1, Math.min(12, Math.round(L / every)));
  const s = sw * 3.2, layer = el("g", { "pointer-events": "none" }, g);
  let j = 1;
  for (let a = 0; a < count; a++) {
    const target = L * (a + 0.5) / count;
    while (j < pts.length - 1 && cum[j] < target) j++;
    // Direction from a short stretch around the chevron (single samples can be tiny).
    const lo = Math.max(0, j - 3), hi = Math.min(pts.length - 1, j + 2);
    const f = (target - cum[j - 1]) / ((cum[j] - cum[j - 1]) || 1);
    const x = pts[j - 1][0] + f * (pts[j][0] - pts[j - 1][0]), y = pts[j - 1][1] + f * (pts[j][1] - pts[j - 1][1]);
    const deg = Math.atan2(pts[hi][1] - pts[lo][1], pts[hi][0] - pts[lo][0]) * 180 / Math.PI;
    el("path", { d: `M ${-s} ${-s * 0.9} L 0 0 L ${-s} ${s * 0.9}`, transform: `translate(${x} ${y}) rotate(${deg})`,
      fill: "none", stroke: color, "stroke-width": sw * 1.3, "stroke-linecap": "round", "stroke-linejoin": "round" }, layer);
  }
}
function pieceColor(id, roots) {
  const root = roots.get(id) ?? id;
  const hue = (root * 137.508) % 360;
  // Cut halves keep their parent's hue; alternate lightness so neighbours differ.
  const shift = id === root ? "" : (id % 2 ? " - 10%" : " + 8%");
  return `hsl(${hue.toFixed(1)} 68% calc(var(--piece-l)${shift}))`;
}

// Fixed bases plus a linkage that can be re-posed. A pose is a flat
// [x1, y1, x2, y2, ...] list, one quadruple per link (board frame).
function makeRobot(g, rg, sw) {
  const layer = el("g", { class: "robot", "pointer-events": "none" }, g);
  const cable = rg.kind === "cable";
  const linkLayer = el("g", {}, layer), jointLayer = el("g", {}, layer);
  const n = ((rg.home || []).length / 4) | 0;
  const links = [], joints = [];
  for (let i = 0; i < n; i++) {
    links.push(el("line", { stroke: "var(--link)", "stroke-width": sw * (cable ? 1 : 5),
      "stroke-linecap": "round", "stroke-opacity": cable ? 0.9 : 0.55 }, linkLayer));
    if (!cable) for (let k = 0; k < 2; k++)
      joints.push(el("circle", { r: sw * 2.2, fill: "var(--board)", stroke: "var(--link)",
        "stroke-width": sw }, jointLayer));
  }
  for (const [x, y] of rg.bases) {
    const s = sw * 6;
    el("rect", { x: x - s, y: y - s, width: 2 * s, height: 2 * s, rx: sw * 1.5, fill: "var(--base)" }, layer);
    el("circle", { cx: x, cy: y, r: sw * 2, fill: "var(--board)" }, layer);
  }
  function set(pose) {
    linkLayer.setAttribute("visibility", pose ? "visible" : "hidden");
    jointLayer.setAttribute("visibility", pose ? "visible" : "hidden");
    if (!pose) return;
    links.forEach((ln, i) => {
      const [a, b, c, d] = pose.slice(4 * i, 4 * i + 4);
      ln.setAttribute("x1", a); ln.setAttribute("y1", b); ln.setAttribute("x2", c); ln.setAttribute("y2", d);
      if (!cable) {
        joints[2 * i].setAttribute("cx", a); joints[2 * i].setAttribute("cy", b);
        joints[2 * i + 1].setAttribute("cx", c); joints[2 * i + 1].setAttribute("cy", d);
      }
    });
  }
  set(rg.home);
  return { set };
}
