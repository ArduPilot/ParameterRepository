import MiniSearch from 'minisearch';
import flatten from './flatten';

const idx = JSON.parse(document.getElementById('idx').textContent);
const $ = id => document.getElementById(id);
const table = $('params'), tbody = table.querySelector('tbody');
const cache = new Map();
let rows = [], ms, hits = [], shown = 0, group = '', gen = 0;

const fmtMap = m => Object.entries(m).map(([k, v]) => `${k}: ${v}`).join(', ');
const esc = t => t.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');

// text with every matched term wrapped in <mark>
function marked(text, terms) {
  const frag = document.createDocumentFragment();
  let last = 0;
  for (const m of terms.length ? text.matchAll(new RegExp(terms.map(esc).join('|'), 'gi')) : []) {
    frag.append(text.slice(last, m.index), Object.assign(document.createElement('mark'), { textContent: m[0] }));
    last = m.index + m[0].length;
  }
  frag.append(text.slice(last));
  return frag;
}

function td(text, cls, terms = []) {
  const e = document.createElement('td');
  e.append(marked(text, terms));
  if (cls) e.className = cls;
  return e;
}

function load(file) {
  if (!cache.has(file)) cache.set(file, fetch(`${table.dataset.base}/${file}.json`).then(r => {
    if (!r.ok) throw new Error(`${file}: ${r.status}`);
    return r.json();
  }).then(flatten).catch(e => { cache.delete(file); throw e; })); // let the next selection retry
  return cache.get(file);
}

function fillVersions() {
  $('version').innerHTML = idx[$('vehicle').value].map((v, i) => `<option value="${v}">${v}${i ? '' : ' (latest)'}</option>`).join('');
}

function fillChips() {
  const groups = new Set(rows.map(r => r.n.split('_')[0]));
  $('chips').innerHTML = [...groups].sort().map(g => `<button aria-pressed="${g === group}" data-g="${g}">${g}</button>`).join('');
}

function search() {
  if (!ms) return; // typed before the first load, reload() searches when done
  const q = $('q').value.trim();
  // every term must match; fuzzy only as a fallback since results are alphabetical, not ranked
  hits = q ? ms.search(q, { prefix: true, combineWith: 'AND' }) : rows.slice();
  if (q && !hits.length) hits = ms.search(q, { prefix: true, combineWith: 'AND', fuzzy: 0.2 });
  hits.sort((a, b) => (a.n > b.n) - (a.n < b.n)); // codepoint order, same as the JSON
  const i = hits.findIndex(h => h.n === q.toUpperCase().replace(/\s+/g, '_'));
  if (i > 0) hits.unshift(...hits.splice(i, 1));
  if (group) hits = hits.filter(h => h.n.split('_')[0] === group);
  $('count').textContent = `${hits.length} parameters`;
  tbody.textContent = ''; shown = 0; more();
  history.replaceState(null, '', '#' + new URLSearchParams({ v: $('vehicle').value, r: $('version').value, q, g: group }));
}

function more() {
  const frag = document.createDocumentFragment();
  for (const h of hits.slice(shown, shown + 300)) {
    const tr = document.createElement('tr');
    tr.className = 'p';
    const terms = h.match ? Object.keys(h.match) : [];
    tr.append(td(h.n, 'n', terms), td(h.d, null, terms), td(h.r ? h.r.join(' to ') : ''), td(h.u || ''));
    tr.onclick = () => {
      const next = tr.nextElementSibling;
      if (next?.classList.contains('x')) return next.remove();
      const x = document.createElement('tr'); x.className = 'x';
      const cell = td([h.v && 'Values: ' + fmtMap(h.v), h.b && 'Bitmask: ' + fmtMap(h.b)].filter(Boolean).join('\n') || 'No values or bitmask');
      cell.colSpan = 4; x.append(cell); tr.after(x);
    };
    frag.append(tr);
  }
  shown += 300; tbody.append(frag); $('more').hidden = shown >= hits.length;
}

async function reload() {
  const file = `${$('vehicle').value}-${$('version').value}`, my = ++gen;
  $('count').textContent = 'Loading…';
  table.setAttribute('aria-busy', 'true');
  try {
    rows = await load(file);
  } catch (e) {
    if (my === gen) $('count').textContent = `Failed to load ${file}`;
    throw e;
  } finally {
    if (my === gen) table.removeAttribute('aria-busy');
  }
  if (my !== gen) return; // a newer selection won
  ms = new MiniSearch({ fields: ['n', 'd'], storeFields: ['n', 'd', 'r', 'u', 'v', 'b'], idField: 'n' });
  ms.addAll(rows);
  fillChips(); search();
}

$('vehicle').onchange = () => { fillVersions(); reload(); };
$('version').onchange = reload;
$('q').oninput = search;
$('more').onclick = more;
$('chips').onclick = e => {
  const g = e.target.dataset.g;
  if (g === undefined) return;
  group = group === g ? '' : g;
  fillChips(); search();
};

const h = new URLSearchParams(location.hash.slice(1));
if (Object.hasOwn(idx, h.get('v'))) $('vehicle').value = h.get('v');
fillVersions();
$('version').value = h.get('r');
if (!$('version').value) $('version').selectedIndex = 0;
$('q').value = h.get('q') || '';
group = h.get('g') || '';
reload();
