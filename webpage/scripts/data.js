// Copy every ../<Vehicle>-<version>/apm.pdef.json into static/data and list them in index.json
import { copyFileSync, mkdirSync, readdirSync, writeFileSync } from 'fs';

const out = 'static/data';
mkdirSync(out, { recursive: true });
const index = {};
for (const dir of readdirSync('..').filter(d => /^\w+-[\d.]+$/.test(d))) {
  const [vehicle, version] = dir.split('-');
  copyFileSync(`../${dir}/apm.pdef.json`, `${out}/${dir}.json`);
  (index[vehicle] ??= []).push(version);
}
for (const versions of Object.values(index)) versions.sort((a, b) => b.localeCompare(a, undefined, { numeric: true }));
// Copter first so it is the default vehicle
const vehicles = Object.entries(index).sort(([a], [b]) => (a !== 'Copter') - (b !== 'Copter') || a.localeCompare(b));
writeFileSync(`${out}/index.json`, JSON.stringify(Object.fromEntries(vehicles)));
