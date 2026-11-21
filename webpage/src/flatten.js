// apm.pdef.json groups parameters by prefix; the page wants one sorted list without duplicates
export default function flatten(pdef) {
  const byName = new Map();
  for (const [group, params] of Object.entries(pdef)) {
    if (group === 'json') continue; // format version marker
    for (const [n, p] of Object.entries(params)) {
      byName.set(n, { n, d: p.Description || p.DisplayName || '', r: p.Range && [p.Range.low, p.Range.high], u: p.Units, v: p.Values, b: p.Bitmask });
    }
  }
  return [...byName.values()].sort((a, b) => (a.n > b.n) - (a.n < b.n));
}
