import { expect, test } from 'bun:test';
import flatten from './flatten';

test('flatten skips the marker, dedupes by name and sorts', () => {
  const pdef = {
    json: { version: 0 },
    WP_: { WP_SPEED: { Description: 'a', Range: { low: '0', high: '1' } } },
    Rover: { WP_SPEED: { Description: 'b' }, ACRO_X: { DisplayName: 'c', Units: 'm' } },
  };
  expect(flatten(pdef)).toEqual([{ n: 'ACRO_X', d: 'c', u: 'm' }, { n: 'WP_SPEED', d: 'b' }]);
});
