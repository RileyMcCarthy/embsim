// A page with a 4 ms timer, and, asked for one in its query, a dedicated
// worker with its own (?worker), or a worker that starts a worker with one
// (?nested). The test reads the ticks while the page's clock is held.
const query = new URLSearchParams(location.search);
window.ticks = [];
setInterval(() => window.ticks.push(performance.now()), 4);
// The worker's last word: its ticks (its own clock) and what its
// navigator.serial was.
window.worker = { ticks: [], serial: null };
const kind = query.has('nested') ? 'nested' : query.has('worker') ? 'worker' : null;
if (kind) {
  const worker = new Worker(kind === 'nested' ? 'outer-worker.js' : 'inner-worker.js');
  worker.onmessage = (e) => { window.worker = e.data; };
}
// connect and disconnect for any port this origin may use, asked for or
// not.
window.events = [];
for (const type of ['connect', 'disconnect']) {
  navigator.serial.addEventListener(type, () => window.events.push(type));
}
window.out = {};
window.run = (name, fn) => {
  window.out[name] = { done: false };
  Promise.resolve().then(fn).then(
    (value) => { window.out[name] = { done: true, value }; },
    (e) => { window.out[name] = { done: true, error: { name: e.name, message: e.message } }; },
  );
  return true;
};
window.ready = true;
