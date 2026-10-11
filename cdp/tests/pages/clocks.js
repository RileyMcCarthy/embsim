// The worker canary: a main thread and a dedicated worker, each with a 4 ms
// timer stamping every tick with its clock. The worker writes its ticks into
// shared memory the page reads while its clock is held.
const MAX = 4096;
const shared = new SharedArrayBuffer(8 * (2 + MAX));
const workerTicks = new Float64Array(shared);
const mainTicks = [];
window.ready = false;
const worker = new Worker('clocks-worker.js');
worker.onmessage = () => {
  setInterval(() => {
    if (mainTicks.length < MAX) mainTicks.push(performance.timeOrigin + performance.now());
  }, 4);
  window.ready = true;
};
worker.postMessage(shared);
// What the test reads, the page's clock held.
window.clocks = () => ({
  now: performance.timeOrigin + performance.now(),
  date: Date.now(),
  main: mainTicks.slice(),
  worker: Array.from(workerTicks.subarray(2, 2 + workerTicks[0])),
});
