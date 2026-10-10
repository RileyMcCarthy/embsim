// The worker canary: a main thread and a dedicated worker, each with a 4 ms
// timer, each stamping every tick with performance.now() and with a WASM
// module's imported clock. The worker writes its ticks into shared memory
// the page reads while its clock is held.
const MAX = 4096;
const shared = new SharedArrayBuffer(8 * (2 + 2 * MAX));
const workerTicks = new Float64Array(shared);
const main = { ticks: [], wasm: [] };
let wasm = null;
window.ready = false;
WebAssembly.instantiate(EMBSIM_TEST_WASM, { env: { now: () => performance.now() } }).then((r) => {
  wasm = r.instance.exports;
  const worker = new Worker('clocks-worker.js');
  worker.onmessage = () => {
    setInterval(() => {
      if (main.ticks.length < MAX) {
        main.ticks.push(performance.timeOrigin + performance.now());
        main.wasm.push(performance.timeOrigin + wasm.now());
      }
    }, 4);
    window.ready = true;
  };
  worker.postMessage(shared);
});
// What the test reads, the page's clock held.
window.clocks = () => ({
  now: performance.timeOrigin + performance.now(),
  wasm: wasm ? performance.timeOrigin + wasm.now() : null,
  date: Date.now(),
  main: main.ticks.slice(),
  mainWasm: main.wasm.slice(),
  worker: Array.from(workerTicks.subarray(2, 2 + workerTicks[0])),
  workerWasm: Array.from(workerTicks.subarray(2 + MAX, 2 + MAX + workerTicks[0])),
});
