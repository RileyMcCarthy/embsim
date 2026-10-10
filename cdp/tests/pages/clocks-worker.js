importScripts('wasm.js');
const MAX = 4096;
onmessage = async (e) => {
  const ticks = new Float64Array(e.data);
  const { instance } = await WebAssembly.instantiate(EMBSIM_TEST_WASM, { env: { now: () => performance.now() } });
  setInterval(() => {
    const n = ticks[0];
    if (n < MAX) {
      ticks[2 + n] = performance.timeOrigin + performance.now();
      ticks[2 + MAX + n] = performance.timeOrigin + instance.exports.now();
      ticks[0] = n + 1;
    }
  }, 4);
  postMessage('ticking');
};
