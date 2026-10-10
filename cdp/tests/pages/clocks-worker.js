// The canary's worker: a 4 ms timer stamping every tick into shared memory
// the page reads while its clock is held.
const MAX = 4096;
onmessage = (e) => {
  const ticks = new Float64Array(e.data);
  setInterval(() => {
    const n = ticks[0];
    if (n < MAX) {
      ticks[2 + n] = performance.timeOrigin + performance.now();
      ticks[0] = n + 1;
    }
  }, 4);
  postMessage('ticking');
};
