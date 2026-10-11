// A worker with a 4 ms timer that says its ticks after each, and, after
// its first, what its own navigator.serial is (Chrome's, not the board's
// line).
const ticks = [];
let serial = null;
setInterval(() => {
  ticks.push(performance.timeOrigin + performance.now());
  if (serial === null) serial = typeof navigator.serial;
  postMessage({ ticks, serial });
}, 4);
