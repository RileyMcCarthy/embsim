// A worker that starts a worker, and passes its ticks on.
const inner = new Worker('inner-worker.js');
inner.onmessage = (e) => postMessage(e.data);
