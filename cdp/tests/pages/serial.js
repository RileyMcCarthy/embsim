// MaD Control's shape: the port opened on the main thread, its streams
// transferred to a dedicated worker that owns them, a WASM module there,
// and Comlink-style calls between the two. The test drives it with
// window.run(name, fn) and reads window.out[name] once it is done.
const BAUD = Number(new URLSearchParams(location.search).get('baud') || 2000000);
const worker = new Worker('serial-worker.js');
let nextId = 1;
const calls = new Map();
worker.onmessage = (e) => {
  const { id, ok, value, error } = e.data;
  const call = calls.get(id);
  if (!call) return;
  calls.delete(id);
  if (ok) call.resolve(value);
  else call.reject(new Error(error));
};
const rpc = (method, args = [], transfer = []) => new Promise((resolve, reject) => {
  const id = nextId++;
  calls.set(id, { resolve, reject });
  worker.postMessage({ id, method, args }, transfer);
});
const caught = (e) => ({ name: e.name, message: e.message });

const ids = new Map();
const idOf = (port) => {
  if (!ids.has(port)) ids.set(port, ids.size + 1);
  return ids.get(port);
};
window.events = [];
navigator.serial.addEventListener('connect', (e) => window.events.push({ type: 'connect', port: idOf(e.target) }));
navigator.serial.addEventListener('disconnect', (e) => window.events.push({ type: 'disconnect', port: idOf(e.target) }));

window.out = {};
window.run = (name, fn) => {
  window.out[name] = { done: false };
  Promise.resolve().then(fn).then(
    (value) => { window.out[name] = { done: true, value }; },
    (e) => { window.out[name] = { done: true, error: caught(e) }; },
  );
  return true;
};

const ports = [];
window.t = {
  // MaD's connect: a granted port, opened here, its streams handed to the
  // worker.
  async connect(scenario) {
    const [port] = await navigator.serial.getPorts();
    if (!port) throw new Error('no port');
    ports.push(port);
    await port.open({ baudRate: BAUD });
    await rpc('attach', [port.readable, port.writable, scenario], [port.readable, port.writable]);
    return idOf(port);
  },
  stats: () => rpc('stats'),
  quiet: () => rpc('quiet'),
  bulk: () => rpc('bulk'),
  // close() while this realm still holds the readable locked.
  async closeWhileLocked() {
    const [port] = await navigator.serial.getPorts();
    ports.push(port);
    await port.open({ baudRate: BAUD });
    const reader = port.readable.getReader();
    const close = await port.close().then(() => 'resolved', caught);
    const reopen = await port.open({ baudRate: BAUD }).then(() => 'resolved', caught);
    reader.releaseLock();
    const closeAfter = await port.close().then(() => 'resolved', caught);
    return { close, reopen, closeAfter, readable: port.readable };
  },
  requestWithoutGesture: () => navigator.serial.requestPort().then(() => 'resolved', caught),
  // A port open with a read pending, for the cable to be pulled under it.
  async openAndRead() {
    const [port] = await navigator.serial.getPorts();
    ports.push(port);
    await port.open({ baudRate: BAUD });
    const reader = port.readable.getReader();
    window.pendingRead = reader.read().then(() => 'resolved', caught);
    return idOf(port);
  },
  async afterReplug() {
    const read = await window.pendingRead;
    const a = ports[ports.length - 1];
    const [b] = await navigator.serial.getPorts();
    const reopenA = await a.open({ baudRate: BAUD }).then(() => 'resolved', caught);
    const openB = b ? await b.open({ baudRate: BAUD }).then(() => 'resolved', caught) : 'no port';
    return {
      read,
      aConnected: a.connected,
      a: idOf(a),
      b: b ? idOf(b) : null,
      same: b === a,
      reopenA,
      openB,
      events: window.events,
    };
  },
};
window.ready = true;
