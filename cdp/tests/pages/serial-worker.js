// The worker that owns the port's streams, as MaD's DeviceSession worker
// does: it reads the board's frames through a WASM module and writes
// requests on its own timer, each with a timeout in its own clock.
importScripts('wasm.js');
let wasm = null;
const ready = WebAssembly.instantiate(EMBSIM_TEST_WASM, { env: { now: () => performance.now() } })
  .then((r) => { wasm = r.instance.exports; });
const st = {
  bytes: 0, sum: 0, samples: 0, bad: 0, requests: 0, replies: 0, timeouts: 0,
  latencies: [], echoed: 0, echoOk: null, readErrors: [], wasmLag: 0,
};
let reader = null;
let writer = null;
let timer = null;
let bulk = false;
const pending = new Map();

const methods = {
  async attach(readable, writable, scenario) {
    await ready;
    reader = readable.getReader();
    writer = writable.getWriter();
    readLoop();
    if (scenario === 'mad') timer = setInterval(request, 25);
    if (scenario === 'echo') echo();
    return true;
  },
  // Stop asking; read everything as a bulk stream from here.
  quiet() {
    clearInterval(timer);
    return true;
  },
  bulk() {
    bulk = true;
    st.bulkBytes = 0;
    st.bulkSum = 0;
    return true;
  },
  stats() {
    const lat = st.latencies.slice().sort((x, y) => x - y);
    const q = (p) => (lat.length ? lat[Math.min(lat.length - 1, Math.floor(p * lat.length))] : null);
    return Object.assign({}, st, { latencies: undefined, latency: { n: lat.length, p50: q(0.5), p99: q(0.99), max: q(1) } });
  },
};
onmessage = async (e) => {
  const { id, method, args } = e.data;
  try {
    postMessage({ id, ok: true, value: await methods[method](...args) });
  } catch (err) {
    postMessage({ id, ok: false, error: String(err) });
  }
};

// The board's frames: a sample (0x55 0x02 seq, 16 bytes, xor) every 10 ms
// of board time, and a reply (0x55 0x01 seq, 5 bytes) to each request.
let buf = [];
function consume(bytes) {
  for (const b of bytes) {
    st.sum = wasm.mix(st.sum, b) >>> 0;
    buf.push(b);
  }
  st.bytes += bytes.length;
  st.wasmLag = Math.max(st.wasmLag, Math.abs(wasm.now() - performance.now()));
  for (;;) {
    const i = buf.indexOf(0x55);
    if (i < 0) { buf = []; return; }
    if (i > 0) buf.splice(0, i);
    if (buf.length < 2) return;
    const type = buf[1];
    const need = type === 0x02 ? 20 : type === 0x01 ? 8 : 0;
    if (need === 0) { buf.shift(); continue; }
    if (buf.length < need) return;
    const frame = buf.splice(0, need);
    if (type === 0x02) {
      let x = 0;
      for (let k = 0; k < 19; k++) x ^= frame[k];
      if (x === frame[19]) st.samples++;
      else st.bad++;
    } else {
      const p = pending.get(frame[2]);
      if (p) {
        clearTimeout(p.timer);
        pending.delete(frame[2]);
        st.replies++;
        st.latencies.push(performance.now() - p.at);
      }
    }
  }
}

let seq = 0;
function request() {
  seq = (seq + 1) & 0xff;
  const s = seq;
  st.requests++;
  pending.set(s, {
    at: performance.now(),
    timer: setTimeout(() => { if (pending.delete(s)) st.timeouts++; }, 200),
  });
  writer.write(new Uint8Array([0x55, 0x00, s]));
}

let expect = null;
async function echo() {
  expect = new Uint8Array(4096);
  for (let i = 0; i < expect.length; i++) expect[i] = (i * 7 + 3) & 0xff;
  await writer.write(expect);
}

async function readLoop() {
  for (;;) {
    let r;
    try {
      r = await reader.read();
    } catch (e) {
      st.readErrors.push({ name: e.name, message: e.message });
      return;
    }
    if (r.done) return;
    if (bulk) {
      for (const b of r.value) st.bulkSum = wasm.mix(st.bulkSum, b) >>> 0;
      st.bulkBytes += r.value.length;
    } else if (expect) {
      for (const b of r.value) {
        if (b !== expect[st.echoed]) st.echoOk = false;
        st.echoed++;
      }
      st.bytes += r.value.length;
      if (st.echoed === expect.length && st.echoOk !== false) st.echoOk = true;
    } else {
      consume(r.value);
    }
  }
}
