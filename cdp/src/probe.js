// embsim chrome-cdp: a dedicated worker's read probe, for the drain barrier.
//
// Evaluated in each dedicated worker the node holds, through the worker's
// own session, before and (until it takes) after the worker is released.
// A stream the worker was sent in a message (a port's readable a page
// transferred to it) is marked as the message's data is read; every read
// call on a reader of a marked stream reports the worker's running count
// through the worker-session binding __embsimDrain: a consumer that asks
// for more has finished with what it was handed. A read of any other
// stream (a fetch's body, a file) is not counted. The worker's own
// navigator.serial is Chrome's, not the board's line: asking for it is
// reported once, as "serial". Returns 1 once installed, 0 while the
// worker's global scope is not ready yet (the node retries).
(() => {
  if (self.__embsimProbe) return 1;
  if (typeof ReadableStream !== 'function' || typeof MessageEvent !== 'function'
      || typeof __embsimDrain !== 'function') {
    return 0;
  }
  const report = __embsimDrain;
  const say = (m) => { try { report(m); } catch (_) { /* the binding went away */ } };
  let n = 0;
  const sent = new WeakSet();
  const mark = (v, depth) => {
    if (v === null || typeof v !== 'object' || depth > 6) return;
    if (v instanceof ReadableStream) { sent.add(v); return; }
    if (ArrayBuffer.isView(v) || v instanceof ArrayBuffer) return;
    if (Array.isArray(v)) {
      for (let i = 0; i < v.length && i < 64; i++) mark(v[i], depth + 1);
      return;
    }
    const proto = Object.getPrototypeOf(v);
    if (proto !== Object.prototype && proto !== null) return;
    let k = 0;
    for (const key in v) {
      if (k++ >= 64) break;
      mark(v[key], depth + 1);
    }
  };
  const data = Object.getOwnPropertyDescriptor(MessageEvent.prototype, 'data');
  if (data && typeof data.get === 'function') {
    Object.defineProperty(MessageEvent.prototype, 'data', {
      configurable: true,
      enumerable: data.enumerable,
      get() {
        const v = data.get.call(this);
        mark(v, 0);
        return v;
      },
    });
  }
  const counted = (read) => function (...a) {
    n++;
    say(String(n));
    return read.apply(this, a);
  };
  const RS = ReadableStream.prototype;
  const getReader = RS.getReader;
  RS.getReader = function (...a) {
    const reader = getReader.apply(this, a);
    if (sent.has(this)) reader.read = counted(reader.read);
    return reader;
  };
  if (typeof RS.values === 'function') {
    const values = RS.values;
    const iterate = function (...a) {
      const it = values.apply(this, a);
      if (sent.has(this)) it.next = counted(it.next);
      return it;
    };
    RS.values = iterate;
    RS[Symbol.asyncIterator] = iterate;
  }
  const WN = typeof WorkerNavigator === 'function' ? WorkerNavigator.prototype : null;
  const serial = WN && Object.getOwnPropertyDescriptor(WN, 'serial');
  if (serial && typeof serial.get === 'function') {
    let told = false;
    Object.defineProperty(WN, 'serial', {
      configurable: true,
      enumerable: serial.enumerable,
      get() {
        if (!told) { told = true; say('serial'); }
        return serial.get.call(this);
      },
    });
  }
  Object.defineProperty(self, '__embsimProbe', { value: true });
  return 1;
})()
