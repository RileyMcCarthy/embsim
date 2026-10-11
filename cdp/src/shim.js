// embsim chrome-cdp: the page's Web Serial, whose far end is the board's line.
//
// Installed by the node in every document of every page it holds, before
// the page's own scripts run (Page.addScriptToEvaluateOnNewDocument), in
// the main world. It replaces navigator.serial. Everything that crosses to
// the device goes through the node's binding (__embsimTx) and is answered
// at the node's next slice through __embsim.slice(), as Chrome's own
// SerialPort answers through an IPC round trip to the browser process. The
// rules are Chrome's (third_party/blink/renderer/modules/serial/): the
// messages, which call rejects with what, when a stream is released, what
// a disconnect does (bytes already in the port's pipe are still read), and
// that a write resolves once its last byte is in the port's buffer. Each
// slice it tells the node the document's clock, whether the page is
// hidden, and whether the port's readable went to a worker; its hello says
// whether the document had run scripts before the shim was installed.
(() => {
  'use strict';
  // The top frame's document only: a frame's Web Serial is its own (and
  // needs the `serial` permissions policy besides).
  if (globalThis.__embsim || window.top !== window) return;
  const CFG = __EMBSIM_CONFIG__;
  const BINDING = '__embsimTx';
  const send = (m) => {
    const f = globalThis[BINDING];
    if (typeof f === 'function') f(JSON.stringify(m));
  };
  const doc = (globalThis.crypto && crypto.randomUUID)
    ? crypto.randomUUID()
    : String(Math.random()).slice(2) + String(performance.timeOrigin);
  const origin = String(location.origin);
  const href = String(location.href);
  // Installed in a document that has run scripts already: the page was
  // not held from birth (a page made with a URL, or one open before the
  // node attached), and its scripts saw Chrome's own Web Serial.
  const blank = href === 'about:blank' || href === 'about:srcdoc';
  const late = !blank && (document.readyState !== 'loading' || document.scripts.length > 0);

  // Chrome's messages (serial_port.cc, serial.cc, the streams).
  const kPortClosed = 'The port is closed.';
  const kOpenError = 'Failed to open serial port.';
  const kDeviceLost = 'The device has been lost.';
  const kNoSignals = 'Signals dictionary must contain at least one member.';
  const kMaxBufferSize = 16 * 1024 * 1024;
  const err = (name, message) =>
    name === 'TypeError' ? new TypeError(message) : new DOMException(message, name);
  // A synchronous refusal, as Blink's bindings word it: the method named.
  const refuse = (op, iface, name, message) =>
    err(name, `Failed to execute '${op}' on '${iface}': ${message}`);

  // What the node last said: the device's generation (a new one at each
  // plug), whether it is plugged, whether this origin may use it without a
  // prompt, and the writer's window.
  let gen = -1;
  let plugged = false;
  let granted = false; // as of the last slice
  let windowBytes = 255;

  // Requests answered at the next slice.
  const pending = new Map();
  let nextId = 1;
  const request = (op, data, owner) => new Promise((resolve, reject) => {
    const id = nextId++;
    pending.set(id, { resolve, reject, owner });
    send(Object.assign({ k: 'req', doc, id, op }, data || {}));
  });
  const settle = (r) => {
    const p = pending.get(r.id);
    if (!p) return;
    pending.delete(r.id);
    if (r.ok) p.resolve(r.v);
    else p.reject(err(r.name, r.message));
  };
  // Every request a port still waits on, rejected (a device that went away).
  const dropRequests = (owner, error) => {
    for (const [id, p] of pending) {
      if (p.owner === owner) {
        pending.delete(id);
        p.reject(error);
      }
    }
  };

  const b64 = (u8) => {
    let s = '';
    for (let i = 0; i < u8.length; i += 0x8000) {
      s += String.fromCharCode.apply(null, u8.subarray(i, i + 0x8000));
    }
    return btoa(s);
  };
  const unb64 = (s) => {
    const t = atob(s);
    const u = new Uint8Array(t.length);
    for (let i = 0; i < t.length; i++) u[i] = t.charCodeAt(i);
    return u;
  };

  // A port's readable this document transferred to another realm (a
  // dedicated worker): the node's drain barrier waits for that worker to
  // read what it is handed, which Chrome's clock does not wait for. A
  // consumer in this realm takes its bytes within the slice's evaluate.
  const portStreams = new WeakSet();
  const transferred = new WeakSet();
  const noteTransfer = (args) => {
    for (const a of args) {
      const list = Array.isArray(a) ? a
        : (a && typeof a === 'object' && Array.isArray(a.transfer)) ? a.transfer : null;
      if (!list) continue;
      for (const x of list) if (portStreams.has(x)) transferred.add(x);
    }
  };
  for (const C of [globalThis.Worker, globalThis.MessagePort]) {
    if (typeof C !== 'function' || typeof C.prototype.postMessage !== 'function') continue;
    const post = C.prototype.postMessage;
    Object.defineProperty(C.prototype, 'postMessage', {
      configurable: true,
      enumerable: true,
      writable: true,
      value: function postMessage(...a) {
        noteTransfer(a.slice(1));
        return post.apply(this, a);
      },
    });
  }

  // An EventTarget whose on<event> attributes are listeners, as an IDL
  // event handler attribute is.
  const handlerAttr = (proto, type) => {
    const slot = Symbol(type);
    Object.defineProperty(proto, 'on' + type, {
      configurable: true,
      enumerable: true,
      get() { return this[slot] ? this[slot].fn : null; },
      set(fn) {
        if (!this[slot]) {
          const holder = { fn: null };
          this[slot] = holder;
          this.addEventListener(type, (e) => {
            if (typeof holder.fn === 'function') holder.fn.call(this, e);
          });
        }
        this[slot].fn = typeof fn === 'function' ? fn : null;
      },
    });
  };

  // ── SerialPort ────────────────────────────────────────────────────────
  // The one open port in this document, if any.
  let openPort = null;
  const portsByGen = new Map();

  class SerialPort extends EventTarget {
    #gen;
    #state = 'closed'; // 'opening' | 'open' | 'closing'
    #readable = null;
    #source = null;
    #writable = null;
    #sink = null;
    #bufferSize = 255;
    #closeResolve = null;
    #closeReject = null;

    constructor(token) {
      if (token === undefined || typeof token !== 'object' || !('gen' in token)) {
        throw new TypeError('Illegal constructor');
      }
      super();
      this.#gen = token.gen;
    }

    get connected() { return plugged && gen === this.#gen; }

    getInfo() {
      const info = {};
      if (CFG.usbVendorId !== null) info.usbVendorId = CFG.usbVendorId;
      if (CFG.usbProductId !== null) info.usbProductId = CFG.usbProductId;
      return info;
    }

    open(options) {
      let o;
      try {
        o = convertOptions(options);
        if (this.#state === 'opening') {
          throw refuse('open', 'SerialPort', 'InvalidStateError', 'A call to open() is already in progress.');
        }
        if (this.#state === 'open' || this.#state === 'closing') {
          throw refuse('open', 'SerialPort', 'InvalidStateError', 'The port is already open.');
        }
        if (o.baudRate === 0) throw refuse('open', 'SerialPort', 'TypeError', 'Requested baud rate must be greater than zero.');
        if (o.dataBits !== 7 && o.dataBits !== 8) throw refuse('open', 'SerialPort', 'TypeError', 'Requested number of data bits must be 7 or 8.');
        if (o.stopBits !== 1 && o.stopBits !== 2) throw refuse('open', 'SerialPort', 'TypeError', 'Requested number of stop bits must be 1 or 2.');
        if (o.bufferSize === 0) {
          throw refuse('open', 'SerialPort', 'TypeError', `Requested buffer size (${o.bufferSize} bytes) must be greater than zero.`);
        }
        if (o.bufferSize > kMaxBufferSize) {
          throw refuse('open', 'SerialPort', 'TypeError', `Requested buffer size (${o.bufferSize} bytes) is greater than the maximum allowed (${kMaxBufferSize} bytes).`);
        }
      } catch (e) {
        return Promise.reject(e);
      }
      this.#state = 'opening';
      return request('open', Object.assign({ gen: this.#gen }, o), this).then(() => {
        this.#state = 'open';
        this.#bufferSize = o.bufferSize;
        openPort = this;
      }, (e) => {
        this.#state = 'closed';
        throw e;
      });
    }

    get readable() {
      if (this.#readable) return this.#readable;
      if (this.#state !== 'open') return null;
      const port = this;
      const bufferSize = this.#bufferSize;
      const source = {
        pipe: [], // the data pipe: bytes the node delivered, not yet read
        length: 0,
        controller: null,
        pull: null,
        closed: false,
        // The device went away with bytes still in the pipe: they are read
        // first, then the stream errors with this (Chrome's
        // SignalErrorOnClose).
        lost: null,
        chunks: 0,
        take(n) {
          const out = new Uint8Array(n);
          let at = 0;
          while (at < n) {
            const head = source.pipe[0];
            const k = Math.min(head.length, n - at);
            out.set(head.subarray(0, k), at);
            at += k;
            if (k === head.length) source.pipe.shift();
            else source.pipe[0] = head.subarray(k);
          }
          source.length -= n;
          return out;
        },
        // Hand a waiting read what the pipe holds, at most a pipe's worth;
        // once a lost device's pipe is empty, error the stream.
        pump() {
          if (!source.pull || source.closed) return;
          let delivered = false;
          if (source.length > 0) {
            delivered = true;
            const byob = source.controller.byobRequest;
            if (byob) {
              const n = Math.min(byob.view.byteLength, source.length, bufferSize);
              new Uint8Array(byob.view.buffer, byob.view.byteOffset, n).set(source.take(n));
              byob.respond(n);
            } else {
              source.controller.enqueue(source.take(Math.min(source.length, bufferSize)));
            }
            source.chunks++;
          }
          if (source.length === 0 && source.lost) {
            const error = source.lost;
            source.lost = null;
            source.closed = true;
            try { source.controller.error(error); } catch (_) { /* already closed */ }
          }
          if (delivered || source.closed) {
            const resolve = source.pull;
            source.pull = null;
            resolve();
          }
        },
        // The node's bytes for this slice: how many reads they take.
        push(bytes) {
          if (source.closed) return 0;
          source.pipe.push(bytes);
          source.length += bytes.length;
          const reads = Math.ceil(source.length / bufferSize);
          source.pump();
          return reads;
        },
        // The device went away: what the pipe holds is still read, then
        // the stream errors.
        lose(error) {
          if (source.closed) return;
          if (source.length > 0) {
            source.lost = error;
            source.pump();
            return;
          }
          source.closed = true;
          try { source.controller.error(error); } catch (_) { /* already closed */ }
          if (source.pull) { const r = source.pull; source.pull = null; r(); }
        },
      };
      const stream = new ReadableStream({
        type: 'bytes',
        start(c) { source.controller = c; },
        pull() {
          return new Promise((resolve) => {
            source.pull = resolve;
            source.pump();
          });
        },
        cancel() {
          source.closed = true;
          source.pipe = [];
          source.length = 0;
          // A port that is closing flushes when it closes.
          if (port.#state === 'closing') {
            port.#sourceClosed();
            return undefined;
          }
          return request('flush', { dir: 'rx' }, port).then(() => port.#sourceClosed());
        },
      });
      portStreams.add(stream);
      this.#readable = stream;
      this.#source = source;
      send({ k: 'reading', doc });
      return stream;
    }

    get writable() {
      if (this.#writable) return this.#writable;
      if (this.#state !== 'open') return null;
      const port = this;
      const sink = {
        controller: null,
        // The write in progress: what of its chunk the node's window has
        // not taken yet.
        waiting: null,
        unsent: 0, // bytes sent since the node last said its backlog
        backlog: 0,
        closed: false,
        // Send what the window has room for; resolve the write once its
        // last byte is in, as Chrome resolves a write once its chunk is in
        // the port's data pipe.
        feed() {
          const w = sink.waiting;
          if (!w) return;
          const room = windowBytes - (sink.backlog + sink.unsent);
          if (room > 0) {
            const part = w.rest.subarray(0, room);
            w.rest = w.rest.subarray(part.length);
            send({ k: 'tx', doc, b: b64(part) });
            sink.unsent += part.length;
          }
          if (w.rest.length === 0) {
            sink.waiting = null;
            w.resolve();
          }
        },
        lose(error) {
          sink.closed = true;
          if (sink.waiting) {
            const w = sink.waiting;
            sink.waiting = null;
            w.reject(error);
          } else {
            try { sink.controller.error(error); } catch (_) { /* already errored */ }
          }
        },
      };
      const stream = new WritableStream({
        start(c) {
          sink.controller = c;
          c.signal.addEventListener('abort', () => {
            if (sink.waiting) {
              const w = sink.waiting;
              sink.waiting = null;
              w.reject(c.signal.reason);
            }
          });
        },
        write(chunk) {
          let bytes;
          if (chunk instanceof ArrayBuffer) bytes = new Uint8Array(chunk);
          else if (ArrayBuffer.isView(chunk)) bytes = new Uint8Array(chunk.buffer, chunk.byteOffset, chunk.byteLength);
          else throw new TypeError("Failed to execute 'write' on 'UnderlyingSinkBase': The provided value is not of type '(ArrayBuffer or ArrayBufferView)'.");
          if (bytes.length === 0 || sink.closed) return undefined;
          // Copied: the caller may reuse its buffer once the write resolves,
          // and the rest is sent at later slices.
          const rest = bytes.slice();
          return new Promise((resolve, reject) => {
            sink.waiting = { rest, resolve, reject };
            sink.feed();
          });
        },
        close() {
          sink.closed = true;
          return request('drain', {}, port).then(() => port.#sinkClosed());
        },
        abort() {
          sink.closed = true;
          if (port.#state === 'closing') {
            port.#sinkClosed();
            return undefined;
          }
          return request('flush', { dir: 'tx' }, port).then(() => port.#sinkClosed());
        },
      }, new CountQueuingStrategy({ highWaterMark: 1 }));
      this.#writable = stream;
      this.#sink = sink;
      return stream;
    }

    getSignals() {
      if (this.#state !== 'open' && this.#state !== 'closing') {
        return Promise.reject(refuse('getSignals', 'SerialPort', 'InvalidStateError', kPortClosed));
      }
      return request('getSignals', {}, this);
    }

    setSignals(signals) {
      if (this.#state !== 'open' && this.#state !== 'closing') {
        return Promise.reject(refuse('setSignals', 'SerialPort', 'InvalidStateError', kPortClosed));
      }
      const s = signals || {};
      const out = {};
      if (s.dataTerminalReady !== undefined) out.dataTerminalReady = !!s.dataTerminalReady;
      if (s.requestToSend !== undefined) out.requestToSend = !!s.requestToSend;
      if (s.break !== undefined) out.break = !!s.break;
      if (Object.keys(out).length === 0) return Promise.reject(refuse('setSignals', 'SerialPort', 'TypeError', kNoSignals));
      return request('setSignals', { signals: out }, this).then(() => undefined);
    }

    close() {
      if (this.#state === 'closed' || this.#state === 'opening') {
        return Promise.reject(refuse('close', 'SerialPort', 'InvalidStateError', 'The port is already closed.'));
      }
      if (this.#state === 'closing') {
        return Promise.reject(refuse('close', 'SerialPort', 'InvalidStateError', 'A call to close() is already in progress.'));
      }
      this.#state = 'closing';
      const promise = new Promise((resolve, reject) => {
        this.#closeResolve = resolve;
        this.#closeReject = reject;
      });
      if (!this.#readable && !this.#writable) {
        this.#streamsClosed();
        return promise;
      }
      if (this.#readable) {
        if (this.#readable.locked) {
          this.#abortClose();
          return Promise.reject(refuse('close', 'SerialPort', 'TypeError', 'Cannot cancel a locked stream'));
        }
        this.#readable.cancel().catch(() => {});
      }
      if (this.#writable) {
        if (this.#writable.locked) {
          this.#abortClose();
          return Promise.reject(refuse('close', 'SerialPort', 'TypeError', 'Cannot abort a locked stream'));
        }
        this.#writable.abort(err('InvalidStateError', kPortClosed)).catch(() => {});
      }
      return promise;
    }

    // The origin gives the port up; an open port is closed, as Chrome
    // closes an origin's connections when its permission goes (a port the
    // policy grants cannot be given up).
    forget() {
      return request('forget', { gen: this.#gen, origin }, this).then((v) => {
        if (v && v.closed) this._lost();
        return undefined;
      });
    }

    // Chrome's AbortClose: the port stays open, the close never resolves.
    #abortClose() {
      this.#state = 'open';
      this.#closeResolve = null;
      this.#closeReject = null;
    }

    #sourceClosed() {
      this.#readable = null;
      this.#source = null;
      if (this.#state === 'closing' && !this.#writable) this.#streamsClosed();
    }

    #sinkClosed() {
      this.#writable = null;
      this.#sink = null;
      if (this.#state === 'closing' && !this.#readable) this.#streamsClosed();
    }

    #streamsClosed() {
      request('close', {}, this).then(() => this.#closed(), () => this.#closed());
    }

    #closed() {
      if (openPort === this) openPort = null;
      this.#state = 'closed';
      const resolve = this.#closeResolve;
      this.#closeResolve = null;
      this.#closeReject = null;
      if (resolve) resolve();
    }

    // The node's side of a slice for this port.
    _slice(rx, backlog) {
      let reads = 0;
      if (rx && this.#source) reads = this.#source.push(rx);
      if (this.#sink) {
        this.#sink.backlog = backlog;
        this.#sink.unsent = 0;
        this.#sink.feed();
      }
      return reads;
    }

    _reading() { return this.#source !== null && !this.#source.closed; }

    _transferred() { return this.#readable !== null && transferred.has(this.#readable); }

    // The device went away (Chrome's OnConnectionError).
    _lost() {
      if (this.#state === 'closed') return;
      const lost = err('NetworkError', kDeviceLost);
      const wasOpening = this.#state === 'opening';
      const wasClosing = this.#state === 'closing';
      this.#state = 'closed';
      if (openPort === this) openPort = null;
      dropRequests(this, wasOpening ? err('NetworkError', kOpenError) : lost);
      if (wasClosing && this.#closeResolve) {
        const resolve = this.#closeResolve;
        this.#closeResolve = null;
        this.#closeReject = null;
        resolve();
      }
      if (this.#source) this.#source.lose(lost);
      if (this.#sink) this.#sink.lose(lost);
      this.#readable = null;
      this.#source = null;
      this.#writable = null;
      this.#sink = null;
    }
  }
  handlerAttr(SerialPort.prototype, 'connect');
  handlerAttr(SerialPort.prototype, 'disconnect');

  // WebIDL's SerialOptions, [EnforceRange] where Chrome's IDL says so.
  const enforce = (v, max, what) => {
    const n = Number(v);
    if (!Number.isFinite(n)) {
      throw err('TypeError', `Failed to execute 'open' on 'SerialPort': Failed to read the '${what}' property from 'SerialOptions': Value is ${Number.isNaN(n) ? 'not a number' : 'infinite'} and cannot be converted to an integer.`);
    }
    const t = Math.trunc(n);
    if (t < 0 || t > max) {
      throw err('TypeError', `Failed to execute 'open' on 'SerialPort': Failed to read the '${what}' property from 'SerialOptions': Value is outside the '${max === 255 ? 'octet' : 'unsigned long'}' value range.`);
    }
    return t;
  };
  const convertOptions = (options) => {
    if (options !== undefined && options !== null && typeof options !== 'object') {
      throw err('TypeError', "Failed to execute 'open' on 'SerialPort': The provided value is not of type 'SerialOptions'.");
    }
    const o = options || {};
    if (o.baudRate === undefined) {
      throw err('TypeError', "Failed to execute 'open' on 'SerialPort': Failed to read the 'baudRate' property from 'SerialOptions': Required member is undefined.");
    }
    const parity = o.parity === undefined ? 'none' : String(o.parity);
    if (!['none', 'even', 'odd'].includes(parity)) {
      throw err('TypeError', `Failed to execute 'open' on 'SerialPort': Failed to read the 'parity' property from 'SerialOptions': The provided value '${parity}' is not a valid enum value of type ParityType.`);
    }
    const flowControl = o.flowControl === undefined ? 'none' : String(o.flowControl);
    if (!['none', 'hardware'].includes(flowControl)) {
      throw err('TypeError', `Failed to execute 'open' on 'SerialPort': Failed to read the 'flowControl' property from 'SerialOptions': The provided value '${flowControl}' is not a valid enum value of type FlowControlType.`);
    }
    return {
      baudRate: enforce(o.baudRate, 0xffffffff, 'baudRate'),
      bufferSize: o.bufferSize === undefined ? 255 : enforce(o.bufferSize, 0xffffffff, 'bufferSize'),
      dataBits: o.dataBits === undefined ? 8 : enforce(o.dataBits, 255, 'dataBits'),
      stopBits: o.stopBits === undefined ? 1 : enforce(o.stopBits, 255, 'stopBits'),
      parity,
      flowControl,
    };
  };

  const portFor = (g) => {
    let port = portsByGen.get(g);
    if (!port) {
      port = new SerialPort({ gen: g });
      portsByGen.set(g, port);
    }
    return port;
  };

  // ── Serial ────────────────────────────────────────────────────────────
  class Serial extends EventTarget {
    constructor(token) {
      if (token !== doc) throw new TypeError('Illegal constructor');
      super();
    }

    getPorts() {
      return request('getPorts', { origin }).then((gens) => gens.map(portFor));
    }

    requestPort(options) {
      const activation = navigator.userActivation;
      if (!activation || !activation.isActive) {
        return Promise.reject(refuse('requestPort', 'Serial', 'SecurityError', 'Must be handling a user gesture to show a permission request.'));
      }
      const o = options || {};
      const filters = o.filters === undefined ? [] : Array.from(o.filters);
      for (const f of filters) {
        const hasVendor = f && f.usbVendorId !== undefined;
        const hasProduct = f && f.usbProductId !== undefined;
        const hasBluetooth = f && f.bluetoothServiceClassId !== undefined;
        if (!hasVendor && !hasProduct && !hasBluetooth) {
          return Promise.reject(refuse('requestPort', 'Serial', 'TypeError', 'A filter must provide a property to filter by.'));
        }
        if (hasProduct && !hasVendor) {
          return Promise.reject(refuse('requestPort', 'Serial', 'TypeError', 'A filter containing a usbProductId must also specify a usbVendorId.'));
        }
      }
      const wanted = filters.map((f) => ({
        usbVendorId: f.usbVendorId === undefined ? null : Number(f.usbVendorId),
        usbProductId: f.usbProductId === undefined ? null : Number(f.usbProductId),
        bluetooth: f.bluetoothServiceClassId !== undefined,
      }));
      return request('requestPort', { origin, filters: wanted }).then(portFor);
    }
  }
  handlerAttr(Serial.prototype, 'connect');
  handlerAttr(Serial.prototype, 'disconnect');
  const serial = new Serial(doc);

  // A connect or disconnect event: at the port, then bubbling to
  // navigator.serial with the port as its target (Chrome's
  // SerialPort::DispatchEventInternal).
  const fire = (port, type) => {
    port.dispatchEvent(new Event(type, { bubbles: true }));
    const up = new Event(type, { bubbles: true });
    Object.defineProperty(up, 'target', { value: port });
    Object.defineProperty(up, 'currentTarget', { value: serial });
    serial.dispatchEvent(up);
  };

  Object.defineProperty(Navigator.prototype, 'serial', {
    configurable: true,
    enumerable: true,
    get() { return serial; },
  });
  for (const [name, value] of [['Serial', Serial], ['SerialPort', SerialPort]]) {
    Object.defineProperty(globalThis, name, { configurable: true, writable: true, value });
  }

  // ── The node's side ───────────────────────────────────────────────────
  const embsim = {
    version: 1,
    doc,
    // Called by the node once a slice, the page's clock held: what the
    // node answers, its link changes and the board's bytes in; this
    // document's clock and its port's state out.
    slice(a) {
      try {
        const wasGen = gen;
        const wasPlugged = plugged;
        const wasGranted = granted;
        gen = a.gen;
        plugged = a.plugged;
        granted = a.granted;
        windowBytes = a.window;
        for (const r of a.replies) settle(r);
        // A port the origin may use fires disconnect whether or not the
        // page has asked for it yet (Chrome's GetOrCreatePort).
        if (wasPlugged && (!plugged || gen !== wasGen)) {
          const old = portsByGen.get(wasGen) || (wasGranted ? portFor(wasGen) : null);
          if (old) {
            old._lost();
            fire(old, 'disconnect');
          }
        }
        if (plugged && (!wasPlugged || gen !== wasGen) && wasGen !== -1 && granted) {
          fire(portFor(gen), 'connect');
        }
        let reads = 0;
        if (openPort) reads = openPort._slice(a.rx ? unb64(a.rx) : null, a.backlog);
        return {
          t: performance.now(),
          doc,
          iso: !!globalThis.crossOriginIsolated,
          hidden: document.visibilityState === 'hidden',
          reading: !!(openPort && openPort._reading()),
          transferred: !!(openPort && openPort._transferred()),
          reads,
        };
      } catch (e) {
        return {
          t: performance.now(),
          doc,
          iso: !!globalThis.crossOriginIsolated,
          error: String(e && e.stack || e),
        };
      }
    },
    // Pull the cable or put it back, at the node's next slice.
    link(op) {
      if (op !== 'unplug' && op !== 'plug') {
        throw new TypeError(`__embsim.link takes 'unplug' or 'plug', not ${JSON.stringify(op)}`);
      }
      send({ k: 'link', doc, op });
    },
    get plugged() { return plugged; },
  };
  Object.defineProperty(globalThis, '__embsim', { configurable: false, enumerable: false, writable: false, value: Object.freeze(embsim) });
  send({ k: 'hello', doc, origin, href, late });
})();
