// embsim chrome-cdp: a dedicated worker's read probe, for the drain barrier.
//
// Evaluated in each dedicated worker the node holds, through the worker's
// own session, before and (until it takes) after the worker is released.
// Every read call on a reader in this worker reports the worker's running
// count through the worker-session binding __embsimDrain: a consumer that
// asks for more has finished with what it was handed. Returns 1 once
// installed, 0 while the worker's global scope is not ready yet (the node
// retries).
(() => {
  if (self.__embsimProbe) return 1;
  if (typeof ReadableStreamDefaultReader !== 'function' || typeof __embsimDrain !== 'function') {
    return 0;
  }
  const report = __embsimDrain;
  let n = 0;
  const counted = (read) => function (...a) {
    n++;
    try { report(String(n)); } catch (_) { /* the binding went away */ }
    return read.apply(this, a);
  };
  const P = ReadableStreamDefaultReader.prototype;
  P.read = counted(P.read);
  if (typeof ReadableStreamBYOBReader === 'function') {
    const B = ReadableStreamBYOBReader.prototype;
    B.read = counted(B.read);
  }
  const RS = ReadableStream.prototype;
  if (typeof RS.values === 'function') {
    const values = RS.values;
    const iterate = function (...a) {
      const it = values.apply(this, a);
      it.next = counted(it.next);
      return it;
    };
    RS.values = iterate;
    RS[Symbol.asyncIterator] = iterate;
  }
  Object.defineProperty(self, '__embsimProbe', { value: true });
  return 1;
})()
