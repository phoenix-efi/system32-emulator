// Pre-js for the Node build of emu/pair/s32pair.c (build/s32pair_node.js): the process environment is the knob
// interface (S32_*), as for the native binary. Node only; the browser bench (build/s32_wasm.js) does not use this.
Module['preRun'] = Module['preRun'] || [];
Module['preRun'].push(function () {
  for (var k in process.env) ENV[k] = process.env[k];
});
