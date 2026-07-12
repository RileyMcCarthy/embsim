# Platform-crate contract

A **platform crate** (e.g. `embsim-p2`) is the glue between one firmware's HAL
and embsim's generic peripherals. It exports a Rust `#[no_mangle] extern "C"`
function for every symbol the firmware leaves undefined, and implements
[`embsim_runtime::Platform`]. This document enumerates the contract.

## Why it exists

The firmware is compiled to a static library with its HAL **left undefined**.
When the emulator binary links that `.a`, the linker resolves each undefined
`HAL_*` (etc.) symbol against the `#[no_mangle]` functions in the platform crate.
At the *linker* level there is no indirection — symbol *names must match the
firmware's HAL headers exactly*, including any historical spellings (the
reference `embsim-p2` platform's firmware spells it `recieve`, so its
trampoline is `HAL_serial_recieveByte`).

At *runtime*, each trampoline delegates to a generic peripheral free function,
and those free functions route by **thread identity** to a per-MCU
`embsim_peripherals::instance::PeripheralInstance` (see "Peripheral instances
& thread routing" below). A single-MCU emulator never notices: unbound threads
fall back to a process-wide default instance that behaves exactly like the
former process globals.

## ABI mapping rules

| C HAL type            | Rust trampoline type            | Notes |
|-----------------------|---------------------------------|-------|
| channel `int`         | `i32`                           | guard `>= 0`, then `as usize` for the peripheral |
| `bool`                | `bool`                          | |
| `uint8_t* / const uint8_t*` | `*mut u8 / *const u8`     | null-check; `slice::from_raw_parts*` |
| `uint32_t`            | `u32`                           | |
| `void*`               | `*mut core::ffi::c_void`        | |
| `void (*)(void*)`     | `Option<unsafe extern "C" fn(*mut c_void)>` | thread entry |

Every trampoline must **null/range-guard its arguments** before delegating to the
generic peripheral (the peripherals also guard, but the platform layer is the
ABI boundary). See [`platforms/p2/src/ffi.rs`](platforms/p2/src/ffi.rs) for the reference pattern.

## Required symbol domains

A platform must provide all symbols its firmware references. For the reference
Propeller 2 platform (`embsim-p2`) these are three domains:

### 1. HAL (the bulk — delegate to `embsim_peripherals`)

```
GPIO     HAL_GPIO_setActive  HAL_GPIO_getActive  HAL_GPIO_toggleActive
Serial   HAL_serial_start  HAL_serial_stop  HAL_serial_transmitData
         HAL_serial_recieveDataTimeout  HAL_serial_recieveByte
Encoder  HAL_encoder_start  HAL_encoder_value  HAL_encoder_set
PulseOut HAL_pulseOut_start  HAL_pulseOut_run  HAL_pulseOut_stop
Timer    HAL_time_getMs  HAL_time_getUs  HAL_time_waitMs  HAL_time_waitUs
         HAL_time_getCycles  HAL_time_getClockFreq
Lock     HAL_lock_create  HAL_lock_try  HAL_lock_release
System   HAL_system_init  HAL_system_reboot  HAL_system_startThread
I2C      i2c_setup  i2c_start  i2c_write  i2c_read  i2c_stop
```

(Your firmware's set will differ — provide what *its* HAL headers declare.)

### 2. Platform VFS / filesystem (FlexC VFS for the P2)

```
mount  umount  _vfs_open_sdcard
```

These bridge the firmware's filesystem calls to the host SD directory configured
via the builder's `.sd_path(...)`.

### 3. MCU intrinsics

```
_clkset  _hubset  _reboot
```

Compiler/CPU intrinsics the firmware emits inline on real silicon; stubbed for
the host.

## Peripheral instances & thread routing

All generic peripheral state (serial FDs/baud/pacing, GPIO state + callbacks,
encoder counters, pulse-out trains, the lock pool, the thread registry, the
filesystem mount, and the per-MCU clock-frequency override) is owned by an
`embsim_peripherals::instance::PeripheralInstance`. The free functions the
trampolines call resolve the calling thread's instance:

1. a thread explicitly bound via `instance::bind_current_thread(arc)` (RAII
   guard; restores the previous binding on drop) routes to its bound instance.
   The guard is `!Send`: it must be dropped on the thread it bound (dropping
   it elsewhere would rewrite the wrong thread's cached binding), which is
   what keeps the thread-local routing cache coherent by construction.
   Nested guards must be dropped in **LIFO order** (innermost first) — an
   out-of-order drop panics rather than leaving the thread bound to a stale
   instance;
2. threads spawned through `system::start_thread` — i.e. through the
   `HAL_system_startThread` trampoline — **inherit their creator's instance**
   (the spawn path binds the new thread before the firmware function runs);
3. any other thread falls back to the lazily-created **default singleton**
   (`instance::default()`), which preserves the historical single-MCU
   behavior. `Emulator::run` initializes peripherals through the free
   functions on an unbound thread, so a whole classic emulator lives on the
   default instance.

To support **multiple MCU instances** in one process, a platform/board layer
must:

- create one `Arc<PeripheralInstance>` per MCU and initialize/wire *that*
  instance (`inst.serial.init(..)`, `inst.gpio.on_change(..)`, …) instead of
  the module free functions;
- bind the thread that runs each firmware entry point with
  `instance::bind_current_thread` **before** calling the entry, and keep the
  guard alive for the entry's lifetime — every thread the firmware spawns
  through `HAL_system_startThread` then inherits the right instance
  automatically;
- keep model/host threads that talk to a specific MCU either bound to that
  instance or using `inst.<peripheral>` directly (free functions on an unbound
  thread hit the default instance, not "the nearest MCU").

**Documented limit:** instance routing de-globalizes the Rust-side peripheral
state only. A given firmware **image's own C statics** (`.data`/`.bss`) exist
once per process, so one process can run at most **one instance of a given
firmware image**; multi-instance means multiple *distinct* images (or
pure-Rust components). Virtual *time* also remains process-wide
(`embsim_core::virtual_clock` is free-running scaled wall time); only the
clock *frequency* used for cycle math is per-instance.

## Init-before-entry ordering (handled by the runtime)

The platform crate provides the *symbols*; the **runtime** owns *initialization
order*. By the time the firmware entry point runs, `Emulator::run` has:

1. initialized the virtual clock (`Platform::clock_freq_hz`),
2. sized every peripheral from `Machine::peripheral_counts` (validated against
   each peripheral's `MAX_*` ceiling),
3. initialized locks/threads from `Platform::max_locks` / `max_cores`,
4. created the host PTY and bridged it to `Machine::host_serial_channel`,
5. run `Machine::wire`.

A platform/machine author never has to remember this sequence — but a trampoline
must tolerate being called the instant the entry point starts.

## Checklist for a new platform

- [ ] Implement `Platform` (clock freq, core count, lock count).
- [ ] One `#[no_mangle] extern "C"` trampoline per firmware HAL symbol, name-exact.
- [ ] Null/range-guard every pointer and channel argument.
- [ ] Stub MCU intrinsics / VFS your firmware references.
- [ ] Build the firmware `.a` with debug info (`-g`) so `memory-inspect` can read
      enums/structs.
