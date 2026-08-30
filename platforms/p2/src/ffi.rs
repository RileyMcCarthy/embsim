//! P2 FFI trampolines — `#[no_mangle] extern "C"` functions matching the
//! firmware's HAL_* interface. Each function delegates to the generic
//! peripheral implementation in `embsim-peripherals`.
//!
//! Native SIL cannot preempt host machine code. [`enter_hal`] charges this
//! cog's quantum so a free-running poll (UART, `LOCKTRY`) is stopped at the
//! next HAL call after a slice, without firmware yield macros.

use embsim_core::virtual_clock;
use embsim_peripherals::{access, encoder, gpio, i2c, lock, pulse_out, serial, system, timer};
use tracing::info;

/// Charge one HAL-proxy unit of work. No-op on threads that are not actors.
fn enter_hal() {
    virtual_clock::charge(1);
}

fn require_channel(kind: &'static str, channel: i32) -> Option<usize> {
    if channel < 0 {
        access::report(kind, &format!("negative channel {channel}"));
        return None;
    }
    Some(channel as usize)
}

// ============================================================
// GPIO
// ============================================================

#[no_mangle]
pub unsafe extern "C" fn HAL_GPIO_setActive(channel: i32, active: bool) {
    enter_hal();
    if let Some(ch) = require_channel("gpio", channel) {
        gpio::set_active(ch, active);
    }
}

#[no_mangle]
pub unsafe extern "C" fn HAL_GPIO_getActive(channel: i32) -> bool {
    enter_hal();
    require_channel("gpio", channel)
        .map(gpio::get_active)
        .unwrap_or(false)
}

#[no_mangle]
pub unsafe extern "C" fn HAL_GPIO_toggleActive(channel: i32) {
    enter_hal();
    if let Some(ch) = require_channel("gpio", channel) {
        gpio::toggle_active(ch);
    }
}

// ============================================================
// Serial
// ============================================================

#[no_mangle]
pub unsafe extern "C" fn HAL_serial_start(channel: i32) {
    enter_hal();
    if let Some(ch) = require_channel("serial", channel) {
        serial::start(ch);
    }
}

#[no_mangle]
pub unsafe extern "C" fn HAL_serial_stop(channel: i32) {
    enter_hal();
    if let Some(ch) = require_channel("serial", channel) {
        serial::stop(ch);
    }
}

#[no_mangle]
pub unsafe extern "C" fn HAL_serial_transmitData(channel: i32, data: *const u8, len: u32) {
    enter_hal();
    if data.is_null() || len == 0 {
        return;
    }
    let Some(ch) = require_channel("serial", channel) else {
        return;
    };
    let buf = std::slice::from_raw_parts(data, len as usize);
    serial::transmit_data(ch, buf);
}

#[no_mangle]
pub unsafe extern "C" fn HAL_serial_recieveDataTimeout(
    channel: i32,
    data: *mut u8,
    len: u32,
    timeout_us: u32,
) -> bool {
    enter_hal();
    if data.is_null() || len == 0 {
        // Guard path: the firmware still observes its full virtual timeout, so
        // a bad argument cannot make a blocking receive return instantly.
        embsim_core::virtual_clock::wait_virtual_us(timeout_us as u64);
        return false;
    }
    let Some(ch) = require_channel("serial", channel) else {
        // Guard path: the firmware still observes its full virtual timeout, so
        // a bad argument cannot make a blocking receive return instantly.
        embsim_core::virtual_clock::wait_virtual_us(timeout_us as u64);
        return false;
    };
    let buf = std::slice::from_raw_parts_mut(data, len as usize);
    serial::receive_data_timeout(ch, buf, timeout_us as u64)
}

#[no_mangle]
pub unsafe extern "C" fn HAL_serial_recieveByte(channel: i32, data: *mut u8) -> bool {
    enter_hal();
    if data.is_null() {
        return false;
    }
    let Some(ch) = require_channel("serial", channel) else {
        return false;
    };
    match serial::receive_byte(ch) {
        Some(byte) => {
            *data = byte;
            true
        }
        None => false,
    }
}

#[no_mangle]
pub unsafe extern "C" fn HAL_serial_recieveBytes(
    channel: i32,
    buf: *mut u8,
    max_bytes: u32,
) -> u32 {
    enter_hal();
    if buf.is_null() || max_bytes == 0 {
        return 0;
    }
    let Some(ch) = require_channel("serial", channel) else {
        return 0;
    };
    let out = std::slice::from_raw_parts_mut(buf, max_bytes as usize);
    serial::receive_bytes(ch, out) as u32
}

// ============================================================
// Encoder
// ============================================================

#[no_mangle]
pub unsafe extern "C" fn HAL_encoder_start(channel: i32) {
    enter_hal();
    if let Some(ch) = require_channel("encoder", channel) {
        encoder::start(ch);
    }
}

#[no_mangle]
pub unsafe extern "C" fn HAL_encoder_value(channel: i32) -> i32 {
    enter_hal();
    require_channel("encoder", channel)
        .map(encoder::value)
        .unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn HAL_encoder_set(channel: i32, value: i32) {
    enter_hal();
    if let Some(ch) = require_channel("encoder", channel) {
        encoder::set(ch, value);
    }
}

// ============================================================
// Pulse Out
// ============================================================

#[no_mangle]
pub unsafe extern "C" fn HAL_pulseOut_start(channel: i32, pulses: u32, frequency: u32) {
    enter_hal();
    if let Some(ch) = require_channel("pulse_out", channel) {
        pulse_out::start(ch, pulses, frequency);
    }
}

#[no_mangle]
pub unsafe extern "C" fn HAL_pulseOut_run(channel: i32, pulses: *mut u32) -> bool {
    enter_hal();
    if pulses.is_null() {
        return true;
    }
    let Some(ch) = require_channel("pulse_out", channel) else {
        return true;
    };
    let (emitted, done) = pulse_out::run(ch);
    *pulses = emitted;
    done
}

#[no_mangle]
pub unsafe extern "C" fn HAL_pulseOut_stop(channel: i32) {
    enter_hal();
    if let Some(ch) = require_channel("pulse_out", channel) {
        pulse_out::stop(ch);
    }
}

#[no_mangle]
pub unsafe extern "C" fn HAL_pulseOut_startVelocity(channel: i32, frequency: u32) {
    enter_hal();
    if let Some(ch) = require_channel("pulse_out", channel) {
        pulse_out::start_velocity(ch, frequency);
    }
}

#[no_mangle]
pub unsafe extern "C" fn HAL_pulseOut_setFrequency(channel: i32, frequency: u32) {
    enter_hal();
    if let Some(ch) = require_channel("pulse_out", channel) {
        pulse_out::set_frequency(ch, frequency);
    }
}

// ============================================================
// Timer
// ============================================================

#[no_mangle]
pub unsafe extern "C" fn HAL_time_getMs() -> u32 {
    enter_hal();
    timer::get_ms()
}

#[no_mangle]
pub unsafe extern "C" fn HAL_time_getUs() -> u32 {
    enter_hal();
    timer::get_us()
}

#[no_mangle]
pub unsafe extern "C" fn HAL_time_waitMs(ms: u32) {
    timer::wait_ms(ms);
}

#[no_mangle]
pub unsafe extern "C" fn HAL_time_waitUs(us: u32) {
    timer::wait_us(us);
}

#[no_mangle]
pub unsafe extern "C" fn HAL_time_getCycles() -> u32 {
    enter_hal();
    timer::get_cycles()
}

#[no_mangle]
pub unsafe extern "C" fn HAL_time_getClockFreq() -> u32 {
    enter_hal();
    timer::get_clock_freq()
}

// ============================================================
// Lock
// ============================================================

#[no_mangle]
pub unsafe extern "C" fn HAL_lock_create() -> i32 {
    enter_hal();
    lock::create()
}

#[no_mangle]
pub unsafe extern "C" fn HAL_lock_try(lock_id: i32) -> bool {
    enter_hal();
    lock::try_acquire(lock_id)
}

#[no_mangle]
pub unsafe extern "C" fn HAL_lock_release(lock_id: i32) {
    enter_hal();
    lock::release(lock_id);
}

// ============================================================
// System
// ============================================================

#[no_mangle]
pub unsafe extern "C" fn HAL_system_init() {
    enter_hal();
    info!("HAL_system_init called (already initialized by embsim)");
}

#[no_mangle]
pub unsafe extern "C" fn HAL_system_reboot() {
    enter_hal();
    info!("HAL_system_reboot: firmware requested reboot");
    std::process::exit(0);
}

#[no_mangle]
pub unsafe extern "C" fn HAL_system_startThread(
    func: Option<unsafe extern "C" fn(*mut std::ffi::c_void)>,
    arg: *mut std::ffi::c_void,
    _stack: *mut std::ffi::c_void,
    _stack_size: u32,
) -> i32 {
    enter_hal();
    system::start_thread(func, arg)
}

// ============================================================
// I2C
// ============================================================

#[no_mangle]
pub unsafe extern "C" fn i2c_setup(self_: *mut i2c::I2C, scl: u8, sda: u8, khz: u32, pullup: i32) {
    enter_hal();
    if let Some(i2c) = self_.as_mut() {
        i2c::setup(i2c, scl, sda, khz, pullup);
    }
}

#[no_mangle]
pub unsafe extern "C" fn i2c_start(self_: *mut i2c::I2C) {
    enter_hal();
    if let Some(i2c) = self_.as_mut() {
        i2c::start(i2c);
    }
}

#[no_mangle]
pub unsafe extern "C" fn i2c_write(self_: *mut i2c::I2C, byte: u8) -> bool {
    enter_hal();
    if let Some(i2c) = self_.as_mut() {
        i2c::write(i2c, byte)
    } else {
        false
    }
}

#[no_mangle]
pub unsafe extern "C" fn i2c_read(self_: *mut i2c::I2C, ack: bool) -> u8 {
    enter_hal();
    if let Some(i2c) = self_.as_mut() {
        i2c::read(i2c, ack)
    } else {
        0
    }
}

#[no_mangle]
pub unsafe extern "C" fn i2c_stop(self_: *mut i2c::I2C) {
    enter_hal();
    if let Some(i2c) = self_.as_mut() {
        i2c::stop(i2c);
    }
}
