//! Time APIs for obtaining the current time and calculating with points in time and durations.
//!
//! These APIs are made for the BLE stack and are not meant to be general-purpose. The APIs here
//! have microsecond resolution and use 32-bit arithmetic wherever possible.

use fugit;

pub type Instant = fugit::Instant<u32, 1, 1_000_000>;
pub type Duration = fugit::Duration<u32, 1, 1_000_000>;
pub const T_IFS: Duration = Duration::micros(150);

/// Trait for time providers.
///
/// The hardware interface has to provide an implementation of `Timer` to the stack. The
/// implementation must have microsecond accuracy.
///
/// This trait can also be implemented by a mock timer for testing.
pub trait Timer {
    /// Obtain the current time as an [`Instant`].
    ///
    /// The [`Instant`]s returned by this function must never move backwards in time, except when
    /// the underlying value wraps around.
    fn now(&self) -> Instant;
}
