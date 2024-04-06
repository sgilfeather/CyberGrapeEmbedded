#![no_std]

// Okay this is fucky so get ready.

// This is the API that talc expects for our lock
use lock_api::{GuardSend, RawMutex};

// This is how we access the real hardware locks on the pico
use rp_pico::hal::sio::{Spinlock, SpinlockValid};

// We need to write an adapter that uses the SpinLock to implement the RawMutex
// API, so that the hardware locks can be used in our custom allocator.

// This struct essentially holds a marker telling the compiler that it exists,
// but it doesn't have any real data in it, this will be our adapter.
// The const type parameter N indicate which hardware lock we are referring to.
pub struct HardwareSpinlock<const N: usize>(core::marker::PhantomData<()>);

// This is obviously wildly unsafe because we, the programmer, are promising
// that the locks will actually block when they are supposed to. It would be
// trivial to write a RawMutex implementation that does not actually lock.
unsafe impl<const N: usize> RawMutex for HardwareSpinlock<N>
where
    // This enforces that N refers to a real hardware lock
    Spinlock<N>: SpinlockValid,
{
    type GuardMarker = GuardSend;
    const INIT: Self = HardwareSpinlock::<N>(core::marker::PhantomData);

    fn lock(&self) {
        Spinlock::<N>::claim();
    }

    fn try_lock(&self) -> bool {
        Spinlock::<N>::try_claim().is_some()
    }

    // RawMutex::unlock is unsafe to call because it must be called in the same
    // scope as RawMutex::lock was called. We normally release locks by letting
    // them go out of scope.
    unsafe fn unlock(&self) {
        Spinlock::<N>::release();
    }
}
