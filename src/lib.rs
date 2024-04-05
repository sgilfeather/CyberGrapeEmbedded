#![no_std]

use lock_api::{GuardSend, RawMutex};
use rp_pico::hal::sio::{Spinlock, SpinlockValid};

pub struct HardwareSpinlock<const N: usize>(core::marker::PhantomData<()>);

unsafe impl<const N: usize> RawMutex for HardwareSpinlock<N>
where
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

    unsafe fn unlock(&self) {
        Spinlock::<N>::release();
    }
}
