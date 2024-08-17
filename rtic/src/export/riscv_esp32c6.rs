use esp32c6::INTERRUPT_CORE0; // Priority threshold control
pub use esp32c6::{Interrupt, Peripherals, INTPRI};
pub use riscv::asm::fence;
pub use riscv::interrupt;
pub use riscv::register::mcause; // Low level interrupt enable/disable
pub use riscv::register::mie; // Low level interrupt enable/disable

#[cfg(all(feature = "riscv-esp32c6", not(feature = "riscv-esp32c6-backend")))]
compile_error!("Building for the esp32c6, but 'riscv-esp32c6-backend' not selected");

#[inline(always)]
pub fn run<F>(priority: u8, f: F)
where
    F: FnOnce(),
{
    if priority == 1 {
        // If priority is 1, priority thresh should be 1
        f();
        unsafe {
            (*INTPRI::ptr())
                .cpu_int_thresh()
                .write(|w| w.cpu_int_thresh().bits(1));
        }
    } else {
        // Read current thresh
        let initial = unsafe {
            (*INTPRI::ptr())
                .cpu_int_thresh()
                .read()
                .cpu_int_thresh()
                .bits()
        };
        f();
        // Write back old thresh
        unsafe {
            (*INTPRI::ptr())
                .cpu_int_thresh()
                .write(|w| w.cpu_int_thresh().bits(initial));
        }
    }
}
/// Lock implementation using threshold and global Critical Section (CS)
///
/// # Safety
///
/// The system ceiling is raised from current to ceiling
/// by either
/// - raising the threshold to the ceiling value, or
/// - disable all interrupts in case we want to
///   mask interrupts with maximum priority
///
/// Dereferencing a raw pointer inside CS
///
/// The priority.set/priority.get can safely be outside the CS
/// as being a context local cell (not affected by preemptions).
/// It is merely used in order to omit masking in case current
/// priority is current priority >= ceiling.
#[inline(always)]
pub unsafe fn lock<T, R>(ptr: *mut T, ceiling: u8, f: impl FnOnce(&mut T) -> R) -> R {
    if ceiling == 15 {
        // Turn off interrupts completely, we're at max priority
        let r = critical_section::with(|_| f(&mut *ptr));
        r
    } else {
        // Read the current threshold
        let current = unsafe {
            (*INTPRI::ptr())
                .cpu_int_thresh()
                .read()
                .cpu_int_thresh()
                .bits()
        };

        // Set the new threshold to ceiling + 1
        unsafe {
            (*INTPRI::ptr())
                .cpu_int_thresh()
                .write(|w| w.cpu_int_thresh().bits(ceiling + 1));
        }

        // Execute the closure while the threshold is raised
        let r = f(&mut *ptr);

        // Restore the original threshold
        unsafe {
            (*INTPRI::ptr())
                .cpu_int_thresh()
                .write(|w| w.cpu_int_thresh().bits(current));
        }

        r
    }
}

#[inline(always)]
pub fn pend(int: Interrupt) {
    unsafe {
        let peripherals = Peripherals::steal();
        match int {
            Interrupt::FROM_CPU_INTR0 => peripherals
                .INTPRI
                .cpu_intr_from_cpu_0()
                .write(|w| w.cpu_intr_from_cpu_0().bit(true)),
            Interrupt::FROM_CPU_INTR1 => peripherals
                .INTPRI
                .cpu_intr_from_cpu_1()
                .write(|w| w.cpu_intr_from_cpu_1().bit(true)),
            Interrupt::FROM_CPU_INTR2 => peripherals
                .INTPRI
                .cpu_intr_from_cpu_2()
                .write(|w| w.cpu_intr_from_cpu_2().bit(true)),
            Interrupt::FROM_CPU_INTR3 => peripherals
                .INTPRI
                .cpu_intr_from_cpu_3()
                .write(|w| w.cpu_intr_from_cpu_3().bit(true)),
            _ => panic!("Unsupported software interrupt"), //should never happen, checked at compile time
        }
    }
}

#[inline(always)]
pub fn unpend(int: Interrupt) {
    unsafe {
        let peripherals = Peripherals::steal();
        match int {
            Interrupt::FROM_CPU_INTR0 => peripherals
                .INTPRI
                .cpu_intr_from_cpu_0()
                .write(|w| w.cpu_intr_from_cpu_0().bit(false)),
            Interrupt::FROM_CPU_INTR1 => peripherals
                .INTPRI
                .cpu_intr_from_cpu_1()
                .write(|w| w.cpu_intr_from_cpu_1().bit(false)),
            Interrupt::FROM_CPU_INTR2 => peripherals
                .INTPRI
                .cpu_intr_from_cpu_2()
                .write(|w| w.cpu_intr_from_cpu_2().bit(false)),
            Interrupt::FROM_CPU_INTR3 => peripherals
                .INTPRI
                .cpu_intr_from_cpu_3()
                .write(|w| w.cpu_intr_from_cpu_3().bit(false)),
            _ => panic!("Unsupported software interrupt"), //should never happen, checked at compile time
        }
    }
}

pub fn enable(int: Interrupt, prio: u8, cpu_int_id: u8) {
    const INTERRUPT_MAP_BASE: *mut u32 =
        unsafe { core::mem::transmute::<_, *mut u32>(INTERRUPT_CORE0::ptr()) };

    let interrupt_number = int as isize;
    let cpu_interrupt_number = cpu_int_id as isize;

    unsafe {
        let intr_map_base = INTERRUPT_MAP_BASE;
        intr_map_base
            .offset(interrupt_number)
            .write_volatile(cpu_interrupt_number as u32);

        (*INTPRI::ptr())
            .cpu_int_enable()
            .modify(|r, w| w.bits((1 << cpu_interrupt_number) | r.bits()));

        (*INTPRI::ptr())
            .cpu_int_pri(cpu_interrupt_number as usize)
            .write(|w| w.bits(prio as u32));

        (*INTPRI::ptr()).cpu_int_type().modify(|r, w| {
            let interrupt_type = 1;
            w.bits(
                r.bits() & !(1 << cpu_interrupt_number) | (interrupt_type << cpu_interrupt_number),
            )
        });
    }
}
