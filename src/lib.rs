#![no_std]

pub use embassy_boot::{
    AlignedBuffer, BlockingFirmwareState, BlockingFirmwareUpdater, BootLoaderConfig, FirmwareState,
    FirmwareUpdater, FirmwareUpdaterConfig, State,
};

#[cfg(feature = "defmt")]
use defmt::trace;

use embassy_time::Duration;
use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};

/// A bootloader for iMXRT devices.
pub struct BootLoader {
    /// The reported state of the bootloader after preparing for boot
    pub state: State,
}

impl BootLoader {
    /// Inspect the bootloader state and perform actions required before booting, such as swapping firmware
    pub fn prepare<ACTIVE: NorFlash, DFU: NorFlash, STATE: NorFlash, const BUFFER_SIZE: usize>(
        config: BootLoaderConfig<ACTIVE, DFU, STATE>,
    ) -> Self {
        let mut aligned_buf = AlignedBuffer([0; BUFFER_SIZE]);
        let mut boot = embassy_boot::BootLoader::new(config);

        let state = boot
            .prepare_boot(aligned_buf.as_mut())
            .expect("Boot prepare error");
        Self { state }
    }

    /// Boots the application.
    ///
    /// # Safety
    ///
    /// This modifies the stack pointer and reset vector and will run code placed in the active partition.
    pub unsafe fn load(self, start: u32) -> ! {
        #[cfg(feature = "defmt")]
        trace!("Loading app at 0x{:x}", start);
        #[allow(unused_mut)]
        let mut p = cortex_m::Peripherals::steal();
        p.SCB.invalidate_icache();
        p.SCB.vtor.write(start);

        cortex_m::asm::bootload(start as *const u32)
    }
}

use embassy_imxrt::flexspi;
use embassy_imxrt::flexspi::nor::*;
use embassy_imxrt::flexspi::Instance;
use embassy_imxrt::rtwdog;

/// A flash implementation that will feed RTWDOG3 when touching flash.
pub struct WatchdogNorFlash<'d, T: flexspi::Instance, const SIZE: usize> {
    flash: flexspi::nor::NorFlash<'d, T, SIZE>,
}

impl<'d, T: flexspi::Instance, const SIZE: usize> WatchdogNorFlash<'d, T, SIZE> {
    /// Start a new watchdog with a given flash and watchdog peripheral and a timeout
    pub fn start(flexspi: T, timeout: Duration) -> Self {
        let flexspi = flexspi::Flexspi::new();
        let flash = flexspi::nor::NorFlash::<_, SIZE>::from_flexspi(flexspi);
        rtwdog::enable(rtwdog::Config::lpo(timeout));
        Self { flash }
    }
}

impl<'d, T: flexspi::Instance, const SIZE: usize> embedded_storage::nor_flash::ErrorType
    for WatchdogNorFlash<'d, T, SIZE>
{
    type Error = <flexspi::nor::NorFlash<'d, T, SIZE> as ErrorType>::Error;
}

impl<'d, T: flexspi::Instance, const SIZE: usize> NorFlash for WatchdogNorFlash<'d, T, SIZE> {
    const WRITE_SIZE: usize = <flexspi::nor::NorFlash<'d, T, SIZE> as NorFlash>::WRITE_SIZE;
    const ERASE_SIZE: usize = <flexspi::nor::NorFlash<'d, T, SIZE> as NorFlash>::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        rtwdog::feed();

        self.flash.erase(from, to)
    }
    fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error> {
        rtwdog::feed();

        self.flash.blocking_write(offset, data)
    }
}

impl<'d, T: flexspi::Instance, const SIZE: usize> ReadNorFlash for WatchdogNorFlash<'d, T, SIZE> {
    const READ_SIZE: usize = <flexspi::nor::NorFlash<'d, T, SIZE> as ReadNorFlash>::READ_SIZE;
    fn read(&mut self, offset: u32, data: &mut [u8]) -> Result<(), Self::Error> {
        rtwdog::feed();

        self.flash.blocking_read(offset, data)
    }
    fn capacity(&self) -> usize {
        self.flash.capacity()
    }
}
