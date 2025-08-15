#include <hal/nrf_power.h>

void board_init(void) {
    // Enable DC-DC converter
    nrf_power_dcdcen_set(NRF_POWER, true);
    
    // Set REGOUT0 to 3.3V (0x05 = 3.3V)
    NRF_POWER->REGOUT0 = (NRF_POWER->REGOUT0 & ~POWER_REGOUT0_VOUT_Msk) | 0x05;
}