/* SPDX-License-Identifier: Apache-2.0 */

#include <fsl_clock.h>

/* Physical strap/reset sequencing uses DWT delays and is unnecessary when the
 * firmware communicates exclusively through the FastDyn ENET model. */
void board_early_init_hook(void)
{
}

/* QEMU owns CPU time. Keep MCUX clock queries internally consistent without
 * polling analog DCDC/PLL status registers that do not exist in rehosting. */
void clock_init(void)
{
	CLOCK_SetXtalFreq(24000000U);
	CLOCK_SetRtcXtalFreq(32768U);
	CLOCK_SetMux(kCLOCK_PeriphClk2Mux, 1U);
	CLOCK_SetMux(kCLOCK_PeriphMux, 1U);
	SystemCoreClock = 24000000U;
}

uint32_t CLOCK_GetAhbFreq(void)
{
	return 24000000U;
}

uint32_t CLOCK_GetIpgFreq(void)
{
	return 24000000U;
}

uint32_t CLOCK_GetPerClkFreq(void)
{
	return 24000000U;
}

uint32_t CLOCK_GetFreq(clock_name_t name)
{
	return name == kCLOCK_RtcClk ? 32768U : 24000000U;
}
