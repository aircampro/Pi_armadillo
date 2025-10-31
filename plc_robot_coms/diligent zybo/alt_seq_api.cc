/* diligent zybo API for GPIO example (custom to zybo) */
#include <stdio.h>
#include <cstdint>
#include "platform.h"
#include "xil_printf.h"
#include "sleep.h"
#include "xgpiops.h"
#include "xil_exception.h"
#include "xipipsu.h"
#include "xparameters.h"
#include "xscugic.h"
#include "xttcps.h"
inline constexpr std::uint32_t GPIO_BANK0_LED_MASK = std::uint32_t{0b1111} << 17;

// sequence to iterate
constexpr std::uint8_t led_patterns[] = {
  0b0000, 0b0001, 0b0011, 0b0110, 0b1100, 0b1000,
  0b0000, 0b1000, 0b1100, 0b0110, 0b0011, 0b0001,
  0b0110, 0b1001, 0b1101, 0b1111, 0b1010, 0b0101,
};
constexpr std::uint8_t no_pattern = 17;

int main()
{
    init_platform();
    print("Zybo sequence example using API\n\r");

    XGpioPs instXGpioPs;
    XGpioPs_Config *configXGpioPs;

    configXGpioPs = XGpioPs_LookupConfig(XPAR_PSU_GPIO_0_DEVICE_ID);
    XGpioPs_CfgInitialize(&instXGpioPs, configXGpioPs,configXGpioPs->BaseAddr);

    auto dir = XGpioPs_GetDirection(&instXGpioPs, 0);
    XGpioPs_SetDirection(&instXGpioPs, 0, dir | GPIO_BANK0_LED_MASK);

    auto oen = XGpioPs_GetOutputEnable(&instXGpioPs, 0);
    XGpioPs_SetOutputEnable(&instXGpioPs, 0, oen | GPIO_BANK0_LED_MASK);
	
    std::uint8_t led_pattern_next = 0;
    for(;;) {
		XGpioPs_WriteReg(&instXGpioPs, XGPIOPS_DATA_MSW_OFFSET, (~GPIO_BANK0_LED_MASK & 0xffff0000) | ((led_patterns[led_pattern_next] & 0b1111) << 1));
        led_pattern_next = (led_pattern_next + 1) % no_pattern;                      // cycle the patterns
        sleep(1);	
    }
    cleanup_platform();
    return 0;

}
