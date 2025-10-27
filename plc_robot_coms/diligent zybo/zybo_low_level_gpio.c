/* simple diligent zybo gpio sequence (processor independant) */
# include <stdio.h>
# include "platform.h"
# include "xil_printf.h"
# include "sleep.h"

/* take the value of the memory address */
# define REG(address) *(volatile unsigned int*)(address)

/* memory map as per manual */
# define GPIOPS_BASE   (0xE000A000)
# define GPIOPS_DATA_0 (GPIOPS_BASE + 0x0040)
# define GPIOPS_DIRM_0 (GPIOPS_BASE + 0x0204)
# define GPIOPS_OEN_0  (GPIOPS_BASE + 0x0208)

int main()
{
    init_platform();

    print("----------------Example Simple Output----------------\n\r");

    /* Set MIO7 as output */
    REG(GPIOPS_DIRM_0) |= 1 << 7;
    REG(GPIOPS_OEN_0)  |= 1 << 7;
    /* Set MIO6 as output */
    REG(GPIOPS_DIRM_0) |= 1 << 6;
    REG(GPIOPS_OEN_0)  |= 1 << 6;
    /* Set MIO0 as input */
    REG(GPIOPS_DIRM_0) &= ~(1 << 0);
    REG(GPIOPS_OEN_0)  |= 1 << 0;
    while(1) {
    	/* Set MIO7 as High */
    	REG(GPIOPS_DATA_0) |= 1 << 7;
    	sleep(1);
    	/* Set MIO7 as Low */
    	REG(GPIOPS_DATA_0) &= ~(1 << 7);
    	sleep(1);
    	/* Set MIO6 as High */
    	REG(GPIOPS_DATA_0) |= 1 << 6;
    	/* Set MIO7 as High */
    	REG(GPIOPS_DATA_0) |= 1 << 7;
    	sleep(1);
    	/* Set MIO6 as Low */
    	REG(GPIOPS_DATA_0) &= ~(1 << 6);
    	sleep(1);
    	/* Set MIO7 as Low */
    	REG(GPIOPS_DATA_0) &= ~(1 << 7);
		/* input is on */
		while (REG(GPIOPS_DATA_0) & 0x1) {
    	    /* Set MIO6 as High */
    	    REG(GPIOPS_DATA_0) |= 1 << 6;
    	    sleep(1);
    	    /* Set MIO6 as Low */
    	    REG(GPIOPS_DATA_0) &= ~(1 << 6);
    	    sleep(1);			
        }
    }

    cleanup_platform();
    return 0;
}