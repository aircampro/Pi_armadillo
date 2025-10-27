/* diligent zybo API for GPIO example (custom to zybo) */
# include <stdio.h>
# include "platform.h"
# include "xil_printf.h"
# include "sleep.h"
# include "xgpiops.h"

#define ACTION_TIMEOUT 10000
/* steps of action for the gpio */
typedef enum {
    START_TRAVERSE,
    BEGIN_ACTION,
    STOP_TRAVERSE,
    CONTINUE_ACTION,
    STOP_ACTION
} seq_steps_e;

int main()
{
    int step = 0;
    int timeout_count = 0;
    init_platform();

    print("Zybo sequence example using API\n\r");

    XGpioPs instXGpioPs;
    XGpioPs_Config *configXGpioPs;

    configXGpioPs = XGpioPs_LookupConfig(XPAR_PS7_GPIO_0_DEVICE_ID);
    XGpioPs_CfgInitialize(&instXGpioPs, configXGpioPs,configXGpioPs->BaseAddr);
    /* Set MIO7 as output */
    XGpioPs_SetDirectionPin(&instXGpioPs, 7, 1);
    XGpioPs_SetOutputEnablePin(&instXGpioPs, 7, 1);
    /* Set MIO6 as output */
    XGpioPs_SetDirectionPin(&instXGpioPs, 6, 1);
    XGpioPs_SetOutputEnablePin(&instXGpioPs, 6, 1);
    /* Set MIO2 as input */
    XGpioPs_SetDirectionPin(&instXGpioPs, 2, 0);
    XGpioPs_SetOutputEnablePin(&instXGpioPs, 2, 1);
    /* Set MIO3 as input */
    XGpioPs_SetDirectionPin(&instXGpioPs, 3, 0);
    XGpioPs_SetOutputEnablePin(&instXGpioPs, 3, 1);
    while(1) {
		/* sequence state engine */
		while (step == START_TRAVERSE) {
    	    /* Set MIO7 as High */
    	    XGpioPs_WritePin(&instXGpioPs, 7, 1);
    	    /* Read MIO2 until High */
            if (XGpioPs_ReadPin(&instXGpioPs, 2)) {
				step = BEGIN_ACTION;
            }			
        }
		while (step == BEGIN_ACTION) {
    	    /* Set MIO6 as High */
    	    XGpioPs_WritePin(&instXGpioPs, 6, 1);
    	    /* Read MIO2 until Low */
            if (!XGpioPs_ReadPin(&instXGpioPs, 2)) {
				step = STOP_TRAVERSE;
            }			
        }
		while (step == STOP_TRAVERSE) {
    	    /* Set MIO7 as Low */
    	    XGpioPs_WritePin(&instXGpioPs, 7, 0);
    	    /* Read MIO3 until High */
            if (XGpioPs_ReadPin(&instXGpioPs, 3)) {
				step = CONTINUE_ACTION;
            }	
        }
		while (step == CONTINUE_ACTION) {
    	    /* Read MIO3 until Low */
            if ((!XGpioPs_ReadPin(&instXGpioPs, 3)) || (timeout_count > ACTION_TIMEOUT)) {
				step = STOP_ACTION;
				timeout_count = 0;
            }	
			timeout_count++;
        }	
		while (step == STOP_ACTION) {
    	    /* Set MIO7 as Low */
    	    XGpioPs_WritePin(&instXGpioPs, 6, 0);	
    	    sleep(10);
			step = START_TRAVERSE;
        }		
    }

    cleanup_platform();
    return 0;
}