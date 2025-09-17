/*
  Reads 1st channel data from a Novus DigiRail-2A analog input module coonected  to uart using modbus and sends to another radio unit using BLE 
  using Nordic NUS protocol

  Modify for UART and BLE as follows :-
  
  prj.conf
  ========
  # Use bluetooth
  CONFIG_BT=y
  CONFIG_BT_DEVICE_NAME="NCS NUS"
  # Enable the NUS service
  CONFIG_BT_NUS=y
  # use UART serial
  CONFIG_SERIAL=y
  CONFIG_UART_ASYNC_API=y
  # low power management for UART
  CONFIG_PM=y
  CONFIG_PM_DEVICE=y
  CONFIG_PM_DEVICE_RUNTIME=y
  CONFIG_CAF_POWER_MANAGER=y
  # enable printk
  CONFIG_PRINTK=y

  deviceTree either 0 or 20 depends on model
&uart20 {
    status = "okay";
    compatible = "nordic,nrf-uarte";
    current-speed = <115200>;
    parity = <UART_CFG_PARITY_EVEN>;  
    stop-bits = <UART_CFG_STOP_BITS_1>;  
    data-bits = <UART_CFG_DATA_BITS_8>;  
    rx-pin = <8>;
    tx-pin = <6>;
    rts-pin = <5>;
    cts-pin = <7>;
    hw-flow-control;  
};
  
  Example product :-
  https://www.braveridge.com/product/
  https://www.switch-science.com/products/2803/
  https://www.digikey.co.uk/en/products/detail/nordic-semiconductor-asa/NRF51-DK/5022449
  
  lower power uart ref:-
  https://qiita.com/Kosuke_Matsui/items/b0eefe66fb627df8c72b

 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <sys/printk.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/nus.h>
#include <drivers/uart.h>
#include <settings/settings.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>

#include <stdint.h>                                              // to use stdint defs
#include "mbproto.h"                                             // modbus library include
#define _ENDIAN_SWAP                                             // uncomment for ENDIAN byte SWAP if required
#define _RAW                                                     // get raw analog value
//#define _SCALED                                                   get scaled float value

#if defined(CONFIG_BOARD_NRF54L15DK_NRF54L15_CPUAPP) || defined(CONFIG_BOARD_NRF54L15DK_NRF54L15_CPUAPP_NS)
#define UART_ID uart20
#else
#define UART_ID uart0
#endif
#define TRANS_BUFF_SIZE 7
#if defined(_RAW)
#define RECEIVE_BUFF_SIZE 6
#elif defined(_SCALE)
#define RECEIVE_BUFF_SIZE 8
#endif

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define MSGINT_TO_SEND(X,a)  do{ sprintf(X, "Data : %d", a); } while(0)
#define MSGFLT_TO_SEND(X,a)  do{ sprintf(X, "Data : %f", a); } while(0)

// states for receiving and sending on the uart port, and re-transmitting through BLE NUS protocol
#define USENDING 1
#define URECVED 2
#define SENTBLE 3
	
static uint8_t tx_buf[TRANS_BUFF_SIZE] = {0};
static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = {0};
// define variable for modbus data
static uint8_t mbusSendMsg[TRANS_BUFF_SIZE];                           // send message bytes to request data from the slave
static uint8_t mbusRcvMsg[RECEIVE_BUFF_SIZE];                          // for raw 16 bit register
static uint8_t mbusFloatRcvMsg[RECEIVE_BUFF_SIZE];                     // for float 32 bit register 
static uint16_t crc = 0;                                               // crc
static uint8_t crc1 = 0;
static uint8_t crc2 = 0;
static uint16_t HReg1Val = 0;                                          // variable containing raw integer counts value
static ModbusRTUDaniels_u floatAsUnion;                                // variable containing scaled float value as an integer or float
static ModbusReadHoldingRegsReq_t mhReq;                               // struct for holding HReg Read Request
static ModbusReadHoldingRegsResp_t mhRsp;                              // struct for holding HReg Read Response 
#define RECEIVE_TIMEOUT 100
#define SLEEP_TIME_MS 1000
volatile uint8_t tx_recv = 0;

#if defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP) || defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP_NS)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#else
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
#endif

K_MUTEX_DEFINE(my_mutex);                                                 // define mutex between BLE and UART

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type) {

    case UART_TX_DONE:
#if defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP) || defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP_NS)
		gpio_pin_toggle_dt(&led1);
#elif defined(CONFIG_BOARD_NRF54L15DK_NRF54L15_CPUAPP) || defined(CONFIG_BOARD_NRF54L15DK_NRF54L15_CPUAPP_NS)
		gpio_pin_toggle_dt(&led1);
#else
		gpio_pin_toggle_dt(&led1);
#endif
	    break;

    case UART_TX_ABORTED:
	    break;

    case UART_RX_STOPPED:
        break;

    case UART_RX_BUF_REQUEST:
        break;
    case UART_RX_BUF_RELEASED:
        break;
		
	case UART_RX_RDY:
#if defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP) || defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP_NS)
		if ((evt->data.rx.len) == 1)  {
			if ((evt->data.rx.buf[evt->data.rx.offset] == '1') || (evt->data.rx.buf[evt->data.rx.offset] == '2')) {
				gpio_pin_toggle_dt(&led0);
			} 
		}
#elif defined(CONFIG_BOARD_NRF54L15DK_NRF54L15_CPUAPP) || defined(CONFIG_BOARD_NRF54L15DK_NRF54L15_CPUAPP_NS)
		if ((evt->data.rx.len) == 1) {
			if ((evt->data.rx.buf[evt->data.rx.offset] == '0') || else if (evt->data.rx.buf[evt->data.rx.offset] == '1')) {
				gpio_pin_toggle_dt(&led0);
			} else if (evt->data.rx.buf[evt->data.rx.offset] == '2') {
				gpio_pin_toggle_dt(&led2);
			}
        }
#else
		if ((evt->data.rx.len) == 1) {
			if ((evt->data.rx.buf[evt->data.rx.offset] == '1') || (evt->data.rx.buf[evt->data.rx.offset] == '2')) {
				gpio_pin_toggle_dt(&led0);
			} else if (evt->data.rx.buf[evt->data.rx.offset] == '3') {
				gpio_pin_toggle_dt(&led2);
			}
		}
#endif
		break;
		
	case UART_RX_DISABLED:
#if defined(_RAW)        // response bytes for integer raw
    uart_rx_enable(dev, mbusRcvMsg, sizeof mbusRcvMsg, RECEIVE_TIMEOUT);
#elif defined(_SCALE)
    uart_rx_enable(dev, mbusFloatRcvMsg, sizeof mbusFloatRcvMsg, RECEIVE_TIMEOUT);     // response bytes for float value
#endif
       break;

	case UART_RX_RDY:                                                                 // Some data was received and receive timeout occurred (if RX timeout is enabled) or when the receive buffer is full
#if defined(_RAW)	   
    if (sizeof(mbusRcvMsg) >= RECEIVE_BUFF_SIZE) {
       memcpy(mhRsp, mbusRcvMsg, 4);
       crc = usMBCRC16((unsigned char *) &mhRsp, 4);                                 // calculate crc from data received and check against what has been sent on the end of msg
#if defined(_ENDIAN_SWAP)
       crc = SWAP_UINT16(crc);
#endif
       crc1 = crc >> 8;                                                          
       crc2 = crc & 0x00ff;
       if ((mhRsp.functionCode == MB_FUNC_READ_HOLDING_REGISTER) && ((crc1 == mbusRcvMsg[4]) && (crc2 == mbusRcvMsg[5]))) {
#if defined(_ENDIAN_SWAP)
          mhRsp.regValue[0] = SWAP_UINT16(mhRsp.regValue[0]);
#endif
        if (k_mutex_lock(&my_mutex, K_MSEC(100)) == 0) {
	       HReg1Val = regValue[0];
           printk("Holding Reg Raw %d\n", HReg1Val);
		   tx_recv = URECVED;
        } else {
           printk("Cannot lock mutex\n");
        }
        k_mutex_unlock(&my_mutex);
       } else {
           printk("error in receiving data wrong function code or crc\n")
       }		
    }
#elif defined(_SCALED)

    if (sizeof(mbusFloatRcvMsg) >= RECEIVE_BUFF_SIZE) {
       memcpy(mhRsp, mbusFloatRcvMsg, 6);
       crc = usMBCRC16((unsigned char *) &mhRsp, 6);
#if defined(_ENDIAN_SWAP)
       crc = SWAP_UINT16(crc);
#endif
       crc1 = crc >> 8; 
       crc2 = crc & 0x00ff;
       if ((mhRsp.functionCode == MB_FUNC_READ_HOLDING_REGISTER) && ((crc1 == mbusFloatRcvMsg[6]) && (crc2 == mbusFloatRcvMsg[7]))) {
        if (k_mutex_lock(&my_mutex, K_MSEC(100)) == 0) {
		   floatAsUnion.holdingReg = (mhRsp.regValue[0] << 16) | (mhRsp.regValue[1] & 0xFFFF);
#if defined(_ENDIAN_SWAP)
	       floatAsUnion.holdingReg = SWAP_UINT32(floatAsUnion.holdingReg);
#endif
	       printk("Scaled Value %f\n", floatAsUnion.danielsFloat);
		   tx_recv = URECVED;
        } else {
           printk("Cannot lock mutex\n");
        }
        k_mutex_unlock(&my_mutex);

       } else {
           printk("error in receiving data wrong function code or crc\n")
       }		
    }
#endif
		break;

    case UART_RX_BUF_RELEASED:
	   printk("completed recv\n");
	   break;

	default:
		break;
	}
}

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
    uint8_t buf[256] = {0};
    printk("Received data\n");
    memcpy(buf, data, len);
    printk("%s", buf);
    printk("\n");
}

static struct bt_nus_cb nus_cb = {
    .received = bt_receive_cb,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    printk("Connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected\n");
}

static struct bt_conn_cb conn_callbacks = {
    .connected    = connected,
    .disconnected = disconnected,
};

void main(void)
{
                   
   mhReq.functionCode = MB_FUNC_READ_HOLDING_REGISTER;             // eMBFunctionCode.MODBUS_FUNCTION_READ_HOLDING_REGS
   uint16_t stAddrRaw = NOVUS_DR2A_PV_CH1;                         // address of raw integer value of NOVUS DigiRail-2A Universal Analog Input Module 
   uint16_t stAddrScaled = NOVUS_DR2A_ENGV_CH1;                    // address scaled float value of NOVUS DigiRail-2A Universal Analog Input Module 

   // for bleData
   uint8_t bleData[15];
   uint16_t bleLen = 10;
   strcpy(bleData, "Data : NaN \n");	                           // initialise to no data message
	
    // set-up UART
    const struct device *uart= DEVICE_DT_GET(DT_NODELABEL(UART_ID));
    if (!device_is_ready(uart)){
		printk("UART device not ready\r\n");
		return 1;
	}
    // set-up GPIO
	if (!device_is_ready(led0.port)){
		printk("GPIO device is not ready\r\n");
		return 1;
	}
    if (gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE) < 0) {
		return 1 ; 
	}
	if (gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE) < 0) {
		return 1 ;
	}
	if (gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE) < 0) {
		return 1 ;
	}	
	if (uart_callback_set(uart, uart_cb, NULL) != 0) {
        return 1;
    }
#if defined(_RAW)
    // modbus read raw value
    uint16_t qtyReg = 1;
    mhReq.startingAddr = stAddrRaw;
    mhReq.quantityOfRegs = qtyReg;
#if defined(_ENDIAN_SWAP)
    mhReq.startingAddr = SWAP_UINT16(mhReq.startingAddr);
    mhReq.quantityOfRegs = SWAP_UINT16(mhReq.quantityOfRegs);
#endif
    memcpy(&mbusSendMsg, &mbReg, 5);
    crc = usMBCRC16((unsigned char *) &mbusSendMsg, 5);
#if defined(_ENDIAN_SWAP)
    crc = SWAP_UINT16(crc);
#endif
    mbusSendMsg[5] = crc >> 8;                              // tag crc on end of send message
    mbusSendMsg[6] = crc & 0x00ff; 
    memcpy(&tx_buf, &mbusSendMsg, 7);
#elif defined(_SCALED)
    // read float value
    uint16_t qtyReg = 2;
    mhReq.quantityOfRegs = qtyReg;
    mhReq.startingAddr = stAddrScaled;
#if defined(_ENDIAN_SWAP)
    mhReq.startingAddr = SWAP_UINT16(mhReq.startingAddr);
    mhReq.quantityOfRegs = SWAP_UINT16(mhReq.quantityOfRegs);
#endif
    memcpy(&mbusSendMsg, &mbReg, 5);
    crc = usMBCRC16((unsigned char *) &mbusSendMsg, 5);
#if defined(_ENDIAN_SWAP)
    crc = SWAP_UINT16(crc);
#endif
    mbusSendMsg[5] = crc >> 8;                                          // tag crc on end of send message
    mbusSendMsg[6] = crc & 0x00ff; 
#endif  

    // transmit the message (NO TIMEOUT)
	if (uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_US) != 0) {
		ptintk("error tx of message through uart\n");
		return 1;                                                                        // exit if we cant send first message
	} else {
		tx_recv = USENDING;
    }
    // receive back call back
	if (uart_rx_enable(uart ,rx_buf,sizeof rx_buf,RECEIVE_TIMEOUT) != 0) {
		return 1;
	}
	
    // send using nordic NUS protocol BLE
    int err = 0;

    if (bt_enable(NULL) != 0) {
        printk("Blutooth failed to start (err %d)\n", err);
        return;
    }

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    if (bt_nus_init(&nus_cb) != 0) {
        printk("Failed to initialize Nordic UART service (err: %d)\n", err);
        return;
    }

    bt_conn_cb_register(&conn_callbacks);

    if (bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0) != 0) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }

    while (1)
    {
		if (tx_recv == USENDING) {                                             // wait for a new message to be received
            k_msleep(100);		
		} else if (tx_recv == URECVED) {
		    // we got new data so disable UART to save power ? (bug suggests do twice)
		    uart_rx_disable(uart);
            pm_device_action_run(uart, PM_DEVICE_ACTION_SUSPEND);
            k_msleep(100);
            pm_device_action_run(uart, PM_DEVICE_ACTION_RESUME);
            k_msleep(100);
            pm_device_action_run(uart, PM_DEVICE_ACTION_SUSPEND);

            if (k_mutex_lock(&my_mutex, K_MSEC(100)) == 0) {                   // waits up to 100 milliseconds for the mutex to become available
#if defined(_RAW)                                                              // get the data received on the uart
	            MSGINT_TO_SEND(bleData, HReg1Val);
#elif defined(_SCALED)
	            MSGFLT_TO_SEND(bleData, floatAsUnion.danielsFloat);
#endif
            } else {
			    printk("cannot lock mutex\n");
            }
            k_mutex_unlock(&my_mutex);
		    bleLen = sizeof(bleData);
            if (bt_nus_send(NULL, bleData, bleLen))                             // Send over BLE
            {
                printk("Failed to send data over BLE connection\n");
            }
            k_msleep(1000);
			tx_recv = SENTBLE;
			
			// re-enable UART for next message (bug suggests do twice)
            uart_rx_enable(uart);
            pm_device_action_run(uart, PM_DEVICE_ACTION_RESUME);
            k_msleep(100);
            pm_device_action_run(uart, PM_DEVICE_ACTION_SUSPEND);
            k_msleep(100);
            pm_device_action_run(uart, PM_DEVICE_ACTION_RESUME);
		} else if (tx_recv == SENTBLE) {		
            // transmit the message (NO TIMEOUT)
	        if (uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_US) != 0) {
		       printk("error tx\n");
	        } else {
               tx_recv = USENDING;
            }
            k_msleep(1000);
		}
    }
}



