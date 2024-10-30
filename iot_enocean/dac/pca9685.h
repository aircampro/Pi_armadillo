#ifndef __pca_lib
#define __pca_lib

#include <stdint.h>
#include <cmath>
#include <iostream>
#include "i2c-dev-wrapper.h"

namespace pwm_obj {

/**
 * PCA 9685 LED controller
 * @see https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf
 * @see adapted from ref:- https://kimitok.hateblo.jp/entry/2021/04/05/080221
 */
class PCA9685 {
    public:
    /**
     * @param[in] period_us period of sending pulses in microsecconds
     */
    PCA9685( uint32_t period_us, uint8_t _adapter_number = DEFAULT_ADAPTER_NUMBER, uint8_t _device_address = DEFAULT_DEVICE_ADDRESS, uint32_t _oscillator_frequency = DEFAULT_OSCILLATOR_FREQUENCY);
    ~PCA9685() = default;

    /**
     * start sending the pulses of each channel
     * @return 0 on success, otherwise error (see errno)
     */
    int start();

    /**
     * reset the chip
     * @return 0 on success, otherwise error (see errno)
     */
    int reset();

    /**
     * init
     * @param[in] channel channel number within 0-15
     * @param[in] width_us pulse width in microsecconds
     * @return -1 0n error
     */
    int init();
    /**
     * set pulse without delay
     * @param[in] channel channel number within 0-15
     * @param[in] width_us pulse width in microsecconds
     * @return 0 on success, otherwise error (see errno)
     */
    int set_pulse(uint8_t channel, uint32_t width_us);
    /**
     * read
     * @param[in] reg addr
     * @param[in] value read into
     * @return 0 on success, otherwise error (see errno)
     */	
    int read(uint8_t register_address, uint8_t &value);
    /**
     * close the i2c device driver handle 
     */
    void close();	
	
    static const uint8_t DEFAULT_ADAPTER_NUMBER = 0x01;
    static const uint8_t DEFAULT_DEVICE_ADDRESS = 0x40;
    static const uint32_t DEFAULT_OSCILLATOR_FREQUENCY = 25 * 1000 * 1000;

    private:
    uint32_t period_us;
};

}//namespace pwm_obj
#endif
