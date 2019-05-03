#ifndef REGISTERS_H
#define	REGISTERS_H

#include <stdint.h>
/**
 * Define registers map
 */
struct registers {
    /**
     * Set read-only register for device id
     * 
     * @see DEVICE_ID
     */
    uint8_t device;
    
    /**
     * Set read-only register for firmware version
     * 
     * @see FIRMWARE_VER
     */
    uint8_t firmware;
    
    /**
     * Set read-only register for serial number
     * 
     * @see SERIAL
     */
    uint32_t serial;
    
    /**
     * Set pin function. Can be;
     * 1 - ADC/DAC
     * 0 - GPIO
     * 
     * @note This apply only for PIN0 and PIN2
     * 
     * Default value: 0x00
     */
    uint8_t function;
    
    /**
     * Set direction. Can be:
     * 1 - input
     * 0 - output
     * 
     * Default value: 0xFF
     */
    uint8_t dir;
    
    
    /**
     * Get input level
     */
    uint8_t input;
    
    /**
     * Set output level
     */
    uint8_t output;
    
    /**
     * Enable disable internal pullups.
     * Only usable if pins are defined as inputs.
     * 1 - Enabled PU
     * 0 - Disabled PU
     * 
     * Default value: 0xFF
     */
    uint8_t pullup;
    
    /**
     * Set outputs as push-pull or open-drain.
     * Only usable for outputs.
     * 1 - Open-drain
     * 0 - Push-pull
     * 
     * Default value: 0x00
     */
    uint8_t mode;
    
    /**
     * @brief Input threshold control
     * 
     * Set input buffer type. Input port can be configured either as Schmitt 
     * Trigger or TTL. 
     * 
     * Values can be:
     *  1 - ST input
     *  0 - TTL input
     * 
     * Default value: 0xFF
     */
    uint8_t buffer;
    
    /**
     * @brief Slew rate control
     * 
     * Sets input slew rate.
     * 
     * Values can be:
     *  1 - Port pin slew rate is limited
     *  0 - Port pin slews at maximum rate
     * 
     * Default value: 0xFF
     */
    uint8_t slew;
    
    
    /**
     * @brief Enable interrupt generation
     * 
     * Values can be:
     *  1 - Enables INT generation
     *  0 - Disables INT generation
     * 
     * Default value: 0x00
     */
    uint8_t interrupt_enable;
    
    union {
        uint16_t interrupt_sense;
        struct {
            /**
             * @brief Configure interrupt sense for GPIO0 - GPIO3
             * 
             * Values can be:
             *  0b00 - No edge detect
             *  0b01 - Rising edge
             *  0b10 - Falling edge             
             *  0b11 - Both
             * 
             * Default value: 0x00
             */
            uint8_t interrupt_sense_lo;
            
            /**
             * @brief Configure interrupt sense for GPIO4 - GPIO7
             * 
             * Values can be:
             *  0b00 - No edge detect
             *  0b01 - Rising edge
             *  0b10 - Falling edge             
             *  0b11 - Both
             * 
             * Default value: 0x00
             */
            uint8_t interrupt_sense_hi;
        };
    };
    
    /**
     * @brief Read interrupt status
     */
    uint8_t interrupt_status;
    
    
};

extern struct registers regmap;
extern uint8_t *pointer;

#endif	/* REGISTERS_H */

