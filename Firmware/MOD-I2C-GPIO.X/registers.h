#ifndef REGISTERS_H
#define	REGISTERS_H

#include <stdint.h>
/**
 * Define registers map
 */
struct registers {
    /**
     * Set direction. Can be:
     * 1 - output
     * 0 - input
     * 
     * Default value: 0x00
     */
    uint8_t dir;
    
    
    /**
     * Set/Get current port data
     */
    uint8_t data;
    
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
};

extern struct registers regmap;
extern uint8_t *pointer;

#endif	/* REGISTERS_H */

