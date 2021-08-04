/*
 * svpwm.cpp
 *
 *  Created on: 2021/08/03
 *      Author: yusaku
 */

#include "svpwm.h"

void SVPWM_Calc(q31_t theta, q31_t modFactor, UVW *duty) {
	// Refer to Application Note AN2154 from STMicroelectronics.

	constexpr uint8_t states[7] = {0b001, 0b011, 0b010, 0b110, 0b100, 0b101, 0b001};
	constexpr q31_t fracPi3 = (0x80000000 / 3) + 1;		// PI/3 plus 1, to make sector < 6.
	constexpr q31_t frac_1_root3 = Q31(1 / 1.73205080756887729352f);

	uint32_t sector = (uint32_t)theta / (uint32_t)fracPi3;
    uint8_t state_a = states[sector];
    uint8_t state_b = states[sector + 1];

    q31_t alpha = theta - (fracPi3 * sector);

	q31_t mul = ((q63_t)modFactor * frac_1_root3) >> 31;
    q31_t state_a_duty = (((q63_t)mul * arm_sin_q31((fracPi3 - alpha)/2)) >> 31);
    q31_t state_b_duty = (((q63_t)mul * arm_sin_q31(alpha/2)) >> 31);
    //q31_t state_o_duty = ((Q31(1.0f) - state_a_duty) - state_b_duty);

    //state_a_duty += state_o_duty/2;
    //state_b_duty += state_o_duty/2;

    // Calculate total fraction of time to drive each of the UVW phases.
    // Note that one of these will always be 0: for any given position SVPWM
    // in general drives two of the phases and keeps the other grounded.
    /*
    duty->U =
		(((state_a & 0b100) != 0) ? state_a_duty : 0) +
		(((state_b & 0b100) != 0) ? state_b_duty : 0);
    duty->V =
		(((state_a & 0b010) != 0) ? state_a_duty : 0) +
		(((state_b & 0b010) != 0) ? state_b_duty : 0);
    duty->W =
		(((state_a & 0b001) != 0) ? state_a_duty : 0) +
		(((state_b & 0b001) != 0) ? state_b_duty : 0);
	*/

    q31_t t0 = Q31(0.5f) - state_a_duty/2 - state_b_duty/2;
    q31_t t1 = Q31(0.5f) + state_a_duty/2 - state_b_duty/2;
    q31_t t2 = Q31(0.5f) - state_a_duty/2 + state_b_duty/2;
    q31_t t3 = Q31(0.5f) + state_a_duty/2 + state_b_duty/2;

    switch(sector) {
    	case 0:
    	    duty->U = t0;
			duty->V = t2;
    		duty->W = t3;
    	break;
    	case 1:
    	    duty->U = t0;
			duty->V = t3;
    		duty->W = t1;
    	break;
    	case 2:
    	    duty->U = t2;
			duty->V = t3;
    		duty->W = t0;
    	break;
    	case 3:
    	    duty->U = t3;
			duty->V = t1;
    		duty->W = t0;
    	break;
    	case 4:
    	    duty->U = t3;
			duty->V = t0;
    		duty->W = t2;
    	break;
    	case 5:
    	    duty->U = t1;
			duty->V = t0;
    		duty->W = t3;
    	break;
    }
}
