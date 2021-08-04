/*
 * svpwm.h
 *
 *  Created on: 2021/08/03
 *      Author: yusaku
 */

#ifndef INC_SVPWM_H_
#define INC_SVPWM_H_

#include "arm_math.h"
#include "coord_types.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif


void SVPWM_Calc(q31_t theta, q31_t modFactor, UVW *duty);


#ifdef __cplusplus
}
#endif


#endif /* INC_SVPWM_H_ */
