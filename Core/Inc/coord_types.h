/*
 * coord_types.h
 *
 *  Created on: 2021/08/03
 *      Author: yusaku
 */

#ifndef INC_COORD_TYPES_H_
#define INC_COORD_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif



typedef struct {
	q31_t U;
	q31_t V;
	q31_t W;
} UVW;



#ifdef __cplusplus
}
#endif

#endif /* INC_COORD_TYPES_H_ */
