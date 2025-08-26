/*
 * wrapper.h
 *
 *  Created on: May 29, 2025
 *      Author: cw
 */

#ifndef THREADS_INC_WRAPPER_H_
#define THREADS_INC_WRAPPER_H_

// For cpp functions that needs to be called by c files
#ifdef __cplusplus
extern "C" {
#endif

void telemetry_setup_wrapper(); // For telemetry::setup called in freertos.c

#ifdef __cplusplus
}
#endif



#endif /* THREADS_INC_WRAPPER_H_ */
