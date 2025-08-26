/*
 * Thread.hpp
 *
 *  Created on: Sep 6, 2023
 *      Author: YassineBakkali
 */

#ifndef THREAD_HPP_
#define THREAD_HPP_


#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "semphr.h"

class Thread {
public:
	Thread(const char* name);
	Thread(const char* name, osPriority priority);
	Thread(const char* name, uint32_t stackSize);
	Thread(const char* name, osPriority priority, uint32_t stackSize);
	virtual ~Thread() {};
	virtual void init() = 0;
	virtual void loop() = 0;
	osThreadId getHandle();
	bool isRunning() { return running; }
	void terminate();
	void setTickDelay(uint32_t ms);
	uint32_t getTickDelay();

private:
	osThreadId handle;
	const char* name;
	bool running = true;
	void task();
	osThreadAttr_t thread_attributes;
	uint32_t delay;
};


#endif /* THREAD_HPP_ */
