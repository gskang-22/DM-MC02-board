/*
 * Thread.cpp
 *
 *  Created on: Sep 6, 2023
 *      Author: YassineBakkali
 */

#include "usart.h"

#include <stdarg.h>
#include <string.h>
#include <Thread.h>

#define DEFAULT_STACK_SIZE (2048) // Please be wary when creating new threads.
								  // We're already using more than 70%
								  // of the RAM :)

static char buffer[128];

void __task_run(void* arg) {
	Thread* thread = (Thread*) arg;

	osDelay(100 / portTICK_PERIOD_MS);

	thread->init();

	while(thread->isRunning()) {
		thread->loop();
		osDelay(1 / portTICK_PERIOD_MS);
	}

	delete thread;

	vTaskDelete(nullptr);

	while(true) {
		osDelay(1000 / portTICK_PERIOD_MS);
	}
}

Thread::Thread(const char* name) : Thread(name, (osPriority) osPriorityNormal) {
	;
}

Thread::Thread(const char* name, osPriority priority) : Thread(name, priority, DEFAULT_STACK_SIZE) {
	;
}

Thread::Thread(const char* name, uint32_t stackSize) : Thread(name, (osPriority) osPriorityNormal, stackSize) {
	;
}

Thread::Thread(const char* name, osPriority priority, uint32_t stackSize) {
	const osThreadAttr_t thread_attributes = {
	  .name = (char*) name,
	  .stack_size = stackSize,
	  .priority = (osPriority_t) priority,
	};
	this->handle = (osThreadId) osThreadNew(&__task_run, this, &thread_attributes);
	this->name = name;
}

osThreadId Thread::getHandle() {
	return handle;
}

void Thread::terminate() {
	this->running = false;
}


void Thread::setTickDelay(uint32_t ms) {
	this->delay = ms;
}

uint32_t Thread::getTickDelay() {
	return this->delay;
}


