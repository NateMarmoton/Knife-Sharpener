/*
 * FreeRTOS Kernel <DEVELOPMENT BRANCH>
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

 /*******************************************************************************
  * This file provides an example FreeRTOSConfig.h header file, inclusive of an
  * abbreviated explanation of each configuration item.  Online and reference
  * documentation provides more information.
  * https://www.freertos.org/a00110.html
  *
  * Constant values enclosed in square brackets ('[' and ']') must be completed
  * before this file will build.
  *
  * Use the FreeRTOSConfig.h supplied with the RTOS port in use rather than this
  * generic file, if one is available.
  ******************************************************************************/

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "main.h" 

#define configCPU_CLOCK_HZ    ( ( unsigned long ) 120000000 )


#define configTICK_RATE_HZ                         	1000
#define configUSE_PREEMPTION                       	1
#define configUSE_TIME_SLICING                     	1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION    	0
#define configMAX_PRIORITIES                       	56
#define configTICK_TYPE_WIDTH_IN_BITS              	TICK_TYPE_WIDTH_32_BITS
#define configIDLE_SHOULD_YIELD                    	1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES      	1
#define configQUEUE_REGISTRY_SIZE                  	0
#define configENABLE_BACKWARD_COMPATIBILITY        	0

#define configMINIMAL_STACK_SIZE                   	128
#define configTOTAL_HEAP_SIZE                      	20000
#define configMAX_TASK_NAME_LEN                    	16
#define configUSE_TRACE_FACILITY                	1

#define configSTACK_DEPTH_TYPE                     size_t
#define configMESSAGE_BUFFER_LENGTH_TYPE           size_t
#define configSTATS_BUFFER_MAX_LENGTH              0xFFFF
	/******************************************************************************/
	/* Software timer related definitions. ****************************************/
	/******************************************************************************/
#define configUSE_TIMERS                1
#define configTIMER_TASK_PRIORITY       (2)
#define configTIMER_TASK_STACK_DEPTH    256
#define configTIMER_QUEUE_LENGTH        10

	/******************************************************************************/
	/* Memory allocation related definitions. *************************************/
	/******************************************************************************/

#define configSUPPORT_STATIC_ALLOCATION              1
#define configSUPPORT_DYNAMIC_ALLOCATION             1

				  /******************************************************************************/
				  /* Interrupt nesting behaviour configuration. *********************************/
				  /******************************************************************************/

				  /* configKERNEL_INTERRUPT_PRIORITY sets the priority of the tick and context
				   * switch performing interrupts.  Not supported by all FreeRTOS ports.  See
				   * https://www.freertos.org/RTOS-Cortex-M3-M4.html for information specific to
				   * ARM Cortex-M devices. */
#define configKERNEL_INTERRUPT_PRIORITY          15

				   /* configMAX_SYSCALL_INTERRUPT_PRIORITY sets the interrupt priority above which
					* FreeRTOS API calls must not be made.  Interrupts above this priority are
					* never disabled, so never delayed by RTOS activity.  The default value is set
					* to the highest interrupt priority (0).  Not supported by all FreeRTOS ports.
					* See https://www.freertos.org/RTOS-Cortex-M3-M4.html for information specific
					* to ARM Cortex-M devices. */
#define configPRIO_BITS 3
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	5
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

					/* Another name for configMAX_SYSCALL_INTERRUPT_PRIORITY - the name used depends
					 * on the FreeRTOS port. */
#define configMAX_API_CALL_INTERRUPT_PRIORITY    0

					 /******************************************************************************/
					 /* Hook and callback function related definitions. ****************************/
					 /******************************************************************************/

					 /* Set the following configUSE_* constants to 1 to include the named hook
					  * functionality in the build.  Set to 0 to exclude the hook functionality from
					  * the build.  The application writer is responsible for providing the hook
					  * function for any set to 1.  See https://www.freertos.org/a00016.html. */
#define configUSE_IDLE_HOOK                   0
#define configUSE_TICK_HOOK                   1
#define configUSE_MALLOC_FAILED_HOOK          0
#define configUSE_DAEMON_TASK_STARTUP_HOOK    0

					  /* Set configUSE_SB_COMPLETED_CALLBACK to 1 to have send and receive completed
					   * callbacks for each instance of a stream buffer or message buffer. When the
					   * option is set to 1, APIs xStreamBufferCreateWithCallback() and
					   * xStreamBufferCreateStaticWithCallback() (and likewise APIs for message
					   * buffer) can be used to create a stream buffer or message buffer instance
					   * with application provided callbacks. Defaults to 0 if left undefined. */
#define configUSE_SB_COMPLETED_CALLBACK       0

					   /* Set configCHECK_FOR_STACK_OVERFLOW to 1 or 2 for FreeRTOS to check for a
						* stack overflow at the time of a context switch.  Set to 0 to not look for a
						* stack overflow.  If configCHECK_FOR_STACK_OVERFLOW is 1 then the check only
						* looks for the stack pointer being out of bounds when a task's context is
						* saved to its stack - this is fast but somewhat ineffective.  If
						* configCHECK_FOR_STACK_OVERFLOW is 2 then the check looks for a pattern
						* written to the end of a task's stack having been overwritten.  This is
						* slower, but will catch most (but not all) stack overflows.  The application
						* writer must provide the stack overflow callback when
						* configCHECK_FOR_STACK_OVERFLOW is set to 1. See
						* https://www.freertos.org/Stacks-and-stack-overflow-checking.html  Defaults to
						* 0 if left undefined. */
#define configCHECK_FOR_STACK_OVERFLOW        2

						/******************************************************************************/
						/* Run time and task stats gathering related definitions. *********************/
						/******************************************************************************/

						/* Set configGENERATE_RUN_TIME_STATS to 1 to have FreeRTOS collect data on the
						 * processing time used by each task.  Set to 0 to not collect the data.  The
						 * application writer needs to provide a clock source if set to 1.  Defaults to
						 * 0 if left undefined.  See https://www.freertos.org/rtos-run-time-stats.html.
						 */
#define configGENERATE_RUN_TIME_STATS           0

						 /* Set to 1 to include the vTaskList() and vTaskGetRunTimeStats() functions in
						  * the build.  Set to 0 to exclude these functions from the build.  These two
						  * functions introduce a dependency on string formatting functions that would
						  * otherwise not exist - hence they are kept separate.  Defaults to 0 if left
						  * undefined. */
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

						  /******************************************************************************/
						  /* Co-routine related definitions. ********************************************/
						  /******************************************************************************/

						  /* Set configUSE_CO_ROUTINES to 1 to include co-routine functionality in the
						   * build, or 0 to omit co-routine functionality from the build. To include
						   * co-routines, croutine.c must be included in the project. Defaults to 0 if
						   * left undefined. */
#define configUSE_CO_ROUTINES              0

						   /* configMAX_CO_ROUTINE_PRIORITIES defines the number of priorities available
							* to the application co-routines. Any number of co-routines can share the same
							* priority. Defaults to 0 if left undefined. */
#define configMAX_CO_ROUTINE_PRIORITIES    1

							/******************************************************************************/
							/* Debugging assistance. ******************************************************/
							/******************************************************************************/

							/* configASSERT() has the same semantics as the standard C assert().  It can
							 * either be defined to take an action when the assertion fails, or not defined
							 * at all (i.e. comment out or delete the definitions) to completely remove
							 * assertions.  configASSERT() can be defined to anything you want, for example
							 * you can call a function if an assert fails that passes the filename and line
							 * number of the failing assert (for example, "vAssertCalled( __FILE__, __LINE__
							 * )" or it can simple disable interrupts and sit in a loop to halt all
							 * execution on the failing line for viewing in a debugger. */

#define configASSERT( x )         ((x) ? (void)0U : vAssertCalled((uint8_t *)__FILE__, __LINE__))

void vAssertCalled(uint8_t* file, uint32_t line);

/******************************************************************************/
/* FreeRTOS MPU specific definitions. *****************************************/
/******************************************************************************/

/* If configINCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS is set to 1 then
 * the application writer can provide functions that execute in privileged mode.
 * See:
 * https://www.freertos.org/a00110.html#configINCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS
 * Defaults to 0 if left undefined.  Only used by the FreeRTOS Cortex-M MPU
 * ports, not the standard ARMv7-M Cortex-M port. */
#define configINCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS    0

 /* Set configTOTAL_MPU_REGIONS to the number of MPU regions implemented on your
  * target hardware.  Normally 8 or 16.  Only used by the FreeRTOS Cortex-M MPU
  * ports, not the standard ARMv7-M Cortex-M port.  Defaults to 8 if left
  * undefined. */
#define configTOTAL_MPU_REGIONS                                   8

  /* configTEX_S_C_B_FLASH allows application writers to override the default
   * values for the for TEX, Shareable (S), Cacheable (C) and Bufferable (B) bits
   * for the MPU region covering Flash.  Defaults to 0x07UL (which means TEX=000,
   * S=1, C=1, B=1) if left undefined.  Only used by the FreeRTOS Cortex-M MPU
   * ports, not the standard ARMv7-M Cortex-M port. */
#define configTEX_S_C_B_FLASH                                     0x07UL

   /* configTEX_S_C_B_SRAM allows application writers to override the default
	* values for the for TEX, Shareable (S), Cacheable (C) and Bufferable (B) bits
	* for the MPU region covering RAM. Defaults to 0x07UL (which means TEX=000,
	* S=1, C=1, B=1) if left undefined.  Only used by the FreeRTOS Cortex-M MPU
	* ports, not the standard ARMv7-M Cortex-M port. */
#define configTEX_S_C_B_SRAM                                      0x07UL

	/* Set configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY to 0 to prevent any privilege
	 * escalations originating from outside of the kernel code itself.  Set to 1 to
	 * allow application tasks to raise privilege.  Defaults to 1 if left undefined.
	 * Only used by the FreeRTOS Cortex-M MPU ports, not the standard ARMv7-M
	 * Cortex-M port. */
#define configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY               1

	 /* Set configALLOW_UNPRIVILEGED_CRITICAL_SECTIONS to 1 to allow unprivileged
	  * tasks enter critical sections (effectively mask interrupts). Set to 0 to
	  * prevent unprivileged tasks entering critical sections.  Defaults to 1 if left
	  * undefined.  Only used by the FreeRTOS Cortex-M MPU ports, not the standard
	  * ARMv7-M Cortex-M port. */
#define configALLOW_UNPRIVILEGED_CRITICAL_SECTIONS                0

	  /* FreeRTOS Kernel version 10.6.0 introduced a new v2 MPU wrapper, namely
	   * mpu_wrappers_v2.c. Set configUSE_MPU_WRAPPERS_V1 to 0 to use the new v2 MPU
	   * wrapper. Set configUSE_MPU_WRAPPERS_V1 to 1 to use the old v1 MPU wrapper
	   * (mpu_wrappers.c). Defaults to 0 if left undefined. */
#define configUSE_MPU_WRAPPERS_V1                                 0

	   /* When using the v2 MPU wrapper, set configPROTECTED_KERNEL_OBJECT_POOL_SIZE to
		* the total number of kernel objects, which includes tasks, queues, semaphores,
		* mutexes, event groups, timers, stream buffers and message buffers, in your
		* application. The application will not be able to have more than
		* configPROTECTED_KERNEL_OBJECT_POOL_SIZE kernel objects at any point of
		* time. */
#define configPROTECTED_KERNEL_OBJECT_POOL_SIZE                   10

		/* When using the v2 MPU wrapper, set configSYSTEM_CALL_STACK_SIZE to the size
		 * of the system call stack in words. Each task has a statically allocated
		 * memory buffer of this size which is used as the stack to execute system
		 * calls. For example, if configSYSTEM_CALL_STACK_SIZE is defined as 128 and
		 * there are 10 tasks in the application, the total amount of memory used for
		 * system call stacks is 128 * 10 = 1280 words. */
#define configSYSTEM_CALL_STACK_SIZE                              1024

		 /* When using the v2 MPU wrapper, set configENABLE_ACCESS_CONTROL_LIST to 1 to
		  * enable Access Control List (ACL) feature. When ACL is enabled, an
		  * unprivileged task by default does not have access to any kernel object other
		  * than itself. The application writer needs to explicitly grant the
		  * unprivileged task access to the kernel objects it needs using the APIs
		  * provided for the same. Defaults to 0 if left undefined. */
#define configENABLE_ACCESS_CONTROL_LIST                          1

		  /******************************************************************************/
		  /* SMP( Symmetric MultiProcessing ) Specific Configuration definitions. *******/
		  /******************************************************************************/

		  /* Set configNUMBER_OF_CORES to the number of available processor cores.
		   * Defaults to 1 if left undefined. */

		   /*
			#define configNUMBER_OF_CORES                     [Num of available cores]
			*/

			/* When using SMP (i.e. configNUMBER_OF_CORES is greater than one), set
			 * configRUN_MULTIPLE_PRIORITIES to 0 to allow multiple tasks to run
			 * simultaneously only if they do not have equal priority, thereby maintaining
			 * the paradigm of a lower priority task never running if a higher priority task
			 * is able to run. If configRUN_MULTIPLE_PRIORITIES is set to 1, multiple tasks
			 * with different priorities may run simultaneously - so a higher and lower
			 * priority task may run on different cores at the same time. */
#define configRUN_MULTIPLE_PRIORITIES             0

			 /* When using SMP (i.e. configNUMBER_OF_CORES is greater than one), set
			  * configUSE_CORE_AFFINITY to 1 to enable core affinity feature. When core
			  * affinity feature is enabled, the vTaskCoreAffinitySet and
			  * vTaskCoreAffinityGet APIs can be used to set and retrieve which cores a task
			  * can run on. If configUSE_CORE_AFFINITY is set to 0 then the FreeRTOS
			  * scheduler is free to run any task on any available core. */
#define configUSE_CORE_AFFINITY                   0

			  /* When using SMP with core affinity feature enabled, set
			   * configTASK_DEFAULT_CORE_AFFINITY to change the default core affinity mask for
			   * tasks created without an affinity mask specified. Setting the define to 1
			   * would make such tasks run on core 0 and setting it to (1 <<
			   * portGET_CORE_ID()) would make such tasks run on the current core. This config
			   * value is useful, if swapping tasks between cores is not supported (e.g.
			   * Tricore) or if legacy code should be controlled. Defaults to tskNO_AFFINITY
			   * if left undefined. */
#define configTASK_DEFAULT_CORE_AFFINITY          tskNO_AFFINITY

			   /* When using SMP (i.e. configNUMBER_OF_CORES is greater than one), if
				* configUSE_TASK_PREEMPTION_DISABLE is set to 1, individual tasks can be set to
				* either pre-emptive or co-operative mode using the vTaskPreemptionDisable and
				* vTaskPreemptionEnable APIs. */
#define configUSE_TASK_PREEMPTION_DISABLE         0

				/* When using SMP (i.e. configNUMBER_OF_CORES is greater than one), set
				 * configUSE_PASSIVE_IDLE_HOOK to 1 to allow the application writer to use
				 * the passive idle task hook to add background functionality without the
				 * overhead of a separate task. Defaults to 0 if left undefined. */
#define configUSE_PASSIVE_IDLE_HOOK               0

				 /* When using SMP (i.e. configNUMBER_OF_CORES is greater than one),
				  * configTIMER_SERVICE_TASK_CORE_AFFINITY allows the application writer to set
				  * the core affinity of the RTOS Daemon/Timer Service task. Defaults to
				  * tskNO_AFFINITY if left undefined. */
#define configTIMER_SERVICE_TASK_CORE_AFFINITY    tskNO_AFFINITY

				  /******************************************************************************/
				  /* ARMv8-M secure side port related definitions. ******************************/
				  /******************************************************************************/

				  /* secureconfigMAX_SECURE_CONTEXTS define the maximum number of tasks that can
				   *  call into the secure side of an ARMv8-M chip.  Not used by any other ports.
				   */
#define secureconfigMAX_SECURE_CONTEXTS        5

				   /* Defines the kernel provided implementation of
					* vApplicationGetIdleTaskMemory() and vApplicationGetTimerTaskMemory()
					* to provide the memory that is used by the Idle task and Timer task
					* respectively. The application can provide it's own implementation of
					* vApplicationGetIdleTaskMemory() and vApplicationGetTimerTaskMemory() by
					* setting configKERNEL_PROVIDED_STATIC_MEMORY to 0 or leaving it undefined. */
#define configKERNEL_PROVIDED_STATIC_MEMORY    1

						  /******************************************************************************/
						  /* Definitions that include or exclude functionality. *************************/
						  /******************************************************************************/

						  /* Set the following configUSE_* constants to 1 to include the named feature in
						   * the build, or 0 to exclude the named feature from the build. */
#define configUSE_TASK_NOTIFICATIONS           1
#define configUSE_MUTEXES                      1
#define configUSE_RECURSIVE_MUTEXES            1
#define configUSE_COUNTING_SEMAPHORES          1
#define configUSE_QUEUE_SETS                   0
#define configUSE_APPLICATION_TASK_TAG         0

						   /* USE_POSIX_ERRNO enables the task global FreeRTOS_errno variable which will
							* contain the most recent error for that task. */
#define configUSE_POSIX_ERRNO                  0

							/* Set the following INCLUDE_* constants to 1 to include the named API function,
							 * or 0 to exclude the named API function.  Most linkers will remove unused
							 * functions even when the constant is 1. */
#define INCLUDE_vTaskPrioritySet               1
#define INCLUDE_uxTaskPriorityGet              1
#define INCLUDE_vTaskDelete                    1
#define INCLUDE_vTaskSuspend                   1
#define INCLUDE_vTaskDelayUntil                1
#define INCLUDE_vTaskDelay                     1
#define INCLUDE_xTaskGetSchedulerState         1
#define INCLUDE_xTaskGetCurrentTaskHandle      1
#define INCLUDE_uxTaskGetStackHighWaterMark    0
#define INCLUDE_xTaskGetIdleTaskHandle         0
#define INCLUDE_eTaskGetState                  0
#define INCLUDE_xTimerPendFunctionCall         0
#define INCLUDE_xTaskAbortDelay                0
#define INCLUDE_xTaskGetHandle                 0
#define INCLUDE_xTaskResumeFromISR             1



/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler


#endif /* FREERTOS_CONFIG_H */