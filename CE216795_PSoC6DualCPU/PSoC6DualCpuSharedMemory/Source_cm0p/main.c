/*****************************************************************************
* File Name: main.c for Cortex-M0+ CPU
*
* Version: 1.10
*
* Description: Demonstrates sharing of a region of SRAM memory between two
* CPUs.
*
* Note: The main.c files for both of the CPUs in the PSoC 6 are part of the
* same PSoC Creator project. They are both compiled at project build time.
* The resultant .hex file has the code for both CPUs. When the project
* executes, the code in main_cm0p.c starts executing first. The Cortex-M0+
* code then turns on the Cortex-M4 CPU, and the code in main_cm4.c starts
* executing.
*
* Related Document: Code example CE216795
*
* Hardware Dependency: See code example CE216795
*
******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
******************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*****************************************************************************/
#include "cy_device_headers.h"
#include "cy_syslib.h"
#include "cy_syspm.h"

#include "ce216795_common.h"

/* Global shared variable is static within this file. The address of this
   variable is given to the CM4 via an IPC channel. */
static uint8_t sharedVar;

static volatile uint32_t mySysError;  /* general-purpose status/error code */

static uint32_t myArray[4]; /* for Cy_IPC_Sema_Init() */


/*******************************************************************************
* Function Name: HandleError
****************************************************************************//**
*
* Placeholder function for handling IPC and other errors. Does nothing -
* implements halt on error.
*
*******************************************************************************/
void HandleError(void)
{
    static uint8_t i;
    /* for now, just halt on error */
    while(1)
    {
        i++;
    }
}


/*******************************************************************************
* Function Name: main
****************************************************************************//**
*
* The main function for the Cortex-M0+ CPU does the following:
*  Initialization:
*   - Enables the other CPU.
*   - Initializes IPC channel for sharing a variable with the other CPU.
*  Do forever loop:
*   - Increments the 4 LS bits of the shared variable, without changing the 4 MS
*     bits.
*   - Waits for the 4 MS bits of the shared variable to equal the 4 LS bits. The
*     other CPU sets the 4 MS bits to be equal to the 4 LS bits.
*
*******************************************************************************/
int main(void)
{
	IPC_STRUCT_Type *myIpcHandle; /* handle for the IPC channel being used */

    /* Always use semaphore set/clear functions to access the shared variable;
       do computations on a local copy of the shared variable. */
    uint8_t copy;

    /* Set up the device based on configurator selections */
    init_cycfg_all();

    /* enable interrupts */
    __enable_irq();

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    /* Do the first read of the shared variable, preparatory to modifying and
       writing it for the first time in the main loop.
       This initial read must be done before notifying the CM4. */
    copy = sharedVar;

    /* Initialize the IPC semaphore subsystem. This must done by this CPU, with definition of
       semaphore array address, before the other CPU starts using the semaphore system. */
    if (Cy_IPC_Sema_Init(CY_IPC_CHAN_SEMA, sizeof(myArray) * 8ul/*bits per byte*/, myArray) != CY_IPC_SEMA_SUCCESS)
    {
        HandleError();
    }

    /* Enable CM4. Do not do so until after IPC semaphore initialization (see above).
       CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
    Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR);

    /* Notify the CM4 with the shared memory address. There is no interrupt associated with this notification.
       Wait for channel to be available, then acquire the lock for it, in a single atomic operation;
       and send the address. */
    myIpcHandle = Cy_IPC_Drv_GetIpcBaseAddress(MY_IPC_CHANNEL);
    while(Cy_IPC_Drv_SendMsgPtr(myIpcHandle, CY_IPC_NO_NOTIFICATION, &sharedVar) != CY_IPC_DRV_SUCCESS);
    /* Wait for release, which indicates other CPU has read the pointer value. */
    while (Cy_IPC_Drv_IsLockAcquired(myIpcHandle));

    for (;;)
    {
        /* Increment the 4 LS bits of the shared variable, without changing the 4 MS bits. */
        copy = (copy & 0xF0u) | ((copy + 1u) & 0x0Fu);
        /* write the shared variable, with semaphore set/clear */
        mySysError = WriteSharedVar(&sharedVar, copy);
        if (mySysError != (uint32_t)CY_IPC_SEMA_SUCCESS) HandleError();

        /* Wait for the 4 MS bits of the shared variable to equal the 4 LS bits.
           The other CPU sets the 4 MS bits to be equal to the 4 LS bits. */
        do
        {
            /* grab a copy of the shared variable, with semaphore set/clear */
            mySysError = ReadSharedVar(&sharedVar, &copy);
            if (mySysError != (uint32_t)CY_IPC_SEMA_SUCCESS) HandleError();
            /* brief delay to give the other CPU a chance to acquire the semaphore */
            CyDelayUs(1ul/*usec*/);
        } while (((copy >> 4) & 0x0Fu) != (copy & 0x0Fu));
    }
}
