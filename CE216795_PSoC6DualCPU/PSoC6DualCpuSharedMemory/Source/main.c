/*****************************************************************************
* File Name: main_cm4.c
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

#include "ce216795_common.h"

static volatile uint32_t mySysError;  /* general-purpose status/error code */

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
* The main function for the Cortex-M4 CPU does the following:<br>
*  Initialization:<br>
*   - Initializes IPC channel for sharing a variable with the other CPU.<br>
*  Do forever loop:<br>
*   - Waits for the 4 MS bits of the shared variable to not equal the 4 LS bits.
*     The other CPU increments the 4 LS bits.<br>
*   - sets the 4 MS bits to equal the 4 LS bits.
*
*******************************************************************************/
int main(void)
{
    IPC_STRUCT_Type *myIpcHandle; /* handle for the IPC channel being used */

    /* A global shared variable is defined in the codefor the other CPU.
       Its address is given to the this CPU via an IPC channel. */
    uint8_t *sharedVar;

    /* Set up the device based on configurator selections */
//    init_cycfg_all();

    __enable_irq();
 
    /* Read the shared memory address, from the other CPU. Wait for success, which indicates that
       (1) the sending CPU acquired the lock, and (2) this CPU read the pointer. */
    myIpcHandle = Cy_IPC_Drv_GetIpcBaseAddress(MY_IPC_CHANNEL);
    while (Cy_IPC_Drv_ReadMsgPtr(myIpcHandle, (void *)&sharedVar) != CY_IPC_DRV_SUCCESS);
    /* Release the lock. This indicates to the other CPU that the read is
       successfully complete. */
    mySysError = Cy_IPC_Drv_LockRelease(myIpcHandle, CY_IPC_NO_NOTIFICATION);
    if (mySysError != CY_IPC_DRV_SUCCESS) HandleError();

    /* Initialize the IPC semaphore subsystem. The other CPU already defined the semaphore
       array address, so this CPU just initializes the IPC channel number. */
    if (Cy_IPC_Sema_Init(CY_IPC_CHAN_SEMA, (uint32_t)NULL, (uint32_t *)NULL) != CY_IPC_SEMA_SUCCESS) HandleError();

    for(;;)
    {
        /* Always use semaphore set/clear functions to access the shared variable;
           do computations on a local copy of the shared variable. */
        uint8_t copy;

        /* Wait for the 4 MS bits of the shared variable to NOT equal the 4 LS bits.
           The other CPU increments the 4 LS bits. */
        do
        {
            /* grab a copy of the shared variable, with semaphore set/clear */
            mySysError = ReadSharedVar(sharedVar, &copy);
            if (mySysError != (uint32_t)CY_IPC_SEMA_SUCCESS) HandleError();
            /* brief delay to give the other CPU a chance to acquire the semaphore */
            CyDelayUs(1ul/*usec*/);
        } while (((copy >> 4) & 0x0Fu) == (copy & 0x0Fu));

        /* Make the 4 MS bits of the shared variable equal the 4 LS bits. */
        copy = ((copy & 0x0Fu) << 4) | (copy & 0x0Fu);
        /* write the shared variable, with semaphore set/clear */
        mySysError = WriteSharedVar(sharedVar, copy);
        if (mySysError != (uint32_t)CY_IPC_SEMA_SUCCESS) HandleError();
    }
}
