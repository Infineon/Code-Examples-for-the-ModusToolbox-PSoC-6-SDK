/*
* Copyright YOUR COMPANY, THE YEAR
* All Rights Reserved
* UNPUBLISHED, LICENSED SOFTWARE.
*
* CONFIDENTIAL AND PROPRIETARY INFORMATION
* WHICH IS THE PROPERTY OF your company.
*/

#include "cy_device_headers.h"
#include "cycfg.h"

#include "cy_dfu.h"
#include "dfu_user.h"
#include <string.h>


/*******************************************************************************
* Function Name: CopyRow
********************************************************************************
* Copies data from a "src" address to a flash row with the address "dest".
* If "src" data is the same as "dest" data then no copy is needed.
*
* Parameters:
*  dest     Destination address. Has to be an address of the start of flash row.
*  src      Source address. Has to be properly aligned.
*  rowSize  Size of flash row.
*
* Returns:
*  CY_BOOTLAOD_SUCCESS if operation is successful.
*  Error code in a case of failure.
*******************************************************************************/
cy_en_bootload_status_t CopyRow(uint32_t dest, uint32_t src, uint32_t rowSize, cy_stc_bootload_params_t * params)
{
    cy_en_bootload_status_t status;

    /* Save params->dataBuffer value */
    uint8_t *buffer = params->dataBuffer;

    /* Compare "dest" and "src" content */
    params->dataBuffer = (uint8_t *)src;
    status = Cy_Bootload_ReadData(dest, rowSize, CY_BOOTLOAD_IOCTL_COMPARE, params);

    /* Restore params->dataBuffer */
    params->dataBuffer = buffer;

    /* If "dest" differs from "src" then copy "src" to "dest" */
    if (status != CY_BOOTLOAD_SUCCESS)
    {
        (void) memcpy((void *) params->dataBuffer, (const void*)src, rowSize);
        status = Cy_Bootload_WriteData(dest, rowSize, CY_BOOTLOAD_IOCTL_WRITE, params);
    }
    /* Restore params->dataBuffer */
    params->dataBuffer = buffer;

    return (status);
}


/*******************************************************************************
* Function Name: HandleMetadata
********************************************************************************
* The goal of this function is to make Bootloader SDK metadata (MD) valid.
* The following algorithm is used (in C-like pseudocode):
* ---
* if (isValid(MD) == true)
* {   if (MDC != MD)
*         MDC = MD;
* } else
* {   if(isValid(MDC) )
*         MD = MDC;
*     else
*         MD = INITIAL_VALUE;
* }
* ---
* Here MD is metadata flash row, MDC is flash row with metadata copy,
* INITIAL_VALUE is known initial value.
*
* In this code example MDC is placed in the next flash row after the MD, and
* INITIAL_VALUE is MD with only CRC, App0 start and size initialized,
* all the other fields are not touched.
*
* Parameters:
*  params   A pointer to a Bootloader SDK parameters structure.
*
* Returns:
* - CY_BOOTLOAD_SUCCESS when finished normally.
* - Any other status code on error.
*******************************************************************************/
cy_en_bootload_status_t HandleMetadata(cy_stc_bootload_params_t *params)
{
    const uint32_t MD     = (uint32_t)(&__cy_boot_metadata_addr   ); /* MD address  */
    const uint32_t mdSize = (uint32_t)(&__cy_boot_metadata_length ); /* MD size, assumed to be one flash row */
    const uint32_t MDC    = MD + mdSize;                             /* MDC address */

    cy_en_bootload_status_t status = CY_BOOTLOAD_SUCCESS;

    status = Cy_Bootload_ValidateMetadata(MD, params);
    if (status == CY_BOOTLOAD_SUCCESS)
    {
        /* Checks if MDC equals to DC, if no then copies MD to MDC */
        status = CopyRow(MDC, MD, mdSize, params);
    }
    else
    {
        status = Cy_Bootload_ValidateMetadata(MDC, params);
        if (status == CY_BOOTLOAD_SUCCESS)
        {
            /* Copy MDC to MD */
            status = CopyRow(MD, MDC, mdSize, params);
        }
        if (status != CY_BOOTLOAD_SUCCESS)
        {
            const uint32_t elfStartAddress = 0x10000000;
            const uint32_t elfAppSize      = 0x8000;
            /* Set MD to INITIAL_VALUE */
            status = Cy_Bootload_SetAppMetadata(0u, elfStartAddress, elfAppSize, params);
        }
    }
    return (status);
}


/*******************************************************************************
* Function Name: counterTimeoutSeconds
********************************************************************************
* Returns number of counts that correspond to number of seconds passed as
* a parameter.
* E.g. comparing counter with 300 seconds is like this.
* ---
* uint32_t counter = 0u;
* for (;;)
* {
*     Cy_SysLib_Delay(UART_TIMEOUT);
*     ++count;
*     if (count >= counterTimeoutSeconds(seconds: 300u, timeout: UART_TIMEOUT))
*     {
*         count = 0u;
*         DoSomething();
*     }
* }
* ---
*
* Both parameters are required to be compile time constants,
* so this function gets optimized out to single constant value.
*
* Parameters:
*  seconds    Number of seconds to pass. Must be less that 4_294_967 seconds.
*  timeout    Timeout for Cy_Bootload_Continue() function, in milliseconds.
*             Must be greater than zero.
*             It is recommended to be a value that produces no reminder
*             for this function to be precise.
* Return:
*  See description.
*******************************************************************************/
static uint32_t counterTimeoutSeconds(uint32_t seconds, uint32_t timeout);
static uint32_t counterTimeoutSeconds(uint32_t seconds, uint32_t timeout)
{
    return (seconds * 1000ul) / timeout;
}


int main(void)
{
    /* timeout for Cy_Bootload_Continue(), in milliseconds */
    const uint32_t paramsTimeout = 20u;

    /* Bootloader params, used to configure bootloader */
    cy_stc_bootload_params_t bootParams;

    /* Status codes for Bootloader SDK API */
    cy_en_bootload_status_t status;

    /*
    * Bootloading state, one of
    * - CY_BOOTLOAD_STATE_NONE
    * - CY_BOOTLOAD_STATE_BOOTLOADING
    * - CY_BOOTLOAD_STATE_FINISHED
    * - CY_BOOTLOAD_STATE_FAILED
    */
    uint32_t state;

    /*
    * Used to count seconds, to convert counts to seconds use
    * counterTimeoutSeconds(SECONDS, paramsTimeout)
    */
    uint32_t count = 0;

#if CY_BOOTLOAD_OPT_CRYPTO_HW != 0
    cy_en_crypto_status_t cryptoStatus;
#endif

    /* Buffer to store bootloader commands */
    CY_ALIGN(4) static uint8_t buffer[CY_BOOTLOAD_SIZEOF_DATA_BUFFER];

    /* Buffer for bootloader packets for Transport API */
    CY_ALIGN(4) static uint8_t packet[CY_BOOTLOAD_SIZEOF_CMD_BUFFER ];

    /* Set up the device based on configurator selections */
    init_cycfg_all();

    __enable_irq();

#if CY_BOOTLOAD_OPT_CRYPTO_HW != 0
    /* Initialize the Crypto Client code */
    cryptoStatus = Cy_Crypto_Init(&cryptoConfig, &cryptoContext);
    if (cryptoStatus != CY_CRYPTO_SUCCESS)
    {
        /* Crypto not initialized, debug what is the problem */
        Cy_SysLib_Halt(0x00u);
    }
#endif /* CY_BOOTLOAD_OPT_CRYPTO_HW != 0 */

    /* Initialize bootParams structure and Bootloader SDK state */
    bootParams.timeout          = paramsTimeout;
    bootParams.dataBuffer       = &buffer[0];
    bootParams.packetBuffer     = &packet[0];

    status = Cy_Bootload_Init(&state, &bootParams);

    /* Ensure Bootloader Metadata is valid */
    status = HandleMetadata(&bootParams);
    if (status != CY_BOOTLOAD_SUCCESS)
    {
        Cy_SysLib_Halt(0x00u);
    }

    /*
    * In the case of non-software reset check if there is a valid app image.
    * If these is - switch to it.
    */

    if (Cy_SysLib_GetResetReason() != CY_SYSLIB_RESET_SOFT)
    {
        status = Cy_Bootload_ValidateApp(1u, &bootParams);
        if (status == CY_BOOTLOAD_SUCCESS)
        {
            /*
            * Clear the reset reason because Cy_Bootload_ExecuteApp() performs a
            * software reset. Without clearing it, two reset reasons would be
            * present.
            */
            do
            {
                Cy_SysLib_ClearResetReason();
            }while(Cy_SysLib_GetResetReason() != 0);

            /* Never returns */
            Cy_Bootload_ExecuteApp(1u);
        }
    }

    /* Initialize bootloader communication */
    Cy_Bootload_TransportStart();

    for(;;)
    {
        status = Cy_Bootload_Continue(&state, &bootParams);
        ++count;

        if (state == CY_BOOTLOAD_STATE_FINISHED)
        {
            /* Finished bootloading the application image */

            /* Validate bootloaded application, if it is valid then switch to it */
            status = Cy_Bootload_ValidateApp(1u, &bootParams);
            if (status == CY_BOOTLOAD_SUCCESS)
            {
                Cy_Bootload_TransportStop();
                Cy_Bootload_ExecuteApp(1u);
            }
            else if (status == CY_BOOTLOAD_ERROR_VERIFY)
            {
                /*
                * Restarts Bootloading, an alternatives are to Halt MCU here
                * or switch to the other app if it is valid.
                * Error code may be handled here, i.e. print to debug UART.
                */
                status = Cy_Bootload_Init(&state, &bootParams);
                Cy_Bootload_TransportReset();
            }
        }
        else if (state == CY_BOOTLOAD_STATE_FAILED)
        {
            /* An error has happened during the bootloading process */
            /* Handle it here */

            /* In this Code Example just restart bootloading process */
            status = Cy_Bootload_Init(&state, &bootParams);
            Cy_Bootload_TransportReset();
        }
        else if (state == CY_BOOTLOAD_STATE_BOOTLOADING)
        {
            uint32_t passed5seconds = (count >= counterTimeoutSeconds(5u, paramsTimeout) ) ? 1u : 0u;
            /*
            * if no command has been received during 5 seconds when the bootloading
            * has started then restart bootloading.
            */
            if (status == CY_BOOTLOAD_SUCCESS)
            {
                count = 0u;
            }
            else if (status == CY_BOOTLOAD_ERROR_TIMEOUT)
            {
                if (passed5seconds != 0u)
                {
                    count = 0u;
                    Cy_Bootload_Init(&state, &bootParams);
                    Cy_Bootload_TransportReset();
                }
            }
            else
            {
                count = 0u;
                /* Delay because Transport still may be sending error response to a host */
                Cy_SysLib_Delay(paramsTimeout);
                Cy_Bootload_Init(&state, &bootParams);
                Cy_Bootload_TransportReset();
            }
        }

        /* No image has been received in 300 seconds, try to load existing image, or sleep */
        if( (count >= counterTimeoutSeconds(300u, paramsTimeout) ) && (state == CY_BOOTLOAD_STATE_NONE) )
        {
            /* Stop bootloading communication */
            Cy_Bootload_TransportStop();
            /* Check if app is valid, if it is then switch to it */
            status = Cy_Bootload_ValidateApp(1u, &bootParams);
            if (status == CY_BOOTLOAD_SUCCESS)
            {
                Cy_Bootload_ExecuteApp(1u);
            }
            /* 300 seconds has passed and App is invalid. Handle that */
            Cy_SysLib_Halt(0x00u);
        }

        /* Blink once per two seconds */
        if ( ( count % counterTimeoutSeconds(1u, paramsTimeout) ) == 0u)
        {
            Cy_GPIO_Inv(KIT_RGB_R_PORT, KIT_RGB_R_PIN);
        }

        /* If Button clicked - Switch to App1 if it is valid */
        if (Cy_GPIO_Read(KIT_BTN1_PORT, KIT_BTN1_PIN) == 0u)
        {
            /* 50 ms delay for button debounce on button press */
            Cy_SysLib_Delay(50u);

            if (Cy_GPIO_Read(KIT_BTN1_PORT, KIT_BTN1_PIN) == 0u)
            {
                while (Cy_GPIO_Read(KIT_BTN1_PORT, KIT_BTN1_PIN) == 0u)
                {   /* 50 ms delay for button debounce on button release */
                    Cy_SysLib_Delay(50u);
                }

                /* Validate and switch to App1 */
                status = Cy_Bootload_ValidateApp(1u, &bootParams);

                if (status == CY_BOOTLOAD_SUCCESS)
                {
                    Cy_Bootload_TransportStop();
                    Cy_Bootload_ExecuteApp(1u);
                }
            }
        }
    }
}
