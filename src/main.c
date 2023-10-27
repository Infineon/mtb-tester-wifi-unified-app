/*
 * Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */
 /** @file
 *
 * Description: This is the source code for mfg_test app example in ModusToolbox.
 *
 * Related Document: See README.md
  *
 */

/* Header file includes */
#include "cyhal.h"
#include "cybsp.h"
#include <cybsp_wifi.h>
#include "cy_retarget_io.h"
#ifndef OS_THREADX
#include "FreeRTOS.h"
#include "task.h"
#include <lwip/tcpip.h>
#include <lwip/api.h>
#endif /* OS_THREADX */
#include "command_console.h"

#include "cy_wcm.h"
#include "whd.h"
#include "cy_network_mw_core.h"
#include "iperf_utility.h"
#include "mfg_test_common_api.h"
#include "wifi_utility.h"
#ifndef H1CP_SUPPORT
#include "wifi_app_intf.h"
#ifdef LPA_ENABLE
#include "lpa_app_config.h"
#include "lpa_offload.h"
#endif /* LPA_ENABLE */
#endif /* H1CP_SUPPORT */
#include <stdarg.h>

/******************************************************************************
* Macros
******************************************************************************/

/* Task parameters for MfgTest app task. */
#define MFG_TEST_CLIENT_TASK_PRIORITY       (2)

#ifndef H1CP_SUPPORT
#define MFG_TEST_CLIENT_TASK_STACK_SIZE     (1024 * 2)
#else
#define MFG_TEST_CLIENT_TASK_STACK_SIZE     (1024 * 4)
#endif

/* LPA task stack size and priority */
#define NETWORK_ACTIVITY_TASK_STACK_SIZE            (256)
#define NETWORK_ACTIVITY_TASK_PRIORITY              (CY_RTOS_PRIORITY_MAX)

#define WCM_INITIALIZED                  (1lu << 0)

/*
 * Command console buffers
 */
#define CONSOLE_COMMAND_MAX_PARAMS     (32)
#define CONSOLE_COMMAND_MAX_LENGTH     (256)

#ifndef H1CP_SUPPORT
#define CONSOLE_COMMAND_HISTORY_LENGTH (10)
#define CONSOLE_THREAD_PRIORITY        (2)
#else
#define CONSOLE_COMMAND_HISTORY_LENGTH (5)
#define CONSOLE_THREAD_PRIORITY        (CY_RTOS_PRIORITY_NORMAL)
#endif

/*
 * CPU CLOCK FREQUENCY
 */
#define CPU_CLOCK_FREQUENCY 144000000

/* Peripheral clock dividier */
#define CPU_PERI_CLOCK_DIV (2)

#define CHECK_SIGMA_APP_RETURN(expr)  { \
        cy_rslt_t rslt = (expr); \
        if (rslt != CY_RSLT_SUCCESS) \
        { \
           return rslt; \
        } \
}

/* Macro to check whether the result of an operation was successful.  When
 * it has failed, print the error message and return EXIT_FAILURE to the
 * calling function.
 */
#define CHECK_RESULT(result, init_mask, error_message...)   \
                     do                                     \
                     {                                      \
                         if ((int)result != CY_RSLT_SUCCESS)   \
                         {                                  \
                             printf(error_message);         \
                             return;                        \
                         }                                  \
                     } while(0)

/* Macro to get the Wifi Unified Tester application version.
 */
#define GET_WIFI_UNIFIED_VER(str) #str
#define GET_WIFI_UNIFIED_STRING(str) GET_WIFI_UNIFIED_VER(str)
#define GET_WIFI_UNIFIED_VER_STRING  GET_WIFI_UNIFIED_STRING(WIFI_UNIFIED_VER)

#define GET_WIFI_BT_VER(str) #str
#define GET_WIFI_BT_STRING(str) GET_WIFI_BT_VER(str)
#define GET_WIFI_BT_VER_STRING  GET_WIFI_BT_STRING(WIFI_BT_VER)

#define GET_WIFI_MFG_VER(str) #str
#define GET_WIFI_MFG_STRING(str) GET_WIFI_MFG_VER(str)
#define GET_WIFI_MFG_VER_STRING  GET_WIFI_MFG_STRING(WIFI_MFG_VER)

#define GET_WIFI_MFG_MW_VER(str) #str
#define GET_WIFI_MFG_MW_STRING(str) GET_WIFI_MFG_MW_VER(str)
#define GET_WIFI_MFG_MW_VER_STRING  GET_WIFI_MFG_MW_STRING(WIFI_MFG_MW_VER)

#define GET_WIFI_CERT_VER(str) #str
#define GET_WIFI_CERT_STRING(str) GET_WIFI_CERT_VER(str)
#define GET_WIFI_CERT_VER_STRING  GET_WIFI_CERT_STRING(WIFI_CERT_VER)

#ifdef LPA_ENABLE
#define GET_WIFI_LPA_VER(str) #str
#define GET_WIFI_LPA_STRING(str) GET_WIFI_LPA_VER(str)
#define GET_WIFI_LPA_VER_STRING  GET_WIFI_LPA_STRING(WIFI_LPA_VER)
#endif

/** WL UART baud rate */
#define WL_UART_BAUDRATE             (115200)

/******************************************************************************
* Function prototypes
*******************************************************************************/
cy_rslt_t cy_wcm_get_whd_interface(cy_wcm_interface_t interface_type, whd_interface_t *whd_iface);
static cy_rslt_t command_console_init_task(void);

#ifndef DISABLE_WL_SUPPORT
static void mfg_test_client_task(void *pvParameters);
cy_rslt_t wl_uart_init(cyhal_gpio_t tx, cyhal_gpio_t rx, uint32_t baudrate);
__attribute__((aligned(8))) uint8_t mfgtest_thread_stack[MFG_TEST_CLIENT_TASK_STACK_SIZE] = {0};
#endif /* DISABLE_WL_SUPPORT */

#ifdef LPA_ENABLE
cy_rslt_t suspend_console_and_debug_tasks(void);
cy_rslt_t resume_console_and_debug_tasks(void);
#endif

/******************************************************************************
* Global variables
******************************************************************************/

const char* console_delimiter_string = ",";

static char command_buffer[CONSOLE_COMMAND_MAX_LENGTH];
static char command_history_buffer[CONSOLE_COMMAND_MAX_LENGTH * CONSOLE_COMMAND_HISTORY_LENGTH];

static cy_wcm_config_t wcm_config;

#ifndef DISABLE_WL_SUPPORT
/* UART HAL object used by BSP for wl command UART port */
cyhal_uart_t wl_uart_obj;
/* Global Task Handle for mfg test client task */
cy_thread_t mfg_task_handle = NULL;
#ifdef OS_THREADX
cy_thread_t           wlthread;
#endif /* OS_THREADX */
#endif /* DISABLE_WL_SUPPORT */

/* Enables RTOS-aware debugging. */
volatile int uxTopUsedPriority;
#if defined(__GNUC__) && !(defined(__ICCARM__) || defined(__clang__) || defined(__CC_ARM))
extern volatile uint32_t __HeapBase;
extern volatile uint32_t __HeapLimit;
uint8_t* base = (uint8_t *)&__HeapBase;
uint8_t* end  = (uint8_t *)&__HeapLimit;
#endif

#ifndef H1CP_SUPPORT
/* set CPU clock frequency */
#if (CYHAL_API_VERSION >= 2)
cy_rslt_t set_cpu_clock_v2 ( uint32_t freq );
#else // HAL API version 1
cy_rslt_t set_cpu_clock ( uint32_t freq );
#endif
#endif

#ifndef OS_THREADX
/*
 * Called first after the initialization of FreeRTOS. It basically connects to Wi-Fi and then starts the
 * task that waits for DHCP.  No tasks will actually run until this function returns.
 */
void vApplicationDaemonTaskStartupHook()
{
    cy_rslt_t result;
    wcm_config.interface = CY_WCM_INTERFACE_TYPE_AP_STA;

    /* Initialize the Wi-Fi connection manager and return if the operation fails. */
    result = cy_wcm_init(&wcm_config);

    CHECK_RESULT(result, WCM_INITIALIZED, "\r\nWi-Fi Connection Manager initialization failed!\r\n");
    printf("\r\nWi-Fi Connection Manager initialized.\r\n");

    /* Initialize command console params and task */
    result = command_console_init_task();
    if ( result != CY_RSLT_SUCCESS ) {
       printf ("Error in command console thread init : %ld \n", (long)result);
    }

#ifndef DISABLE_WL_SUPPORT
    /* Create the mfg_test client task. */
    xTaskCreate(mfg_test_client_task, "Mfg-test task", MFG_TEST_CLIENT_TASK_STACK_SIZE,
                NULL, MFG_TEST_CLIENT_TASK_PRIORITY, &mfg_task_handle);
#endif /* DISABLE_WL_SUPPORT */

#ifdef LPA_ENABLE
    /*
     * Suspend/Resume network stack.
     * This task will cause PSoC 6 MCU to go into deep-sleep power mode. The PSoC 6 MCU
     * wakes up only when the network stack resumes due to Tx/Rx activity detected
     * by the Offload Manager (OLM).
     */
    xTaskCreate(network_idle_task, "low-power task", NETWORK_ACTIVITY_TASK_STACK_SIZE,
                NULL, NETWORK_ACTIVITY_TASK_PRIORITY, NULL);
#endif

}
#else
void init_console_and_wl_threads()
{
    cy_rslt_t result;
    wcm_config.interface = CY_WCM_INTERFACE_TYPE_AP_STA;

#ifdef H1CP_SUPPORT
    /* In H1 CP the main app booting from ROM is itself a Thread, in specific is the mpaf_thread_Main thread */
    TX_THREAD* Thread_Main;
    Thread_Main = tx_thread_identify();
    UINT old_prio = 0;
    tx_thread_priority_change(Thread_Main, CY_RTOS_PRIORITY_BELOWNORMAL, &old_prio);
#endif /* H1CP_SUPPORT */

    /* Initialize the Wi-Fi connection manager and return if the operation fails. */
    result = cy_wcm_init(&wcm_config);

    CHECK_RESULT(result, WCM_INITIALIZED, "\r\nWi-Fi Connection Manager initialization failed!\r\n");
    printf("\r\nWi-Fi Connection Manager initialized.\r\n");

    /* Initialize command console params and task */
    result = command_console_init_task();
    if ( result != CY_RSLT_SUCCESS ) {
       printf ("Error in command console thread init : %ld \n", (long)result);
    }

#ifndef DISABLE_WL_SUPPORT
    /* Create the mfg_test client task. */
    result = cy_rtos_thread_create(&wlthread, (cy_thread_entry_fn_t)mfg_test_client_task, "Mfg-test task", (void *)mfgtest_thread_stack,
                                    MFG_TEST_CLIENT_TASK_STACK_SIZE, CY_RTOS_PRIORITY_NORMAL, (cy_thread_arg_t)NULL);
    if ( result != CY_RSLT_SUCCESS ) {
       printf ("Error in wl init thread : %ld \n", (long)result);
    }
#endif /* DISABLE_WL_SUPPORT */
}
#endif /* OS_THREADX */

/************************************************************************************
* Main initializes the hardware and low power support, and starts the task scheduler
*************************************************************************************/
int main()
{
    cy_rslt_t result ;

#ifndef OS_THREADX
    /* Enables RTOS-aware debugging in OpenOCD */
    uxTopUsedPriority = configMAX_PRIORITIES - 1;
#endif

    /* Initializes the board support package */
    result = cybsp_init() ;
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Dummy read to avoid warning for Release build */
    (void) result;

    /* Enables global interrupts */
    __enable_irq();

#ifndef H1CP_SUPPORT
/* set CPU clock to CPU_CLOCK_FREQUENCY */
#if (CYHAL_API_VERSION >= 2)
    result = set_cpu_clock_v2(CPU_CLOCK_FREQUENCY);
#else
    result = set_cpu_clock(CPU_CLOCK_FREQUENCY);
#endif
    CY_ASSERT(result == CY_RSLT_SUCCESS) ;
#endif /* H1CP_SUPPORT */

    /* Initializes retarget-io to use the debug UART port */
#ifndef H1CP_SUPPORT
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
#ifndef DISABLE_WL_SUPPORT
    wl_uart_init(CYBSP_MIKROBUS_UART_TX, CYBSP_MIKROBUS_UART_RX, CY_RETARGET_IO_BAUDRATE);
#endif /* DISABLE_WL_SUPPORT */
#else
    cy_retarget_io_init(BT_UART_TXD , BT_UART_RXD , 115200);
#ifndef DISABLE_WL_SUPPORT
    wl_uart_init(BT_GPIO_3, BT_GPIO_2, 115200);
#endif /* DISABLE_WL_SUPPORT */
#endif /* H1CP_SUPPORT */

    /* Clears the screen */
    printf("\x1b[2J\x1b[;H");
    printf("=============================================\r\n");
    printf("FreeRTOS Unified Test Application\r\n") ;
    printf("Wi-Fi Unified App : %s\r\n", GET_WIFI_UNIFIED_VER_STRING);
    printf("- Wi-Fi Bluetooth Tester : %s\r\n", GET_WIFI_BT_VER_STRING);
    printf("- Wi-Fi Mfg Tester       : %s\r\n", GET_WIFI_MFG_VER_STRING);
    printf("- Wi-Fi Mfg Middleware   : %s\r\n", GET_WIFI_MFG_MW_VER_STRING);
#ifndef H1CP_SUPPORT
    printf("- Wi-Fi Cert Tester      : %s\r\n", GET_WIFI_CERT_VER_STRING);
#endif /* H1CP_SUPPORT */
#ifdef LPA_ENABLE
    printf("- Wi-Fi LPA Middleware   : %s\r\n", GET_WIFI_LPA_VER_STRING);
#endif
    printf("=============================================\r\n");

#ifndef OS_THREADX
    /* Starts the FreeRTOS scheduler */
    vTaskStartScheduler() ;
#else
    init_console_and_wl_threads();
#endif /* OS_THREADX */

#ifndef OS_THREADX
    /* Should never get here */
    CY_ASSERT(0);
#else
    /* For threadx when the main exits the child threads are not killed */
    return 0;
#endif
}

static cy_rslt_t command_console_init_task(void)
{
    cy_rslt_t result;
    cy_command_console_cfg_t console_cfg;

#ifndef OS_THREADX
    /* Set I/O to No Buffering */
    setvbuf( stdin, NULL, _IONBF, 0 );

    /* Set I/O to No Buffering */
    setvbuf( stdout, NULL, _IONBF, 0 );
#endif

    console_cfg.serial             = (void *)&cy_retarget_io_uart_obj;
    console_cfg.line_len           = sizeof(command_buffer);
    console_cfg.buffer             = command_buffer;
    console_cfg.history_len        = CONSOLE_COMMAND_HISTORY_LENGTH;
    console_cfg.history_buffer_ptr = command_history_buffer;
    console_cfg.delimiter_string   = console_delimiter_string;
    console_cfg.params_num         = CONSOLE_COMMAND_MAX_PARAMS;
    console_cfg.thread_priority    = (cy_thread_priority_t)CONSOLE_THREAD_PRIORITY;

    /* Initialize command console library */
    result = cy_command_console_init(&console_cfg);
    if ( result != CY_RSLT_SUCCESS ) {
       printf ("Error in command console init task : %ld \n", (long)result);
       return result;
    }

    /* Initialize Wi-Fi utility and add Wi-Fi commands */
    result = wifi_utility_init();
    if ( result != CY_RSLT_SUCCESS ) {
       printf ("Error in wifi initialization : %ld \n", (long)result);
       return result;
    }

#ifndef H1CP_SUPPORT /* For CP, cert will be enabled later */
    /* Initialize SigmaDUT library */
    result = cywifi_init_sigmadut();
    if ( result != CY_RSLT_SUCCESS )
    {
       printf ("Error in initializing command console library : %ld \n", (long)result);
    }
#endif

#ifndef OS_THREADX
    printf("CY_SRAM_SIZE     : %lu\n", CY_SRAM_SIZE);
#if defined(__GNUC__) && !(defined(__ICCARM__) || defined(__clang__) || defined(__CC_ARM))
    printf("Heap Base        : %p\n", base);
    printf("Heap size        : %d\n", (end - base));
#else
    printf("Heap size        : %d\n", configTOTAL_HEAP_SIZE);
#endif
    printf("SystemCoreClock  : %u\n", (unsigned int)SystemCoreClock);
#endif /* OS_THREADX */

#ifndef DISABLE_COMMAND_CONSOLE_IPERF
    /* Initialize IPERF utility and add IPERF commands */
    iperf_utility_init(&wcm_config.interface);
#endif

    return result;

}

#ifndef DISABLE_WL_SUPPORT
/******************************************************************************
 * Function Name: mfg_test_client_task
 ******************************************************************************
 * Summary:
 *  Task for handling the initialization and connection of Wi-Fi and the MQTT Client.
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void mfg_test_client_task(void *pvParameters)
{
    cy_rslt_t result;
    whd_interface_t sta_interface = NULL;

    /* Avoid compiler warnings */
    (void)pvParameters;

#ifndef OS_THREADX
    /* Set I/O to No Buffering */
    setvbuf( stdin, NULL, _IONBF, 0 );

    /* Set I/O to No Buffering */
    setvbuf( stdout, NULL, _IONBF, 0 );
#endif

    /* Configure the interface as a Wi-Fi STA (i.e. Client). */
    result = cy_wcm_get_whd_interface(CY_WCM_INTERFACE_TYPE_STA, &sta_interface);
    if (result != CY_RSLT_SUCCESS) {
        printf("\r\nWifi STA Interface Not found, Wi-Fi Connection Manager initialization failed!\r\n");
        return;
    }
    wl_set_sta_interface_handle(sta_interface);
    printf("\r\nSTA interface initialized for Mfg Test Application.\r\n");

    while (true)
    {
        wl_remote_command_handler();
    }

    /* Should never get here */
}

//////////////////////The APIs for UART2//////////////////////
static cy_mutex_t wl_uart_mutex;
static bool       wl_uart_mutex_initialized = false;
//--------------------------------------------------------------------------------------------------
// cy_retarget_io_mutex_init
//--------------------------------------------------------------------------------------------------
static cy_rslt_t wl_uart_mutex_init(void)
{
    cy_rslt_t rslt;
    if (wl_uart_mutex_initialized)
    {
        rslt = CY_RSLT_SUCCESS;
    }
    else if (CY_RSLT_SUCCESS == (rslt = cy_rtos_init_mutex(&wl_uart_mutex)))
    {
        wl_uart_mutex_initialized = true;
    }
    return rslt;
}


//--------------------------------------------------------------------------------------------------
// cy_retarget_io_getchar
//--------------------------------------------------------------------------------------------------
cy_rslt_t wl_uart_getchar(char* c)
{
#ifdef H1CP_SUPPORT
    while(1)
    {
        if (cy_isreadable((void *)&wl_uart_obj) == true)
        {
            return cyhal_uart_getc(&wl_uart_obj, (uint8_t*)c, 0);
        }
        //tx_thread_relinquish();
        cy_rtos_delay_milliseconds(1);
    }
#else
    return cyhal_uart_getc(&wl_uart_obj, (uint8_t*)c, 0);
#endif /* H1CP_SUPPORT */
}

//--------------------------------------------------------------------------------------------------
// cy_retarget_io_putchar
//--------------------------------------------------------------------------------------------------
cy_rslt_t wl_uart_putchar(char c)
{
    return cyhal_uart_putc(&wl_uart_obj, (uint8_t)c);
}

cy_rslt_t wl_uart_init(cyhal_gpio_t tx, cyhal_gpio_t rx, uint32_t baudrate)
{
    const cyhal_uart_cfg_t uart_config =
    {
        .data_bits          = 8,
        .stop_bits          = 1,
        .parity             = CYHAL_UART_PARITY_NONE,
        .rx_buffer          = NULL,
        .rx_buffer_size     = 0
    };

#if (CYHAL_API_VERSION >= 2)
    cy_rslt_t result =
        cyhal_uart_init(&wl_uart_obj, tx, rx, NC, NC, NULL, &uart_config);
#else // HAL API version 1
    cy_rslt_t result = cyhal_uart_init(&wl_uart_obj, tx, rx, NULL, &uart_config);
#endif

    if (result == CY_RSLT_SUCCESS)
    {
        result = cyhal_uart_set_baud(&wl_uart_obj, baudrate, NULL);
    }

    if (result == CY_RSLT_SUCCESS)
    {
        result = wl_uart_mutex_init();
    }

    return result;
}
#endif /* DISABLE_WL_SUPPORT */

#ifdef LPA_ENABLE
/* For LPA, the IDLE task to be triggered to make MCU to sleep and SDIO clock to disable.
 * if console or debug tasks are running and it will keep on polling UART, which makes the
 * MCU always wake and idle task is not getting triggered, in order to make MCU to enter deep,
 * we have to disable UART and other any debug task, if it is running
 */
cy_rslt_t suspend_console_and_debug_tasks(void)
{
    extern cy_thread_t console_task_handle;

    if(mfg_task_handle != NULL)
    {
        vTaskSuspend(mfg_task_handle);
    }
    else
    {
        printf("Suspending mfg task handle failed \n");
        return CY_RTOS_BAD_PARAM;
    }

    if(console_task_handle != NULL)
    {
        vTaskSuspend(console_task_handle);
    }
    else
    {
        printf("Suspending Console task handle failed \n");
        return CY_RTOS_BAD_PARAM;
    }

    return CY_RSLT_SUCCESS;
}
cy_rslt_t resume_console_and_debug_tasks(void)
{
    extern cy_thread_t console_task_handle;

    if(console_task_handle != NULL)
    {
        vTaskResume(console_task_handle);
    }
    else
    {
        printf("Resuming Console task handle failed \n");
        return CY_RTOS_BAD_PARAM;
    }

    if(mfg_task_handle != NULL)
    {
        vTaskResume(mfg_task_handle);
    }
    else
    {
        printf("Resuming mfg task handle failed \n");
        return CY_RTOS_BAD_PARAM;
    }

    return CY_RSLT_SUCCESS;
}
#endif /* LPA_ENABLE */

#ifndef H1CP_SUPPORT
#if (CYHAL_API_VERSION >= 2)
cy_rslt_t set_cpu_clock_v2 ( uint32_t freq )
{
    cy_rslt_t ret = CY_RSLT_SUCCESS;
    uint32_t old_freq;
    cyhal_clock_t clock_pll, clock_hf0 , clock_peri;

    old_freq = cyhal_clock_get_frequency(&CYHAL_CLOCK_HF[0]);
    if ( freq != old_freq )
    {
        /* Initialize, take ownership of, the PLL instance */
        CHECK_SIGMA_APP_RETURN(cyhal_clock_reserve(&clock_pll, &CYHAL_CLOCK_PLL[0]));

        /* Set CPU clock to freq */
        CHECK_SIGMA_APP_RETURN(cyhal_clock_set_frequency(&clock_pll, freq, NULL));

        /* If the PLL is not already enabled, enable it */
        if (!cyhal_clock_is_enabled(&clock_pll))
        {
            CHECK_SIGMA_APP_RETURN(cyhal_clock_set_enabled(&clock_pll, true, true));
        }
        /* Initialize, take ownership of, the PERI instance */
        CHECK_SIGMA_APP_RETURN(cyhal_clock_reserve(&clock_peri, &CYHAL_CLOCK_PERI));

        /* Initialize, take ownership of, the HF0 instance */
        CHECK_SIGMA_APP_RETURN(cyhal_clock_reserve(&clock_hf0, &CYHAL_CLOCK_HF[0]));

        /* Set peri clock divider */
        CHECK_SIGMA_APP_RETURN(cyhal_clock_set_divider(&clock_peri, CPU_PERI_CLOCK_DIV));

        /* set HF0 Clock source to PLL */
        CHECK_SIGMA_APP_RETURN(cyhal_clock_set_source(&clock_hf0, &clock_pll));
    }
    return ret;
}
#else // HAL API version 1
cy_rslt_t set_cpu_clock ( uint32_t freq )
{
    cy_rslt_t ret = CY_RSLT_SUCCESS;
    uint32_t old_freq;
    cyhal_clock_t clock_pll, clock_hf0 , clock_peri;

    /* Get the HF0 resource */
    CHECK_SIGMA_APP_RETURN(cyhal_clock_get(&clock_hf0 , (const cyhal_resource_inst_t *)&CYHAL_CLOCK_HF));

    old_freq = cyhal_clock_get_frequency(&clock_hf0);
    if ( freq != old_freq )
    {
        /* Get the PLL and PERI resource */
        CHECK_SIGMA_APP_RETURN(cyhal_clock_get(&clock_pll,  (const cyhal_resource_inst_t *)&CYHAL_CLOCK_PLL));
        CHECK_SIGMA_APP_RETURN(cyhal_clock_get(&clock_peri, (const cyhal_resource_inst_t *)&CYHAL_CLOCK_PERI));

        /* Initialize, take ownership of, the PLL instance */
        CHECK_SIGMA_APP_RETURN(cyhal_clock_init(&clock_pll));

        /* Set CPU clock to freq */
        CHECK_SIGMA_APP_RETURN(cyhal_clock_set_frequency(&clock_pll, freq, NULL));

        /* If the PLL is not already enabled, enable it */
        if (!cyhal_clock_is_enabled(&clock_pll))
        {
            CHECK_SIGMA_APP_RETURN(cyhal_clock_set_enabled(&clock_pll, true, true));
        }

        /* Initialize, take ownership of, the PERI instance */
        CHECK_SIGMA_APP_RETURN(cyhal_clock_init(&clock_peri));

        /* Initialize, take ownership of, the HF0 instance */
        CHECK_SIGMA_APP_RETURN(cyhal_clock_init(&clock_hf0));

        /* Set peri clock divider */
        CHECK_SIGMA_APP_RETURN(cyhal_clock_set_divider(&clock_peri, CPU_PERI_CLOCK_DIV));

        /* set HF0 Clock source to PLL */
        CHECK_SIGMA_APP_RETURN(cyhal_clock_set_source(&clock_hf0, &clock_pll));
    }
    return ret;
}
#endif
#endif

#if defined(__GNUC__) && !(defined(__ICCARM__) || defined(__clang__) || defined(__CC_ARM))
void optimized_wordsize_memcpy(void *dst, const void *src, size_t len)
{
    volatile unsigned int *pdest1 = (unsigned int *)dst;
    volatile unsigned int *psrc1  = (unsigned int*)src;
    volatile unsigned char *psrc;
    volatile unsigned char *pdest;
    int index;

    if ( (len != 0 ) && ( len > 4 ))
    {
        do
        {
            *pdest1 = *psrc1;
            pdest1++;
            psrc1++;
            len -= 4;
        } while ( len >= 4 );
   }
   psrc  = (unsigned char *)psrc1;
   pdest = (unsigned char *)pdest1;

   for ( index = 0; index < len; index++)
   {
       *pdest = *psrc;
       pdest++;
       psrc++;
   }
}
#endif

/* [] END OF FILE */
