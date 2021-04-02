/**
 *  \file   GpioTsk.c
 *
 *  \brief
 *
 */

/* this define must precede inclusion of any xdc header file */
#define Registry_CURDESC gpiotsk_desc
#define MODULE_NAME "GpioTsk"

/* xdctools header files */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Registry.h>

/* package header files */
#include <ti/ipc/Ipc.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdio.h>
#include <ti/drv/gpio/GPIO.h>
#include <ti/csl/soc.h>
#include <ti/drv/gpio/soc/GPIO_soc.h>

/* Example/Board Header files */
#include <ti/board/board.h>

/* Port and pin number mask for input and output
   Bits 7-0: Pin number  and Bits 15-8: Port number */
#define GPIO_IN  GPIO_DEVICE_CONFIG(1, 31) // gpio1_31
#define GPIO_OUT GPIO_DEVICE_CONFIG(1, 28) // gpio1_28

#define GPIO_PIN_VAL_LOW     (0U)
#define GPIO_PIN_VAL_HIGH    (1U)

enum gpio_pins {
    GpioDspIn = 0,
    GpioDspOut
};

/* GPIO Driver board specific pin configuration structure */
GPIO_PinConfig gpioPinConfigs[] = {
	GPIO_IN | GPIO_CFG_IN_INT_BOTH_EDGES | GPIO_CFG_INPUT,
	GPIO_OUT | GPIO_CFG_OUT_LOW,
};

/* GPIO Driver call back functions */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    NULL,
    NULL,
};

/* GPIO Driver configuration structure - Is read by ti/drv/gpio/src/v1/GPIO_v1.c */
GPIO_v1_Config GPIO_v1_config = {
    gpioPinConfigs,
    gpioCallbackFunctions,
    sizeof(gpioPinConfigs) / sizeof(GPIO_PinConfig),
    sizeof(gpioCallbackFunctions) / sizeof(GPIO_CallbackFxn),
    0,
};

/* private data */
Registry_Desc gpiotsk_desc;

#define USE_GPIO_INTERRUPT
#ifdef USE_GPIO_INTERRUPT

/*
 *  ======== Callback function ========
 */
Void GpioCallbackFnx(void)
{
    int val = GPIO_read(GpioDspIn);
    GPIO_write(GpioDspOut, val);
}
#endif

Void gpioSetup(void)
{
    /* GPIO initialization */
    GPIO_init();
    Log_print0(Diags_INFO, "Gpio Init complete");

#ifdef USE_GPIO_INTERRUPT
    GPIO_setCallback(GpioDspIn, GpioCallbackFnx);

    GPIO_enableInt(GpioDspIn);
    Log_print0(Diags_INFO, "Gpio interupt enabled");

    /* Call to ensure pin output matches */
    GpioCallbackFnx();
#endif
}


/*
 *  ======== gpioTsk ========
 */
Void gpioTsk(UArg arg0, UArg arg1)
{
    Int                 status = 0;
    Error_Block         eb;
    Bool                running = TRUE;

    Registry_Result result;

    /* register with xdc.runtime to get a diags mask */
    result = Registry_addModule(&gpiotsk_desc, MODULE_NAME);
    Assert_isTrue(result == Registry_SUCCESS, (Assert_Id)NULL);

    /* enable some log events */
    Diags_setMask(MODULE_NAME"+EXF");

    Log_print0(Diags_ENTRY | Diags_INFO, "--> gpioTsk:");

    Error_init(&eb);

/* Do this if we aren't using interupts */
#ifndef USE_GPIO_INTERRUPTS
    while(running) {
        int val = GPIO_read(GpioDspIn);
        GPIO_write(GpioDspOut, val);

        /* Sleep to yield */
        Task_yield();
    } /* while (running) */
#endif

leave:
    if (status < 0)
        Log_print1(Diags_INFO, "DSP: Error %d", status);

    Log_print1(Diags_EXIT, "<-- gpioTsk: %d", (IArg)status);
    return;
}
