#include <zephyr/kernel.h>
#include <zephyr/irq.h>

#define RAD_IRQ  24       /* device uses IRQ 24 */ // find the number
#define RAD_IRQ_PRIO  2       /* device uses interrupt priority 2 */
/* argument passed to my_isr(), in this case a pointer to the device */
#define MY_ISR_ARG  DEVICE_GET(my_device) // can pass arguments to the isr function
#define MY_IRQ_FLAGS 0       /* IRQ flags */ //configure how the isr is ran, 0 means run normally.

void rad_isr(void *arg)
{
    //switch state
}

void rad_isr_init(void)
{
   IRQ_CONNECT(RAD_IRQ, RAD_IRQ_PRIO, my_isr, MY_ISR_ARG, MY_IRQ_FLAGS);
}
