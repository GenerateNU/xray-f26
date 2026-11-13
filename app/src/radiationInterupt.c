#include <zephyr/kernel.h>
#include <zephyr/irq.h>

#define MY_DEV_IRQ  24       /* device uses IRQ 24 */ // find the number
#define MY_DEV_PRIO  2       /* device uses interrupt priority 2 */
/* argument passed to my_isr(), in this case a pointer to the device */
#define MY_ISR_ARG  DEVICE_GET(my_device) // can pass arguments to the isr function
#define MY_IRQ_FLAGS 0       /* IRQ flags */ //configure how the isr is ran, 0 means run normally.

void my_isr(void *arg)
{
    //switch state
}

void my_isr_installer(void)
{
   IRQ_CONNECT(MY_DEV_IRQ, MY_DEV_PRIO, my_isr, MY_ISR_ARG, MY_IRQ_FLAGS);
   // if (radiation data > threshold) {
    // irq_enable(MY_DEV_IRQ);
   // }
}
