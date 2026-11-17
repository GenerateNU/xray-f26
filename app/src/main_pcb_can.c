#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/printk.h>

#define ESTOP_ID 0x101
#define COMMAND_ID 0x102

CAN_MSGQ_DEFINE(rx_msgq_main, 10);

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

void can_rx_main(void *p1, void *p2, void *p3) {
    struct can_frame frame;

	while (1) {
		k_msgq_get(&rx_msgq_main, &frame, K_FOREVER);

        printk("%d, %d", frame.id, frame.data);
    }
}

void send_can_msg(int id, int data) {
    // can sending frame, it only sends 8 bits at a time, so data can't be longer than a byte.
    // I don't think the data is greater than 255, so it should be fine for now.
    struct can_frame frame = {
		.id = id,
		.flags = 0,
		.dlc = 1,
		.data = { data },
	};

	return can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);
}

void main_pcb_can_init() {

    if (!device_is_ready(can_dev)) {
		return -1;
	}

    const struct can_filter can_filter_main = {
        .id = 0,
        .mask = 0,
        .flags = 0,
    };

    can_add_rx_filter_msgq(can_dev, &rx_msgq_main, &can_filter_main);

    can_start(can_dev);

    return 0;
}

//main
/*

K_THREAD_DEFINE(can_rx, 1024, can_rx_main, NULL, NULL, NULL, 5, 0, 0);

int main(void) {
    if (can_setup() != 0) {
		printk("CAN setup failed");
	}

    while (true) {
        send_can_msg(COMMAND_ID, 2);
        k_msleep(1000);
    }
}
*/