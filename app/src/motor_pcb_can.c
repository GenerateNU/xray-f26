#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/printk.h>

#define IMU_ID 0x201
#define DISTANCE_ID 0x202
#define PAN_ID  0x203
#define TILT_ID 0x204

CAN_MSGQ_DEFINE(rx_msgq_motor, 10);

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

void can_rx_motor(void *p1, void *p2, void *p3) {
    struct can_frame frame;

	while (1) {
		k_msgq_get(&rx_msgq_motor, &frame, K_FOREVER);

        printk("%d, %d", frame.id, frame.data);
    }
}

int send_can_msg(int id, int data) {
    // can sending frame, it only sends 8 bits at a time, so data can't be longer than a byte.
    // I don't think the data is longer than 255, so it should be fine for now.
    struct can_frame frame = {
		.id = id,
		.flags = 0,
		.dlc = 1,
		.data = { data },
	};

	return can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);
}

void motor_pcb_can_init() {
    if (!device_is_ready(can_dev)) {
		return -1;
	}

    const struct can_filter can_filter_main = {
        .id = 0,
        .mask = 0,
        .flags = 0,
    };

    can_add_rx_filter_msgq(can_dev, &rx_msgq_motor, &can_filter_main);

    if (can_start(can_dev) != 0) {
        return -1;
    };

    return 0;
}

