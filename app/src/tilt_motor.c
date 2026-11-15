/*
 * tilt_motor.c - open-loop control of the tilt stepper (StepperOnline 1.16 Nm + DM542T)
 *
 * How it works:
 *   - You type an angle in the serial terminal (e.g. 90, -90, 180).
 *   - The console thread converts it to a number of step pulses, checks it
 *     against the hard-coded angle limits, and sends it to the rotation thread.
 *   - The rotation thread sets the direction pin, then sends exactly that many
 *     pulses on the step pin, speeding up and slowing down gently so the motor
 *     doesn't miss steps. It never steps past a limit.
 *   - When the move is done, the console thread reports where the motor is and
 *     asks if you want to move again (y) or stop (n).
 *
 * "Open loop" means the position comes only from counting pulses. There is no
 * sensor yet; the IMU check will be added on top of this later.
 *
 * Position 0 deg = wherever the tilt axis is when the board starts or is reset.
 * Put the RadEye in its reference position before pressing RESET.
 *
 * Wiring (common anode, see app.overlay):
 *   DM542T PUL+ and DIR+ -> Nucleo 5V
 *   DM542T PUL-          -> D5 (step pulses)
 *   DM542T DIR-          -> D6 (direction)
 *   DM542T ENA+ / ENA-   -> not connected (driver always enabled)
 * The pins are open-drain and active-low, so "set to 1" pulls the pin low,
 * which is what switches the DM542T input on.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/console/console.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

/* ---- Mechanics: these must match the real hardware ---- */
#define TILT_MOTOR_STEPS_PER_REV 200   /* 1.8 deg per full step */
#define TILT_MICROSTEPS          4     /* DM542T SW5-SW8 set to 800 pulses/rev (200 x 4) */
#define TILT_GEAR_RATIO          1.0   /* motor turns per tilt-axis turn (1.0 = direct drive) */
#define TILT_PULSES_PER_DEG \
    ((double)TILT_MOTOR_STEPS_PER_REV * TILT_MICROSTEPS * TILT_GEAR_RATIO / 360.0)

/* ---- Safety limits, in degrees from the start position ---- */
#define TILT_MIN_DEG (-180.0)
#define TILT_MAX_DEG (180.0)

/* ---- Speed profile ---- */
#define TILT_PULSE_WIDTH_US   10    /* DM542T needs the pulse held for at least ~2.5 us */
#define TILT_START_PERIOD_US  4000  /* 250 pulses/s at the start and end of each move */
#define TILT_CRUISE_PERIOD_US 2000  /* 500 pulses/s at full speed */
#define TILT_RAMP_PULSES      100   /* pulses spent speeding up, and again slowing down */
#define TILT_DIR_SETUP_US     10    /* DIR must be set at least 5 us before the first pulse */
#define TILT_INVERT_DIR       0     /* change to 1 if positive angles turn the wrong way */

/* ---- Threads ---- */
#define ROTATION_THREAD_PRIORITY 3  /* pulse timing matters, so it runs above the console */
#define CONSOLE_THREAD_PRIORITY  8
#define TILT_THREAD_STACK        2048

static const struct gpio_dt_spec step_pin =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), tilt_step_gpios);
static const struct gpio_dt_spec dir_pin =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), tilt_dir_gpios);

/* What the rotation thread reports back after each move */
struct tilt_result {
    int32_t pulses_moved;   /* signed: negative means it moved in the minus direction */
    int64_t duration_ms;
};

/* Console -> rotation thread: one move command (signed number of pulses) */
K_MSGQ_DEFINE(tilt_cmd_q, sizeof(int32_t), 1, 4);
/* Rotation thread -> console: "move finished" */
K_SEM_DEFINE(tilt_done_sem, 0, 1);

static struct tilt_result last_result;
/* Current position in pulses from the start position. Only the rotation thread
 * changes it, and the console only reads it while no move is running. */
static volatile int32_t tilt_position;

static int32_t deg_to_pulses(double deg)
{
    return (int32_t)lround(deg * TILT_PULSES_PER_DEG);
}

static double pulses_to_deg(int32_t pulses)
{
    return pulses / TILT_PULSES_PER_DEG;
}

/*
 * Time between pulses for pulse number i out of total. Starts slow, speeds up
 * over TILT_RAMP_PULSES, cruises, then slows down again before the end. A
 * stepper started at full speed can stall and lose steps, and the tilt axis
 * has little torque margin against gravity.
 */
static uint32_t pulse_period_us(int32_t i, int32_t total)
{
    int32_t from_edge = MIN(i, total - 1 - i);

    if (from_edge >= TILT_RAMP_PULSES) {
        return TILT_CRUISE_PERIOD_US;
    }
    return TILT_START_PERIOD_US -
           (TILT_START_PERIOD_US - TILT_CRUISE_PERIOD_US) * from_edge / TILT_RAMP_PULSES;
}

static void rotation_thread(void *a, void *b, void *c)
{
    int32_t command;
    int32_t min_pulses = deg_to_pulses(TILT_MIN_DEG);
    int32_t max_pulses = deg_to_pulses(TILT_MAX_DEG);

    if (!gpio_is_ready_dt(&step_pin) || !gpio_is_ready_dt(&dir_pin)) {
        printk("Tilt motor GPIO not ready\n");
        return;
    }
    /* Start with both DM542T inputs off */
    gpio_pin_configure_dt(&step_pin, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&dir_pin, GPIO_OUTPUT_INACTIVE);

    while (1) {
        /* Sleep until the console sends a move */
        k_msgq_get(&tilt_cmd_q, &command, K_FOREVER);

        int32_t step_dir = (command >= 0) ? 1 : -1;
        int32_t total = (command >= 0) ? command : -command;
        int32_t moved = 0;
        int64_t start_ms = k_uptime_get();

        gpio_pin_set_dt(&dir_pin, (step_dir > 0) ^ TILT_INVERT_DIR);
        k_busy_wait(TILT_DIR_SETUP_US);

        for (int32_t i = 0; i < total; i++) {
            int32_t next = tilt_position + step_dir;

            /* Second safety net: never step past a limit, even if a bad command got through */
            if (next < min_pulses || next > max_pulses) {
                break;
            }

            /* One step: pulse the step pin, then wait for the rest of the period */
            gpio_pin_set_dt(&step_pin, 1);
            k_busy_wait(TILT_PULSE_WIDTH_US);
            gpio_pin_set_dt(&step_pin, 0);

            tilt_position = next;
            moved += step_dir;
            k_usleep(pulse_period_us(i, total) - TILT_PULSE_WIDTH_US);
        }

        last_result.pulses_moved = moved;
        last_result.duration_ms = k_uptime_get() - start_ms;
        k_sem_give(&tilt_done_sem);
    }
}

/* Returns 'y' or 'n', asking again until the user types one of them */
static char ask_yes_no(void)
{
    while (1) {
        printk("Measure another angle? (y/n): ");
        char *line = console_getline();

        while (*line == ' ') {
            line++;
        }
        char answer = (char)tolower((unsigned char)line[0]);

        if (answer == 'y' || answer == 'n') {
            return answer;
        }
        printk("Please type y or n\n");
    }
}

static void console_thread(void *a, void *b, void *c)
{
    int32_t min_pulses = deg_to_pulses(TILT_MIN_DEG);
    int32_t max_pulses = deg_to_pulses(TILT_MAX_DEG);

    console_getline_init();
    k_msleep(500);   /* let the boot messages print first */

    printk("\n=== Tilt motor open-loop test ===\n");
    printk("%.3f pulses per degree, limits %.1f to %.1f deg\n",
           TILT_PULSES_PER_DEG, TILT_MIN_DEG, TILT_MAX_DEG);
    printk("Start position counts as 0.0 deg\n");

    while (1) {
        printk("\nTilt is at %.1f deg. Enter degrees to move (e.g. 90, -90, 180): ",
               pulses_to_deg(tilt_position));
        char *line = console_getline();
        char *end;
        double degrees = strtod(line, &end);

        if (end == line) {
            printk("\"%s\" is not a number\n", line);
            continue;
        }

        int32_t command = deg_to_pulses(degrees);
        int32_t target = tilt_position + command;

        if (command == 0) {
            printk("That's less than one step, nothing to do\n");
            continue;
        }
        /* First safety check: reject any move that would end outside the limits */
        if (target < min_pulses || target > max_pulses) {
            printk("Rejected: that would end at %.1f deg, outside the limits (%.1f to %.1f).\n",
                   pulses_to_deg(target), TILT_MIN_DEG, TILT_MAX_DEG);
            printk("From here you can move between %.1f and %+.1f deg.\n",
                   pulses_to_deg(min_pulses - tilt_position),
                   pulses_to_deg(max_pulses - tilt_position));
            continue;
        }

        printk("Moving %+.1f deg (%d pulses)...\n", degrees, (int)command);
        k_msgq_put(&tilt_cmd_q, &command, K_FOREVER);
        k_sem_take(&tilt_done_sem, K_FOREVER);   /* wait for the rotation thread */

        if (last_result.pulses_moved == command) {
            printk("Target reached and stopped.\n");
        } else {
            printk("Stopped early at a limit.\n");
        }
        printk("  Commanded: %+.1f deg (%d pulses)\n", degrees, (int)command);
        printk("  Moved:     %+.1f deg (%d pulses) in %d ms\n",
               pulses_to_deg(last_result.pulses_moved),
               (int)last_result.pulses_moved, (int)last_result.duration_ms);
        printk("  Tilt now:  %.1f deg from start (from step count, open loop)\n",
               pulses_to_deg(tilt_position));

        if (ask_yes_no() == 'n') {
            printk("\nStopped. No more moves will be made.\n");
            printk("The motor stays powered and holds the tilt at %.1f deg.\n",
                   pulses_to_deg(tilt_position));
            printk("Press RESET to start again (the new start position becomes 0.0 deg).\n");
            return;
        }
    }
}

K_THREAD_DEFINE(tilt_rotation_tid, TILT_THREAD_STACK, rotation_thread, NULL, NULL, NULL,
                ROTATION_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(tilt_console_tid, TILT_THREAD_STACK, console_thread, NULL, NULL, NULL,
                CONSOLE_THREAD_PRIORITY, 0, 0);
