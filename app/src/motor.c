


#define USER_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec x_step_pin = GPIO_DT_SPEC_GET(USER_NODE, x_step_gpios);
static const struct gpio_dt_spec y_step_pin = GPIO_DT_SPEC_GET(USER_NODE, y_step_gpios);
static const struct gpio_dt_spec z_step_pin = GPIO_DT_SPEC_GET(USER_NODE, z_step_gpios);

static const struct gpio_dt_spec x_dir_pin = GPIO_DT_SPEC_GET(USER_NODE, x_dir_gpios);
static const struct gpio_dt_spec y_dir_pin = GPIO_DT_SPEC_GET(USER_NODE, y_dir_gpios);
static const struct gpio_dt_spec z_dir_pin = GPIO_DT_SPEC_GET(USER_NODE, z_dir_gpios);

static const struct gpio_dt_spec x_ena_pin = GPIO_DT_SPEC_GET(USER_NODE, x_ena_gpios);
static const struct gpio_dt_spec y_ena_pin = GPIO_DT_SPEC_GET(USER_NODE, y_ena_gpios);
static const struct gpio_dt_spec z_ena_pin = GPIO_DT_SPEC_GET(USER_NODE, z_ena_gpios);

static const struct gpio_dt_spec x_limit_pin = GPIO_DT_SPEC_GET(USER_NODE, x_limit_gpios);
static const struct gpio_dt_spec y_limit_pin = GPIO_DT_SPEC_GET(USER_NODE, y_limit_gpios);
static const struct gpio_dt_spec z_limit_pin = GPIO_DT_SPEC_GET(USER_NODE, z_limit_gpios);

static const struct gpio_dt_spec x_limit_pin = GPIO_DT_SPEC_GET(USER_NODE, x_sleep_gpios);
static const struct gpio_dt_spec y_limit_pin = GPIO_DT_SPEC_GET(USER_NODE, y_sleep_gpios);
static const struct gpio_dt_spec z_limit_pin = GPIO_DT_SPEC_GET(USER_NODE, z_sleep_gpios);

static const struct gpio_dt_spec x_nfault_pin = GPIO_DT_SPEC_GET(USER_NODE, x_nfault_gpios);
static const struct gpio_dt_spec y_nfault_pin = GPIO_DT_SPEC_GET(USER_NODE, y_nfault_gpios);
static const struct gpio_dt_spec z_nfault_pin = GPIO_DT_SPEC_GET(USER_NODE, z_nfault_gpios);

static const struct gpio_dt_spec x_cs_pin = GPIO_DT_SPEC_GET(USER_NODE, x_cs_gpios);
static const struct gpio_dt_spec y_cs_pin = GPIO_DT_SPEC_GET(USER_NODE, y_cs_gpios);
static const struct gpio_dt_spec z_cs_pin = GPIO_DT_SPEC_GET(USER_NODE, z_cs_gpios);
