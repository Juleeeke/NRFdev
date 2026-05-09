#include <stdio.h>
#include <string.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/services/nus.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/sys/printk.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define CTRL_LED       DK_LED1
#define CON_STATUS_LED DK_LED2
#define BLE_INIT_LED   DK_LED3
#define ERR_STATUS_LED DK_LED4

static const struct gpio_dt_spec dht_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(dht_pin), gpios);
const struct device *max30102_dev = DEVICE_DT_GET_ANY(maxim_max30101);
static struct bt_conn *current_conn;

// 传感器数据缓存
static uint8_t raw_dht[5];
static float real_temp = 0, real_hum = 0;
static float sim_acc_x, sim_acc_y, sim_acc_z;

// 蓝牙广播定义
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};
static const struct bt_data sd[] = { BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL) };

static void start_advertising(void) {
    bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
}

static void connected(struct bt_conn *conn, uint8_t err) {
    if (!err) { current_conn = bt_conn_ref(conn); dk_set_led_on(CON_STATUS_LED); }
}
static void disconnected(struct bt_conn *conn, uint8_t reason) {
    if (current_conn) { bt_conn_unref(current_conn); current_conn = NULL; dk_set_led_off(CON_STATUS_LED); }
    start_advertising();
}
BT_CONN_CB_DEFINE(conn_callbacks) = { .connected = connected, .disconnected = disconnected };

// 接收控制指令 (开/关 LED)
static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len) {
    if (strncmp(data, "LED_ON", 6) == 0) dk_set_led_on(CTRL_LED);
    else if (strncmp(data, "LED_OFF", 7) == 0) dk_set_led_off(CTRL_LED);
}
static struct bt_nus_cb nus_cb = { .received = bt_receive_cb };

// DHT11 驱动逻辑
static int wait_state(int state, int timeout_us) {
    int us = 0;
    while (gpio_pin_get_dt(&dht_gpio) != state) { if (++us > timeout_us) return -1; k_busy_wait(1); }
    return us;
}
static int read_byte(uint8_t *out) {
    uint8_t val = 0;
    for (int i = 0; i < 8; i++) {
        if (wait_state(1, 100) < 0) return -1;
        k_busy_wait(30);
        if (gpio_pin_get_dt(&dht_gpio) == 1) { val |= (1 << (7 - i)); if (wait_state(0, 100) < 0) return -1; }
    }
    *out = val; return 0;
}
int do_dht_read(void) {
    gpio_pin_configure_dt(&dht_gpio, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&dht_gpio, 0); k_msleep(20);
    gpio_pin_set_dt(&dht_gpio, 1); k_busy_wait(30);
    gpio_pin_configure_dt(&dht_gpio, GPIO_INPUT);
    if (wait_state(0, 100) < 0 || wait_state(1, 100) < 0 || wait_state(0, 100) < 0) return -1;
    for (int i = 0; i < 5; i++) if (read_byte(&raw_dht[i]) < 0) return -1;
    if (raw_dht[4] != ((raw_dht[0] + raw_dht[1] + raw_dht[2] + raw_dht[3]) & 0xFF)) return -1;
    return 0;
}

// 模拟行走数据
void simulate_lis3dh_walking(void) {
    static float time = 0;
    sim_acc_z = 2.0f + 0.5f * sinf(time) + (float)(k_cycle_get_32() % 100 - 50) / 500.0f;
    sim_acc_x = 0.1f + 0.1f * sinf(time * 0.5f);
    sim_acc_y = -0.1f + 0.1f * sinf(time * 1.3f);
    time += 0.2f;
}

int main(void) {
    dk_leds_init();
    gpio_is_ready_dt(&dht_gpio);
    
    if (max30102_dev && device_is_ready(max30102_dev)) printk("[OK] MAX30102 I2C Ready.\n");
    else printk("[ERR] MAX30102 Not Found.\n");

    bt_enable(NULL);
    bt_nus_init(&nus_cb);
    start_advertising();
    dk_set_led_on(BLE_INIT_LED);

    for (;;) {
        // 1. 读温湿度
        if (do_dht_read() == 0) {
            real_temp = (float)raw_dht[2]; real_hum = (float)raw_dht[0];
        } else {
            dk_set_led_on(ERR_STATUS_LED); k_msleep(100); dk_set_led_off(ERR_STATUS_LED);
        }

        // 2. 读光强 (红光/红外)
        uint32_t raw_red = 0, raw_ir = 0;
        if (max30102_dev) {
            struct sensor_value red_val, ir_val;
            sensor_sample_fetch(max30102_dev);
            sensor_channel_get(max30102_dev, SENSOR_CHAN_RED, &red_val);
            sensor_channel_get(max30102_dev, SENSOR_CHAN_IR, &ir_val);
            raw_red = red_val.val1; raw_ir = ir_val.val1;
        }

        // 3. 模拟加速度
        simulate_lis3dh_walking();

        // 4. 打包发送
        if (current_conn) {
            char buf[128];
            snprintf(buf, sizeof(buf), "T:%.1f,H:%.1f,X:%.2f,Y:%.2f,Z:%.2f,RED:%u,IR:%u\n", 
                     (double)real_temp, (double)real_hum, (double)sim_acc_x, (double)sim_acc_y, (double)sim_acc_z, raw_red, raw_ir);
            bt_nus_send(NULL, (uint8_t *)buf, strlen(buf));
            printk("[TX] %s", buf);
        }
        k_sleep(K_MSEC(2000));
    }
}