#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include "Fusion.h"
#include "hardware/adc.h"


#define SAMPLE_PERIOD (0.01f) // replace this with actual sample period

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;
QueueHandle_t xQueuePos;
static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}
typedef struct dat {
    FusionVector posicoes;
    int click;
 } data;

void mpu6050_task(void *p) {
    // configuracao do I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    int click = 1;
    while(1) {
        // leitura da MPU, sem fusao de dados
        mpu6050_read_raw(acceleration, gyro, &temp);
        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f, // Conversão para graus/s
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };
  
        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f, // Conversão para g
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };  
        
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
  
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        if(accelerometer.axis.x > 0.500){
            click = 1;
        }
        else{
            click = 0;
        }
        if( -0.060 <= accelerometer.axis.y  && accelerometer.axis.y < -0.020){
            accelerometer.axis.y = 0.000;
        }
        if( 1.030<= accelerometer.axis.z && accelerometer.axis.z  <1.060){
            accelerometer.axis.z = 0.0000;
        }
        data position;
        position.posicoes = accelerometer;
        position.click = click;
        // printf("X %d, Y %0.3f, Z %0.3f\n", click,accelerometer.axis.y,accelerometer.axis.z);

        xQueueSend(xQueuePos, &position, 0);

        vTaskDelay(pdMS_TO_TICKS(10));
        }
}

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1
void send_uart_packet(uint8_t axis, int32_t valor) {
    if (valor <= 0) return;

    uint8_t bytes[4];
    bytes[0] = axis;
    bytes[1] = (valor >> 8) & 0xFF;
    bytes[2] = valor & 0xFF;
    bytes[3] = 0xFF;
    uart_write_blocking(uart0, bytes, 4);
}
void uart_task(void *p) {

    data pin_data;

    uart_init( uart0, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));
    while (1) {
        if (xQueueReceive(xQueuePos, &pin_data, portMAX_DELAY)) {
            int axis_y = 0;
            float val_y = pin_data.posicoes.axis.y;
            int32_t val_y_int = (int32_t)(val_y * 100);
            send_uart_packet(axis_y,val_y_int);

            int axis_z = 1;
            float val_z = pin_data.posicoes.axis.z;
            int32_t val_z_int = (int32_t)(val_z * 100);
            send_uart_packet(axis_z,val_z_int);

            int click_val = 2;
            int val_click = pin_data.click;
            send_uart_packet(axis_z,val_click);
            // printf("X %d, Y %0.3f, Z %0.3f\n", val_click,val_y_int,val_z_int);

        }
    }
}

int main() {
    stdio_init_all();
    xQueuePos = xQueueCreate(32, sizeof(data));

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);
    xTaskCreate(uart_task, "uart_task", 8192, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
