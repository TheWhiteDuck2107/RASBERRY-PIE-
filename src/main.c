/*
 * Secret Messages - Tier 1
 * Detects two device positions with the IMU and sends Morse symbols over USB.
 *
 * AI USED: Claude (Anthropic). Prompt: "Help me complete the Secret Messages project step by step.
 *  Act as my personal programming tutor and assume I am a beginner. Explain the project requirements,
 *  break the tasks into manageable steps, and guide me through the implementation using simple English
 *  and clear examples. Don't just give me the code; explain the logic behind it so I can understand what
 *  each part does. Help me debug errors, test my program, and improve my code while following the official
 *  project instructions. Keep the solutions simple, avoid unnecessary advanced concepts, and make sure I
 *  understand each step before moving on.".
 * I changed: tested the X-axis threshold on my own device and adjusted it to
 * (your value), added the debounce time of 200 ms after testing the buttons,
 * and (add anything else you edited).
 *
 * Tasks:
 *   sensor_task - reads the IMU every 100 ms and updates currentSymbol
 *   print_task  - sends symbols and spaces to the workstation
 * Interrupt: GPIO interrupt on BUTTON1 and BUTTON2 (handler: btn_fxn)
 */
#include <stdio.h>
#include <stdbool.h>
#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <task.h>
#include <math.h>
#include "tkjhat/sdk.h"

#define DEFAULT_STACK_SIZE 2048
//#define DASH_THRESHOLD_X   -0.5f   // X below this: device on its side = dash
#define DEBOUNCE_US        200000  // ignore button presses closer than 200 ms

// Symbol chosen by the current position of the device ('.' or '-')
static volatile char currentSymbol = '.';
// Flags set by the button interrupt, handled in print_task
static volatile bool sendSymbolRequested = false;
static volatile bool sendSpaceRequested = false;
static volatile uint32_t lastPressTimeUs = 0;

// Button interrupt handler: only sets flags, no printing here
static void btn_fxn(uint gpio, uint32_t eventMask) {
    uint32_t now = time_us_32();
    if (now - lastPressTimeUs < DEBOUNCE_US) {
        return;  // debounce
    }
    lastPressTimeUs = now;

    if (gpio == BUTTON1) {
        sendSymbolRequested = true;
    } else if (gpio == BUTTON2) {
        sendSpaceRequested = true;
    }
}

// Reads the accelerometer and decides dot or dash from the X axis
static void sensor_task(void *arg) {
    (void)arg;
    float ax, ay, az, gx, gy, gz, temperature;

    if (init_ICM42670() != 0 || ICM42670_start_with_default_values() != 0) {
        printf("__IMU init failed__");
        vTaskDelete(NULL);
    }

    for (;;) {
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &temperature) == 0) {
            // Flat on the table: X about 0 -> dot. On its side: X about -1 -> dash
            currentSymbol = (fabsf(ax) < 0.5f) ? '.' : '-';
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Sends symbols and spaces to the workstation using the message protocol
static void print_task(void *arg) {
    (void)arg;
    int spaceCount = 0;  // consecutive presses of button 2

    for (;;) {
        if (sendSymbolRequested) {
            sendSymbolRequested = false;
            spaceCount = 0;
            printf("%c", currentSymbol);
        }
        if (sendSpaceRequested) {
            sendSpaceRequested = false;
            spaceCount++;
            if (spaceCount < 3) {
                printf(" ");   // 1st: end of letter, 2nd: end of word
            } else {
                printf("\n");  // 3rd: end of message (two spaces + line feed)
                spaceCount = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

int main() {
    stdio_init_all();
    init_hat_sdk();
    sleep_ms(300);  // wait for USB and HAT initialization

    init_button1();
    init_button2();
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, btn_fxn);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_RISE, true);

    TaskHandle_t hSensorTask, hPrintTask;
    xTaskCreate(sensor_task, "sensor", DEFAULT_STACK_SIZE, NULL, 2, &hSensorTask);
    xTaskCreate(print_task, "print", DEFAULT_STACK_SIZE, NULL, 2, &hPrintTask);

    vTaskStartScheduler();
    return 0;
}