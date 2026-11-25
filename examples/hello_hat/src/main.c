/*
 * Secret Messages - Tier 2
 * Detects two device positions with the IMU and sends Morse symbols over USB.
 * Receives Morse symbols from the workstation and shows them on the LCD,
 * buzzer and LED. Uses a state machine (SEND, SENT, RECEIVE).
 *
 * AI USED: Claude (Anthropic).
 * Prompt 1: "Help me complete the Secret Messages project step by step.
 *  Act as my personal programming tutor and assume I am a beginner. Explain the project requirements,
 *  break the tasks into manageable steps, and guide me through the implementation using simple English
 *  and clear examples. Don't just give me the code; explain the logic behind it so I can understand what
 *  each part does. Help me debug errors, test my program, and improve my code while following the official
 *  project instructions. Keep the solutions simple, avoid unnecessary advanced concepts, and make sure I
 *  understand each step before moving on."

 * Tasks:
 *   sensor_task - reads the IMU every 100 ms and updates currentSymbol
 *   print_task  - state machine: sends symbols, receives symbols, gives feedback
 * Interrupt: GPIO interrupt on BUTTON1 and BUTTON2 (handler: btn_fxn)
 * Synchronization: i2cMutex protects the I2C bus (IMU and display)
 */
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "tkjhat/sdk.h"

#define DEFAULT_STACK_SIZE 2048
#define FLAT_THRESHOLD_X   0.5f    // use the value you tested: |X| below this = dot
#define DEBOUNCE_US        200000  // ignore button presses closer than 200 ms
#define RX_TIMEOUT_TICKS   250     // 250 x 20 ms = 5 s without data -> back to SEND
#define RX_TEXT_SIZE       17      // max 16 received symbols shown on the LCD

// States of the device
typedef enum {
    STATE_SEND,     // normal mode: device sends symbols to the workstation
    STATE_SENT,     // feedback after a message was sent
    STATE_RECEIVE   // a message from the workstation is arriving
} DeviceState;

// Symbol chosen by the current position of the device ('.' or '-')
static volatile char currentSymbol = '.';
// Flags set by the button interrupt, handled in print_task
static volatile bool sendSymbolRequested = false;
static volatile bool sendSpaceRequested = false;
static volatile uint32_t lastPressTimeUs = 0;

// Mutex so the IMU and the display do not use the I2C bus at the same time
static SemaphoreHandle_t i2cMutex;

// Received symbols that are shown on the LCD
static char rxText[RX_TEXT_SIZE];
static int rxLen = 0;

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

// Clears the LCD and writes one line of text (protected by the mutex)
static void show_text(const char *text) {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    clear_display();
    write_text((char *)text);
    xSemaphoreGive(i2cMutex);
}

// Shows a received symbol on the LCD, lights the LED and beeps
// Dot = short beep, dash = long beep
static void show_received_symbol(char symbol) {
    char line[24];
    if (rxLen < RX_TEXT_SIZE - 1) {
        rxText[rxLen] = symbol;
        rxLen++;
        rxText[rxLen] = '\0';
    }
    snprintf(line, sizeof(line), "RX %s", rxText);
    show_text(line);

    toggle_led();                                   // LED on
    buzzer_play_tone(880, (symbol == '.') ? 100 : 300);
    toggle_led();                                   // LED off
}

// Feedback after a message was sent: text, LED and beep
static void show_sent_feedback(void) {
    show_text("Sent!");
    toggle_led();
    buzzer_play_tone(1000, 200);
    toggle_led();
}

// Reads the accelerometer and decides dot or dash from the X axis
static void sensor_task(void *arg) {
    (void)arg;
    float ax, ay, az, gx, gy, gz, temperature;

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    bool initFailed = (init_ICM42670() != 0 || ICM42670_start_with_default_values() != 0);
    xSemaphoreGive(i2cMutex);
    if (initFailed) {
        printf("__IMU init failed__");
        vTaskDelete(NULL);
    }

    for (;;) {
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
        int readResult = ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &temperature);
        xSemaphoreGive(i2cMutex);

        if (readResult == 0) {
            // Flat on the table: X about 0 -> dot. On its side: |X| about 1 -> dash
            currentSymbol = (fabsf(ax) < FLAT_THRESHOLD_X) ? '.' : '-';
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// State machine: sends and receives symbols (half-duplex)
static void print_task(void *arg) {
    (void)arg;
    DeviceState state = STATE_SEND;
    int spaceCount = 0;   // consecutive presses of button 2
    int idleTicks = 0;    // how long nothing arrived in RECEIVE
    int ch;               // character read from the workstation

    // Initialize display, LED and buzzer
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    init_display();
    xSemaphoreGive(i2cMutex);
    init_led();
    init_buzzer();
    show_text("Ready");

    for (;;) {
        switch (state) {

        case STATE_SEND:
            if (sendSymbolRequested) {
                sendSymbolRequested = false;
                spaceCount = 0;
                printf("%c", currentSymbol);
            }
            if (sendSpaceRequested) {
                sendSpaceRequested = false;
                spaceCount++;
                if (spaceCount < 3) {
                    printf(" ");     // 1st: end of letter, 2nd: end of word
                } else {
                    printf("\n");    // 3rd: end of message (two spaces + line feed)
                    spaceCount = 0;
                    state = STATE_SENT;
                }
            }
            // Did the workstation start sending a message to us?
            ch = getchar_timeout_us(0);
            if (ch == '.' || ch == '-') {
                spaceCount = 0;
                idleTicks = 0;
                rxLen = 0;
                rxText[0] = '\0';
                show_received_symbol((char)ch);
                state = STATE_RECEIVE;
            }
            break;

        case STATE_SENT:
            show_sent_feedback();
            vTaskDelay(pdMS_TO_TICKS(1000));
            show_text("Ready");
            sendSymbolRequested = false;   // ignore presses during feedback
            sendSpaceRequested = false;
            state = STATE_SEND;
            break;

        case STATE_RECEIVE:
            ch = getchar_timeout_us(0);
            if (ch == PICO_ERROR_TIMEOUT) {
                idleTicks++;
                if (idleTicks > RX_TIMEOUT_TICKS) {   // nothing for 5 s
                    show_text("Ready");
                    state = STATE_SEND;
                }
            } else {
                idleTicks = 0;
                if (ch == '.' || ch == '-') {
                    show_received_symbol((char)ch);
                } else if (ch == '\n') {              // end of received message
                    vTaskDelay(pdMS_TO_TICKS(2000));  // keep the message visible
                    show_text("Ready");
                    sendSymbolRequested = false;
                    sendSpaceRequested = false;
                    state = STATE_SEND;
                }
            }
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

int main() {
    stdio_init_all();
    init_hat_sdk();
    sleep_ms(300);  // wait for USB and HAT initialization

    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL) {
        printf("__mutex creation failed__");
        return 0;
    }

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