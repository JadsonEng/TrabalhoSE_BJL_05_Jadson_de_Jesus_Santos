/*
 *  Por: Wilton Lacerda Silva
 *  Data: 10/05/2025
 *
 *  Exemplo do uso de Filas queue no FreeRTOS com Raspberry Pi Pico
 *
 *  Descrição: Leitura do valor do joystick e exibição no display OLED SSD1306
 *  com comunicação I2C. O valor do joystick é lido a cada 100ms e enviado para a fila.
 *  A task de exibição recebe os dados da fila e atualiza o display a cada 100ms.
 *  Os leds são controlados por PWM, com brilho proporcional ao desvio do joystick.
 *  O led verde controla o eixo X e o led azul controla o eixo Y.
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include "hardware/clocks.h"
#include "ws2818b.pio.h"
#include "hardware/pio.h"

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
#define ADC_JOYSTICK_X 26
#define ADC_JOYSTICK_Y 27
#define LED_RED 13
#define LED_BLUE 12
#define LED_GREEN  11
#define tam_quad 10
#define BUZZER 21
#define LED_PIN 7
#define LED_COUNT 25
uint sm;

//////////////////////////////////////////////////////////////////////////////////////////
// =====     Configurações para o Buzzer     ===== //
void init_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM); // Configura o GPIO como PWM
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_clkdiv(slice_num, 125.0f);     // Define o divisor do clock para 1 MHz
    pwm_set_wrap(slice_num, 1000);        // Define o TOP para frequência de 1 kHz
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 0); // Razão cíclica inicial
    pwm_set_enabled(slice_num, true);     // Habilita o PWM
}   
void set_buzzer_tone(uint gpio, uint freq) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    uint top = 1000000 / freq;            // Calcula o TOP para a frequência desejada
    pwm_set_wrap(slice_num, top);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), top / 2); // 50% duty cycle
}   
void stop_buzzer(uint gpio) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 0); // Desliga o PWM
}

//////////////////////////////////////////////////////////////////////////////////////////
// =====     Configurações para a Matriz de LEDs     ===== //
// Estrutura para representar um pixel RGB na matriz de LEDs
struct pixel_t {
    uint8_t G, R, B;           // Componentes de cor: verde, vermelho, azul
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t;       // Tipo para LEDs NeoPixel (WS2812B)
npLED_t leds[LED_COUNT];       // Array que armazena o estado de cada LED
PIO np_pio;                    // Instância do PIO para controlar a matriz
void npDisplayDigit(int digit);
// Matrizes que definem os padrões de exibição na matriz de LEDs (5x5 pixels)
const uint8_t digits[4][5][5][3] = {
    // Matriz desligada
    {
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    },
    // Atenção Vermelho
    {
        {{0, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    },
    // Atenção Verde
    {
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}}
    },
    // Atenção Azul
    {
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 100}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 100}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 100}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 100}, {0, 0, 0}, {0, 0, 0}}
    }
};

// Define as cores de um LED na matriz
void npSetLED(uint index, uint8_t r, uint8_t g, uint8_t b) {
    leds[index].R = r; // Componente vermelho
    leds[index].G = g; // Componente verde
    leds[index].B = b; // Componente azul
}

// Limpa a matriz de LEDs, exibindo o padrão de dígito 4 (padrão para limpar)
void npClear() {
    npDisplayDigit(4);
}

// Inicializa a matriz de LEDs WS2812B usando o PIO
void npInit(uint pin) {
    uint offset = pio_add_program(pio0, &ws2818b_program); // Carrega programa PIO
    np_pio = pio0; // Usa PIO0
    sm = pio_claim_unused_sm(np_pio, true); // Reserva uma máquina de estado
    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f); // Inicializa PIO
    npClear(); // Limpa a matriz ao inicializar
}

// Escreve os dados dos LEDs na matriz
void npWrite() {
    for (uint i = 0; i < LED_COUNT; i++) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G); // Envia componente verde
        pio_sm_put_blocking(np_pio, sm, leds[i].R); // Envia componente vermelho
        pio_sm_put_blocking(np_pio, sm, leds[i].B); // Envia componente azul
    }
    sleep_us(100); // Pequeno atraso para estabilizar a comunicação
}

// Calcula o índice de um LED na matriz com base nas coordenadas (x, y)
int getIndex(int x, int y) {
    if (y % 2 == 0) {
        return 24 - (y * 5 + x); // Linhas pares: ordem direta
    } else {
        return 24 - (y * 5 + (4 - x)); // Linhas ímpares: ordem invertida
    }
}

// Exibe um dígito ou padrão na matriz de LEDs
void npDisplayDigit(int digit) {
    for (int coluna = 0; coluna < 5; coluna++) {
        for (int linha = 0; linha < 5; linha++) {
            int posicao = getIndex(linha, coluna); // Calcula índice do LED
            npSetLED(posicao, digits[digit][coluna][linha][0], // Componente R
                              digits[digit][coluna][linha][1], // Componente G
                              digits[digit][coluna][linha][2]); // Componente B
        }
    }
    npWrite(); // Atualiza a matriz com os novos dados
}


//////////////////////////////////////////////////////////////////////////////////////////
// =====     Configurações para a Queue (fila)     ===== //
typedef struct
{
    uint16_t x_pos;
    uint16_t y_pos;
} joystick_data_t;
QueueHandle_t xQueueJoystickData;

//////////////////////////////////////////////////////////////////////////////////////////
// =====     Task: Joystick     ===== //

void vJoystickTask(void *params)
{
    adc_gpio_init(ADC_JOYSTICK_Y);
    adc_gpio_init(ADC_JOYSTICK_X);
    adc_init();

    joystick_data_t joydata;
    while (true)
    {
        adc_select_input(0); // GPIO 26 = ADC0
        joydata.y_pos = adc_read();

        adc_select_input(1); // GPIO 27 = ADC1
        joydata.x_pos = adc_read();

        xQueueSend(xQueueJoystickData, &joydata, 0); // Envia o valor do joystick para a fila
        vTaskDelay(pdMS_TO_TICKS(100));              // 10 Hz de leitura
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
// =====     Task: Display     ===== //
void vDisplayTask(void *params)
{
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ssd1306_t ssd;
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    char buffer[32];

    joystick_data_t joydata;
    bool cor = true;
    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            uint16_t percentual_x = (joydata.x_pos * 100) / 4095;
            uint16_t percentual_y = (joydata.y_pos * 100) / 4095;

            ssd1306_fill(&ssd, !cor);                          // Limpa o display
            ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);      // Desenha um retângulo
            ssd1306_line(&ssd, 3, 15, 123, 15, cor);           // Desenha uma linha
            ssd1306_draw_string(&ssd, "EMBARCATECH", 20, 6); // Desenha uma string
            
            ssd1306_draw_string(&ssd, "Niv. agua", 5, 20);   // Desenha uma string
            snprintf(buffer, sizeof(buffer), "%u%%", percentual_x);
            ssd1306_draw_string(&ssd, buffer, 95, 20); 

            ssd1306_draw_string(&ssd, "Vol. chuva", 5, 30);   // Desenha uma string
            snprintf(buffer, sizeof(buffer), "%u%%", percentual_y);
            ssd1306_draw_string(&ssd, buffer, 95, 30);

            if(joydata.x_pos >= 2866 || joydata.y_pos >= 3276){
                ssd1306_draw_string(&ssd, "ALERTA", 40, 45);   // Desenha uma string
            }
            else{
                
            }
            ssd1306_send_data(&ssd);                          // Atualiza o display
            vTaskDelay(pdMS_TO_TICKS(100)); // Atualiza a cada 50ms
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Atualiza a cada 50ms
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
// =====     Task: LED RGB     ===== //
void vLedRGBTask(void *params)
{
    gpio_init(LED_RED);
    gpio_init(LED_GREEN);
    gpio_init(LED_BLUE);
    gpio_set_dir(LED_RED, GPIO_OUT);
    gpio_set_dir(LED_GREEN, GPIO_OUT);
    gpio_set_dir(LED_BLUE, GPIO_OUT);
    
    joystick_data_t joydata;
    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            if (joydata.x_pos >= 2866 && joydata.y_pos >= 3276){
                gpio_put(LED_RED, 1);
                gpio_put(LED_BLUE, 0);
                gpio_put(LED_GREEN, 0);
            } else {
                gpio_put(LED_RED, 0);
                if(joydata.x_pos >= 2866){
                gpio_put(LED_GREEN, 1);
                }else{
                    gpio_put(LED_GREEN, 0);
                }   
                if (joydata.y_pos >= 3276){
                    gpio_put(LED_BLUE, 1);
                }else{
                    gpio_put(LED_BLUE, 0);
            }
            }
            
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza a cada 50ms
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
// =====     Task: Buzzer     ===== //
void vBuzzerTask(void *params)
{
    gpio_init(BUZZER);
    gpio_set_dir(BUZZER, GPIO_OUT);
    init_pwm(BUZZER);

    joystick_data_t joydata;
    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            if(joydata.x_pos >= 2866 && joydata.y_pos >= 3276){
                set_buzzer_tone(BUZZER, 440);
                vTaskDelay(pdMS_TO_TICKS(200));
                stop_buzzer(BUZZER);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            else if (joydata.x_pos >= 2866){
                set_buzzer_tone(BUZZER, 330);

            } else if (joydata.y_pos >= 3276){
                set_buzzer_tone(BUZZER, 220);

            } else {
                stop_buzzer(BUZZER);
            }

            
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza a cada 50ms
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
// =====     Task: Matriz de LEDs     ===== //
void vMatrizTask(void *params)
{
    npInit(LED_PIN);

    joystick_data_t joydata;
    while (true)
    {
        if (xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) == pdTRUE)
        {
            if(joydata.x_pos >= 2866 && joydata.y_pos >= 3276){
                npDisplayDigit(1);

            } else if (joydata.x_pos >= 2866) {
                npDisplayDigit(2);

            } else if (joydata.y_pos >= 3276) {
                npDisplayDigit(3);

            } else {
                npDisplayDigit(0);
            }
        vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza a cada 50ms
        }
    }
}

// Modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6
void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}


//////////////////////////////////////////////////////////////////////////////////////////
// =====     MAIN     ===== //
int main()
{
    // Ativa BOOTSEL via botão
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    stdio_init_all();

    // Cria a fila para compartilhamento de valor do joystick
    xQueueJoystickData = xQueueCreate(5, sizeof(joystick_data_t));

    // Criação das tasks
    xTaskCreate(vJoystickTask, "Joystick Task", 256, NULL, 1, NULL);
    xTaskCreate(vDisplayTask, "Display Task", 512, NULL, 1, NULL);
    xTaskCreate(vLedRGBTask, "LED red Task", 256, NULL, 1, NULL);
    xTaskCreate(vBuzzerTask, "Buzzer Task", 256, NULL, 1, NULL);
    xTaskCreate(vMatrizTask, "Matriz Task", 256, NULL, 1, NULL);
    // Inicia o agendador
    vTaskStartScheduler();
    panic_unsupported();
}