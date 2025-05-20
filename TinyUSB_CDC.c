#include <stdio.h>
#include <string.h> // Para strcmp(), strstr(), memset() e strlen()
#include <ctype.h>  // Para tolower()

#include "pico/stdlib.h" // Para timers, gpio, etc.
#include "hardware/pwm.h" // Para controle PWM do buzzer
#include "hardware/i2c.h" // REQ_8: Para comunicação I2C com o OLED
#include "tusb.h"      // Biblioteca TinyUSB

// REQ_8: Incluir o header principal da biblioteca do display OLED SSD1306
#include "inc/ssd1306.h"

// Definição dos LEDs (conforme BitDogLab)
// REQ_6: GPIO 11 - Verde; GPIO 12 – Azul; GPIO 13 – Vermelho
#define LED_VERDE     11
#define LED_AZUL      12
#define LED_VERMELHO  13

// REQ_7: Acionar o Buzzer (GPIO – 21) [...] com pwm
#define BUZZER_PIN_A  21
#define BUZZER_DEFAULT_FREQ 2000 // Frequência de 2kHz para o som do buzzer
#define SOUND_ON_DURATION_MS 1000 // Som do buzzer por 1 segundo

// REQ_8: Definições para I2C e Display OLED (conforme manual BitDogLab)
const uint I2C_SDA_PIN = 14; // Pino SDA do I2C (GPIO14)
const uint I2C_SCL_PIN = 15; // Pino SCL do I2C (GPIO15)
// A biblioteca ssd1306_i2c.c usa i2c1 e o endereço 0x3C por padrão.

// Variáveis globais para controle dos LEDs
static absolute_time_t led_off_time;
static int led_ativo = -1; // -1 significa nenhum LED ativo, ou o GPIO do LED ativo

// Variáveis globais para controle do Buzzer
static absolute_time_t sound_off_time;
static bool sound_ativo = false;
static uint buzzer_pwm_slice_num;
static uint buzzer_pwm_chan;

// REQ_8: Variáveis globais para o Display OLED
// O buffer para o conteúdo do display. Seu tamanho é definido em ssd1306_i2c.h (ssd1306_buffer_length).
static uint8_t oled_screen_buffer[ssd1306_buffer_length];
// Estrutura para definir a área de renderização (tela inteira neste caso).
static struct render_area oled_full_screen_area;


// Inicializa os LEDs
void init_leds() {
    gpio_init(LED_VERDE);
    gpio_set_dir(LED_VERDE, GPIO_OUT);
    gpio_init(LED_AZUL);
    gpio_set_dir(LED_AZUL, GPIO_OUT);
    gpio_init(LED_VERMELHO);
    gpio_set_dir(LED_VERMELHO, GPIO_OUT);
    
    gpio_put(LED_VERDE, 0);
    gpio_put(LED_AZUL, 0);
    gpio_put(LED_VERMELHO, 0);
}

// Aciona o LED correspondente por 1 segundo
// REQ_3: fazer a identificação visual do comando
// REQ_6: acender o LED correspondente à cor [...] durante 1 segundo
void aciona_led(int led_pin) {
    if (led_ativo != -1) {
        gpio_put(led_ativo, 0);
    }
    if (led_pin == LED_VERMELHO || led_pin == LED_VERDE || led_pin == LED_AZUL) {
        gpio_put(led_pin, 1);
        led_ativo = led_pin;
        led_off_time = make_timeout_time_ms(1000); // Configura timer de 1s para REQ_6
    } else {
        led_ativo = -1;
    }
}

// Verifica se deve desligar o LED após 1s
// REQ_6: [...] durante 1 segundo (lógica de desligamento)
void verifica_led_timer() {
    if (led_ativo != -1 && time_reached(led_off_time)) {
        gpio_put(led_ativo, 0);
        led_ativo = -1;
    }
}

// --- Funções do Buzzer com PWM (REQ_7) ---
void setup_buzzer_pwm() {
    gpio_set_function(BUZZER_PIN_A, GPIO_FUNC_PWM);
    buzzer_pwm_slice_num = pwm_gpio_to_slice_num(BUZZER_PIN_A);
    buzzer_pwm_chan = pwm_gpio_to_channel(BUZZER_PIN_A);
    pwm_set_enabled(buzzer_pwm_slice_num, false); // Inicia desligado
}

void play_buzzer_tone(uint16_t freq) {
    if (freq == 0) {
        pwm_set_enabled(buzzer_pwm_slice_num, false);
        return;
    }
    pwm_set_clkdiv_int_frac(buzzer_pwm_slice_num, 100, 0); 
    float wrap_float = 1250000.0f / freq; 
    if (wrap_float < 1.0f) wrap_float = 1.0f;
    if (wrap_float > 65535.0f) { 
        pwm_set_enabled(buzzer_pwm_slice_num, false);
        return;
    }
    uint16_t wrap_value = (uint16_t)wrap_float;
    pwm_set_wrap(buzzer_pwm_slice_num, wrap_value);
    pwm_set_chan_level(buzzer_pwm_slice_num, buzzer_pwm_chan, wrap_value / 2); 
    pwm_set_enabled(buzzer_pwm_slice_num, true);
}

void stop_buzzer_tone() {
    pwm_set_enabled(buzzer_pwm_slice_num, false);
}

void aciona_som_buzzer() {
    // REQ_7: Ao realizar o eco, o dispositivo CDC deverá acionar o Buzzer
    play_buzzer_tone(BUZZER_DEFAULT_FREQ);
    sound_ativo = true;
    sound_off_time = make_timeout_time_ms(SOUND_ON_DURATION_MS);
}

void verifica_sound_timer() {
    if (sound_ativo && time_reached(sound_off_time)) {
        stop_buzzer_tone();
        sound_ativo = false;
    }
}

// --- Funções do Display OLED (REQ_8) ---
void setup_oled() {
    // Inicializa I2C1 usando os pinos definidos
    // A biblioteca ssd1306_i2c.c usa i2c1 internamente.
    i2c_init(i2c1, 400 * 1000); // 400kHz clock speed
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN); // Pull-ups externos são recomendados, mas internos podem ajudar
    gpio_pull_up(I2C_SCL_PIN);

    // Inicializa o display SSD1306 usando a função da sua biblioteca
    ssd1306_init();

    // Configura a área de renderização para a tela inteira
    // Assumindo que ssd1306_width e ssd1306_height são definidos em ssd1306_i2c.h
    oled_full_screen_area.start_column = 0;
    oled_full_screen_area.end_column = ssd1306_width - 1;
    oled_full_screen_area.start_page = 0;
    oled_full_screen_area.end_page = (ssd1306_height / ssd1306_page_height) - 1; // ssd1306_page_height é 8
    calculate_render_area_buffer_length(&oled_full_screen_area); // Função da sua biblioteca

    // Limpa o display e mostra uma mensagem inicial
    memset(oled_screen_buffer, 0, sizeof(oled_screen_buffer));
    ssd1306_draw_string(oled_screen_buffer, 0, 0, "BitDogLab"); // Exemplo de string, x, y_pixel, texto
    ssd1306_draw_string(oled_screen_buffer, 0, 16, "CDC Ativo!");
    render_on_display(oled_screen_buffer, &oled_full_screen_area);
}

void oled_display_command_echo(const char* command_eco) {
    // REQ_8: Utilizar o Display OLED [...] para impressão dos comandos (durante o eco)
    memset(oled_screen_buffer, 0, sizeof(oled_screen_buffer)); // Limpa o buffer do display

    char display_line1[32] = "Eco Serial:"; // Max 16 chars por linha com fonte 8x8 em 128px
    char display_line2[32] = "";

    // Prepara a string do comando para exibição, truncando se necessário
    strncpy(display_line2, command_eco, 15); // Copia até 15 caracteres para caber
    display_line2[15] = '\0'; // Garante terminação nula

    ssd1306_draw_string(oled_screen_buffer, 0, 0, display_line1);
    ssd1306_draw_string(oled_screen_buffer, 0, 16, display_line2); // Exibe o comando na segunda linha

    render_on_display(oled_screen_buffer, &oled_full_screen_area);
}


// Função para enviar mensagens para o host via CDC
void cdc_send_message(const char* message) {
    if (tud_cdc_connected()) {
        tud_cdc_write_str(message);
        tud_cdc_write_flush();
    }
}

int main() {
    // REQ_1: Implementar um dispositivo CDC [...] conectado via porta USB (inicialização da TinyUSB)
    tusb_init(); // Inicializa a stack TinyUSB
    
    init_leds(); // Inicializa os GPIOs dos LEDs
    setup_buzzer_pwm(); // REQ_7: Configura PWM para o buzzer
    setup_oled();       // REQ_8: Configura I2C e Display OLED

    // Aguarda conexão USB CDC ser estabelecida pelo host
    // REQ_1: [...] dispositivo CDC que será conectado ao computador
    while (!tud_cdc_connected()) {
        tud_task(); 
        sleep_ms(10); 
    }
    // REQ_4: Comandos simulados através do envio [...] no Monitor Serial (preparação para receber)
    cdc_send_message("BitDogLab CDC: Conectado! Envie 'vermelho', 'verde', 'azul' ou 'som'.\r\n");

    uint8_t buf[64]; 
    char command_to_check[65]; 

    while (true) {
        // REQ_1: Parte da manutenção da conexão CDC
        tud_task(); 

        if (tud_cdc_available()) {
            // REQ_2: [...] receber [...] o comando dado no computador
            // REQ_4: [...] envio das palavras [...] no Monitor Serial
            uint32_t count = tud_cdc_read(buf, sizeof(buf) -1); 
            
            if (count > 0) {
                // Prepara o comando para verificação (minúsculas, null-terminado)
                // E também para exibição no OLED (antes de ser modificado para o eco serial)
                char original_command_for_oled[65];
                memcpy(original_command_for_oled, buf, count);
                original_command_for_oled[count] = '\0';

                memcpy(command_to_check, buf, count);
                command_to_check[count] = '\0'; 

                for(int i = 0; command_to_check[i]; i++){
                    command_to_check[i] = tolower(command_to_check[i]);
                }
                
                // REQ_2: [...] e, em seguida, realizar o ECO (enviar de volta ao computador)
                // REQ_5: [...] fazer o eco enviando-os de volta ao computador para que sejam impressos no Monitor Serial
                // REQ_7: Ao realizar o eco (para o comando "som")
                // REQ_8: [...] impressão dos comandos (durante o eco), além do Monitor Serial.
                if (tud_cdc_connected()) {
                    tud_cdc_write_str("Eco: ");
                    tud_cdc_write(buf, count); // Ecoa os bytes originais recebidos (antes do tolower)
                    tud_cdc_write_flush();
                }
                // Mostra o comando original (antes do tolower) no OLED
                oled_display_command_echo(original_command_for_oled); // REQ_8

                // REQ_2: [...] e identificar o comando
                // REQ_5: O dispositivo CDC deverá reconhecer estes comandos
                // REQ_3: [...] fazer a identificação visual do comando
                // REQ_6: Ao realizar o ECO, o dispositivo CDC deverá acender o LED correspondente
                // REQ_7: Acrescentar um “comando” com a palavra “som”
                if (strstr(command_to_check, "vermelho")) {
                    aciona_led(LED_VERMELHO); // REQ_3 e REQ_6
                    cdc_send_message("Feedback: LED Vermelho ativado!\r\n");
                } 
                else if (strstr(command_to_check, "verde")) {
                    aciona_led(LED_VERDE); // REQ_3 e REQ_6
                    cdc_send_message("Feedback: LED Verde ativado!\r\n");
                } 
                else if (strstr(command_to_check, "azul")) {
                    aciona_led(LED_AZUL); // REQ_3 e REQ_6
                    cdc_send_message("Feedback: LED Azul ativado!\r\n");
                }
                else if (strstr(command_to_check, "som")) { // REQ_7
                    aciona_som_buzzer(); // REQ_7
                    cdc_send_message("Feedback: Som do buzzer ativado!\r\n");
                }
                memset(buf, 0, sizeof(buf)); 
            }
        }

        verifica_led_timer(); // Verifica e desliga LEDs após 1s (parte do REQ_6)
        verifica_sound_timer(); // REQ_7: Verifica e desliga o som do buzzer após 1s
    }

    return 0; 
}
