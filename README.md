# ATIVCap6_cdc_tinyusb_led_echo

**Dispositivo CDC USB com Eco e Feedback Completo (BitDogLab)**  

## Visão Geral

Este projeto transforma a placa BitDogLab (baseada no Raspberry Pi Pico) em um dispositivo USB CDC (Communication Device Class) interativo, utilizando a biblioteca TinyUSB. O sistema é projetado para receber comandos de texto específicos ("vermelho", "verde", "azul", "som") enviados através de um Monitor Serial (como o do VSCode) a partir de um computador.

Ao receber um comando válido, o dispositivo realiza as seguintes ações:
1. **Eco no Monitor Serial:** O comando original é enviado de volta para o Monitor Serial do computador.
2. **Eco no Display OLED:** O comando original também é exibido no display OLED da placa BitDogLab.
3. **Feedback Visual (LEDs):** Para os comandos de cor, o LED RGB correspondente na placa acende por 1 segundo.
4. **Feedback Sonoro (Buzzer):** Para o comando "som", o buzzer da placa emite um tom audível (via PWM) por 1 segundo.

O projeto demonstra a comunicação USB CDC, o processamento de strings, o controle de periféricos de saída (LEDs, Buzzer, Display OLED) e a temporização não bloqueante.

## Código-fonte Inicial

O desenvolvimento partiu deste código base que implementava a comunicação CDC básica:

```c
#include <stdio.h>
#include "pico/stdlib.h"
#include "tusb.h"

int main() {
    // Inicializa o USB
    stdio_init_all();
    
    // Aguarda o USB ser montado
    while (!tud_cdc_connected()) {
        sleep_ms(100);
    }
    
    printf("USB conectado!\n");

    // Loop principal: ecoa o que receber
    while (true) {
        if (tud_cdc_available()) {
            uint8_t buf[64];
            uint32_t count = tud_cdc_read(buf, sizeof(buf));

            // Para cada byte recebido, imprime no terminal
            for (uint32_t i = 0; i < count; i++) {
                printf("Recebido: %c\n", buf[i]);
            }

            tud_cdc_write(buf, count); // Eco
            tud_cdc_write_flush();
        }
        tud_task(); // Executa tarefas USB
    }

    return 0;
}

```

## Hardware Utilizado (Placa BitDogLab)

- **Porta USB:** Para comunicação CDC com o computador
- **LED RGB:**
  - Vermelho (R): GPIO 13
  - Verde (G): GPIO 11
  - Azul (B): GPIO 12
- **Buzzer A (Controlado por Transistor):** GPIO 21 (para geração de tons via PWM)
- **Display OLED (128x64, I2C):**
  - SDA: GPIO 14
  - SCL: GPIO 15
  - (Conectado à interface i2c1 do RP2040)

## Funcionalidades Implementadas

- **Dispositivo USB CDC:** A placa é reconhecida como uma porta serial virtual
- **Recepção de Comandos:** Aceita "vermelho", "verde", "azul" e "som"
- **Eco Duplo:** Comandos ecoados para o Monitor Serial e display OLED
- **Feedback Visual Temporizado:** LEDs acendem por 1 segundo
- **Feedback Sonoro Temporizado:** Buzzer emite tom de ~2kHz por 1 segundo
- **Interface de Usuário:** Mensagens no Monitor Serial e Display OLED

## Bibliotecas Utilizadas

- **TinyUSB:** Para gestão da comunicação USB
- **SSD1306:** Para controle do display OLED

### Ambiente de Desenvolvimento
- SDK do Raspberry Pi Pico configurado

### Arquivos do Projeto
- `main.c`: Código principal
- Biblioteca OLED: `ssd1306.h`, `ssd1306_font.h`, `ssd1306_i2c.c`, `ssd1306_i2c.h`

## Cumprimento dos Requisitos

| Requisito | Status |
|-----------|--------|
| Dispositivo CDC via USB | ✅ CUMPRIDO |
| Receber e ecoar comandos | ✅ CUMPRIDO |
| Identificação visual com LEDs | ✅ CUMPRIDO |
| Simulação via Monitor Serial | ✅ CUMPRIDO |
| Reconhecimento de comandos | ✅ CUMPRIDO |
| Acionar LEDs por 1s | ✅ CUMPRIDO |
| Comando "som" com buzzer | ✅ CUMPRIDO |
| Display OLED para eco | ✅ CUMPRIDO |

## Propósito



Este projeto foi desenvolvido com fins estritamente educacionais e aprendizdo durante a residência em sistemas embarcados pelo embarcatech
