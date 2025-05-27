/**
 * Sistema de Monitoramento TOPADAS - Detecção de Posicionamento e Risco de Queda
 */
// TODO BIBLIOTECAS
  // ===== BIBLIOTECAS =====
  #include <Arduino.h>
  #include <HX711.h>
  #include <Wire.h>
  #include <LiquidCrystal_I2C.h>
  #include "esp_pm.h"       // Para controle de energia
  #include "esp_system.h"   // Para controle da frequência da CPU


// TODO Conexões GPIO

  // ! PARA CODIGO NA PCB (led vermelho 2 p/ 5 | botao  mod 5 p/ 2)
  // =====  FTDI  =====  
  #define FTDI_TX_PIN 23    // Pino TX do ESP32 para RX do FTDI
  #define FTDI_RX_PIN 18    // Pino RX do ESP32 para TX do FTDI

  //=====  BOTOES  =====  
  #define BOTAO_TARA 4      // Botão de tara (GPIO 4)
  #define BOTAO_MODO 5      // Botão para alternar modo de exibição (GPIO 5)
  #define BOTAO_PAUSA 0     // Pino para o botão de pausa (GPIO 0)
  #define BOTAO_INATIVO 15  // Botão para modo inativo (GPIO 15)

  // =====  LEDs =====   
  #define LED_FTDI 19       // LED azul para indicar comunicação com FTDI
  #define LED_ALERTA 2      // LED vermelho para alerta de risco de queda (GPIO 2)
  #define LED_AMARELO 17    // LED amarelo - pisca quando sistema pausado (GPIO 17)
  #define LED_VERDE 16      // LED verde - aceso quando sistema operando (GPIO 16)

  // Sensores de Peso
  // Sensor 1 - Pé esquerdo
  #define pinDT  36    // Input only - S_VP / ADC1_0
  #define pinSCK  32   // 32K_XP / TOUCH9
  // Sensor 2 - Pé direito
  #define pinDT2  39   // Input only - S_VN / ADC1_3
  #define pinSCK2  33  // 32K_XN / TOUCH8
  // Sensor 3 - Cabeceira direita
  #define pinDT3  34   // Input only - VDET_1 / ADC1_6
  #define pinSCK3  25  // DAC_1 / ADC1_8
  // Sensor 4 - Cabeceira esquerda
  #define pinDT4  35   // Input only - VDET_2 / ADC1_7
  #define pinSCK4  26  // DAC_2 / ADC2_9

  // Pinos I2C
  #define SDA_PIN 21
  #define SCL_PIN 22

//TODO VARIÁVEIS CONSTANTES
  // ===== DEFINIÇÕES FRAMES =====
  #define FRAME_HEAD 0x55   // Byte de início do frame
  #define DELIMITER 0x5A    // Byte delimitador
  #define CMD_POSITION 0x1A // Comando para "mudou de posição"
  #define CMD_STATUS 0x1B   // Comando para "status do sistema" (ativo/pausado)

  // ===== CÓDIGOS DE RISCO =====
  #define RISK_NO 0
  #define RISK_YES 1

  // ===== MODOS DE DISPLAY =====
  #define DISPLAY_SENSORES_1_2 0
  #define DISPLAY_SENSORES_3_4 1
  #define DISPLAY_TOTAL 2

  // ===== CÓDIGOS DE POSIÇÃO =====
  #define POS_PE_ESQUERDO 1
  #define POS_PE_DIREITO 2
  #define POS_CAB_DIREITA 3
  #define POS_CAB_ESQUERDA 4

  // ===== PARÂMETROS DO LCD I2C =====
  #define LCD_ENDERECO 0x27  // Endereço I2C (normalmente 0x27 ou 0x3F)
  #define LCD_COLUNAS 16
  #define LCD_LINHAS 2
  #define LCD_BACKLIGHT_LEVEL 255  // Nível de backlight (0-255)

//TODO VARIÁVEIS DE CONTROLE
  #define ZONA_MORTA 0.1f         // Ignora variações de 100g
  #define NUM_LEITURAS 2          // Número de leituras para média móvel
  #define AMOSTRAS_HX711 2        // Número de amostras por leitura do sensor
  #define TAXA_ATUALIZACAO 150    // Tempo entre atualizações do display (ms)
  #define DEBOUNCE_DELAY 100      // Tempo de debounce para os botões (ms)
  #define MSG_DELAY 1500          // Tempo para mostrar mensagem de status (ms)
  #define LIMITE_PESO_MINIMO 0.05 // Peso mínimo para considerar ocupação
  #define LIMITE_BORDA 0.30       // Limite para considerar borda da cama
  #define I2C_FREQUENCIA 800000   // Frequência do barramento I2C (Hz)
  #define TEMPO_PISCA_LED 100     // Tempo em ms que o LED fica aceso após envio

  // ===== CONFIGURAÇÕES DE TEMPO =====
  const unsigned long TEMPO_ALTERNANCIA_MODO_NORMAL = 4000;  // ms
  const unsigned long TEMPO_MOSTRAR_POSICAO = 1500;          // ms
  const unsigned long TEMPO_CICLO_QUADRANTES = 6000;         // ms
  const unsigned long TEMPO_PISCA_LED_AMARELO = 500;         // ms

  // ===== CONFIGURAÇÕES DE RISCO DE QUEDA =====
  int CONFIRMACOES_RISCO_QUEDA = 6;  // Confirmações necessárias para alerta
  const int MAX_CONFIRMACOES_RISCO = 10;  // Limite máximo de confirmações
  const int MIN_CONFIRMACOES_RISCO = 1;   // Limite mínimo de confirmações

//  TODO OBJETOS =====
  // Sensores de peso (células de carga)
  HX711 scale; 
  HX711 scale2; 
  HX711 scale3; 
  HX711 scale4;

  // LCD
  LiquidCrystal_I2C lcd(LCD_ENDERECO, LCD_COLUNAS, LCD_LINHAS);

//TODOS CARACTERES PERSONALIZADOS =====
  // Ícone de pessoa para o LCD
  byte icone_pessoa[8] = {
    B01110, B01110, B00100, B01110, B10101, B00100, B01010, B01010
  };

  // Ícone de alerta para risco de queda
  byte icone_alerta[8] = {
    B00100, B01110, B01110, B01110, B01110, B11111, B00100, B00100
  };

//TODO VARIÁVEIS DE ESTADO
  // Medidas dos sensores
  float medida1 = 0;
  float medida2 = 0;
  float medida3 = 0;
  float medida4 = 0;

  // Estado dos botões
  bool estadoBotaoAnterior = HIGH;
  bool estadoBotaoModoAnterior = HIGH;
  bool estadoBotaoInativoAnterior = HIGH;
  bool estado_anterior_botao_pausa = HIGH;

  // Modo de operação
  int modo_operacao = 0;  // 0: Normal, 1: Quadrantes, 2: Risco de Queda
  int modo_display = DISPLAY_SENSORES_1_2;

  // Variáveis de status
  bool mostrando_status = false;
  bool risco_queda = false;
  bool led_ftdi_ativo = false;
  bool estado_led_amarelo = false;
  bool sistema_monitorando = true;
  bool sistema_ativo = true;

  // Variáveis de risco
  int condicao_risco = 0;
  int condicao_risco_atual = 0;
  int contagem_confirmacao_risco = 0;
  int ultimo_risco_detectado = 0;
  int ultima_condicao = 0;

  // Variáveis para posição
  float porc_quad1 = 0, porc_quad2 = 0, porc_quad3 = 0, porc_quad4 = 0;
  String posicao_atual = "Indefinida";
  String posicao_anterior = "Indefinida";

  // Variáveis para média móvel - Sensor 1
  float leituras1[NUM_LEITURAS];
  int indice_leitura1 = 0;
  float total_leituras1 = 0;
  bool array_inicializado1 = false;

  // Variáveis para média móvel - Sensor 2
  float leituras2[NUM_LEITURAS];
  int indice_leitura2 = 0;
  float total_leituras2 = 0;
  bool array_inicializado2 = false;

  // Variáveis para média móvel - Sensor 3
  float leituras3[NUM_LEITURAS];
  int indice_leitura3 = 0;
  float total_leituras3 = 0;
  bool array_inicializado3 = false;

  // Variáveis para média móvel - Sensor 4
  float leituras4[NUM_LEITURAS];
  int indice_leitura4 = 0;
  float total_leituras4 = 0;
  bool array_inicializado4 = false;
  
  // Controle do LCD
  bool lcd_encontrado = false;
  bool lcd_backlight_on = true;
  bool lcd_atualizado = false;

  // Strings para o LCD
  char linha1[17];
  char linha2[17];
  char linha1_anterior[17] = {0};
  char linha2_anterior[17] = {0};

  // Temporizadores
  unsigned long ult_exibicao = 0;
  unsigned long ult_alternancia = 0;
  unsigned long ult_tempo_botao_tara = 0;
  unsigned long ult_tempo_botao_modo = 0;
  unsigned long ult_tempo_botao_inativo = 0;
  unsigned long ultimo_debounce_botao_pausa = 0;
  unsigned long ultimo_tempo_pisca_amarelo = 0;
  unsigned long msg_status_tempo = 0;
  unsigned long tempo_led_ftdi = 0;

// TODO FUNÇÕES =====
  void garantir_backlight();
  void copiar_para_buffer(const char* texto, char* buffer);
  void adicionar_icone(char* buffer, uint8_t icone_index);
  void atualizar_lcd(const char* linha1_nova, const char* linha2_nova);
  String determinar_posicao(float s1, float s2, float s3, float s4, float total);
  int verificar_tipo_risco();
  void ajustar_confirmacoes_risco(int delta);
  float ler_peso_estabilizado(HX711 &sensor, float *leituras, int &indice, float &total, bool &inicializado);
  void float_para_string(float valor, char* buffer);
  void enviar_posicao_ftdi(String posicao);
  void enviar_risco_ftdi(String posicao, bool tem_risco);
  void enviar_status_sistema();
  void reduzir_frequencia_cpu();
  void restaurar_frequencia_cpu();
  void entrar_modo_inativo();
  void reativar_sistema();
  void processar_botao_pausa();
  void processar_botao_tara(unsigned long tempo_atual);
  void processar_botao_modo(unsigned long tempo_atual);
  void atualizar_leds_status();
  void setup();
  void loop();


// TODO FUNÇÕES LCD

  //Garante que o backlight do LCD esteja ligado

  void garantir_backlight() {
    if (!lcd_backlight_on) {
      lcd.backlight();
      lcd_backlight_on = true;
    }
   }

  //Copia texto para buffer do LCD de forma segura
   
  void copiar_para_buffer(const char* texto, char* buffer) {
    strncpy(buffer, texto, 16);
    buffer[16] = '\0'; // Garante que a string seja terminada corretamente
   }

  //Adiciona ícone na posição inicial da linha
   
  void adicionar_icone(char* buffer, uint8_t icone_index) {
    // Utiliza memmove para deslocar o conteúdo de forma mais eficiente
    memmove(buffer + 1, buffer, strlen(buffer) + 1);
    // Insere o código de ícone na primeira posição
    buffer[0] = icone_index;
   }

  //Atualiza o LCD apenas se o conteúdo mudou
   
  void atualizar_lcd(const char* linha1_nova, const char* linha2_nova) {
    if (!lcd_encontrado) return;
    
    // Garante que o backlight esteja ligado
    garantir_backlight();
    
    // Verificar se a linha 1 mudou
    if (strcmp(linha1_nova, linha1_anterior) != 0) {
      lcd.setCursor(0, 0);
      lcd.print("                "); // Limpa a linha (16 espaços)
      lcd.setCursor(0, 0);
      
      // Verificar se o primeiro caractere é um código de ícone (0 ou 1)
      if (linha1_nova[0] == 0) {
        lcd.write(byte(0)); // Ícone de pessoa
        lcd.print(linha1_nova + 1); // Imprime o resto da string
      } else if (linha1_nova[0] == 1) {
        lcd.write(byte(1)); // Ícone de alerta
        lcd.print(linha1_nova + 1); // Imprime o resto da string
      } else {
        lcd.print(linha1_nova); // Imprime a string normal
      }
      
      strcpy(linha1_anterior, linha1_nova);
    }
    
    // Verificar se a linha 2 mudou
    if (strcmp(linha2_nova, linha2_anterior) != 0) {
      lcd.setCursor(0, 1);
      lcd.print("                "); // Limpa a linha (16 espaços)
      lcd.setCursor(0, 1);
      
      // Verificar se o primeiro caractere é um código de ícone (0 ou 1)
      if (linha2_nova[0] == 0) {
        lcd.write(byte(0)); // Ícone de pessoa
        lcd.print(linha2_nova + 1); // Imprime o resto da string
      } else if (linha2_nova[0] == 1) {
        lcd.write(byte(1)); // Ícone de alerta
        lcd.print(linha2_nova + 1); // Imprime o resto da string
      } else {
        lcd.print(linha2_nova); // Imprime a string normal
      }
      
      strcpy(linha2_anterior, linha2_nova);
    }
   }

// TODO Determina a posição

  String determinar_posicao(float s1, float s2, float s3, float s4, float total) {
    // Se não há peso significativo, cama está vazia
    if (total < LIMITE_PESO_MINIMO) {
      // Zerar todas as porcentagens quando não há peso significativo
      porc_quad1 = porc_quad2 = porc_quad3 = porc_quad4 = 0;
      return "TOPADAS VAZIO";
    }
    
    // Calcula porcentagens de cada quadrante de forma otimizada
    float inv_total = 100.0f / total; 
    porc_quad1 = s1 * inv_total; // Pé esquerdo
    porc_quad2 = s2 * inv_total; // Pé direito
    porc_quad3 = s3 * inv_total; // Cabeceira direita
    porc_quad4 = s4 * inv_total; // Cabeceira esquerda

    // Verifica se está próximo do centro - condições otimizadas
    if ((porc_quad4 >= 30.0f && porc_quad2 >= 30.0f) && porc_quad1 <= 20.0f) { 
      return "CENTRO";
    }
    
    // Determina a posição
    float limite = LIMITE_BORDA * 100;
    if (porc_quad1 > limite || porc_quad3 <= -30.0f) {
      return "PE ESQUERDO";
    } 
    if (porc_quad2 > limite && porc_quad1 <= 30.0f && porc_quad4 <= 30.0f) {
      return "PE DIREITO";
    } 
    if (porc_quad3 > limite && porc_quad3 > porc_quad4) {
      return "CAB DIREITA";
    } 
    if (porc_quad4 > limite && porc_quad4 > porc_quad3 && porc_quad1 <= 30.0f) {
      return "CAB ESQUERDA";
    }
    
    return "INDEFINIDO";
  }

// TODO Verifica se há risco de queda 

    int verificar_tipo_risco() {
      // Lado esquerdo - condições de risco 
      if (porc_quad1 > 55.0f && porc_quad2 < 47.0f) {
        return 1; // Concentração excessiva no pé esquerdo
      }
      if (porc_quad1 > 32.0f && porc_quad4 > 39.0f) {
        return 2; // Desbalanceamento lado esquerdo
      }
      if (porc_quad1 > 4.0f && porc_quad3 < 0.1f && porc_quad4 > 65.0f && porc_quad2 < 40.0f) {
        return 3; // Concentração cabeceira esquerda
      }
      if (porc_quad1 > 22.0f && porc_quad3 < -8.0f && porc_quad4 > 55.0f && porc_quad2 < 40.0f) {
        return 4; // Desbalanceamento diagonal
      }
      if (porc_quad4 > 45.0f && porc_quad2 < 5.0f && porc_quad3 < 21.0f) {
        return 5; // Saindo pela lateral esquerda
      }
      if (porc_quad4 > 45.0f && porc_quad2 < -10.0f && porc_quad3 > 19.0f) {
        return 6; // Desbalanceamento cabeceira
      }

      // Lado direito - condições de risco
      if (porc_quad2 > 55.0f && porc_quad1 < 27.0f && porc_quad3 < 15.0f) {
        return 10; // Concentração excessiva no pé direito
      }
      if (porc_quad2 > 50.0f && porc_quad3 > 12.0f) {
        return 11; // Desbalanceamento lado direito
      }
      if (porc_quad2 > 40.0f && porc_quad2 < 60.0f && porc_quad3 > 30.0f) {
        return 12; // Concentração cabeceira direita
      }
      if (porc_quad3 > 40.0f && porc_quad4 < 25.0f) {
        return 13; // Saindo pela lateral direita
      }
      if (porc_quad3 > 60.0f && porc_quad4 < 45.0f) {
        return 14; // Desbalanceamento extremo direito
      }
      
      return 0; // Sem risco detectado
    }

// TODO Ajusta o número de confirmações necessárias para detectar risco
  
  void ajustar_confirmacoes_risco(int delta) {
    CONFIRMACOES_RISCO_QUEDA += delta;
    
    // Garantir que o valor está dentro dos limites
    if (CONFIRMACOES_RISCO_QUEDA < MIN_CONFIRMACOES_RISCO) {
      CONFIRMACOES_RISCO_QUEDA = MIN_CONFIRMACOES_RISCO;
    }
    if (CONFIRMACOES_RISCO_QUEDA > MAX_CONFIRMACOES_RISCO) {
      CONFIRMACOES_RISCO_QUEDA = MAX_CONFIRMACOES_RISCO;
    }
    
    // Reseta o contador de confirmação ao ajustar o valor
    contagem_confirmacao_risco = 0;
  }

// TODO Lê o peso com estabilização por média móvel
 
  float ler_peso_estabilizado(HX711 &sensor, float *leituras, int &indice, float &total, bool &inicializado) {
    float valor = sensor.get_units(AMOSTRAS_HX711);
    
    if (!inicializado) {
      // Na primeira execução, preenche o array com o valor atual
      for (int i = 0; i < NUM_LEITURAS; i++) {
        leituras[i] = valor;
      }
      total = valor * NUM_LEITURAS;
      inicializado = true;
      return valor;
    }
    
    // Subtrai a leitura mais antiga e adiciona a nova
    total = total - leituras[indice] + valor;
    leituras[indice] = valor;
    
    // Avança o índice
    indice = (indice + 1) % NUM_LEITURAS;
    
    // Retorna a média
    return total / NUM_LEITURAS;
  }

//Formata um valor float para string com tratamento de valores pequenos
 
void float_para_string(float valor, char* buffer) {
  if (fabsf(valor) < 0.001f) {
    strcpy(buffer, "0.00");
  } else {
    dtostrf(valor, 5, 2, buffer);
    // Remover espaços iniciais se houver
    if (buffer[0] == ' ') {
      for (int i = 0; buffer[i]; i++) {
        buffer[i] = buffer[i+1];
      }
    }
  }
}

// TODO Envia posição via UART para o módulo FTDI

  void enviar_posicao_ftdi(String posicao) {
    // Se o sistema estiver pausado ou inativo, não envia nada
    if (!sistema_monitorando || !sistema_ativo) {
      return;
    }
    
    byte codigo_posicao = 0;
    byte codigo_risco = 0; // Quando só envia posição, risco é 0
    
    // Mapear a string da posição para o código numérico
    if (posicao == "PE ESQUERDO") {
      codigo_posicao = POS_PE_ESQUERDO;
    } else if (posicao == "PE DIREITO") {
      codigo_posicao = POS_PE_DIREITO;
    } else if (posicao == "CAB DIREITA") {
      codigo_posicao = POS_CAB_DIREITA;
    } else if (posicao == "CAB ESQUERDA") {
      codigo_posicao = POS_CAB_ESQUERDA;
    } else {
      // Se não for nenhuma das posições específicas, não envia nada
      return;
    }
    
    // Envia o frame completo com dois bytes de dados
    Serial2.write(FRAME_HEAD);   // 0x55
    Serial2.write(DELIMITER);    // 0x5A
    Serial2.write(CMD_POSITION); // 0x1A (mudou de posição)
    Serial2.write(codigo_posicao); // 1, 2, 3 ou 4
    Serial2.write(codigo_risco);   // 0 (sem risco)
    
    // Acende o LED azul para indicar a transmissão
    digitalWrite(LED_FTDI, HIGH);
    led_ftdi_ativo = true;
    tempo_led_ftdi = millis();
  }

  // Envia informação de risco via UART
   
  void enviar_risco_ftdi(String posicao, bool tem_risco) {
    // Se o sistema estiver pausado ou inativo, não envia nada
    if (!sistema_monitorando || !sistema_ativo) {
      return;
    }
    
    byte codigo_posicao = 0;
    byte codigo_risco = tem_risco ? RISK_YES : RISK_NO;
    
    // Mapear a string da posição para o código numérico
    if (posicao == "PE ESQUERDO") {
      codigo_posicao = POS_PE_ESQUERDO;
    } else if (posicao == "PE DIREITO") {
      codigo_posicao = POS_PE_DIREITO;
    } else if (posicao == "CAB DIREITA") {
      codigo_posicao = POS_CAB_DIREITA;
    } else if (posicao == "CAB ESQUERDA") {
      codigo_posicao = POS_CAB_ESQUERDA;
    } else {
      // Para posições indefinidas
      codigo_posicao = 0;
    }
    
    // Envia o frame completo com dois bytes de dados
    Serial2.write(FRAME_HEAD);   // 0x55
    Serial2.write(DELIMITER);    // 0x5A
    Serial2.write(CMD_POSITION); // 0x1A (mudou de posição)  
    Serial2.write(codigo_posicao); // 0-4 (posição)
    Serial2.write(codigo_risco);   // 0 ou 1 (sem/com risco)
    
    // Acende o LED azul para indicar a transmissão
    digitalWrite(LED_FTDI, HIGH);
    led_ftdi_ativo = true;
    tempo_led_ftdi = millis();
  }

  /**
   * Envia o status do sistema via UART
   */
  void enviar_status_sistema() {
    // Determinar os bytes de status com base no estado do sistema
    byte dt1 = sistema_ativo ? 0x01 : 0x00;
    byte dt2 = (!sistema_ativo || !sistema_monitorando) ? 0x01 : 0x00;
    
    // Envia o frame completo
    Serial2.write(FRAME_HEAD);  // 0x55
    Serial2.write(DELIMITER);   // 0x5A
    Serial2.write(CMD_STATUS);  // 0x1B (status do sistema)
    Serial2.write(dt1);         // 0x01=ativo, 0x00=inativo
    Serial2.write(dt2);         // 0x00=monitorando, 0x01=pausado/inativo
    
    // Acende o LED azul para indicar a transmissão
    digitalWrite(LED_FTDI, HIGH);
    led_ftdi_ativo = true;
    tempo_led_ftdi = millis();
    
    // Atualiza os LEDs de status
    atualizar_leds_status();
    
    // Mostra mensagem no Serial para depuração
    Serial.print("Enviado status do sistema: ");
    if (!sistema_ativo) {
      Serial.println("INATIVO (dt1=00, dt2=01)");
    } else if (!sistema_monitorando) {
      Serial.println("ATIVO mas PAUSADO (dt1=01, dt2=01)");
    } else {
      Serial.println("ATIVO e MONITORANDO (dt1=01, dt2=00)");
    }
  }

 /**
 * Atualiza o estado dos LEDs de status
 */
 void atualizar_leds_status() {
  if (!sistema_ativo) {
    // Sistema inativo - ambos LEDs desligados
    digitalWrite(LED_VERDE, LOW);
    digitalWrite(LED_AMARELO, LOW);
  } else {
    // Sistema ativo - LED verde sempre aceso
    digitalWrite(LED_VERDE, HIGH);
    
    // LED amarelo depende do estado de monitoramento
    if (sistema_monitorando) {
      // Sistema operando - LED amarelo apagado
      digitalWrite(LED_AMARELO, LOW);
    } else {
      // Sistema pausado - o LED amarelo pisca no loop principal
      // O controle é feito no loop principal para piscar
    }
  }
 }

// TODO SISTEMA INATIVO
 
  void reduzir_frequencia_cpu() {
    setCpuFrequencyMhz(80); // Reduz de 240MHz para 80MHz
  }

  /**
   * Restaura a frequência normal da CPU
   */
  void restaurar_frequencia_cpu() {
    setCpuFrequencyMhz(240); // Volta para frequência normal
  }

  /**
   * Coloca o sistema em modo inativo (baixo consumo)
   */
  void entrar_modo_inativo() {
    sistema_ativo = false;
    
    // Desligar sensores para economizar energia
    scale.power_down();
    scale2.power_down();
    scale3.power_down();
    scale4.power_down();
    
    // Desligar LEDs não essenciais
    digitalWrite(LED_ALERTA, LOW);
    digitalWrite(LED_AMARELO, LOW);
    digitalWrite(LED_VERDE, LOW);
    
    // Reduzir frequência da CPU
    reduzir_frequencia_cpu();
    
    // Feedback para o usuário
    if (lcd_encontrado) {
      copiar_para_buffer("SISTEMA", linha1);
      copiar_para_buffer("INATIVO", linha2);
      atualizar_lcd(linha1, linha2);
      delay(1000);
      lcd.noBacklight(); // Desliga backlight para economizar energia
      lcd_backlight_on = false;
    }
    
    Serial.println("Sistema INATIVO - Modo de baixo consumo ativado");
    
    // Enviar status para a outra ESP
    enviar_status_sistema();
  }

  /**
   * Reativa o sistema do modo inativo
   */
  void reativar_sistema() {
    sistema_ativo = true;
    
    // Restaurar frequência da CPU
    restaurar_frequencia_cpu();
    
    // Reativar sensores
    scale.power_up();
    scale2.power_up();
    scale3.power_up();
    scale4.power_up();
    
    // Configuração dos LEDs de estado
    atualizar_leds_status();
    
    // Feedback para o usuário
    if (lcd_encontrado) {
      garantir_backlight(); // Liga backlight
      copiar_para_buffer("SISTEMA", linha1);
      copiar_para_buffer("REATIVADO", linha2);
      atualizar_lcd(linha1, linha2);
    }
    
    Serial.println("Sistema REATIVADO");
    
    // Enviar novo status
    enviar_status_sistema();
  }

//TODO botão de pausa

  void processar_botao_pausa() {
    // Lê o estado atual do botão de pausa
    bool botao_estado_atual = digitalRead(BOTAO_PAUSA);
    unsigned long tempo_atual = millis();
    
    // Verifica se houve pressionamento com debounce
    if (botao_estado_atual == LOW && estado_anterior_botao_pausa == HIGH && 
        (tempo_atual - ultimo_debounce_botao_pausa > DEBOUNCE_DELAY)) {
      
      ultimo_debounce_botao_pausa = tempo_atual;
      
      // Alterna o estado de monitoramento
      sistema_monitorando = !sistema_monitorando;
      
      // Envia o novo status para a ESP secundária
      enviar_status_sistema();
      
      // Log para depuração
      if (sistema_monitorando) {
        Serial.println("Sistema MONITORANDO - Enviando dados para a outra ESP");
      } else {
        Serial.println("Sistema PAUSADO - Envio de dados interrompido");
      }
      
      // Atualiza LCD com o novo status
      if (lcd_encontrado) {
        if (sistema_monitorando) {
          copiar_para_buffer("MONITORANDO", linha1);
          copiar_para_buffer("Dados enviados", linha2);
        } else {
          copiar_para_buffer("PAUSADO", linha1);
          copiar_para_buffer("Envio suspenso", linha2);
        }
        atualizar_lcd(linha1, linha2);
        
        // Inicia contador para mostrar a mensagem temporariamente
        msg_status_tempo = tempo_atual;
        mostrando_status = true;
      }
    }
    
    // Atualiza o estado anterior para a próxima iteração
    estado_anterior_botao_pausa = botao_estado_atual;
   }

// TODO  botão de tara

  void processar_botao_tara(unsigned long tempo_atual) {
    // Lê o estado atual do botão de tara
    bool estadoBotaoAtual = digitalRead(BOTAO_TARA);
    
    // Verifica se houve pressionamento com debounce
    if (estadoBotaoAtual == LOW && estadoBotaoAnterior == HIGH && 
        (tempo_atual - ult_tempo_botao_tara > DEBOUNCE_DELAY)) {
      
      ult_tempo_botao_tara = tempo_atual;
      
      // Garante que o backlight esteja ligado
      garantir_backlight();
      
      // Executa tara em todos os sensores
      scale.tare(1);  // Tara rápida com menos amostras
      scale2.tare(1);
      scale3.tare(1);
      scale4.tare(1);
      
      // Reset das médias móveis
      array_inicializado1 = false;
      array_inicializado2 = false;
      array_inicializado3 = false;
      array_inicializado4 = false;
      
      // Reset dos contadores de risco
      contagem_confirmacao_risco = 0;
      ultimo_risco_detectado = 0;
      condicao_risco = 0;
      risco_queda = false;
      
      Serial.println("Tara realizada");
      
      // Atualiza o LCD
      if (lcd_encontrado) {
        copiar_para_buffer("Tarando...", linha1);
        copiar_para_buffer("", linha2);
        atualizar_lcd(linha1, linha2);
        
        // Mensagem temporária
        msg_status_tempo = tempo_atual;
        mostrando_status = true;
      }
    }
    
    // Atualiza o estado anterior para a próxima iteração
    estadoBotaoAnterior = estadoBotaoAtual;
  }

// TODO botão de modo

  void processar_botao_modo(unsigned long tempo_atual) {
    // Lê o estado atual do botão de modo
    bool estadoBotaoAtual = digitalRead(BOTAO_MODO);
    
    // Verifica se houve pressionamento com debounce
    if (estadoBotaoAtual == LOW && estadoBotaoModoAnterior == HIGH && 
        (tempo_atual - ult_tempo_botao_modo > DEBOUNCE_DELAY)) {
      
      ult_tempo_botao_modo = tempo_atual;
      
      // Garante que o backlight esteja ligado
      garantir_backlight();
      
      // Alterna entre os modos de operação
      modo_operacao = (modo_operacao + 1) % 3;
      
      // Log do novo modo
      if (modo_operacao == 0) {
        Serial.println("Modo: Normal");
      } else if (modo_operacao == 1) {
        Serial.println("Modo: Quadrantes");
      } else {
        Serial.println("Modo: Risco de Queda");
      }
      
      // Notifica a mudança no LCD
      if (lcd_encontrado) {
        strcpy(linha1, "Modo: ");
        if (modo_operacao == 0) {
          strcat(linha1, "Normal");
        } else if (modo_operacao == 1) {
          strcat(linha1, "Quadrantes");
        } else {
          strcat(linha1, "Risco de Queda");
        }
        copiar_para_buffer("", linha2);
        atualizar_lcd(linha1, linha2);
        
        // Mensagem temporária
        msg_status_tempo = tempo_atual;
        mostrando_status = true;
      }
    }
    
    // Atualiza o estado anterior para a próxima iteração
    estadoBotaoModoAnterior = estadoBotaoAtual;
  }

// TODO SETUP Inicialização do sistema

  void setup() {
    // Inicializa a comunicação serial
    Serial.begin(115200);
    Serial.println("Iniciando sistema TOPADAS...");
    
    // Inicializa a UART para comunicação com FTDI
    Serial2.begin(115200, SERIAL_8N1, FTDI_RX_PIN, FTDI_TX_PIN);
    Serial.println("UART2 para FTDI inicializada");
    
    // Configura LEDs
    pinMode(LED_FTDI, OUTPUT);
    digitalWrite(LED_FTDI, LOW);
    pinMode(LED_ALERTA, OUTPUT);
    digitalWrite(LED_ALERTA, LOW);
    
    // Configura LEDs de status
    pinMode(LED_AMARELO, OUTPUT);
    digitalWrite(LED_AMARELO, LOW);
    pinMode(LED_VERDE, OUTPUT);
    digitalWrite(LED_VERDE, LOW);
    
    Serial.println("LEDs configurados - LED_VERDE (GPIO 16), LED_AMARELO (GPIO 17), LED_ALERTA (GPIO 2), LED_FTDI (GPIO 19)");
    
    // Configura os pinos dos botões com pull-up interno
    pinMode(BOTAO_TARA, INPUT_PULLUP);
    pinMode(BOTAO_MODO, INPUT_PULLUP);
    pinMode(BOTAO_INATIVO, INPUT_PULLUP);
    pinMode(BOTAO_PAUSA, INPUT_PULLUP);
    
    Serial.println("Botões configurados com pull-up interno - TARA (GPIO 4), MODO (GPIO 5), PAUSA (GPIO 0), INATIVO (GPIO 15)");
    
    // Configura comunicação I2C em alta velocidade
    Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQUENCIA);
    
    // Tenta o primeiro endereço (0x27)
    Wire.beginTransmission(LCD_ENDERECO);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      lcd_encontrado = true;
      Serial.println("LCD encontrado no endereço 0x27");
      
      lcd.init();
      lcd.backlight();  // Ativa o backlight
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("INICIALIZANDO");
      lcd.setCursor(0, 1);
      lcd.print("PROJETO TOPADAS");
      
      // Cria caracteres personalizados
      lcd.createChar(0, icone_pessoa);
      lcd.createChar(1, icone_alerta);
    } 
    
    // Sequência de inicialização com LEDs
    digitalWrite(LED_AMARELO, HIGH);
    delay(200);
    digitalWrite(LED_VERDE, HIGH);
    delay(200);
    digitalWrite(LED_AMARELO, LOW);
    delay(200);
    digitalWrite(LED_VERDE, LOW);
    
    // Inicialização dos sensores
    Serial.println("Iniciando sensores de peso...");
    
    // Inicialização dos sensores
    scale.begin(pinDT, pinSCK);
    scale2.begin(pinDT2, pinSCK2);
    scale3.begin(pinDT3, pinSCK3);
    scale4.begin(pinDT4, pinSCK4);

    // Definição das escalas dos sensores
    scale.set_scale(-426400);
    scale2.set_scale(-448225);
    scale3.set_scale(-426506.61);
    scale4.set_scale(-437955);
    
    // Tara de todos os sensores
    scale.tare();
    scale2.tare();
    scale3.tare();
    scale4.tare();
    
    // Exibe informações de inicialização
    Serial.println("Calibração concluída");
    Serial.println("Sistema pronto para uso");
    Serial.println("Sistema em modo MONITORANDO - LED verde aceso quando ativo, LED amarelo pisca quando pausado");
    
    if (lcd_encontrado) {
      garantir_backlight();
      copiar_para_buffer("Calibrado", linha1);
      copiar_para_buffer("Pronto p/ Uso", linha2);
      atualizar_lcd(linha1, linha2);
      delay(1000);
      
      // Mostra o status do monitoramento
      copiar_para_buffer("MONITORANDO", linha1);
      copiar_para_buffer("Dados enviados", linha2);
      atualizar_lcd(linha1, linha2);
      delay(1000);
    }
    
    // Pisca o LED do FTDI três vezes para indicar inicialização
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_FTDI, HIGH);
      delay(100);
      digitalWrite(LED_FTDI, LOW);
      delay(100);
    }
    
    // Configura os LEDs de status
    digitalWrite(LED_VERDE, HIGH);
    digitalWrite(LED_AMARELO, LOW);
    
    // Envia o status inicial para o módulo FTDI
    enviar_status_sistema();
  }

// TODO Loop principal do sistema
 
  void loop() {
    unsigned long tempo_atual = millis();
    
    // Controle do LED do FTDI - desliga após o tempo definido
    if (led_ftdi_ativo && (tempo_atual - tempo_led_ftdi > TEMPO_PISCA_LED)) {
      digitalWrite(LED_FTDI, LOW);
      led_ftdi_ativo = false;
    }
    
    // Controle do LED amarelo piscante quando pausado
    if (sistema_ativo && !sistema_monitorando) {
      // LED amarelo pisca quando o sistema está pausado
      if (tempo_atual - ultimo_tempo_pisca_amarelo >= TEMPO_PISCA_LED_AMARELO) {
        ultimo_tempo_pisca_amarelo = tempo_atual;
        estado_led_amarelo = !estado_led_amarelo; // Inverte o estado
        digitalWrite(LED_AMARELO, estado_led_amarelo ? HIGH : LOW);
      }
    }
    
    // Verificar botão de inativação com debounce
    bool estadoBotaoInativoAtual = digitalRead(BOTAO_INATIVO);
    if (estadoBotaoInativoAtual == LOW && estadoBotaoInativoAnterior == HIGH && 
        (tempo_atual - ult_tempo_botao_inativo) > DEBOUNCE_DELAY) {
      ult_tempo_botao_inativo = tempo_atual;
      
      // Alternar entre ativo e inativo
      if (sistema_ativo) {
        entrar_modo_inativo();
      } else {
        reativar_sistema();
      }
    }
    estadoBotaoInativoAnterior = estadoBotaoInativoAtual;
    
    // Se o sistema estiver inativo, não executa o resto do loop
    if (!sistema_ativo) {
      // Apenas verifica o botão periodicamente
      delay(100);  // Pequeno delay para economizar processamento
      yield();     // Permite que o ESP32 processe outras tarefas importantes
      return;      // Sai do loop sem processar o resto
    }
    
    // Processa o botão de pausa
    processar_botao_pausa();
    
    // Verifica se há dados disponíveis no Serial para ajuste de confirmações
    if (Serial.available() > 0) {
      String comando = Serial.readStringUntil('\n');
      comando.trim();
      
      if (comando == "r+") {
        ajustar_confirmacoes_risco(1); // Aumenta as confirmações de risco
      } else if (comando == "r-") {
        ajustar_confirmacoes_risco(-1); // Diminui as confirmações de risco
      } else if (comando.startsWith("r=")) {
        // Define um valor específico para as confirmações de risco (ex: r=5)
        int valor = comando.substring(2).toInt();
        if (valor >= MIN_CONFIRMACOES_RISCO && valor <= MAX_CONFIRMACOES_RISCO) {
          CONFIRMACOES_RISCO_QUEDA = valor;
          contagem_confirmacao_risco = 0; // Reset do contador
          Serial.print("Confirmações para risco de queda definidas para: ");
          Serial.println(CONFIRMACOES_RISCO_QUEDA);
        }
      }
    }
    
    // Verificação para não atualizar LCD durante a exibição da mensagem de status
    if (mostrando_status) {
      // Apenas garantir backlight durante o tempo de exibição da mensagem
      if (lcd_encontrado && !lcd_backlight_on) {
        garantir_backlight();
      }
      
      // Verifica se o tempo de exibição da mensagem terminou
      if (tempo_atual - msg_status_tempo >= MSG_DELAY) {
        mostrando_status = false;
      }
    } else {
      // Garantir que o backlight do LCD esteja sempre ligado
      if (lcd_encontrado && !lcd_backlight_on) {
        garantir_backlight();
      }
      
      // Processa os botões
      processar_botao_tara(tempo_atual);
      processar_botao_modo(tempo_atual);
      
      // Configuração de exibição de pesos no LCD
      // Se estiver no modo normal, alterna a visualização a cada X segundos
      if (modo_operacao == 0 && tempo_atual - ult_alternancia >= TEMPO_ALTERNANCIA_MODO_NORMAL) {
        ult_alternancia = tempo_atual;
        modo_display = (modo_display + 1) % 3; // Alterna entre os modos de display
      }
      
      // Limita a atualização do display
      if (tempo_atual - ult_exibicao >= TAXA_ATUALIZACAO) {
        ult_exibicao = tempo_atual;
        
        // Garante que o backlight esteja ligado a cada atualização
        garantir_backlight();
        
        // Obtém as medidas com estabilização
        medida1 = ler_peso_estabilizado(scale, leituras1, indice_leitura1, total_leituras1, array_inicializado1);
        medida2 = ler_peso_estabilizado(scale2, leituras2, indice_leitura2, total_leituras2, array_inicializado2);
        medida3 = ler_peso_estabilizado(scale3, leituras3, indice_leitura3, total_leituras3, array_inicializado3);
        medida4 = ler_peso_estabilizado(scale4, leituras4, indice_leitura4, total_leituras4, array_inicializado4);
        
        // Aplica zona morta para pequenas variações
        medida1 = (fabsf(medida1) < ZONA_MORTA) ? 0 : medida1;
        medida2 = (fabsf(medida2) < ZONA_MORTA) ? 0 : medida2;
        medida3 = (fabsf(medida3) < ZONA_MORTA) ? 0 : medida3;
        medida4 = (fabsf(medida4) < ZONA_MORTA) ? 0 : medida4;
        
        // Calcula o peso total
        float peso_total = medida1 + medida2 + medida3 + medida4; 
        
        // Determina a posição 
        String nova_posicao = determinar_posicao(medida1, medida2, medida3, medida4, peso_total);
        
        // Verifica se a posição mudou e envia via UART
        if (nova_posicao != posicao_anterior) {
          posicao_anterior = nova_posicao;
          posicao_atual = nova_posicao;
          
          // Envia a nova posição
          enviar_posicao_ftdi(nova_posicao);
        }
        
        // Verifica o risco de queda
        condicao_risco_atual = verificar_tipo_risco();
        
        // Verifica se o tipo de risco mudou
        if (condicao_risco_atual != ultimo_risco_detectado) {
          // Tipo de risco mudou, reseta o contador
          contagem_confirmacao_risco = 0;
        }
        
        // Atualiza o último risco detectado
        ultimo_risco_detectado = condicao_risco_atual;
        
        // Processamento de confirmação de risco
        if (condicao_risco_atual > 0) {
          contagem_confirmacao_risco++;
        } else if (contagem_confirmacao_risco > 0) {
          contagem_confirmacao_risco--;
        }
        
        // Verifica se o risco foi confirmado
        if (contagem_confirmacao_risco >= CONFIRMACOES_RISCO_QUEDA) {
          // Risco confirmado!
          if (!risco_queda || condicao_risco != condicao_risco_atual) {
            // Novo risco detectado ou tipo de risco mudou
            risco_queda = true;
            condicao_risco = condicao_risco_atual;
            
            // Envia o status de risco
            enviar_risco_ftdi(posicao_atual, true);
            
            // Mostra no monitor serial
            Serial.print("ALERTA: Risco de Queda - Condição ");
            Serial.print(condicao_risco);
            Serial.print(" confirmada após ");
            Serial.print(CONFIRMACOES_RISCO_QUEDA);
            Serial.println(" leituras!"); 
          }
        } else if (contagem_confirmacao_risco == 0 && risco_queda) {
          // Risco cessou
          risco_queda = false;
          
          // Envia o status de sem risco
          enviar_risco_ftdi(posicao_atual, false);
          
          Serial.println("ALERTA CESSADO: Sem risco de queda");
        }
        
        // Controla o LED de alerta
        digitalWrite(LED_ALERTA, risco_queda ? HIGH : LOW);
        
        // Prepara dados para o LCD
        if (lcd_encontrado && !mostrando_status) {
          char buffer[17];
          char valor1[8], valor2[8], valor3[8], valor4[8], total_str[8];
          
          // Converte todos os valores para strings
          float_para_string(medida1, valor1);
          float_para_string(medida2, valor2);
          float_para_string(medida3, valor3);
          float_para_string(medida4, valor4);
          float_para_string(peso_total, total_str);
          
          // Atualiza o LCD de acordo com o modo atual
          if (modo_operacao == 0) {
            // Modo normal - alternar entre diferentes visualizações
            switch (modo_display) {
              case DISPLAY_SENSORES_1_2:
                sprintf(linha1, "PE: %s %s", valor1, valor2);
                sprintf(linha2, "Total: %s kg", total_str);
                break;
              case DISPLAY_SENSORES_3_4:
                sprintf(linha1, "CAB: %s %s", valor3, valor4);
                sprintf(linha2, "Total: %s kg", total_str);
                break;
              case DISPLAY_TOTAL:
                if (!sistema_monitorando) {
                  sprintf(linha1, "PAUSADO");
                } else {
                  sprintf(linha1, "Peso: %s kg", total_str);
                }
                sprintf(linha2, "%s", posicao_atual.c_str());
                break;
            }
          } else if (modo_operacao == 1) {
            // Modo quadrantes - exibe posição e porcentagens
            if (tempo_atual % TEMPO_CICLO_QUADRANTES < TEMPO_MOSTRAR_POSICAO) {
              // Mostrar posição com ícone de pessoa
              strcpy(linha1, " POSICAO:");
              adicionar_icone(linha1, 0);
              
              // Ajusta o texto para caber no LCD
              if (posicao_atual.length() > 16) {
                posicao_atual = posicao_atual.substring(0, 16);
              }
              strcpy(linha2, posicao_atual.c_str());
            } else {  
              // Mostrar porcentagens
              if (peso_total < LIMITE_PESO_MINIMO) {
                sprintf(linha1, "P.E:0%% P.D:0%%");
                sprintf(linha2, "C.D:0%% C.E:0%%");
              } else {
                sprintf(linha1, "P.E:%d%% P.D:%d%%", (int)porc_quad1, (int)porc_quad2);
                sprintf(linha2, "C.D:%d%% C.E:%d%%", (int)porc_quad3, (int)porc_quad4);
              }
            }
          } else {
            // Modo risco de queda
            if (risco_queda) {
              strcpy(linha1, " RISCO ");
              adicionar_icone(linha1, 1);
              char num[3];
              sprintf(num, "%d", condicao_risco);
              strcat(linha1, num);
              
              // Mostra o progresso na segunda linha
              sprintf(linha2, "Verificado %d/%d", CONFIRMACOES_RISCO_QUEDA, CONFIRMACOES_RISCO_QUEDA);
            } else if (condicao_risco_atual > 0) {
              // Há risco, mas ainda não confirmado
              strcpy(linha1, "CONF. RISCO... ");
              sprintf(linha2, "Progresso: %d/%d", contagem_confirmacao_risco, CONFIRMACOES_RISCO_QUEDA);
            } else {
              strcpy(linha1, "SEM RISCO");
              
              // Limita o tamanho da posição
              String pos_truncada = posicao_atual;
              if (pos_truncada.length() > 10) {
                pos_truncada = pos_truncada.substring(0, 10);
              }
              sprintf(linha2, "Pos: %s", pos_truncada.c_str());
            }
          }
          
          // Atualiza o LCD
          atualizar_lcd(linha1, linha2);
        }
      }
    }
    
    // Permite que o ESP32 processe outras tarefas
    yield();
  }