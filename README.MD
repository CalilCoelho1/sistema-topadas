# 🏥 Sistema TOPADAS

Sistema inteligente de monitoramento de posicionamento e prevenção de quedas em leitos hospitalares

## 🚀 Início Rápido

### Hardware Necessário
- **ESP32 Dev Module**
- **4x Células de carga + HX711**
- **LCD 16x2 I2C**
- **4x Botões + 4x LEDs**

### Instalação
```bash
git clone https://github.com/CalilCoelho1/sistema-topadas.git
cd sistema-topadas
pio run --target upload
```

### Uso
1. **Conectar hardware** conforme pinout
2. **Compilar e upload** com PlatformIO
3. **Calibrar** usando botão TARA
4. **Monitorar** via LCD e Serial

## 🔧 Características

- ✅ **4 sensores de peso** para detecção precisa
- ✅ **Algoritmo inteligente** de risco de queda  
- ✅ **3 modos de operação** (Normal, Quadrantes, Risco)
- ✅ **Interface LCD** com ícones personalizados
- ✅ **Comunicação UART/FTDI** 
- ✅ **LEDs de status** e controle por botões
- ✅ **Modo de baixo consumo**

## 📌 Pinout Rápido

**Sensores HX711:**
- Sensor 1 (Pé Esquerdo): DT=36, SCK=32
- Sensor 2 (Pé Direito): DT=39, SCK=33
- Sensor 3 (Cabeceira Dir): DT=34, SCK=25
- Sensor 4 (Cabeceira Esq): DT=35, SCK=26

**LCD I2C:**
- SDA=21, SCL=22

**LEDs:**
- Verde=16, Amarelo=17, Vermelho=2, Azul=19

**Botões:**
- Tara=4, Modo=5, Pausa=0, Inativo=15

## 🎛️ Controles

**TARA:** Zera todos os sensores  
**MODO:** Alterna visualização (Normal/Quadrantes/Risco)  
**PAUSA:** Para/retoma envio de dados  
**INATIVO:** Modo de baixo consumo  

## 🚨 Sistema de Alertas

- 🟢 **LED Verde**: Sistema ativo
- 🟡 **LED Amarelo**: Sistema pausado (pisca)
- 🔴 **LED Vermelho**: Risco de queda detectado
- 🔵 **LED Azul**: Transmissão de dados

## 📁 Estrutura do Projeto

```
├── src/main.cpp           # Código principal
├── docs/                  # 📚 Documentação completa
├── platformio.ini         # Configurações PlatformIO
└── README.md             # Este arquivo
```

## 📚 Documentação

**[📋 Índice Completo](docs/README.md)** - Navegação da documentação  
**[⚡ Comandos Rápidos](docs/COMANDOS-RAPIDOS.md)** - Cola de consulta  
**[🔧 Git Tutorial](docs/GIT-TUTORIAL.md)** - Comandos Git essenciais  
**[⚡ PlatformIO Guide](docs/PLATFORMIO-GUIDE.md)** - Guia completo  
**[🛠️ Troubleshooting](docs/TROUBLESHOOTING.md)** - Soluções de problemas

## ⚙️ Configuração Rápida

### platformio.ini
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps = 
    bogde/HX711@^0.7.5
    marcoschwartz/LiquidCrystal_I2C@^1.1.4
```

### Primeira execução
```bash
# Compilar
pio run

# Upload para ESP32
pio run --target upload

# Monitor Serial
pio device monitor
```

## 🔄 Desenvolvimento

### Comandos essenciais
```bash
# Salvar mudanças
git add . && git commit -m "✨ Nova funcionalidade" && git push

# Testar código
pio run --target upload --target monitor

# Limpar build
pio run --target clean
```

### Workflow
1. **Criar branch**: `git checkout -b feature/nova-funcionalidade`
2. **Desenvolver e testar**
3. **Commit**: `git commit -m "✨ Add: funcionalidade"`
4. **Push**: `git push origin feature/nova-funcionalidade`
5. **Pull Request** no GitHub

## 📊 Status do Projeto

- **Detecção de posição**: ✅ Funcionando
- **Algoritmo de risco**: ✅ Funcionando  
- **Interface LCD**: ✅ Funcionando
- **Comunicação UART**: ✅ Funcionando
- **Sistema de alertas**: ✅ Funcionando
- **Documentação**: ✅ Completa

## 🤝 Contribuindo

1. **Fork** o projeto
2. **Criar branch**: `git checkout -b feature/AmazingFeature`
3. **Commit**: `git commit -m 'Add some AmazingFeature'`
4. **Push**: `git push origin feature/AmazingFeature`
5. **Pull Request**


## 👥 Autor

**CalilCoelho1** - GitHub: https://github.com/CalilCoelho1

## 🔗 Links Úteis

**Documentação Completa:** docs/  
**Comandos Rápidos:** docs/COMANDOS-RAPIDOS.md  
**Problemas?** docs/TROUBLESHOOTING.md  

**💡 Dica**: Consulte `docs/COMANDOS-RAPIDOS.md` para referência rápida de comandos Git e PlatformIO!