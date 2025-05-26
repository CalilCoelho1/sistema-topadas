# Compilar projeto
pio run

# Upload para placa
pio run --target upload

# Compilar + Upload
pio run --target upload

# Monitor Serial
pio device monitor

# Upload + Monitor (sequencial)
pio run --target upload --target monitor

# Limpar build
pio run --target clean

# Atualizar plataformas
pio platform update

# Atualizar PlatformIO
pio upgrade

# Info da placa
pio boards esp32

# Buscar biblioteca
pio pkg search ArduinoJson

# Instalar biblioteca
pio pkg install --library "bblanchon/ArduinoJson@^6.19.4"

# Listar bibliotecas instaladas
pio pkg list

# Remover biblioteca
pio pkg uninstall --library ArduinoJson