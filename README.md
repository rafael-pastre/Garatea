# Garatea
## Arquivos Finais:
- Biblioteca com implementações dos sensores: Garatea/Arduino/libraries/ZenithLib/src
- Código microcontrolador Master: Garatea/Arduino/Master_Slave/Master_Slave.ino
- Código microcontrolador Slave: Garatea/Arduino/Slave/Slave.ino
### Breve descrição do funcionamento da biblioteca:
- O arquivo "project.h" contém as definições de quais sensores serão utilizados, e da pinagem utilizada
- Os arquivos no formato "zSENSOR.h" e "zSENSOR.cpp" implementam a inteface dos sensores e variaveis globais para o micro que utiliza o sensor
- Os microcontroladores que utilizam a biblioteca incluem o arquivo "zenith.h", que por sua vez adiciona os sensores definidos em "project.h"
- Os arquivos "zstdlib.h" e "zstdlib.cpp" possuem funções básicas que podem ser utilizadas por todos os sensores
