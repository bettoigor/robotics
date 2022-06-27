# state
int32 activeState
# string
string stateString

##Tabela de Estados possiveis
# 0 - Desligado - Estado inicial do robo. E necessario modificar o estado atraves do controle pelo celular para alterar a velocidade
# 1 - Controle Manual - Controle realizado atraves do celular
# 2 - Autonomo (com velocidade constante) - Controle autonomo para corrigir a orientacao do robo, andando a uma velocidade constante
# 3 - Erro de Comunicacao - Estado para desligamento de velocidade do robo caso perca comunicacao com o celular. E necessario voltar ao Estado 0 para voltar a andar
# 4 - Controle ponto a ponto - Controle implementado para andar autonomamente uma distancia fornecida pelo usuario.
# 5 - Bateria Fraca - Estado de desligamento do robo devido a Bateria Fraca
# 6 - Aquisicao de fotos - Estado de desligamento da velocidade do robo e aquisicao das fotos na plantacao durante 5 segundos
# 10 - Emergencia - Estado de desligamento do robo, devido ao acionamento do botao de emergencia

# idle
int32 IDLE = 0

# manual 
int32 MANUAL = 1

# autonomous
int32 AUTONOMOUS = 2

# heartbeat error
int32 HEARTBEAT_ERROR = 3

# P2P_CONTROL
int32 P2P_CONTROL = 4

# LOW BATTERY
int32 LOW_BATTERY = 5

# PHOTOS
int32 TAKE_PHOTOS = 6

# EMERGENCY
int32 EMERGENCY = 10
