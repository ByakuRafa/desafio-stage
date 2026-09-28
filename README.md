# Desafio Stage ROS 2

Este pacote implementa um nó ROS 2 em Python para o controle autônomo de um robô móvel no simulador Stage. A solução utiliza o algoritmo de navegação integrado a dados de odometria (usando ground truth) e telemetria LiDAR para alcance de objetivos e desvio de obstáculos.

## Arquitetura do Nó (`pursuer.py`)

A navegação baseia-se numa máquina de estados finitos que avalia continuamente a posição do robô face ao alvo e aos obstáculos circundantes.

### Interface ROS 2

| Direção | Tópico | Mensagem | Função |
| --- | --- | --- | --- |
| **Subscrição** | `/ground_truth` | `nav_msgs/Odometry` | Recebe a posição global absoluta do robô no simulador.

 |
| **Subscrição** | `/base_scan` | `sensor_msgs/LaserScan` | Processa as distâncias frontais e laterais direitas obtidas pelo LiDAR.

 |
| **Subscrição** | `/goal_pose` | `geometry_msgs/PoseStamped` | Recebe as coordenadas (X, Y) para definição dinâmica do alvo.

 |
| **Publicação** | `/cmd_vel` | `geometry_msgs/Twist` | Envia comandos cinemáticos de velocidade linear e angular aos motores.

 |

## Lógica de Controle State Machine

O ciclo de controlo opera a 20 Hz (0.05 segundos) através de quatro estados rigorosos:

1. **WAIT_FOR_GOAL (0):** O robô permanece estático a aguardar o recebimento de uma coordenada global através de `/goal_pose`.


2. **GO_TO_GOAL (1):** Regista a coordenada inicial e traça a linha reta imaginária até ao destino. O robô orienta-se para o alvo e se move em linha reta. Se o Sensor detetar um obstáculo a $< 0.60\text{ m}$ de distância frontal, guarda a distância atual e transita para o estado de contorno.


3. **WALL_FOLLOW (2):** Contorna o obstáculo pela direita mantendo uma distância de $0.45\text{ m}$ da parede utilizando um controlador proporcional na velocidade angular. O robô abandona este estado apenas quando cruza a M-Line a uma distância estritamente menor do alvo em comparação ao ponto inicial de impacto.


4. **FINISHED (3):** O robô atinge o destino com uma margem radial de $0.25\text{ m}$, envia comandos de velocidade nula para travar e regressa à escuta de novos comandos.



## Instruções de Execução

**1. Inicialização do Simulador**

**2. Execução do pursuer**
Ececutar o worksppace do ros2 com o pursuer:

```bash
ros2 run desafio_stage_control pursuer

```

**3. Envio de Comando de Navegação**
Num terminal secundário tem que publicar a coordenada a ser alcançada
ex para atingir o ponto $X=6.0, Y=6.0$:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{pose: {position: {x: 6.0, y: 6.0, z: 0.0}}}"

```# My New Project
