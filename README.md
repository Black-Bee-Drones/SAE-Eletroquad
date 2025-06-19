# Pacote Hook - Missão 2 (M2): "Hang the Hook"

Este pacote implementa a lógica de controle para a Missão 2 da competição SAE Aerodesign Eletroquad, utilizando uma Máquina de Estados Finitos (FSM) baseada na biblioteca YASMIN.

## Descrição da Missão

O objetivo desta missão é decolar transportando um gancho e realizar a colocação ou soltura deste gancho de forma que ele fique preso, por gravidade, a uma mangueira vermelha com 12.7 mm de diâmetro, representando uma linha de transmissão, erguida a 2 metros de altura. O veículo deverá decolar de uma base quadrada no solo contendo um círculo azul e localizar uma mangueira vermelha na qual o gancho deverá ser pendurado. Ao realizar a soltura, o veículo deverá voltar e pousar na base de onde decolou. Para auxiliar na localização da mangueira vermelha, o drone poderá (sem obrigatoriedade) seguir uma linha azul no solo, com 25 cm de espessura.

![](./assets/mission_ilustrate.png)


## Implementação

A lógica da missão é orquestrada por uma Máquina de Estados Finitos (FSM) implementada com a biblioteca [YASMIN](https://github.com/uleroboticsgroup/yasmin). O ponto de entrada principal é o script `hook/hook/states/hang_the_hook_sm.py`, que define a sequência de estados e transições.

Cada estado representa uma fase da missão (ex: decolagem, busca da linha, centralização, etc.) e encapsula a lógica específica usando principalmente a `Mirela SDK` para controle do drone (via MAVROS) e processamento de visão (detecção de linhas e blobs). Controladores PID são utilizados para tarefas de navegação como seguir a linha e centralizar sobre alvos.

O fluxo geral da máquina de estados é visualizado abaixo:

```mermaid
stateDiagram-v2
    direction TB
    
    %% Main State Machine (HangTheHookSM)
    [*] --> INITIALIZE
    INITIALIZE --> TAKEOFF : SUCCEED
    INITIALIZE --> END : ABORT
    TAKEOFF --> SEARCH_BLUE_LINE : SUCCEED
    TAKEOFF --> RETURN_TO_LAUNCH : ABORT
    SEARCH_BLUE_LINE --> FOLLOW_BLUE_LINE : SUCCEED
    SEARCH_BLUE_LINE --> RETURN_TO_LAUNCH : ABORT
    FOLLOW_BLUE_LINE --> CENTER_RED_BLOB : SUCCEED
    FOLLOW_BLUE_LINE --> RETURN_TO_LAUNCH : ABORT
    CENTER_RED_BLOB --> DESCEND_TO_HOOK : SUCCEED
    CENTER_RED_BLOB --> RETURN_TO_LAUNCH : ABORT
    DESCEND_TO_HOOK --> RELEASE_HOOK : SUCCEED
    DESCEND_TO_HOOK --> RETURN_TO_LAUNCH : ABORT
    RELEASE_HOOK --> RETURN_TO_LAUNCH
    RETURN_TO_LAUNCH --> END
    END --> [*]
    
    %% Notes for each state
    note right of INITIALIZE
        Conecta ao drone via MavDrone
        Verifica status inicial
    end note
    
    note right of TAKEOFF
        Decola para TAKEOFF_ALTITUDE (5.0m)
        Monitora altitude até atingir setpoint
    end note
    
    note right of SEARCH_BLUE_LINE
        Inicia line_detection_node (azul)
        Move para frente (FORWARD_SPEED_SEARCH_BLUE_LINE)
        Confirma detecção (MIN_BLUE_LINE_DETECTIONS)
    end note
    
    note right of FOLLOW_BLUE_LINE
        Inicia line_detection_node (azul, vermelho)
        Inicia PIDs de centralização e ângulo
        Segue linha azul e detecta mangueira vermelha
        (MIN_RED_COUNT_CONFIRMATIONS)
    end note
    
    note right of CENTER_RED_BLOB
        Inicia line_detection_node (vermelho)
        Inicia PID de centralização
        Centraliza sobre a mangueira vermelha
        (CENTERING_CONFIRMATIONS)
    end note
    
    note right of DESCEND_TO_HOOK
        Estima distância até a mangueira
        Desce até TARGET_DISTANCE_CM (60cm)
        Mantém centralização durante descida
    end note
    
    note right of RELEASE_HOOK
        Para movimento
        Aciona servo (HOCK_SERVO_CHANNEL: 3)
        HOCK_HOLD_PWM → HOCK_RELEASE_PWM
    end note
    
    note right of RETURN_TO_LAUNCH
        Retorna ao ponto de decolagem
        Altitude: RETURN_ALTITUDE (2m)
    end note
    
    note right of END
        Finaliza processos (detecção, PIDs)
        Encerra missão
    end note
```

## Submáquinas de Estados

A implementação utiliza submáquinas de estados para gerenciar tarefas complexas. Abaixo estão os diagramas das principais submáquinas:

### SearchBlueLine

```mermaid
stateDiagram-v2
    direction LR
    [*] --> START_DETECTION
    START_DETECTION --> SEARCH_LINE : SUCCEED
    START_DETECTION --> CLEANUP_RESOURCES : ABORT
    SEARCH_LINE --> CLEANUP_RESOURCES : SUCCEED/ABORT
    CLEANUP_RESOURCES --> [*] : SUCCEED/ABORT
    
    note right of START_DETECTION
        StartBlueLineDetection
        Inicia line_detection_node para cor azul
    end note
    
    note right of SEARCH_LINE
        SearchForBlueLine
        Move para frente (FORWARD_SPEED_SEARCH_BLUE_LINE)
        Monitora detecções consecutivas
    end note
    
    note right of CLEANUP_RESOURCES
        CleanupResources
        Finaliza processos de detecção
    end note
```

### FollowBlueLineWithRedDetection

```mermaid
stateDiagram-v2
    direction LR
    [*] --> START_LINE_DETECTION
    START_LINE_DETECTION --> SETUP_REPUBLISHER : SUCCEED
    START_LINE_DETECTION --> CLEANUP_PROCESSES : ABORT
    SETUP_REPUBLISHER --> START_PID_CONTROLLERS : SUCCEED
    SETUP_REPUBLISHER --> CLEANUP_PROCESSES : ABORT
    START_PID_CONTROLLERS --> FOLLOW_LINE : SUCCEED
    START_PID_CONTROLLERS --> CLEANUP_PROCESSES : ABORT
    FOLLOW_LINE --> CLEANUP_PROCESSES : SUCCEED/ABORT
    FOLLOW_LINE --> [*] : red_detected
    CLEANUP_PROCESSES --> [*] : SUCCEED
    
    note right of START_LINE_DETECTION
        StartLineDetection
        Inicia line_detection_node para azul e vermelho
    end note
    
    note right of SETUP_REPUBLISHER
        SetupLineStateRepublisher
        Configura publicadores para PIDs
    end note
    
    note right of START_PID_CONTROLLERS
        StartPIDControllers
        Inicia PIDs para centralização e ângulo
    end note
    
    note right of FOLLOW_LINE
        FollowLineWithDetection
        Segue linha azul e detecta mangueira vermelha
    end note
    
    note right of CLEANUP_PROCESSES
        CleanupProcesses
        Finaliza PIDs e processos de detecção
    end note
```

### CenterRedBlob

```mermaid
stateDiagram-v2
    direction LR
    [*] --> START_RED_LINE_DETECTION
    START_RED_LINE_DETECTION --> SETUP_RED_LINE_STATE_REPUBLISHER : SUCCEED
    START_RED_LINE_DETECTION --> CLEANUP_PROCESSES : ABORT
    SETUP_RED_LINE_STATE_REPUBLISHER --> START_CENTERING_PID : SUCCEED
    SETUP_RED_LINE_STATE_REPUBLISHER --> CLEANUP_PROCESSES : ABORT
    START_CENTERING_PID --> PERFORM_CENTERING : SUCCEED
    START_CENTERING_PID --> CLEANUP_PROCESSES : ABORT
    PERFORM_CENTERING --> CLEANUP_PROCESSES : SUCCEED/ABORT
    CLEANUP_PROCESSES --> [*] : SUCCEED
    
    note right of START_RED_LINE_DETECTION
        StartRedLineDetection
        Inicia line_detection_node para cor vermelha
    end note
    
    note right of SETUP_RED_LINE_STATE_REPUBLISHER
        SetupRedLineStateRepublisher
        Configura publicadores para PID de centralização
    end note
    
    note right of START_CENTERING_PID
        StartCenteringPID
        Inicia PID para centralização na mangueira
    end note
    
    note right of PERFORM_CENTERING
        PerformCentering
        Centraliza drone sobre a mangueira vermelha
    end note
    
    note right of CLEANUP_PROCESSES
        CleanupProcesses
        Finaliza PID e processos de detecção
    end note
```

## Estrutura do Diretório

-   [`hook/`](./hook/): Contém o código Python principal do pacote.
    -   [`hook/states/`](./hook/hook/states/): Define a máquina de estados principal (`hang_the_hook_sm.py`) e as classes de estado individuais.
        -   [`hook/states/line_following/`](./hook/hook/states/line_following/): Estados relacionados ao seguimento da linha azul.
        -   [`hook/states/hook_operations/`](./hook/hook/states/hook_operations/): Estados relacionados à centralização, descida e liberação do gancho.
        -   [`basic_states.py`](./hook/hook/states/basic_states.py): Estados básicos como Initialize, Takeoff, RTL, End.
        -   [`constants.py`](./hook/hook/states/constants.py): Constantes usadas nos estados (altitudes, velocidades, parâmetros PID, etc.).
-   [`package.xml`](./hook/package.xml): Metadados do pacote ROS 2, incluindo dependências.
-   [`setup.py`](./hook/setup.py): Script de build para pacotes Python ament.

## Pré-requisitos

-   ROS 2 (Recomendado: Humble Hawksbill)
-   [YASMIN](https://github.com/uleroboticsgroup/yasmin) (Biblioteca de Máquina de Estados)
-   [Mirela SDK](https://github.com/Black-Bee-Drones/mirela-sdk) (Biblioteca interna de controle e visão)
-   [PID Controller](https://github.com/Black-Bee-Drones/pid-controller) (Biblioteca interna de controle PID)

## Instalação

1.  **Instalar YASMIN:**
    ```bash
    sudo apt update
    sudo apt install ros-$ROS_DISTRO-yasmin ros-$ROS_DISTRO-yasmin-*
    ```
    *(Consulte a [documentação oficial do YASMIN](https://github.com/uleroboticsgroup/yasmin?tab=readme-ov-file#installation) para mais detalhes)*

2.  **Clonar o Repositório:**
    Navegue até o diretório `src` do seu workspace ROS 2 (ex: `~/ros2_ws/src`) e clone este repositório:
    ```bash
    git clone https://github.com/Black-Bee-Drones/SAE-Eletroquad.git
    ```

3.  **Instalar Dependências:**
    Navegue até a raiz do seu workspace (ex: `~/ros2_ws`) e instale as dependências listadas nos `package.xml` dos pacotes:
    ```bash
    cd ~/ros2_ws # Ou o diretório raiz do seu workspace
    rosdep install --from-paths src --ignore-src -r -y
    ```

## Build

Navegue até a raiz do seu workspace e compile o pacote `hook`:

```bash
cd ~/ros2_ws # Ou o diretório raiz do seu workspace
colcon build --packages-select hook
```

## Execução

1.  **Abra um novo terminal.**
2.  **Faça o source do setup do seu workspace:**
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```
3.  **Execute a máquina de estados principal:**
    ```bash
    ros2 run hook mangalarga
    ```

4.  **(Opcional) Visualizar a Máquina de Estados:**
    Enquanto a máquina de estados está rodando, você pode visualizá-la em tempo real usando o `yasmin-viewer`. Abra outro terminal, faça o source novamente e execute:
    ```bash
    ros2 run yasmin_viewer yasmin_viewer_node
    ```
    Acesse `http://localhost:5000` (ou o endereço/porta configurado) no seu navegador.
