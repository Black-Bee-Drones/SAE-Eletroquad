# Pacote Slalom - Missão 1 (M1): "Slalom"
    Descreve a lógica utilizada para realização da missão 1 (Slalom) na competição SAE Aerodesign Eletroquad.

## Descrição da missão
    A missão Slalom consiste em 4 em canos de pvc erguidos verticalmente ao solo e dispostos um atrás do outro de maneira não alinhada.
    O objetivo da missão é fazer com que o drone levante voo, passe por todos os canos, alternando entre passar à direita e à esquerda a cada cano concluído, e pouse seguramente após passar pelo último. 
    Os canos de pvc são de diferentes cores, sendo elas: rosa, vermelho, azul e preto. A ordem que essas cores estão dispostas é fornecida apenas no dia da competição, assim como o lado pelo qual o drone deve passar pelo primeiro cano (direita ou esquerda).

## Implementação
    Para realizar esta missão, foram utilizadas 3 classes:
    DepthMeasurement: que identifica o cano através de um filtro de cor e calcula a distância entre o drone e o cano.
    Depth_St: máquina de estados que define a cor que está sendo filtrada.
    Movement: dita todas as movimentações do drone para facilitar identificação dos canos, aproximar-se, centralizar-se e passar por eles. Define uma máquina de estados para coordenar o que o drone deve fazer em cada estágio da missão.

    Máquina de estado de Depth_St:
    '''mermaid
    ---
    config:
    layout: dagre
    look: classic
    ---
    stateDiagram
    direction TB
    [*] --> START
    START --> BLACK
    BLACK --> PINK
    PINK --> RED
    RED --> BLUE
    BLUE --> END
    END --> [*]
    '''

    Máquina de estados de Movement:
    '''mermaid
    ---
    config:
    look: classic
    layout: dagre
    ---
    %%Movement State Machine
    stateDiagram
    direction TB
    [*] --> STATE_0
    STATE_0 --> STATE_1
    STATE_1 --> STATE_2
    STATE_2 --> STATE_3
    STATE_3 --> STATE_4
    STATE_4 --> STATE_5:Passed by all 4 pipes
    STATE_4 --> STATE_1:Repeats for next pipe
    STATE_5 --> [*]
    note left of STATE_0 : ARM_TAKEOFF
    note left of STATE_1 
    Yaws the drone left and right until it finds the pipe.
                If not found, the drone goes foward, left and then right repeating the yaws.
    end note
    note right of STATE_2 : Moves drone to where the pipe was found (left or right) until it becomes aligned with the pipe.
    note right of STATE_3 : Moves drone foward until it reaches a certain distance from the pipe.
    note left of STATE_4 : Moves drone to the side it is supposed to pass by the pipe, then moves it foward, counting the succeed pipe and leaving it behind.
    note right of STATE_5 : LAND
    '''
