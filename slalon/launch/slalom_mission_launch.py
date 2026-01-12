from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch description para a missão de Slalom.

    Inicia os seguintes nós:
    - depth_st: Responsável por gerenciar os filtros de cor
    - slalom_mission: Implementa a máquina de estados para a missão
    """
    return LaunchDescription(
        [
            # Nó de detecção de cor e distância
            Node(
                package="slalon",
                executable="depth_st",
                name="depth_st",
                output="screen",
                parameters=[
                    {"cap": 0}
                ],  # Ajuste o índice da câmera conforme necessário
            ),
            # Nó principal da missão slalom (nova implementação)
            Node(
                package="slalon",
                executable="slalom_mission",
                name="slalom_mission",
                output="screen",
            ),
        ]
    )
