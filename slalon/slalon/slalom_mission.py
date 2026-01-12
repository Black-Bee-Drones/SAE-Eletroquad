#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from enum import Enum, auto
import time
from std_msgs.msg import Float32, Int8
from mirela_sdk.control.mavros.mavros_api import MavDrone

# Constantes para a localização do objeto na imagem
NOWHERE = 0
CENTER = 1
LEFT = 2
RIGHT = 3


class SlalomState(Enum):
    """Estados da missão de slalom"""

    INIT = auto()  # Inicialização
    TAKEOFF = auto()  # Decolagem
    SEARCH = auto()  # Procurando o cano
    CENTER = auto()  # Centralizando com o cano
    APPROACH = auto()  # Aproximando do cano
    PASS = auto()  # Passando pelo lado do cano
    NEXT_PIPE = auto()  # Preparando para o próximo cano
    LAND = auto()  # Pousando
    FINISH = auto()  # Missão finalizada


class SlalomMission(Node):
    """
    Nó para executar a missão de slalom da competição SAE EletroQuad.

    O drone deve passar alternadamente pelos lados de 4 traves de cores diferentes,
    mantendo altura abaixo de 2.5m, e então pousar.
    """

    def __init__(self):
        super().__init__("slalom_mission")

        # Inicializa o drone
        self.drone = MavDrone(self, False)

        # Estado atual da máquina de estados
        self.state = SlalomState.INIT

        # Lado pelo qual o drone deve passar (começa com LEFT, mas será definido no briefing)
        self.next_side = LEFT  # Atualizar conforme informado no dia da competição

        # Contagem de canos
        self.pipe_count = 0
        self.max_pipes = 4

        # Valores de controle
        self.pipe_location = NOWHERE  # Posição do cano (LEFT, RIGHT, CENTER, NOWHERE)
        self.distance_to_pipe = 0.0  # Distância estimada até o cano
        self.color_changed = False  # Flag para troca de cor

        # Timeout e controle de tempo
        self.state_start_time = 0.0
        self.timeout_duration = 10.0  # Timeout padrão (segundos)

        # Publishers e Subscribers
        self.depth_st_pub = self.create_publisher(Int8, "switch_state", 10)
        self.depth_sub = self.create_subscription(
            Float32, "depth_topic", self.distance_callback, 10
        )
        self.find_sub = self.create_subscription(
            Int8, "where_is_it", self.find_object_callback, 10
        )
        self.changed_color_sub = self.create_subscription(
            Int8, "color_changed", self.color_changed_callback, 10
        )

        # Timer para a máquina de estados (executa a 10Hz)
        self.create_timer(0.1, self.state_machine_callback)

        # Log de inicialização
        self.get_logger().info("Missão Slalom inicializada!")

    def distance_callback(self, msg):
        """Callback para receber a distância estimada até o cano."""
        self.distance_to_pipe = msg.data

    def find_object_callback(self, msg):
        """Callback para receber a posição do cano na imagem."""
        self.pipe_location = msg.data

    def color_changed_callback(self, msg):
        """Callback para saber quando a troca de cor foi concluída."""
        if msg.data == 1:
            self.color_changed = True
            self.get_logger().info("Cor alterada com sucesso!")

    def state_machine_callback(self):
        """Callback principal da máquina de estados."""
        # Executar o estado atual
        if self.state == SlalomState.INIT:
            self.execute_init()

        elif self.state == SlalomState.TAKEOFF:
            self.execute_takeoff()

        elif self.state == SlalomState.SEARCH:
            self.execute_search()

        elif self.state == SlalomState.CENTER:
            self.execute_center()

        elif self.state == SlalomState.APPROACH:
            self.execute_approach()

        elif self.state == SlalomState.PASS:
            self.execute_pass()

        elif self.state == SlalomState.NEXT_PIPE:
            self.execute_next_pipe()

        elif self.state == SlalomState.LAND:
            self.execute_land()

        elif self.state == SlalomState.FINISH:
            # Nada a fazer, missão completa
            pass

    def change_state(self, new_state):
        """
        Muda para um novo estado e reinicia o temporizador de timeout.

        Args:
            new_state: O novo estado para transicionar
        """
        self.get_logger().info(f"Mudando estado: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.state_start_time = time.time()

    def check_timeout(self):
        """
        Verifica se o estado atual ultrapassou o tempo limite.

        Returns:
            bool: True se o timeout foi atingido, False caso contrário
        """
        return (time.time() - self.state_start_time) > self.timeout_duration

    # Implementação dos estados

    def execute_init(self):
        """Estado inicial, prepara a missão e vai para decolagem."""
        self.get_logger().info("Iniciando missão Slalom")
        self.change_state(SlalomState.TAKEOFF)

    def execute_takeoff(self):
        """Decola e passa para o estado de busca."""
        self.get_logger().info("Decolando para altura de 1.5m")
        self.drone.arm_takeoff(1.5)
        # Aguarda 8 segundos para estabilizar após a decolagem
        time.sleep(8)
        self.change_state(SlalomState.SEARCH)

    def execute_search(self):
        """
        Procura pelo cano atual. Usa a detecção de cor para localizar o cano.
        Se não encontrar após alguns movimentos, continua procurando em padrão expandido.
        """
        self.get_logger().info(f"Procurando cano {self.pipe_count + 1}/4")

        # Se já encontrou o cano, prossegue para centralização
        if self.pipe_location != NOWHERE:
            self.get_logger().info(f"Cano encontrado na posição: {self.pipe_location}")
            self.change_state(SlalomState.CENTER)
            return

        # Timeout para busca (se demorar muito, tenta uma estratégia diferente)
        if self.check_timeout():
            self.get_logger().warn(
                "Timeout na busca. Tentando padrão de busca expandido."
            )
            self.execute_expanded_search()
            return

        # Estratégia de busca: rotação em padrão de varredura
        # Alternar direção de rotação a cada 5 segundos
        elapsed_time = time.time() - self.state_start_time
        rotation_period = 5.0  # segundos para cada direção

        # Determinar direção com base no tempo decorrido
        if (int(elapsed_time / rotation_period) % 2) == 0:
            # Rotação no sentido horário
            self.drone.offboard_velocity(0.0, 0.0, 0.0, 0.3)
            self.get_logger().info("Buscando: Rotação horária")
        else:
            # Rotação no sentido anti-horário
            self.drone.offboard_velocity(0.0, 0.0, 0.0, -0.3)
            self.get_logger().info("Buscando: Rotação anti-horária")

    def execute_expanded_search(self):
        """
        Implementa um padrão de busca expandido quando o cano não é encontrado facilmente.
        Move-se em um padrão de espiral para fora, procurando o cano em uma área maior.
        """
        self.get_logger().info("Iniciando padrão de busca expandido")

        # Sequência de movimentos em padrão espiral
        search_pattern = [
            # (velocidade_x, velocidade_y, tempo, descrição)
            (0.3, 0.0, 2.0, "avançando"),
            (0.0, 0.3, 2.0, "movendo para esquerda"),
            (0.0, -0.6, 3.0, "movendo para direita"),
            (0.0, 0.3, 2.0, "retornando ao centro"),
            (-0.3, 0.0, 1.0, "recuando"),
            (0.0, 0.0, 3.0, "girando 360°"),
        ]

        # Executar cada movimento do padrão
        for vx, vy, t, desc in search_pattern:
            # Se for movimento de rotação (último da lista)
            if vx == 0.0 and vy == 0.0:
                self.get_logger().info(f"Busca expandida: {desc}")
                # Faz uma rotação completa
                self.drone.offboard_velocity_timer(0.0, 0.0, 0.0, 0.3, time=t)
            else:
                self.get_logger().info(f"Busca expandida: {desc}")
                self.drone.offboard_velocity_timer(vx, vy, 0.0, 0.0, time=t)

            # Verifica se encontrou o cano após este movimento
            if self.pipe_location != NOWHERE:
                self.get_logger().info(
                    f"Cano encontrado durante busca expandida: {self.pipe_location}"
                )
                self.change_state(SlalomState.CENTER)
                return

        # Se após todos os movimentos ainda não encontrou, reinicia o timer para busca normal
        self.get_logger().info(
            "Busca expandida concluída sem sucesso, voltando para busca normal"
        )
        self.state_start_time = time.time()  # Reinicia o timer

    def execute_center(self):
        """
        Centraliza o drone em relação ao cano.
        Objetivo: mover o drone até que o cano apareça no centro da imagem.
        """
        self.get_logger().info(
            f"Centralizando com o cano (posição atual: {self.pipe_location})"
        )

        # Se perder o cano de vista, voltar para busca
        if self.pipe_location == NOWHERE:
            self.get_logger().warn(
                "Cano perdido durante centralização. Voltando para busca."
            )
            self.change_state(SlalomState.SEARCH)
            return

        # Se estiver centralizado, ir para aproximação
        if self.pipe_location == CENTER:
            self.get_logger().info("Cano centralizado!")
            self.change_state(SlalomState.APPROACH)
            return

        # Se o timeout foi atingido, tenta novamente a busca
        if self.check_timeout():
            self.get_logger().warn("Timeout na centralização. Voltando para busca.")
            self.change_state(SlalomState.SEARCH)
            return

        # Movimentação para centralizar
        if self.pipe_location == LEFT:
            # Cano está à esquerda, mover-se para esquerda para centralizar
            self.drone.offboard_velocity(0.0, 0.2, 0.0, 0.0)
        elif self.pipe_location == RIGHT:
            # Cano está à direita, mover-se para direita para centralizar
            self.drone.offboard_velocity(0.0, -0.2, 0.0, 0.0)

    def execute_approach(self):
        """
        Aproxima-se do cano até atingir a distância desejada.
        """
        # Distância alvo para preparar a passagem
        target_distance = 150.0  # ajustar conforme necessário

        self.get_logger().info(
            f"Aproximando do cano. Distância atual: {self.distance_to_pipe:.2f}"
        )

        # Se perder o cano de vista, voltar para busca
        if self.pipe_location == NOWHERE:
            self.get_logger().warn(
                "Cano perdido durante aproximação. Voltando para busca."
            )
            self.change_state(SlalomState.SEARCH)
            return

        # Se estiver próximo o suficiente, ir para passagem
        if 0 < self.distance_to_pipe <= target_distance:
            self.get_logger().info(
                f"Distância de {self.distance_to_pipe:.2f} atingida. Preparando passagem."
            )
            self.change_state(SlalomState.PASS)
            return

        # Se o timeout foi atingido, força a passagem
        if self.check_timeout():
            self.get_logger().warn("Timeout na aproximação. Forçando passagem.")
            self.change_state(SlalomState.PASS)
            return

        # Velocidade de aproximação proporcional à distância
        # Mais lento quando próximo, mais rápido quando longe
        speed = max(1, self.distance_to_pipe * 0.2)
        self.drone.offboard_velocity(speed, 0.0, 0.0, 0.0)

    def execute_pass(self):
        """
        Passa pelo lado do cano conforme o lado definido.
        Alterna o lado de passagem para o próximo cano.
        """
        self.get_logger().info(
            f"Passando pelo cano {self.pipe_count + 1}/4 pelo lado {'ESQUERDO' if self.next_side == LEFT else 'DIREITO'}"
        )

        # Velocidade lateral para passar pelo lado correto
        lateral_speed = 0.5
        forward_speed = 0.4

        # Tempo de movimento lateral (ajustar conforme necessário)
        lateral_time = 3.0

        if self.next_side == LEFT:
            # Passar pelo lado esquerdo do cano
            self.drone.offboard_velocity_timer(
                0.0, lateral_speed, 0.0, 0.0, time=lateral_time
            )
        else:
            # Passar pelo lado direito do cano
            self.drone.offboard_velocity_timer(
                0.0, -lateral_speed, 0.0, 0.0, time=lateral_time
            )

        # Avançar para passar completamente pelo cano
        forward_time = self.calculate_forward_time()
        self.drone.offboard_velocity_timer(
            forward_speed, 0.0, 0.0, 0.0, time=forward_time
        )

        # Incrementa a contagem de canos e alterna o lado de passagem
        self.pipe_count += 1
        self.next_side = RIGHT if self.next_side == LEFT else LEFT

        # Verifica se completou todos os canos
        if self.pipe_count >= self.max_pipes:
            self.get_logger().info("Todos os canos passados. Preparando para pouso.")
            self.change_state(SlalomState.LAND)
        else:
            self.get_logger().info(
                f"Preparando para o próximo cano ({self.pipe_count + 1}/4)"
            )
            self.change_state(SlalomState.NEXT_PIPE)

    def calculate_forward_time(self):
        """
        Calcula o tempo necessário para avançar após passar pelo cano,
        com base na distância medida.
        """
        # Ajustar esses valores conforme necessário
        if self.distance_to_pipe < 100:
            return 1.5
        elif self.distance_to_pipe < 200:
            return 3.0
        else:
            return 3.5

    def execute_next_pipe(self):
        """
        Prepara para o próximo cano, trocando o filtro de cor.
        """
        self.get_logger().info("Trocando filtro de cor para o próximo cano")

        # Solicita a troca de filtro de cor
        msg = Int8()
        msg.data = 1  # Sinal para trocar a cor
        self.depth_st_pub.publish(msg)

        # Reseta a flag de cor alterada
        self.color_changed = False

        # Espera pela confirmação de troca de cor
        start_time = time.time()
        while not self.color_changed:
            if time.time() - start_time > 5.0:  # Timeout de 5s
                self.get_logger().warn(
                    "Timeout na troca de cor. Prosseguindo mesmo assim."
                )
                break
            time.sleep(0.1)

        # Reinicia a busca pelo próximo cano
        self.change_state(SlalomState.SEARCH)

    def execute_land(self):
        """
        Realiza o pouso do drone e finaliza a missão.
        """
        self.get_logger().info("Iniciando procedimento de pouso")
        self.drone.land()
        time.sleep(5)  # Aguarda o pouso completar
        self.get_logger().info("Missão de Slalom concluída com sucesso!")
        self.change_state(SlalomState.FINISH)


def main():
    rclpy.init()
    mission = SlalomMission()

    try:
        rclpy.spin(mission)
    except KeyboardInterrupt:
        mission.get_logger().info("Missão interrompida pelo usuário")
    except Exception as e:
        mission.get_logger().error(f"Erro durante a missão: {e}")
    finally:
        # Garantir que o drone pouse em caso de erro
        if mission.state != SlalomState.FINISH:
            mission.get_logger().warn("Pousando em procedimento de emergência")
            mission.drone.land()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
