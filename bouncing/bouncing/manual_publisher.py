import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

class ManualPublisher(Node):
    def __init__(self):
        super().__init__('manual_publisher_node')
        
        self.pub_state = self.create_publisher(String, '/current_state', 10)
        self.pub_cmd = self.create_publisher(String, '/mission_cmd', 10)
        self.pub_status = self.create_publisher(String, '/movement_status', 10)
        self.pub_error = self.create_publisher(Float32MultiArray, '/figure_error', 10)

        self.get_logger().info("Manual Publisher Node started.")
        self.get_logger().info("Digite o nome do tópico e o conteúdo da mensagem.")
        self.menu_loop()

    def menu_loop(self):
        while rclpy.ok():
            print("\n--- PUBLICADOR MANUAL ---")
            print("1 - /current_state (String)")
            print("2 - /mission_cmd (String)")
            print("3 - /movement_status (String)")
            print("4 - /figure_error (Float32MultiArray)")
            print("0 - Sair")

            opcao = input("Selecione uma opção: ").strip()

            if opcao == '1':
                data = input("Digite o estado (ex: circle, square, none...): ")
                msg = String()
                msg.data = data
                self.pub_state.publish(msg)
                self.get_logger().info(f"[Publicado] /current_state: {data}")

            elif opcao == '2':
                data = input("Digite o comando de missão (ex: takeoff): ")
                msg = String()
                msg.data = data
                self.pub_cmd.publish(msg)
                self.get_logger().info(f"[Publicado] /mission_cmd: {data}")

            elif opcao == '3':
                data = input("Digite o status do movimento (ex: landed): ")
                msg = String()
                msg.data = data
                self.pub_status.publish(msg)
                self.get_logger().info(f"[Publicado] /movement_status: {data}")

            elif opcao == '4':
                dx = input("Digite dx (float): ")
                dy = input("Digite dy (float): ")
                try:
                    msg = Float32MultiArray()
                    msg.data = [float(dx), float(dy)]
                    self.pub_error.publish(msg)
                    self.get_logger().info(f"[Publicado] /figure_error: dx={dx}, dy={dy}")
                except ValueError:
                    self.get_logger().error("Erro: dx e dy devem ser números.")

            elif opcao == '0':
                self.get_logger().info("Encerrando o nó...")
                break

            else:
                print("Opção inválida.")

def main(args=None):
    rclpy.init(args=args)
    node = ManualPublisher()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
