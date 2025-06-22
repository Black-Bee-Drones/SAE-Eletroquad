import rclpy

import yasmin
from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub
import atexit

from hook.states import (
    Initialize,
    Takeoff,
    SearchBlueLine,
    FollowBlueLineWithRedDetection,
    CenterRedBlob,
    PerformDescent,
    ReleaseHook,
    ReturnToLaunch,
    End,
)

from mirela_sdk.utils.process import ProcessUtils
from hook.constants import (
    LINE_DETECT_NODE_NAME,
    RED_DETECT_NODE_NAME,
    CENTER_PID_PROCESS,
    ANGLE_PID_PROCESS,
    CENTERING_PID_PROCESS,
)


class HangTheHookSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "INITIALIZE",
            Initialize(),
            transitions={SUCCEED: "TAKEOFF", ABORT: "END"},
        )

        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={SUCCEED: "SEARCH_BLUE_LINE", ABORT: "RETURN_TO_LAUNCH"},
        )

        self.add_state(
            "SEARCH_BLUE_LINE",
            SearchBlueLine(),
            transitions={SUCCEED: "CENTER_RED_BLOB", ABORT: "RETURN_TO_LAUNCH"},
        )

        self.add_state(
            "FOLLOW_BLUE_LINE",
            FollowBlueLineWithRedDetection(),
            transitions={
                SUCCEED: "CENTER_RED_BLOB",
                ABORT: "RETURN_TO_LAUNCH",
            },
        )

        self.add_state(
            "CENTER_RED_BLOB",
            CenterRedBlob(),
            transitions={SUCCEED: "DESCEND_TO_HOOK", ABORT: "RETURN_TO_LAUNCH"},
        )

        self.add_state(
            "DESCEND_TO_HOOK",
            PerformDescent(),
            transitions={SUCCEED: "RELEASE_HOOK", ABORT: "RETURN_TO_LAUNCH"},
        )

        self.add_state(
            "RELEASE_HOOK",
            ReleaseHook(),
            transitions={
                SUCCEED: "RETURN_TO_LAUNCH",
                ABORT: "RETURN_TO_LAUNCH",
            },
        )

        self.add_state(
            "RETURN_TO_LAUNCH",
            ReturnToLaunch(rtl_strategy="gps_return"),
            transitions={SUCCEED: "END", ABORT: "END"},
        )

        self.add_state("END", End(), transitions={SUCCEED: SUCCEED})

        self.cleanup_all_processes()

    def __del__(self):
        """Destrutor da classe - chama o cleanup quando a instância é destruída."""
        self.cleanup_all_processes()

    def cleanup_all_processes(self):
        """Método para encerrar todos os subprocessos abertos pelos subestados."""
        yasmin.YASMIN_LOG_INFO("Limpando todos os processos da máquina de estados...")

        processes_to_kill = [
            LINE_DETECT_NODE_NAME,
            RED_DETECT_NODE_NAME,
            CENTER_PID_PROCESS,
            ANGLE_PID_PROCESS,
            CENTERING_PID_PROCESS,
        ]

        for process in processes_to_kill:
            try:
                if ProcessUtils.kill_process(process):
                    yasmin.YASMIN_LOG_INFO(f"Processo {process} encerrado com sucesso.")
                else:
                    yasmin.YASMIN_LOG_INFO(
                        f"Processo {process} não estava em execução."
                    )
            except Exception as e:
                yasmin.YASMIN_LOG_ERROR(f"Erro ao encerrar o processo {process}: {e}")

        # Executa tmux kill-server para garantir que todas as sessões tmux sejam encerradas
        try:
            import subprocess

            yasmin.YASMIN_LOG_INFO(
                "Executando tmux kill-server para garantir encerramento completo..."
            )
            subprocess.run(["tmux", "kill-server"], check=False)
            yasmin.YASMIN_LOG_INFO("Comando tmux kill-server executado com sucesso.")
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Erro ao executar tmux kill-server: {e}")

        yasmin.YASMIN_LOG_INFO("Limpeza de processos concluída.")


def main(args=None):
    yasmin.YASMIN_LOG_INFO("HANG THE HOOK STATE MACHINE")

    rclpy.init(args=args)

    set_ros_loggers()

    mangalarga = HangTheHookSM()
    viewer = YasminViewerPub("mangalarga_state_machine", mangalarga)

    try:
        avante = mangalarga()
        print(avante)
    except Exception as e:
        yasmin.YASMIN_LOG_ERROR(f"Erro durante a execução da máquina de estados: {e}")
        # Certifique-se de que a limpeza seja feita mesmo se ocorrer uma exceção
        mangalarga.cleanup_all_processes()
    finally:
        # Limpeza final antes de encerrar
        rclpy.shutdown()


if __name__ == "__main__":
    main()
