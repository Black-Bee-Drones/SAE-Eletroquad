# Pacote `bouncing`

## Descrição

O pacote `bouncing` foi desenvolvido para executar a missão "Bouncing" da competição SAE BRASIL Eletrobras, conforme descrito no regulamento oficial. Ele integra módulos de visão computacional, controle e navegação para permitir que um drone autônomo identifique figuras geométricas específicas em uma área delimitada, navegue até elas, e pouse com todos os pés do trem de pouso dentro da figura.

## Funcionalidades Principais

- **Detecção de Figuras:** Utiliza modelos YOLO (You Only Look Once) para identificar figuras geométricas (ex: cruz, círculo, quadrado) em tempo real.
- **Geofencing:** Delimita a área de atuação do robô com base em coordenadas GPS fornecidas via argumentos de lançamento.
- **Controle PID:** Realiza o controle de navegação e centralização do robô em relação às figuras detectadas.
- **Calibração e Comandos de Pouso:** Inclui módulos para calibração da camera e execução de comandos de pouso.

## Estrutura do Projeto

```
bouncing/
├── bouncing/
│   ├── bouncing_node.py
│   ├── calibrate.py
│   ├── camera_node.py
│   ├── geofence.py
│   ├── land_command.py
│   └── ai/
│       └── yolo/
│           ├── inference_onnx.py
│           ├── yolo_inference_node.py
│           ├── yolov11n.onnx
│           └── YOLOv11p.onnx
└── launch/
    └── bouncing_launch.py

```

## Como Executar

1. **Instalação das Dependências**

   Certifique-se de ter o ROS 2 instalado e as dependências listadas no `setup.py` e `package.xml` resolvidas.

2. **Compilação**

   No diretório do workspace ROS 2:
   ```sh
   colcon build --packages-select bouncing --symlink-install
   source install/setup.bash
   ```

3. **Execução**

   Para iniciar a missão padrão:
   ```sh
   ros2 launch bouncing bouncing_launch.py
   ```

   Para interromper a missão, a qualquer instante, com um pouso vertical:
   ```sh
   ros2 run bouncing land_command_node
   ```

## Nós Principais

- `bouncing.bouncing_node`: Lógica principal da missão.
- `bouncing.camera_node`: Captura de imagens.
- `bouncing.ai.yolo.yolo_inference_node`: Inferência de figuras usando YOLO.
- `bouncing.geofence`: Delimitação da área de atuação.

## Modelos de IA

Os modelos `.onnx` utilizados para detecção estão em `bouncing/bouncing/ai/yolo` e `models/`.

## Referência

Para detalhes completos da missão, consulte o regulamento oficial.

---

**Autor:** lipedras  
**Licença:** Apache-2.0

---

> Este pacote foi desenvolvido especificamente para a missão "Bouncing" da SAE BRASIL Eletrobras. Para dúvidas técnicas, consulte os scripts em `bouncing/bouncing` e os arquivos de launch.