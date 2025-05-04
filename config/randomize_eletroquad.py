import yaml
import random
import math
import sys
import subprocess

# Template YAML original (não altere os campos que não devem ser modificados)
yaml_template = """
world_name: default
origin:
  latitude: -22.8627458
  longitude: -43.2297722
  altitude: 2
drones:
- model_type: x500_px4
  model_name: x500_px4
  flight_time: 60
  xyz:
  - 0.2
  - 0.0
  - 0.5
  payload:
  - model_type: gps
    model_name: gps
  - model_type: hd_camera
    model_name: camera
    xyz:
      - 0.04
      - 0.0
      - -0.1
    rpy:
      - 0.0
      - 0.0
      - 0.0
  - model_type: gancho
    model_name: gancho
    xyz:
      - 0.0
      - 0.0
      - -0.05
objects:
- model_name: eletroquad_arena
  model_type: eletroquad_arena
  xyz:
  - 0.0
  - 0.0
  - 0.1
  rpy:
  - 0.0
  - 0.0
  - 0.0
- model_name: base_inicial_slalom
  model_type: circulo_azul
  xyz:
  - -12.5
  - -10
  - 0.11
- model_name: barra_1
  model_type: barra_rosa
  object_bridges:
    - "pose"
  xyz:
  - 1.05
  - -3.14
  - 0.1
- model_name: barra_2
  model_type: barra_azul
  object_bridges:
    - "pose"
  xyz:
  - 2.77
  - -1.35
  - 0.1
- model_name: barra_3
  model_type: barra_preto_fosco
  object_bridges:
    - "pose"
  xyz:
  - 3.54
  - -0.8
  - 0.1
- model_name: barra_4
  model_type: barra_vermelha
  object_bridges:
    - "pose"
  xyz:
  - 4.97
  - 1.39
  - 0.1
- model_name: marker_1
  model_type: circulo_azul
  xyz:
  - -4.13
  - 1.76
  - 0.11
- model_name: marker_2
  model_type: cruz_rosa
  xyz:
  - -1.03
  - -2.02
  - 0.11
- model_name: marker_3
  model_type: estrela_verde
  xyz:
  - 2.0
  - 3.94
  - 0.11
- model_name: marker_4
  model_type: hexagono_vermelho
  xyz:
  - -3.52
  - -2.43
  - 0.11
- model_name: marker_5
  model_type: pentagono_marrom
  xyz:
  - -1.1
  - -1.37
  - 0.11
- model_name: marker_6
  model_type: triangulo_azul
  xyz:
  - -2.1
  - 0.22
  - 0.11
- model_name: marker_7
  model_type: casa_laranja
  xyz:
  - -2.1
  - 0.22
  - 0.11
- model_name: marker_8
  model_type: triangulo_azul
  xyz:
  - -2.1
  - 0.22
  - 0.11
- model_name: base_inicial_hangthehook
  model_type: circulo_azul
  xyz:
  - -12.5
  - -12.5
  - 0.11
- model_name: mangueira_e_sustentacao
  model_type: truss_with_hose
  xyz:
  - -12.5
  - -12.5
  - 0.12
- model_name: cubo_suporte
  model_type: cubo_suporte
  xyz:
  - -12.5
  - -12.5
  - 0.12
- model_name: gancho
  model_type: gancho
  xyz:
  - -12.5
  - -12.5
  - 0.2
- model_name: fita
  model_type: fita
  xyz:
  - 0.0
  - 0.0
  - 0.11
"""

# Carrega o template YAML
data = yaml.safe_load(yaml_template)

# Lê a missão (tarefa) a partir do argumento de linha de comando, se fornecido
if len(sys.argv) > 1:
    try:
        tarefa = int(sys.argv[1])
        if tarefa not in [1, 2, 3]:
            raise ValueError
    except ValueError:
        print("Argumento de missão inválido. Use 1, 2 ou 3.")
        sys.exit(1)
else:
    tarefa = int(input("Qual tarefa (1, 2 ou 3)? ").strip())

print("TAREFA:", tarefa)

# Separa os objetos em grupos conforme suas regras
barras = []
markers = []
hanghook = None
mangueira = None
base_slalom = None
gancho = None
cubo_suporte = None

for obj in data["objects"]:
    nome = obj["model_name"]
    if nome.startswith("barra"):
        barras.append(obj)
    elif nome.startswith("marker"):
        markers.append(obj)
    elif nome == "base_inicial_hangthehook":
        hanghook = obj
    elif nome == "mangueira_e_sustentacao":
        mangueira = obj
    elif nome == "base_inicial_slalom":
        base_slalom = obj
    elif nome == "gancho":
        gancho = obj
    elif nome == "cubo_suporte":
        cubo_suporte = obj

# Atualiza posições das barras
def generate_bar_positions(n_barras, x_min=-10, x_max=12, min_dist=3):
    positions = []
    while len(positions) < n_barras:
        candidate = random.uniform(x_min, x_max)
        if all(abs(candidate - x) >= min_dist for x in positions):
            positions.append(candidate)
    return sorted(positions)

n_barras = len(barras)
x_positions = generate_bar_positions(n_barras)
for i, barra in enumerate(barras):
    barra["xyz"][0] = round(x_positions[i], 2)
    barra["xyz"][1] = round(random.uniform(-13, -7), 2)

# Atualiza posições dos markers
def is_valid_marker(pos, pos_list, min_dist=2):
    for p in pos_list:
        if math.sqrt((pos[0] - p[0])**2 + (pos[1] - p[1])**2) < min_dist:
            return False
    return True

novas_posicoes_markers = []
for marker in markers:
    for _ in range(100):
        x = random.uniform(-13, -2)
        y = random.uniform(-3, 13)
        if is_valid_marker((x, y), novas_posicoes_markers):
            novas_posicoes_markers.append((x, y))
            marker["xyz"][0] = round(x, 2)
            marker["xyz"][1] = round(y, 2)
            break

# Remove aleatoriamente 0, 1 ou 2 markers do YAML
remover = random.choice([0, 1, 2])
if remover > 0 and len(markers) > remover:
    markers_remover = random.sample(markers, remover)
    for m in markers_remover:
        data["objects"].remove(m)
    markers = [m for m in markers if m not in markers_remover]

# Atualiza as posições de base_inicial_hangthehook e mangueira_e_sustentacao
def posicao_aleatoria():
    return (random.uniform(2, 13), random.uniform(-3, 13))

if hanghook is not None and mangueira is not None:
    for _ in range(100):
        pos1 = posicao_aleatoria()
        pos2 = posicao_aleatoria()
        if math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2) >= 9:
            hanghook["xyz"][0] = round(pos1[0], 2)
            hanghook["xyz"][1] = round(pos1[1], 2)
            mangueira["xyz"][0] = round(pos2[0], 2)
            mangueira["xyz"][1] = round(pos2[1], 2)
            break

# Atualiza a posição do drone conforme a tarefa selecionada
drone = data["drones"][0]
if tarefa == 1:
    drone["xyz"][0] = -12.5
    drone["xyz"][1] = -10
elif tarefa == 2:
    if hanghook is not None:
        drone["xyz"][0] = hanghook["xyz"][0]
        drone["xyz"][1] = hanghook["xyz"][1]
elif tarefa == 3:
    if markers:
        escolhido = random.choice(markers)
        drone["xyz"][0] = escolhido["xyz"][0]
        drone["xyz"][1] = escolhido["xyz"][1]
else:
    print("Tarefa inválida. Escolha 1, 2 ou 3.")
    sys.exit(1)

# Armazena posições iniciais antes de modificações
initial_drone_pos = list(data["drones"][0]["xyz"])
initial_mangueira_pos = []
for obj in data["objects"]:
    if obj.get("model_name") == "mangueira_e_sustentacao":
        initial_mangueira_pos = list(obj["xyz"])
        break
        
# Modifica dinamicamente as posições de gancho e cubo_suporte
if not tarefa == 2:
  if gancho is not None:
      data["objects"].remove(gancho)
  if cubo_suporte is not None:
      data["objects"].remove(cubo_suporte)

# Chamada de randomize_curva e inclusão/remoção da fita
if tarefa == 2:
    # Parâmetros para randomize_curva usando posições iniciais
    x0, y0 = initial_drone_pos[0], initial_drone_pos[1]
    yaw0 = 0.0  # ajustar se tiver valor de yaw disponível
    x1, y1 = initial_mangueira_pos[0], initial_mangueira_pos[1]
    yaw1 = 0.0  # ajustar se tiver valor de yaw disponível

    # Gera a curva suave entre drone e truss_with_hose
    print(str(x0), str(y0), str(yaw0), str(x1), str(y1), str(yaw1))
    subprocess.run([
        sys.executable,
        "randomize_curva.py",
        "--x0", str(x0+0.4),
        "--y0", str(y0),
        "--yaw0", str(yaw0),
        "--x1", str(x1),
        "--y1", str(y1),
        "--yaw1", str(yaw1-1.5707)
    ], check=True)

    # Mantém a fita no ambiente (padrão no template)
else:
    # Remove a fita para missões diferentes de 2
    data["objects"] = [obj for obj in data["objects"] if obj.get("model_name") != "fita"]

# Atualiza o campo model_type para ghost_marker conforme a missão
if tarefa == 1:
    for marker in markers:
        marker["model_type"] = "ghost_marker"
    if hanghook is not None:
        hanghook["model_type"] = "ghost_marker"
elif tarefa == 2:
    for marker in markers:
        marker["model_type"] = "ghost_marker"
    if base_slalom is not None:
        base_slalom["model_type"] = "ghost_marker"
elif tarefa == 3:
    if base_slalom is not None:
        base_slalom["model_type"] = "ghost_marker"
    if hanghook is not None:
        hanghook["model_type"] = "ghost_marker"

# Grava o novo YAML no arquivo "world_eletroquad.yaml"
with open("world_eletroquad.yaml", "w") as f:
    yaml.dump(data, f, default_flow_style=False)

print("Arquivo 'world_eletroquad.yaml' gerado com sucesso!")