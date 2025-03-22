import yaml
import random
import math
import sys

# Template YAML original (não altere os campos que não devem ser modificados)
yaml_template = """
world_name: eletroquad
origin:
  latitude: -22.8627458
  longitude: -43.2297722
  altitude: 2
drones:
- model_type: x500_with_suction
  model_name: drone0
  flight_time: 60
  xyz:
  - 0.2
  - 0.0
  - 0.3
  payload:
  - model_type: gps
    model_name: gps
  - model_type: gimbal_speed
    model_name: gimbal
    payload:
      model_type: hd_camera
      model_name: camera
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
  xyz:
  - 1.05
  - -3.14
  - 0.1
- model_name: barra_2
  model_type: barra_azul
  xyz:
  - 2.77
  - -1.35
  - 0.1
- model_name: barra_3
  model_type: barra_preto_fosco
  xyz:
  - 3.54
  - -0.8
  - 0.1
- model_name: barra_4
  model_type: barra_vermelha
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

print("TAREFA: ", tarefa)

# Separa os objetos em grupos conforme suas regras
barras = []
markers = []
hanghook = None
mangueira = None
base_slalom = None

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
    # objetos como eletroquad_arena não serão alterados

# Atualiza posições das barras
# Regras para barras:
# - Os valores de x devem ser sequenciais, entre -12 e 12, com espaçamento entre 2.5 e 5 metros
# - Os valores de y devem ser sorteados aleatoriamente entre -13 e -7

def generate_bar_positions(n_barras, x_min=-10, x_max=12, min_dist=3):
    positions = []
    while len(positions) < n_barras:
        candidate = random.uniform(x_min, x_max)
        if all(abs(candidate - x) >= min_dist for x in positions):
            positions.append(candidate)
    return sorted(positions)

n_barras = len(barras)
x_positions = generate_bar_positions(n_barras)

# Atualiza cada barra
for i, barra in enumerate(barras):
    barra["xyz"][0] = round(x_positions[i], 2)
    barra["xyz"][1] = round(random.uniform(-13, -7), 2)
    # z permanece inalterado

# Atualiza posições dos markers
# Regras para markers:
# - Cada marker deve ficar dentro de: -13 < x < -2 e -3 < y < 13
# - Distância mínima entre qualquer par de markers: 2 metros
def is_valid_marker(pos, pos_list, min_dist=2):
    for p in pos_list:
        if math.sqrt((pos[0] - p[0])**2 + (pos[1] - p[1])**2) < min_dist:
            return False
    return True

novas_posicoes_markers = []
for marker in markers:
    for _ in range(100):  # tenta 100 vezes encontrar uma posição válida
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
    # Atualiza a lista de markers restantes
    markers = [m for m in markers if m not in markers_remover]

# Atualiza as posições de base_inicial_hangthehook e mangueira_e_sustentacao
# Regras:
# - Novas posições: 2 < x < 13 e -3 < y < 13
# - Devem estar separados por pelo menos 9 metros
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
    exit(1)

# Atualiza o campo model_type para ghost_marker conforme a missão:
if tarefa == 1:
    # Missão 1: todos os markers e a base_inicial_hangthehook
    for marker in markers:
        marker["model_type"] = "ghost_marker"
    if hanghook is not None:
        hanghook["model_type"] = "ghost_marker"
elif tarefa == 2:
    # Missão 2: todos os markers e a base_inicial_slalom
    for marker in markers:
        marker["model_type"] = "ghost_marker"
    if base_slalom is not None:
        base_slalom["model_type"] = "ghost_marker"
elif tarefa == 3:
    # Missão 3: base_inicial_slalom e base_inicial_hangthehook
    if base_slalom is not None:
        base_slalom["model_type"] = "ghost_marker"
    if hanghook is not None:
        hanghook["model_type"] = "ghost_marker"

# Grava o novo YAML no arquivo "world_eletroquad.yaml"
with open("world_eletroquad.yaml", "w") as f:
    yaml.dump(data, f, default_flow_style=False)

print("Arquivo 'world_eletroquad.yaml' gerado com sucesso!")
