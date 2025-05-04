import yaml
import random
import math
import sys
import subprocess

# Template YAML original (não altere campos não modificados)
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
  - -6.5
  - 0.0
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
  - 0.0
  - 0.0
  - 0.11
- model_name: mangueira_e_sustentacao
  model_type: truss_with_hose
  xyz:
  - 0.0
  - 0.0
  - 0.12
  rpy:
  - 0.0
  - 0.0
  - 0.0
- model_name: fita
  model_type: fita
  xyz:
  - 0.0
  - 0.0
  - 0.1
"""

# Carrega o template YAML
data = yaml.safe_load(yaml_template)

# Lê a tarefa
tarefa = int(sys.argv[1]) if len(sys.argv)>1 else int(input("Qual tarefa (1=slalom, 2=hangthehook, 3=bouncing)? ").strip())
if tarefa not in [1,2,3]: sys.exit(1)

# Atualiza arena
fase_map={1:'slalom',2:'hangthehook',3:'bouncing'}
arena=next(o for o in data['objects'] if o['model_name']=='eletroquad_arena')
arena['model_name']=arena['model_type']=fase_map[tarefa]

# Indexa
barras=[o for o in data['objects'] if o['model_name'].startswith('barra')]
markers=[o for o in data['objects'] if o['model_name'].startswith('marker')]
hanghook=next((o for o in data['objects'] if o['model_name']=='base_inicial_hangthehook'),None)
mangueira=next((o for o in data['objects'] if o['model_name']=='mangueira_e_sustentacao'),None)
fita=next((o for o in data['objects'] if o['model_name']=='fita'),None)
drone=data['drones'][0]
if 'xyz' not in drone: drone['xyz']=[0.0,0.0,0.0]

# Auxiliares
def gen_bar_positions(n,x_min=-5.5,x_max=7.0,min_dist=1.5):
    pos=[];att=0
    while len(pos)<n and att<1000:
        c=random.uniform(x_min,x_max)
        if all(abs(c-p)>=min_dist for p in pos): pos.append(c)
        att+=1
    return sorted(round(x,2) for x in pos)
def valid(p,l): return all(math.hypot(p[0]-q[0],p[1]-q[1])>=1.0 for q in l)

# Aplica
if tarefa==1:
    # randomiza cores
    types=[b['model_type'] for b in barras]
    random.shuffle(types)
    for b,t in zip(barras,types): b['model_type']=t
    # posições
    xs=gen_bar_positions(len(barras))
    for i,b in enumerate(barras): b['xyz'][0],b['xyz'][1]=xs[i],round(random.uniform(-3.5,3.5),2)
    data['objects']=[o for o in data['objects'] if o['model_name']!='fita']
elif tarefa==3:
    # resto inalterado...
    placed=[]
    for m in markers:
        for _ in range(100):
            x,y=random.uniform(-3.6,3.6),random.uniform(-3.6,3.6)
            if valid((x,y),placed): placed.append((x,y));m['xyz'][0],m['xyz'][1]=round(x,2),round(y,2);break
    for m in random.sample(markers,random.choice([0,1,2])): data['objects'].remove(m)
    data['objects']=[o for o in data['objects'] if o['model_name']!='fita']
elif tarefa==2:
    yaw=random.uniform(0,2*math.pi)
    for _ in range(100):
        x,y=random.uniform(-3,3),random.uniform(-3,3)
        if hanghook and math.hypot(x-hanghook['xyz'][0],y-hanghook['xyz'][1])>=2.5:
            mangueira['xyz'][0],mangueira['xyz'][1]=round(x,2),round(y,2)
            mangueira['rpy'][2]=round(yaw,2);break
    x0,y0=drone['xyz'][0],drone['xyz'][1]
    x1,y1=mangueira['xyz'][0],mangueira['xyz'][1]
    subprocess.run([sys.executable,'randomize_curva.py','--x0',str(x0),'--y0',str(y0),'--yaw0','0.0','--x1',str(x1),'--y1',str(y1),'--yaw1',str(round(yaw-1.5707,2))],check=True)

# filtra
objs=[]
for o in data['objects']:
    n=o['model_name']
    if n.startswith('barra')and tarefa!=1:continue
    if n.startswith('marker')and tarefa!=3:continue
    if n=='base_inicial_slalom'and tarefa!=1:continue
    if n=='base_inicial_hangthehook'and tarefa!=2:continue
    if n=='mangueira_e_sustentacao'and tarefa!=2:continue
    objs.append(o)
data['objects']=objs

# drone
if tarefa==1:drone['xyz']=[-6.5,0,drone['xyz'][2]]
else:drone['xyz']=[0,0,drone['xyz'][2]]

# grava
with open('world_eletroquad.yaml','w') as f: yaml.dump(data,f,default_flow_style=False)
print('world_eletroquad.yaml gerado!')