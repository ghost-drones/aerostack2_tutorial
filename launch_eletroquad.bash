#!/bin/bash

usage() {
    echo "usage: $0 [options]"
    echo "  options:"
    echo "      -m: multi agent. Default not set"
    echo "      -n: select drones namespace to launch, values are comma separated. By default, it will get all drones from world description file"
    echo "      -s: if set, the simulation will not be launched. Default launch simulation"
    echo "      -g: launch using gnome-terminal instead of tmux. Default not set"
    echo "      -y: launch YOLO node (caso contrário, o nó do YOLO não será iniciado)"
    echo "      -t: tipo de plataforma (px4, mavros, gazebo). Default: px4"
    echo "      -e: usar simulação (true ou false). Default: true"
    echo "      -1, -2 ou -3: seleciona a missão a ser executada (ex.: -1 para missão 1)"
}

# Variáveis padrão
swarm="false"
drones_namespace_comma=""
launch_simulation="true"
use_gnome="false"
yolo_launch="false"
mission=""
type="px4"
sim="true"

# Adiciona -t e -e ao getopt
TEMP=$(getopt -o "mn:sgyt:e:123" -n "$0" -- "$@")
if [ $? != 0 ] ; then
    echo "Erro ao analisar os argumentos." >&2
    usage
    exit 1
fi
eval set -- "$TEMP"

while true; do
    case "$1" in
        -m) swarm="true"; shift ;;
        -n) drones_namespace_comma="$2"; shift 2 ;;
        -s) launch_simulation="false"; shift ;;
        -g) use_gnome="true"; shift ;;
        -y) yolo_launch="true"; shift ;;
        -t) type="$2"; shift 2 ;;
        -e) sim="$2"; shift 2 ;;
        -1) mission="1"; shift ;;
        -2) mission="2"; shift ;;
        -3) mission="3"; shift ;;
        --) shift; break ;;
        *) echo "Erro interno!"; exit 1 ;;
    esac
done

# Missão padrão
if [ -z "$mission" ]; then
    mission="1"
fi

# Arquivo de mundo
if [[ ${swarm} == "true" ]]; then
    simulation_config="config/world_swarm.yaml"
else
    simulation_config="config/world_eletroquad.yaml"
fi

# Namespaces
if [ -z "$drones_namespace_comma" ]; then
    drones_namespace_comma=$(python3 utils/get_drones.py -p "${simulation_config}" --sep ',')
fi
IFS=',' read -r -a drone_namespaces <<< "$drones_namespace_comma"

# tmux vs gnome-terminal
tmuxinator_mode="start"
tmuxinator_end="wait"
tmp_file="/tmp/as2_project_launch_${drone_namespaces[@]}.txt"
if [[ ${use_gnome} == "true" ]]; then
    tmuxinator_mode="debug"
    tmuxinator_end="> ${tmp_file} && python3 utils/tmuxinator_to_genome.py -p ${tmp_file} && wait"
fi

# Lançamento por namespace
for namespace in "${drone_namespaces[@]}"; do
    base_launch="false"
    if [[ ${namespace} == "${drone_namespaces[0]}" && ${launch_simulation} == "true" ]]; then
        base_launch="true"
    fi
    eval "tmuxinator ${tmuxinator_mode} -n ${namespace} -p tmuxinator/eletroquad.yaml \
      drone_namespace=${namespace} \
      simulation_config_file=${simulation_config} \
      base_launch=${base_launch} \
      yolo_launch=${yolo_launch} \
      mission=${mission} \
      type=${type} \
      sim=${sim} \
      ${tmuxinator_end}"
    sleep 0.1
done

# Pós-processamento
if [[ ${use_gnome} == "false" ]]; then
    tmux attach-session -t "${drone_namespaces[0]}"
elif [[ -f ${tmp_file} ]]; then
    rm ${tmp_file}
fi
