#!/bin/bash

usage() {
    echo "usage: $0 [options]"
    echo "  options:"
    echo "      -m: multi agent. Default not set"
    echo "      -n: select drones namespace to launch, values are comma separated. By default, it will get all drones from world description file"
    echo "      -s: if set, the simulation will not be launched. Default launch simulation"
    echo "      -g: launch using gnome-terminal instead of tmux. Default not set"
    echo "      -y: launch YOLO node (caso contrário, o nó do YOLO não será iniciado)"
    echo "      -1, -2 ou -3: seleciona a missão a ser executada (ex.: -1 para missão 1)"
}

# Inicializa variáveis com valores padrão
swarm="false"
drones_namespace_comma=""
launch_simulation="true"
use_gnome="false"
yolo_launch="false"
mission=""

# Usa getopt para tratar as opções, inclusive as numéricas
TEMP=$(getopt -o "mn:sgy123" -n "$0" -- "$@")
if [ $? != 0 ] ; then
    echo "Erro ao analisar os argumentos." >&2
    usage
    exit 1
fi
eval set -- "$TEMP"

while true; do
    case "$1" in
        -m)
            swarm="true"
            shift
            ;;
        -n)
            drones_namespace_comma="$2"
            shift 2
            ;;
        -s)
            launch_simulation="false"
            shift
            ;;
        -g)
            use_gnome="true"
            shift
            ;;
        -y)
            yolo_launch="true"
            shift
            ;;
        -1)
            mission="1"
            shift
            ;;
        -2)
            mission="2"
            shift
            ;;
        -3)
            mission="3"
            shift
            ;;
        --)
            shift
            break
            ;;
        *)
            echo "Erro interno!"
            exit 1
            ;;
    esac
done

# Se a missão não foi informada, define missão padrão (1)
if [ -z "$mission" ]; then
    mission="1"
fi

# Define o arquivo de configuração de mundo
if [[ ${swarm} == "true" ]]; then
    simulation_config="config/world_swarm.yaml"
else
    simulation_config="config/world_eletroquad.yaml"
fi

# Se não houver namespaces de drones fornecidos, obtém-os do arquivo de configuração
if [ -z "$drones_namespace_comma" ]; then
    drones_namespace_comma=$(python3 utils/get_drones.py -p "${simulation_config}" --sep ',')
fi
IFS=',' read -r -a drone_namespaces <<< "$drones_namespace_comma"

# Seleciona entre tmux e gnome-terminal
tmuxinator_mode="start"
tmuxinator_end="wait"
tmp_file="/tmp/as2_project_launch_${drone_namespaces[@]}.txt"
if [[ ${use_gnome} == "true" ]]; then
    tmuxinator_mode="debug"
    tmuxinator_end="> ${tmp_file} && python3 utils/tmuxinator_to_genome.py -p ${tmp_file} && wait"
fi

# Lança aerostack2 para cada namespace de drone, repassando também a missão
for namespace in "${drone_namespaces[@]}"; do
    base_launch="false"
    if [[ ${namespace} == "${drone_namespaces[0]}" && ${launch_simulation} == "true" ]]; then
        base_launch="true"
    fi
    eval "tmuxinator ${tmuxinator_mode} -n ${namespace} -p tmuxinator/aerostack2_eletroquad_px4.yaml \
      drone_namespace=${namespace} \
      simulation_config_file=${simulation_config} \
      base_launch=${base_launch} \
      yolo_launch=${yolo_launch} \
      mission=${mission} \
      ${tmuxinator_end}"
    sleep 0.1
done

# Anexa à sessão tmux (ou limpa arquivo temporário se estiver usando gnome-terminal)
if [[ ${use_gnome} == "false" ]]; then
    tmux attach-session -t "${drone_namespaces[0]}"
elif [[ -f ${tmp_file} ]]; then
    rm ${tmp_file}
fi
