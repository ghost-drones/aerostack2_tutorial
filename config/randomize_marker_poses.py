#!/usr/bin/env python3
import yaml
import random
import math

def gerar_posicao_aleatoria(min_dist=0.5, max_dist=4.5):
    """
    Gera uma posição (x, y) aleatória de forma uniforme na área
    do anel definido pelas distâncias min_dist e max_dist, com no máximo 2 casas decimais.
    """
    theta = random.uniform(0, 2 * math.pi)
    # Sorteia um valor u uniformemente entre os quadrados dos limites
    u = random.uniform(min_dist**2, max_dist**2)
    r = math.sqrt(u)
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return round(x, 2), round(y, 2)

def atualizar_marcadores(data):
    """
    Para cada objeto cujo model_name inicia com 'marker', atualiza a posição (x, y)
    com novos valores aleatórios arredondados para 2 casas decimais.
    """
    for obj in data.get("objects", []):
        if obj.get("model_name", "").startswith("marker"):
            x, y = gerar_posicao_aleatoria()
            # Atualiza as coordenadas x e y; z permanece inalterado (0.01)
            obj["xyz"][0] = x
            obj["xyz"][1] = y
    return data

def main():
    # Nome do arquivo de entrada e saída
    arquivo_entrada = "world.yaml"
    arquivo_saida = "world.yaml"

    # Lê o YAML original
    with open(arquivo_entrada, "r") as f:
        data = yaml.safe_load(f)

    # Atualiza as posições dos marcadores
    data_atualizada = atualizar_marcadores(data)

    # Salva o YAML atualizado em um novo arquivo
    with open(arquivo_saida, "w") as f:
        yaml.dump(data_atualizada, f, default_flow_style=False, sort_keys=False)

    print(f"Arquivo atualizado salvo em '{arquivo_saida}'.")

if __name__ == "__main__":
    main()
