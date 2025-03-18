#!/usr/bin/env python3
import yaml
import random

def randomize_positions_and_colors(data):
    """
    Atualiza as posições x e y dos objetos que representam as barras e a base_final.
    - Para cada objeto (exceto 'base_inicial'):
      - O espaçamento em x é incremental, com valor entre 0.6m e 2.6m (em relação ao objeto anterior).
      - O valor y é sorteado entre -3.5m e 3.5m.
      - Os valores são arredondados para 2 casas decimais.
    - Para cada barra (model_name iniciando com "barra" exceto "base_final"):
      - Escolhe aleatoriamente "esquerda" ou "direita" e imprime a escolha.
      - Randomiza a cor (model_type) escolhendo dentre: "barra_azul", "barra_preto_fosco", "barra_rosa" e "barra_vermelha".
    - A "base_final" é atualizada com as posições, mas sua cor permanece inalterada e não recebe escolha de lado.
    """
    # Cores disponíveis para as barras
    available_colors = ["barra_azul", "barra_preto_fosco", "barra_rosa", "barra_vermelha"]
    
    # Encontra a base_inicial para usar sua posição x como referência
    base_inicial = next((obj for obj in data.get("objects", []) if obj.get("model_name") == "base_inicial"), None)
    current_x = base_inicial["xyz"][0] if base_inicial is not None else 0.0

    for obj in data.get("objects", []):
        model_name = obj.get("model_name", "")
        # Atualiza apenas barras e a base_final
        if model_name.startswith("barra") or model_name == "base_final":
            # Gera espaçamento aleatório para x entre 0.6 e 2.6
            spacing = random.uniform(0.6, 2.6)
            new_x = current_x + spacing
            current_x = new_x  # atualiza para o próximo objeto

            # Gera y aleatório entre -3.5 e 3.5
            new_y = random.uniform(-3.5, 3.5)

            # Arredonda os valores para 2 casas decimais
            new_x = round(new_x, 2)
            new_y = round(new_y, 2)

            # Mantém z inalterado, se existir (ou usa 0.0)
            z = obj["xyz"][2] if len(obj.get("xyz", [])) > 2 else 0.0
            obj["xyz"] = [new_x, new_y, z]

            # Se o objeto for uma barra (não for a base_final), randomiza a cor e escolhe um lado
            if model_name != "base_final":
                # Randomiza a cor
                new_color = random.choice(available_colors)
                obj["model_type"] = new_color

                # Escolhe aleatoriamente "esquerda" ou "direita" e imprime a escolha
                side = random.choice(["esquerda", "direita"])
                print(f"{side}")
    return data

def main():
    input_file = "world_slalom.yaml"
    output_file = "world_slalom.yaml"

    # Lê o arquivo YAML de entrada
    with open(input_file, "r") as f:
        data = yaml.safe_load(f)

    # Atualiza posições e cores conforme as regras definidas
    updated_data = randomize_positions_and_colors(data)

    # Salva o YAML atualizado em um novo arquivo
    with open(output_file, "w") as f:
        yaml.dump(updated_data, f, default_flow_style=False, sort_keys=False)

    print(f"\nArquivo atualizado salvo em '{output_file}'.")

if __name__ == "__main__":
    main()
