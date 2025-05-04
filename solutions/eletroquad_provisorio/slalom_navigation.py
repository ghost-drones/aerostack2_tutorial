"""
Navegação, Slalom

• Geração de 4 waypoints laterais (esquerda/direita) a cada barra, segundo uma sequência de comandos aleatória (“direita”, “esquerda”, …)
• Ajuste de curva B-spline que passe pelos 4 waypoints

Inputs:  
• Poses das barras no espaço: `barra_{cor}/pose`  
• Sequência de direções: Argumentos na execução do script

Outputs:  
• Lista de waypoints: `slalom_waypoints`  
• Trajetória contínua (B-spline): `slalom_path`
"""