"""
Navegação, Bouncing

• Zig-zag de busca pela arena para localizar o `target_marker`
• Precision Landing sobre `target_marker/pose` após identificação

Inputs:  
• Perímetro da arena: `arena_perimeter`  
• Posição e orientação atual do drone: `drone_state`

Outputs:  
• Caminho de busca em zig-zag: `search_path`  
• Sequência de setpoints para pouso de precisão: `landing_setpoints`
"""
