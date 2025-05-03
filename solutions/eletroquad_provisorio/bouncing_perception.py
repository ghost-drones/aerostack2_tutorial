"""
Percepção, Bouncing

• Localização de marcadores no solo
• Classificação do marcador selecionado usando ID e distância ao centro da imagem

Inputs:  
• Parâmetros de calibragem da câmera: `camera_info`  
• Imagem crua: `image_raw`  
• Lista de IDs candidatos: `marker_ids`

Outputs:  
• Posições 2D dos cantos dos marcadores: `marker_{id}/corners`  
• Marcador alvo e sua pose aproximada: `target_marker/id`, `target_marker/pose`
"""
