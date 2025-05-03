"""
Percepção, HangTheHook

• Detecção da linha azul com a câmera inferior; cálculo aproximado da derivada do traçado
• Identificação da mangueira vermelha pela câmera frontal
• Identificação da mangueira vermelha pela câmera inferior
• Detecção de soltura do gancho via mudança brusca de pixels em região específica

Inputs:  
• Calibração câmeras frontal e inferior: `camera_info_front`, `camera_info_bottom`  
• Poses relativas câmeras: `drone_pose` → `{cam_front, cam_bottom}`  
• Streams de imagem: `image_front_raw`, `image_bottom_raw`

Outputs:  
• Polilinha aproximada da curva azul: `blue_curve/points`, `blue_curve/derivative`  
• Máscara e bounding box da mangueira vermelha: `hose_mask`, `hose_bbox`  
• Flag de soltura do gancho: `hook_released`
"""