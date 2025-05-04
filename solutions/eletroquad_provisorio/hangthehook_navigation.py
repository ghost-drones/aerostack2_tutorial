"""
Navegação, HangTheHook

• Seguimento da curva azul gerando waypoints ao longo de `blue_curve/derivative`
• Aproximação e alinhamento acima da parte superior da mangueira (Assim que encontrar a mangueira vermelha, subir 2m e 
seguir para frente, olhando diretamente para o meio da mangueira vermelha)
• Descida controlada até engatar o gancho (utilizando controle de velocidade)
• Saída da área da mangueira e retorno à base (Após soltura do gancho, ir para trás e depois subir para sair de perto do gancho e da mangueira)

Inputs:  
• Curva e derivada: `blue_curve/points`, `blue_curve/derivative`
• Posição estimada da mangueira: `hose_bbox`
• Poses atual do drone: `drone_state`

Outputs:  
• Trajetória de seguimento: `hook_path`
• Sequência de comandos de posição: `setpoint_{x,y,z,yaw}`
"""
