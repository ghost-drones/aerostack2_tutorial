import numpy as np
import argparse
import xml.etree.ElementTree as ET

def bezier_cubic(p0, p1, p2, p3, t):
    return (1 - t)**3 * p0 + 3 * (1 - t)**2 * t * p1 + \
           3 * (1 - t) * t**2 * p2 + t**3 * p3

def bezier_derivative(p0, p1, p2, p3, t):
    return -3 * (1 - t)**2 * p0 + \
           3 * (1 - t)**2 * p1 - 6 * (1 - t) * t * p1 - \
           3 * t**2 * p2 + 6 * (1 - t) * t * p2 + 3 * t**2 * p3

def gerar_sdf_fita_retangular(x0, y0, yaw0, x1, y1, yaw1,
                              largura=0.25, N=50,
                              z_plane=0.0, nome_arquivo="fita.sdf"):

    # Pontos de controle em 3D (z = 0)
    p0 = np.array([x0, y0, 0.0])
    p3 = np.array([x1, y1, 0.0])
    dist = np.linalg.norm(p3[:2] - p0[:2]) / 2.0
    p1 = p0 + np.array([np.cos(yaw0), np.sin(yaw0), 0.0]) * dist * 0.5
    p2 = p3 - np.array([np.cos(yaw1), np.sin(yaw1), 0.0]) * dist * 0.5

    # Amostragem da curva
    ts = np.linspace(0, 1, N)
    centros = np.array([bezier_cubic(p0, p1, p2, p3, t) for t in ts])
    derivs  = np.array([bezier_derivative(p0, p1, p2, p3, t) for t in ts])
    tangentes = derivs / np.linalg.norm(derivs, axis=1)[:, None]

    # Cria estrutura SDF
    sdf = ET.Element('sdf', version="1.6")
    model = ET.SubElement(sdf, 'model', name="fita_curva")
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name="link_fita")

    meio_z = z_plane
    half_w = largura / 2.0

    for i in range(N-1):
        p1c = centros[i]
        p2c = centros[i+1]
        length = np.linalg.norm(p2c - p1c)
        meio_xy = (p1c + p2c) / 2.0
        yaw = np.arctan2(tangentes[i][1], tangentes[i][0])

        # ... cálculos de p1c, p2c, length, meio_xy, yaw ...

        # pose central do retângulo
        pose = f"{meio_xy[0]} {meio_xy[1]} {meio_z} 0 0 {yaw}"

        # Visual
        vis = ET.SubElement(link, 'visual', name=f"visual_{i}")
        ET.SubElement(vis, 'pose').text = pose
        geom = ET.SubElement(vis, 'geometry')
        box = ET.SubElement(geom, 'box')
        size = ET.SubElement(box, 'size')
        size.text = f"{length} {largura} 0.01"

        mat = ET.SubElement(vis, 'material')
        # define todos os componentes para azul
        ET.SubElement(mat, 'ambient').text = "0 0 1 1"
        ET.SubElement(mat, 'diffuse').text = "0 0 1 1"
        ET.SubElement(mat, 'specular').text = "0 0 1 1"
        ET.SubElement(mat, 'emissive').text = "0 0 1 1"

        # Collision permanece igual
        col = ET.SubElement(link, 'collision', name=f"collision_{i}")
        ET.SubElement(col, 'pose').text = pose
        geom_c = ET.SubElement(col, 'geometry')
        box_c = ET.SubElement(geom_c, 'box')
        size_c = ET.SubElement(box_c, 'size')
        size_c.text = f"{length} {largura} 0.01"

    # Exporta arquivo .sdf
    tree = ET.ElementTree(sdf)
    tree.write(nome_arquivo, encoding='utf-8', xml_declaration=True)
    print(f"SDF salvo como: {nome_arquivo}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Gera fita curva (retângulos) em SDF no plano z=0.11.")
    parser.add_argument('--x0',   type=float, required=True)
    parser.add_argument('--y0',   type=float, required=True)
    parser.add_argument('--yaw0', type=float, required=True,
                        help="radianos")
    parser.add_argument('--x1',   type=float, required=True)
    parser.add_argument('--y1',   type=float, required=True)
    parser.add_argument('--yaw1', type=float, required=True,
                        help="radianos")
    parser.add_argument('--larg', type=float, default=0.25,
                        help="largura da fita (m)")
    parser.add_argument('--n',    type=int, default=200,
                        help="número de retângulos")
    parser.add_argument('--z',    type=float, default=0.0,
                        help="altura em z (m)")
    parser.add_argument('--out',  type=str, default="/root/tutorials/src/aerostack2_tutorial/models/fita/fita.sdf",
                        help="arquivo de saída .sdf")

    args = parser.parse_args()
    gerar_sdf_fita_retangular(
        args.x0, args.y0, args.yaw0,
        args.x1, args.y1, args.yaw1,
        largura=args.larg,
        N=args.n,
        z_plane=args.z,
        nome_arquivo=args.out
    )
