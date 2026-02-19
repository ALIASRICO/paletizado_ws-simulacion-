#!/usr/bin/env python3
"""
Script para analizar el archivo URDF del robot CR20.
Extrae información de links, joints, meshes y propiedades físicas.
"""

import os
import xml.etree.ElementTree as ET
from pathlib import Path


def find_urdf_file():
    """Busca el archivo URDF del CR20."""
    # Possible paths to search
    possible_paths = [
        os.path.expanduser("~/dobot_ws/src/DOBOT_6Axis_ROS2_V4/cr20_moveit/urdf/cr20.urdf"),
        os.path.expanduser("~/dobot_ws/src/DOBOT_6Axis_ROS2_V4/cra_description/urdf/cr20_robot.urdf"),
        os.path.expanduser("~/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_rviz/urdf/cr20.urdf"),
    ]
    
    # Check if any path exists
    for path in possible_paths:
        if os.path.exists(path):
            return path
    
    # Search for any cr20*.urdf file
    base_dir = os.path.expanduser("~/dobot_ws/src")
    for root, dirs, files in os.walk(base_dir):
        for f in files:
            if f.endswith('.urdf') and 'cr20' in f.lower():
                return os.path.join(root, f)
    
    return None


def parse_urdf(filepath):
    """Parsea el archivo URDF y extrae información."""
    tree = ET.parse(filepath)
    root = tree.getroot()
    
    # Get robot name
    robot_name = root.get('name', 'Unknown')
    
    links = []
    joints = []
    
    # Parse links
    for link in root.findall('link'):
        link_info = {
            'name': link.get('name', ''),
            'visual_geometry': None,
            'collision_geometry': None,
            'mass': None,
            'ixx': None, 'ixy': None, 'ixz': None,
            'iyy': None, 'iyz': None, 'izz': None,
            'origin_xyz': None,
            'origin_rpy': None,
        }
        
        # Visual geometry
        visual = link.find('visual')
        if visual is not None:
            geometry = visual.find('geometry')
            if geometry is not None:
                # Check for mesh
                mesh = geometry.find('mesh')
                if mesh is not None:
                    link_info['visual_geometry'] = {
                        'type': 'mesh',
                        'filename': mesh.get('filename', ''),
                        'scale': mesh.get('scale', '')
                    }
                # Check for box
                box = geometry.find('box')
                if box is not None:
                    link_info['visual_geometry'] = {
                        'type': 'box',
                        'size': box.get('size', '')
                    }
                # Check for cylinder
                cylinder = geometry.find('cylinder')
                if cylinder is not None:
                    link_info['visual_geometry'] = {
                        'type': 'cylinder',
                        'radius': cylinder.get('radius', ''),
                        'length': cylinder.get('length', '')
                    }
                # Check for sphere
                sphere = geometry.find('sphere')
                if sphere is not None:
                    link_info['visual_geometry'] = {
                        'type': 'sphere',
                        'radius': sphere.get('radius', '')
                    }
            
            # Origin
            origin = visual.find('origin')
            if origin is not None:
                link_info['origin_xyz'] = origin.get('xyz', '')
                link_info['origin_rpy'] = origin.get('rpy', '')
        
        # Collision geometry
        collision = link.find('collision')
        if collision is not None:
            geometry = collision.find('geometry')
            if geometry is not None:
                mesh = geometry.find('mesh')
                if mesh is not None:
                    link_info['collision_geometry'] = {
                        'type': 'mesh',
                        'filename': mesh.get('filename', ''),
                        'scale': mesh.get('scale', '')
                    }
                box = geometry.find('box')
                if box is not None:
                    link_info['collision_geometry'] = {
                        'type': 'box',
                        'size': box.get('size', '')
                    }
        
        # Inertial properties
        inertial = link.find('inertial')
        if inertial is not None:
            mass = inertial.find('mass')
            if mass is not None:
                link_info['mass'] = mass.get('value', '')
            
            inertia = inertial.find('inertia')
            if inertia is not None:
                link_info['ixx'] = inertia.get('ixx', '')
                link_info['ixy'] = inertia.get('ixy', '')
                link_info['ixz'] = inertia.get('ixz', '')
                link_info['iyy'] = inertia.get('iyy', '')
                link_info['iyz'] = inertia.get('iyz', '')
                link_info['izz'] = inertia.get('izz', '')
            
            origin = inertial.find('origin')
            if origin is not None:
                link_info['inertial_origin_xyz'] = origin.get('xyz', '')
                link_info['inertial_origin_rpy'] = origin.get('rpy', '')
        
        links.append(link_info)
    
    # Parse joints
    for joint in root.findall('joint'):
        joint_info = {
            'name': joint.get('name', ''),
            'type': joint.get('type', ''),
            'parent': '',
            'child': '',
            'axis': '',
            'origin_xyz': '',
            'origin_rpy': '',
            'limit_lower': '',
            'limit_upper': '',
            'limit_effort': '',
            'limit_velocity': '',
            'dynamics_damping': '',
            'dynamics_friction': '',
        }
        
        # Parent and child
        parent = joint.find('parent')
        if parent is not None:
            joint_info['parent'] = parent.get('link', '')
        
        child = joint.find('child')
        if child is not None:
            joint_info['child'] = child.get('link', '')
        
        # Axis
        axis = joint.find('axis')
        if axis is not None:
            joint_info['axis'] = axis.get('xyz', '')
        
        # Origin
        origin = joint.find('origin')
        if origin is not None:
            joint_info['origin_xyz'] = origin.get('xyz', '')
            joint_info['origin_rpy'] = origin.get('rpy', '')
        
        # Limits
        limit = joint.find('limit')
        if limit is not None:
            joint_info['limit_lower'] = limit.get('lower', '')
            joint_info['limit_upper'] = limit.get('upper', '')
            joint_info['limit_effort'] = limit.get('effort', '')
            joint_info['limit_velocity'] = limit.get('velocity', '')
        
        # Dynamics
        dynamics = joint.find('dynamics')
        if dynamics is not None:
            joint_info['dynamics_damping'] = dynamics.get('damping', '')
            joint_info['dynamics_friction'] = dynamics.get('friction', '')
        
        joints.append(joint_info)
    
    return {
        'robot_name': robot_name,
        'filepath': filepath,
        'links': links,
        'joints': joints
    }


def generate_ascii_diagram(urdf_data):
    """Genera un diagrama ASCII de la estructura del robot."""
    joints = urdf_data['joints']
    links = urdf_data['links']
    
    # Build link -> joint mapping
    link_chain = {}
    for joint in joints:
        parent = joint['parent']
        child = joint['child']
        joint_type = joint['type']
        
        if parent not in link_chain:
            link_chain[parent] = []
        link_chain[parent].append({
            'joint_name': joint['name'],
            'joint_type': joint_type,
            'child': child
        })
    
    # Find base link (no parent)
    all_children = set()
    for joint in joints:
        all_children.add(joint['child'])
    
    base_links = []
    for joint in joints:
        if joint['parent'] not in all_children:
            base_links.append(joint['parent'])
    
    # Generate diagram
    lines = []
    lines.append("=" * 60)
    lines.append("DIAGRAMA DE ESTRUCTURA DEL ROBOT")
    lines.append("=" * 60)
    lines.append("")
    
    if base_links:
        lines.append(f"Base: {base_links[0]}")
        lines.append("")
        lines.append("     +---------+")
        lines.append(f"     | {base_links[0][:9]:<9} |")
        lines.append("     +---------+")
        
        # Follow chain
        def print_chain(parent, indent=0):
            prefix = "  " * indent
            if parent in link_chain:
                for item in link_chain[parent]:
                    joint_name = item['joint_name']
                    joint_type = item['joint_type']
                    child = item['child']
                    
                    type_symbol = {
                        'revolute': '↻',
                        'prismatic': '↔',
                        'fixed': '⚡',
                        'continuous': '○'
                    }.get(joint_type, '?')
                    
                    lines.append(f"{prefix}  | {type_symbol} {joint_type[:8]:<8} |")
                    lines.append(f"{prefix}  +---------+")
                    lines.append(f"{prefix}  | {child[:9]:<9} |")
                    lines.append(f"{prefix}  +---------+")
                    
                    print_chain(child, indent + 1)
        
        print_chain(base_links[0])
    else:
        lines.append("No se pudo determinar la cadena cinemática")
    
    lines.append("")
    lines.append("Leyenda:")
    lines.append("  ↻ = Revolute (rotación)")
    lines.append("  ↔ = Prismatic (deslizamiento)")
    lines.append("  ⚡ = Fixed (fijo)")
    lines.append("  ○ = Continuous (continuo)")
    
    return '\n'.join(lines)


def generate_markdown_report(urdf_data):
    """Genera el reporte en formato Markdown."""
    
    md = []
    
    # Header
    md.append("# Análisis del URDF - Robot CR20\n")
    md.append(f"**Archivo**: `{urdf_data['filepath']}`\n")
    md.append(f"**Robot Name**: {urdf_data['robot_name']}\n")
    md.append(f"**Total Links**: {len(urdf_data['links'])}\n")
    md.append(f"**Total Joints**: {len(urdf_data['joints'])}\n")
    md.append("\n---\n")
    
    # ASCII Diagram
    md.append("## 1. Diagrama de Estructura\n\n")
    md.append("```\n")
    md.append(generate_ascii_diagram(urdf_data))
    md.append("\n```\n")
    md.append("\n---\n")
    
    # Joints table
    md.append("## 2. Tabla de Articulaciones (Joints)\n\n")
    md.append("| # | Nombre | Tipo | Parent | Child | Eje | Límite Inferior | Límite Superior | Esfuerzo Max | Velocidad Max |\n")
    md.append("|---|--------|------|--------|-------|-----|-----------------|-----------------|--------------|---------------|\n")
    
    for i, joint in enumerate(urdf_data['joints'], 1):
        md.append(f"| {i} | {joint['name']} | {joint['type']} | {joint['parent']} | {joint['child']} | {joint['axis']} | {joint['limit_lower']} | {joint['limit_upper']} | {joint['limit_effort']} | {joint['limit_velocity']} |\n")
    
    md.append("\n### Descripción de tipos de joints:\n")
    md.append("- **revolute**: Rotación limitada entre dos ángulos\n")
    md.append("- **prismatic**: Movimiento lineal limitado\n")
    md.append("- **continuous**: Rotación ilimitada\n")
    md.append("- **fixed**: Sin movimiento (fijo)\n")
    md.append("\n---\n")
    
    # Links table
    md.append("## 3. Tabla de Eslabones (Links)\n\n")
    md.append("| # | Nombre | Masa (kg) | Geometría Visual | Geometría Colisión |\n")
    md.append("|---|--------|-----------|-------------------|--------------------|\n")
    
    for i, link in enumerate(urdf_data['links'], 1):
        visual = link.get('visual_geometry', {})
        if visual:
            if visual.get('type') == 'mesh':
                v_geom = f"Mesh: {os.path.basename(visual.get('filename', ''))}"
            else:
                v_geom = f"{visual.get('type')}: {visual.get('size') or visual.get('radius') or ''}"
        else:
            v_geom = '-'
        
        collision = link.get('collision_geometry', {})
        if collision:
            if collision.get('type') == 'mesh':
                c_geom = f"Mesh: {os.path.basename(collision.get('filename', ''))}"
            else:
                c_geom = f"{collision.get('type')}: {collision.get('size') or collision.get('radius') or ''}"
        else:
            c_geom = '-'
        
        md.append(f"| {i} | {link['name']} | {link['mass'] or '-'} | {v_geom} | {c_geom} |\n")
    
    md.append("\n---\n")
    
    # Detailed joints info
    md.append("## 4. Detalle de Cada Articulación\n\n")
    
    for joint in urdf_data['joints']:
        md.append(f"### {joint['name']}\n")
        md.append(f"\n**Tipo**: `{joint['type']}`\n")
        md.append(f"\n**Conexión**:\n")
        md.append(f"- Padre (Parent): `{joint['parent']}`\n")
        md.append(f"- Hijo (Child): `{joint['child']}`\n")
        
        if joint['axis']:
            md.append(f"\n**Eje de rotación**: `{joint['axis']}`\n")
        
        if joint['origin_xyz'] or joint['origin_rpy']:
            md.append(f"\n**Origen**:\n")
            if joint['origin_xyz']:
                md.append(f"- Posición (xyz): `{joint['origin_xyz']}`\n")
            if joint['origin_rpy']:
                md.append(f"- Rotación (rpy): `{joint['origin_rpy']}`\n")
        
        md.append(f"\n**Límites de movimiento**:\n")
        if joint['limit_lower']:
            md.append(f"- Mínimo: {joint['limit_lower']} rad\n")
        if joint['limit_upper']:
            md.append(f"- Máximo: {joint['limit_upper']} rad\n")
        if joint['limit_effort']:
            md.append(f"- Esfuerzo máximo: {joint['limit_effort']} Nm\n")
        if joint['limit_velocity']:
            md.append(f"- Velocidad máxima: {joint['limit_velocity']} rad/s\n")
        
        if joint['dynamics_damping'] or joint['dynamics_friction']:
            md.append(f"\n**Dinámica**:\n")
            if joint['dynamics_damping']:
                md.append(f"- Amortiguación: {joint['dynamics_damping']}\n")
            if joint['dynamics_friction']:
                md.append(f"- Fricción: {joint['dynamics_friction']}\n")
        
        md.append("\n---\n")
    
    # Detailed links info
    md.append("## 5. Detalle de Cada Eslabón\n\n")
    
    for link in urdf_data['links']:
        md.append(f"### {link['name']}\n")
        
        # Mass and inertia
        if link['mass']:
            md.append(f"**Masa**: {link['mass']} kg\n")
        
        if any([link.get('ixx'), link.get('iyy'), link.get('izz')]):
            md.append(f"\n**Tensor de Inercia**:\n")
            md.append("```\n")
            md.append(f"[{link.get('ixx', '0') or '0'}, {link.get('ixy', '0') or '0'}, {link.get('ixz', '0') or '0'}]\n")
            md.append(f"[{link.get('ixy', '0') or '0'}, {link.get('iyy', '0') or '0'}, {link.get('iyz', '0') or '0'}]\n")
            md.append(f"[{link.get('ixz', '0') or '0'}, {link.get('iyz', '0') or '0'}, {link.get('izz', '0') or '0'}]\n")
            md.append("```\n")
        
        # Visual geometry
        visual = link.get('visual_geometry')
        if visual:
            md.append(f"\n**Geometría Visual**:\n")
            md.append(f"- Tipo: `{visual.get('type', 'desconocido')}`\n")
            if visual.get('type') == 'mesh':
                md.append(f"- Archivo: `{visual.get('filename', '')}`\n")
                md.append(f"- Escala: `{visual.get('scale', '1 1 1')}`\n")
            elif visual.get('type') == 'box':
                md.append(f"- Dimensiones: `{visual.get('size', '')}` (largo x ancho x alto)\n")
            elif visual.get('type') == 'cylinder':
                md.append(f"- Radio: {visual.get('radius', '')} m\n")
                md.append(f"- Longitud: {visual.get('length', '')} m\n")
            elif visual.get('type') == 'sphere':
                md.append(f"- Radio: {visual.get('radius', '')} m\n")
        
        # Collision geometry
        collision = link.get('collision_geometry')
        if collision:
            md.append(f"\n**Geometría de Colisión**:\n")
            md.append(f"- Tipo: `{collision.get('type', 'desconocido')}`\n")
            if collision.get('type') == 'mesh':
                md.append(f"- Archivo: `{collision.get('filename', '')}`\n")
            elif collision.get('type') == 'box':
                md.append(f"- Dimensiones: `{collision.get('size', '')}`\n")
        
        # Origin
        if link.get('origin_xyz') or link.get('origin_rpy'):
            md.append(f"\n**Origen**:\n")
            if link.get('origin_xyz'):
                md.append(f"- Posición (xyz): `{link.get('origin_xyz')}`\n")
            if link.get('origin_rpy'):
                md.append(f"- Rotación (rpy): `{link.get('origin_rpy')}`\n")
        
        md.append("\n---\n")
    
    # Mesh files
    md.append("## 6. Archivos de Mesh (Modelos 3D)\n\n")
    md.append("| Link | Ruta del Mesh |\n")
    md.append("|------|---------------|\n")
    
    for link in urdf_data['links']:
        visual = link.get('visual_geometry', {})
        if visual and visual.get('type') == 'mesh':
            filename = visual.get('filename', '')
            md.append(f"| {link['name']} | `{filename}` |\n")
    
    md.append("\n---\n")
    
    # Movement limits summary
    md.append("## 7. Resumen de Límites de Movimiento\n\n")
    md.append("| Joint | Tipo | Rango de Movimiento | Velocidad Máx |\n")
    md.append("|-------|------|-------------------|---------------|\n")
    
    for joint in urdf_data['joints']:
        if joint['type'] == 'revolute' or joint['type'] == 'prismatic':
            range_str = f"{joint['limit_lower']} a {joint['limit_upper']}"
            if joint['type'] == 'revolute':
                range_str += " rad"
            else:
                range_str += " m"
        elif joint['type'] == 'continuous':
            range_str = "-360° a 360° (ilimitado)"
        else:
            range_str = "N/A (fijo)"
        
        md.append(f"| {joint['name']} | {joint['type']} | {range_str} | {joint['limit_velocity']} |\n")
    
    md.append("\n---\n")
    
    # Examples section
    md.append("## 8. Ejemplos de Valores\n\n")
    md.append("### Posición Home (inicial)\n")
    md.append("```\n")
    md.append("joint1: 0 rad\n")
    md.append("joint2: -45° (-0.785 rad)\n")
    md.append("joint3: 60° (1.047 rad)\n")
    md.append("joint4: -90° (-1.571 rad)\n")
    md.append("joint5: -90° (-1.571 rad)\n")
    md.append("joint6: 0 rad\n")
    md.append("```\n")
    
    md.append("### Ejemplo de comando de trayectoria\n")
    md.append("```python\n")
    md.append("# Ejemplo de posiciones articulares (en radianes)\n")
    md.append("positions = [\n")
    md.append("    0.0,      # joint1\n")
    md.append("    -0.785,   # joint2 (-45°)\n")
    md.append("    1.047,   # joint3 (60°)\n")
    md.append("    -1.571,  # joint4 (-90°)\n")
    md.append("    -1.571,  # joint5 (-90°)\n")
    md.append("    0.0      # joint6\n")
    md.append("]\n")
    md.append("```\n")
    
    md.append("\n---\n")
    md.append("*Reporte generado automáticamente por analyze_urdf.py*\n")
    
    return '\n'.join(md)


def main():
    """Función principal."""
    print("Buscando archivo URDF del CR20...")
    
    urdf_path = find_urdf_file()
    
    if urdf_path is None:
        print("ERROR: No se encontró ningún archivo URDF del CR20")
        print("Rutas buscadas:")
        print("  - ~/dobot_ws/src/DOBOT_6Axis_ROS2_V4/cr20_moveit/urdf/cr20.urdf")
        print("  - ~/dobot_ws/src/DOBOT_6Axis_ROS2_V4/cra_description/urdf/cr20_robot.urdf")
        print("  - ~/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_rviz/urdf/cr20.urdf")
        return
    
    print(f"Archivo encontrado: {urdf_path}")
    print("Parseando URDF...")
    
    urdf_data = parse_urdf(urdf_path)
    
    print(f"Robot: {urdf_data['robot_name']}")
    print(f"Links: {len(urdf_data['links'])}")
    print(f"Joints: {len(urdf_data['joints'])}")
    
    # Generate report
    report = generate_markdown_report(urdf_data)
    
    # Save to file
    output_file = os.path.expanduser("~/dobot_ws/URDF_ANALYSIS.md")
    with open(output_file, 'w') as f:
        f.write(report)
    
    print(f"\nReporte guardado en: {output_file}")
    
    # Print summary
    print("\n=== RESUMEN ===")
    print(f"Robot: {urdf_data['robot_name']}")
    print(f"Total de links: {len(urdf_data['links'])}")
    print(f"Total de joints: {len(urdf_data['joints'])}")
    
    print("\nJoints encontrados:")
    for joint in urdf_data['joints']:
        print(f"  - {joint['name']}: {joint['type']}")
    
    print("\nLinks con mesh:")
    for link in urdf_data['links']:
        visual = link.get('visual_geometry', {})
        if visual and visual.get('type') == 'mesh':
            print(f"  - {link['name']}: {visual.get('filename', '')}")


if __name__ == '__main__':
    main()
