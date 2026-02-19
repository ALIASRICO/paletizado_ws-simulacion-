#!/usr/bin/env python3
"""
Script para analizar todos los package.xml del repositorio.
Extrae: nombre, versión, versión ROS2, dependencias, descripción.
Genera reporte comparativo e identifica incompatibilidades.
"""

import os
import xml.etree.ElementTree as ET
from pathlib import Path
from collections import defaultdict
import re


def find_package_xml_files(root_dir):
    """Encuentra todos los package.xml en el directorio."""
    package_files = []
    for root, dirs, files in os.walk(root_dir):
        if 'package.xml' in files:
            package_files.append(os.path.join(root, 'package.xml'))
    return package_files


def parse_package_xml(filepath):
    """Parsea un archivo package.xml y extrae información."""
    try:
        tree = ET.parse(filepath)
        root = tree.getroot()
        
        # Namespace commonly used in ROS2 packages
        ns = {'ros': 'http://ros.org/wiki/cmake'}
        
        # Try to find name - handles both formats
        name_elem = root.find('name')
        if name_elem is None:
            name_elem = root.find('.//name')
        name = name_elem.text if name_elem is not None else 'UNKNOWN'
        
        # Version
        version_elem = root.find('version')
        if version_elem is None:
            version_elem = root.find('.//version')
        version = version_elem.text if version_elem is not None else 'UNKNOWN'
        
        # Description
        desc_elem = root.find('description')
        if desc_elem is None:
            desc_elem = root.find('.//description')
        description = desc_elem.text if desc_elem is not None else ''
        if description:
            description = ' '.join(description.split())[:100]  # Clean and limit
        else:
            description = 'No description'
        
        # Dependencies
        dependencies = {
            'build_depend': [],
            'buildtool_depend': [],
            'exec_depend': [],
            'depend': [],
            'test_depend': [],
            'doc_depend': [],
        }
        
        for dep_type in dependencies.keys():
            deps = root.findall(dep_type)
            if deps:
                dependencies[dep_type] = [d.text for d in deps if d.text]
        
        # Also check for <depend> tag which is shorthand
        depend_tags = root.findall('.//depend')
        if depend_tags:
            for d in depend_tags:
                if d.text and d.text not in dependencies['depend']:
                    dependencies['depend'].append(d.text)
        
        # Package format version (ROS2 specific)
        format_elem = root.find('format')
        if format_elem is None:
            format_elem = root.find('.//format')
        format_version = format_elem.attrib.get('version', '2') if format_elem is not None else '2'
        
        # Build type
        build_type = 'ament_cmake'
        build_type_elem = root.find('build_type')
        if build_type_elem is not None:
            build_type = build_type_elem.text
        
        # Check for ROS2 version mentions in description or dependencies
        ros_version = detect_ros_version(dependencies, description)
        
        return {
            'name': name,
            'version': version,
            'format_version': format_version,
            'build_type': build_type,
            'description': description,
            'dependencies': dependencies,
            'ros_version': ros_version,
            'filepath': filepath
        }
        
    except Exception as e:
        return {
            'name': 'ERROR',
            'version': 'ERROR',
            'format_version': 'ERROR',
            'build_type': 'ERROR',
            'description': f'Error parsing: {str(e)}',
            'dependencies': {},
            'ros_version': 'UNKNOWN',
            'filepath': filepath
        }


def detect_ros_version(dependencies, description):
    """Detecta la versión de ROS2 declarada o inferida."""
    all_deps = []
    for dep_list in dependencies.values():
        all_deps.extend(dep_list)
    
    all_deps_str = ' '.join(all_deps).lower() + ' ' + description.lower()
    
    # Check for explicit version mentions
    if 'jazzy' in all_deps_str:
        return 'Jazzy'
    elif 'humble' in all_deps_str:
        return 'Humble'
    elif 'iron' in all_deps_str:
        return 'Iron'
    elif 'galactic' in all_deps_str:
        return 'Galactic'
    elif 'foxy' in all_deps_str:
        return 'Foxy'
    
    # Infer from dependencies
    if 'moveit_ros' in all_deps_str or 'ros_gz' in all_deps_str:
        return 'Jazzy (inferred)'
    
    return 'Not specified'


def categorize_package(name, dependencies):
    """Categoriza el paquete según su tipo."""
    name_lower = name.lower()
    all_deps = ' '.join([d for deps in dependencies.values() for d in deps]).lower()
    
    if 'moveit' in name_lower or 'moveit' in all_deps:
        return 'MoveIt'
    elif 'gazebo' in name_lower or 'gz_' in all_deps or 'ros_gz' in all_deps:
        return 'Gazebo'
    elif 'msg' in name_lower or 'msgs' in name_lower or 'srv' in name_lower:
        return 'Messages/Services'
    elif 'bringup' in name_lower:
        return 'Bringup/Control'
    elif 'description' in name_lower or 'urdf' in name_lower:
        return 'Robot Description'
    elif 'demo' in name_lower:
        return 'Demo'
    elif 'rviz' in name_lower:
        return 'RViz'
    elif 'servo' in name_lower:
        return 'Servo'
    elif 'vision' in name_lower or 'yolo' in name_lower:
        return 'Vision'
    elif 'conveyor' in name_lower:
        return 'Conveyor'
    else:
        return 'Other'


def check_incompatibilities(package_info):
    """Identifica incompatibilidades potenciales."""
    issues = []
    deps = package_info['dependencies']
    all_deps = [d for dep_list in deps.values() for d in dep_list]
    all_deps_str = ' '.join(all_deps).lower()
    
    # Check for Gazebo Classic vs Gazebo Sim
    if 'gazebo_ros' in all_deps_str or 'gazebo_ros2_control' in all_deps_str:
        issues.append('⚠️ Usa Gazebo Classic (gzclient)')
    
    if 'gz_ros' in all_deps_str or 'gz-sim' in all_deps_str:
        issues.append('✓ Usa Gazebo Sim (Harmonic)')
    
    # Check for ros2_control
    if 'ros2_control' in all_deps_str or 'ros2_controllers' in all_deps_str:
        issues.append('✓ Usa ros2_control')
    
    # Check MoveIt2
    if 'moveit_ros' in all_deps_str:
        issues.append('✓ Usa MoveIt2')
    
    # Check for older ROS2 versions
    if 'foxy' in all_deps_str or 'galactic' in all_deps_str:
        issues.append('⚠️ Puede requerir ROS2 antiguo')
    
    return issues


def generate_markdown_report(packages):
    """Genera el reporte en formato Markdown."""
    
    # Group by category
    categorized = defaultdict(list)
    for pkg in packages:
        category = categorize_package(pkg['name'], pkg['dependencies'])
        categorized[category].append(pkg)
    
    md = """# Análisis de Dependencias - Repositorio dobot_ws

## Resumen Ejecutivo

| Métrica | Valor |
|---------|-------|
| Total de paquetes | {total} |
| Paquetes con formato ROS2 | {format_count} |
| Paquetes mentioning Jazzy | {jazzy_count} |
| Paquetes mencionando Humble | {humble_count} |

---

## 1. Resumen de Paquetes por Categoría

""".format(
        total=len(packages),
        format_count=sum(1 for p in packages if p['format_version'] != '2'),
        jazzy_count=sum(1 for p in packages if 'jazzy' in p['ros_version'].lower()),
        humble_count=sum(1 for p in packages if 'humble' in p['ros_version'].lower())
    )
    
    # Add category summary
    for category, pkgs in sorted(categorized.items()):
        md += f"### {category}: {len(pkgs)} paquete(s)\n"
        for pkg in pkgs:
            md += f"- [{pkg['name']}]({pkg['filepath']}) (v{pkg['version']})\n"
        md += "\n"
    
    # Detailed tables per category
    for category, pkgs in sorted(categorized.items()):
        md += f"\n## 2. Detalle de Paquetes: {category}\n\n"
        md += f"| Paquete | Versión | Formato | Build Type | ROS Version |\n"
        md += f"|---------|---------|---------|------------|-------------|\n"
        
        for pkg in sorted(pkgs, key=lambda x: x['name']):
            md += f"| {pkg['name']} | {pkg['version']} | {pkg['format_version']} | {pkg['build_type']} | {pkg['ros_version']} |\n"
        
        md += f"\n### Descripciones\n\n"
        for pkg in sorted(pkgs, key=lambda x: x['name']):
            md += f"**{pkg['name']}**: {pkg['description']}\n\n"
        
        md += f"\n### Dependencias\n\n"
        for pkg in sorted(pkgs, key=lambda x: x['name']):
            all_deps = []
            for dep_type, dep_list in pkg['dependencies'].items():
                if dep_list:
                    for d in dep_list:
                        all_deps.append(d)
            
            if all_deps:
                md += f"**{pkg['name']}**: {', '.join(all_deps)}\n\n"
            else:
                md += f"**{pkg['name']}**: (sin dependencias explícitas)\n\n"
    
    # Incompatibilities section
    md += "\n## 3. Análisis de Incompatibilidades\n\n"
    md += "| Paquete | Problemas Potenciales |\n"
    md += "|---------|----------------------|\n"
    
    for pkg in sorted(packages, key=lambda x: x['name']):
        issues = check_incompatibilities(pkg)
        if issues:
            issues_str = '<br>'.join(issues)
            md += f"| {pkg['name']} | {issues_str} |\n"
        else:
            md += f"| {pkg['name']} | ✓ Sin problemas detectados |\n"
    
    # Isaac Sim suggestions
    md += """
---

## 4. Sugerencias para Isaac Sim

### Dependencias a reemplazar para Isaac Sim

| Paquete Original | Alternativa Isaac Sim |
|-----------------|----------------------|
| `ros_gz_sim` | `ros_ign_bridge` o APIs nativas de Isaac Sim |
| `gz-sim8` | `ignition-gazebo` o `gz-sim` |
| `gazebo_ros` | Eliminar (usar bridge ROS-Isaac) |
| `ros2_controllers` | Usar Isaac Actuation API |
| `robot_state_publisher` | Usar Isaac OM |

### Cambios recomendados

1. **Bridge ROS2-Isaac**: Crear bridges personalizados para topic translation
2. **URDF/USD**: Convertir meshes STL a USD para mejor rendimiento
3. **Controladores**: Reemplazar `joint_trajectory_controller` con Isaac Codelets
4. **MoveIt**: Mantener para planificación, usar Isaac execution cuando sea posible
5. **Simulación**: El archivo `Modelo/CR20V_isaac.usd` ya existe y puede ser usado directamente

### Paquetes compatibles con Isaac Sim

Los siguientes paquetes NO requieren cambios:
"""
    
    # List compatible packages
    compatible = []
    for pkg in packages:
        issues = check_incompatibilities(pkg)
        if not issues or all('✓' in i for i in issues):
            compatible.append(pkg['name'])
    
    for name in sorted(compatible):
        md += f"- {name}\n"
    
    md += """

### Paquetes que requieren modificación

"""
    # List packages that need changes
    needs_changes = []
    for pkg in packages:
        issues = check_incompatibilities(pkg)
        if issues and not all('✓' in i for i in issues):
            needs_changes.append((pkg['name'], issues))
    
    for name, issues in sorted(needs_changes, key=lambda x: x[0]):
        md += f"- **{name}**:\n"
        for issue in issues:
            md += f"  - {issue}\n"
        md += "\n"
    
    # Full dependency table
    md += """
---

## 5. Dependencias Completas por Paquete

"""
    
    for category, pkgs in sorted(categorized.items()):
        md += f"### {category}\n\n"
        
        for pkg in sorted(pkgs, key=lambda x: x['name']):
            md += f"#### {pkg['name']} (`{pkg['version']}`)\n\n"
            md += f"**Descripción**: {pkg['description']}\n\n"
            md += f"**Ubicación**: `{pkg['filepath']}`\n\n"
            md += f"**Build Type**: `{pkg['build_type']}`\n\n"
            
            for dep_type, dep_list in pkg['dependencies'].items():
                if dep_list:
                    md += f"- **{dep_type}**: {', '.join(dep_list)}\n"
            
            md += "\n"
    
    # Footer
    md += """
---

*Reporte generado automáticamente por analyze_packages.py*
"""
    
    return md


def main():
    """Función principal."""
    workspace_root = os.path.expanduser('~/dobot_ws')
    src_dir = os.path.join(workspace_root, 'src')
    
    print(f"Buscando package.xml en: {src_dir}")
    
    # Find all package.xml files
    package_files = find_package_xml_files(src_dir)
    print(f"Encontrados {len(package_files)} archivos package.xml")
    
    # Parse each package.xml
    packages = []
    for filepath in package_files:
        pkg_info = parse_package_xml(filepath)
        packages.append(pkg_info)
    
    # Sort by name
    packages.sort(key=lambda x: x['name'])
    
    # Generate markdown report
    report = generate_markdown_report(packages)
    
    # Save to file
    output_file = os.path.join(workspace_root, 'DEPENDENCIES_ANALYSIS.md')
    with open(output_file, 'w') as f:
        f.write(report)
    
    print(f"\nReporte guardado en: {output_file}")
    
    # Print summary
    print("\n=== RESUMEN ===")
    print(f"Total de paquetes analizados: {len(packages)}")
    
    # Count by ROS version
    ros_versions = defaultdict(int)
    for pkg in packages:
        ros_versions[pkg['ros_version']] += 1
    
    print("\nVersiones ROS2 detectadas:")
    for version, count in sorted(ros_versions.items()):
        print(f"  {version}: {count}")
    
    # Categories
    categories = defaultdict(int)
    for pkg in packages:
        cat = categorize_package(pkg['name'], pkg['dependencies'])
        categories[cat] += 1
    
    print("\nCategorías de paquetes:")
    for cat, count in sorted(categories.items()):
        print(f"  {cat}: {count}")


if __name__ == '__main__':
    main()
