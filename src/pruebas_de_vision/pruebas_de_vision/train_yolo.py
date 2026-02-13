#!/usr/bin/env python3
"""
=================================================================
Entrenamiento Robusto de YOLOv8-OBB para Detección de Cajas
=================================================================
Dataset: Kartonger (cajas de cartón) desde Roboflow

Uso:
    python3 train_yolo.py

Requisitos:
    pip install ultralytics roboflow
=================================================================
"""

import os
import sys
from pathlib import Path

def install_dependencies():
    """Instala las dependencias necesarias."""
    import subprocess
    
    packages = ['ultralytics', 'roboflow']
    
    for pkg in packages:
        try:
            __import__(pkg)
            print(f"✓ {pkg} ya está instalado")
        except ImportError:
            print(f"Instalando {pkg}...")
            subprocess.check_call([sys.executable, '-m', 'pip', 'install', pkg])
            print(f"✓ {pkg} instalado correctamente")


def download_dataset():
    """Descarga el dataset desde Roboflow."""
    from roboflow import Roboflow
    
    print("\n" + "="*60)
    print("DESCARGANDO DATASET DESDE ROBOFLOW")
    print("="*60)
    
    # Inicializar Roboflow con tu API key
    rf = Roboflow(api_key="c1YaLJpaj0Gege3lLmCS")
    
    # Obtener proyecto y versión
    project = rf.workspace("jugos").project("kartonger-8b52f")
    version = project.version(1)
    
    # Descargar en formato YOLOv8-OBB
    dataset = version.download("yolov8-obb")
    
    print(f"\n✓ Dataset descargado en: {dataset.location}")
    return dataset.location


def train_model(dataset_path: str, epochs: int = 100, imgsz: int = 640):
    """
    Entrena el modelo YOLOv8-OBB con configuración robusta.
    
    Args:
        dataset_path: Ruta al dataset descargado
        epochs: Número de épocas (default: 100)
        imgsz: Tamaño de imagen (default: 640)
    """
    from ultralytics import YOLO
    
    print("\n" + "="*60)
    print("ENTRENAMIENTO YOLOv8-OBB - CONFIGURACIÓN ROBUSTA")
    print("="*60)
    
    # Buscar el archivo data.yaml
    data_yaml = Path(dataset_path) / "data.yaml"
    if not data_yaml.exists():
        # Buscar en subdirectorios
        for f in Path(dataset_path).rglob("data.yaml"):
            data_yaml = f
            break
    
    print(f"\nArchivo de configuración: {data_yaml}")
    
    # Cargar modelo preentrenado (transfer learning)
    # YOLOv8l es más grande y preciso que YOLOv8n/s/m
    model = YOLO('yolov8l-obb.pt')  # Large model para mejor precisión
    
    # Configuración de entrenamiento robusta
    results = model.train(
        data=str(data_yaml),
        
        # Parámetros de entrenamiento principales
        epochs=epochs,              # Más épocas para mejor convergencia
        imgsz=imgsz,                # Tamaño de imagen
        batch=16,                   # Batch size (ajustar según GPU)
        
        # Optimización
        optimizer='AdamW',          # Optimizador AdamW (mejor que SGD para OBB)
        lr0=0.001,                  # Learning rate inicial
        lrf=0.01,                   # Learning rate final (lr0 * lrf)
        momentum=0.937,             # Momentum
        weight_decay=0.0005,        # Regularización L2
        
        # Augmentación de datos robusta
        hsv_h=0.015,                # Augmentación HSV - Hue
        hsv_s=0.7,                  # Augmentación HSV - Saturation
        hsv_v=0.4,                  # Augmentación HSV - Value
        degrees=45.0,               # Rotación aleatoria (±45°)
        translate=0.1,              # Traslación aleatoria
        scale=0.5,                  # Escala aleatoria
        shear=0.0,                  # Shear
        perspective=0.0005,         # Perspectiva
        flipud=0.0,                 # Flip vertical (no para cajas)
        fliplr=0.5,                 # Flip horizontal
        mosaic=1.0,                 # Mosaic augmentación
        mixup=0.1,                  # Mixup augmentación
        copy_paste=0.1,             # Copy-paste augmentación
        
        # Parámetros de pérdida
        box=7.5,                    # Peso de pérdida de caja
        cls=0.5,                    # Peso de pérdida de clasificación
        dfl=1.5,                    # Peso de distribución focal loss
        
        # Guardado y logging
        project='runs/obb',
        name='kartonger_yolov8l',
        exist_ok=True,
        save=True,
        save_period=10,             # Guardar cada 10 épocas
        plots=True,                 # Generar gráficos
        val=True,                   # Validar durante entrenamiento
        
        # Hardware
        device=0,                   # GPU (0) o CPU ('cpu')
        workers=8,                  # Workers para data loading
        amp=True,                   # Automatic Mixed Precision
        
        # Early stopping
        patience=50,                # Early stopping después de 50 épocas sin mejora
        
        # Misc
        verbose=True,
        seed=42,                    # Reproducibilidad
    )
    
    print("\n" + "="*60)
    print("ENTRENAMIENTO COMPLETADO")
    print("="*60)
    
    return results


def export_model(model_path: str = None):
    """Exporta el modelo a formato ONNX para inferencia rápida."""
    from ultralytics import YOLO
    
    # Buscar el mejor modelo
    if model_path is None:
        runs_dir = Path('runs/obb')
        best_models = list(runs_dir.rglob('best.pt'))
        if best_models:
            model_path = str(best_models[-1])
        else:
            print("No se encontró modelo entrenado")
            return None
    
    print(f"\nExportando modelo: {model_path}")
    
    model = YOLO(model_path)
    
    # Exportar a ONNX
    onnx_path = model.export(
        format='onnx',
        imgsz=640,
        simplify=True,
        opset=12,
    )
    
    print(f"✓ Modelo exportado a: {onnx_path}")
    return onnx_path


def validate_model(model_path: str = None, dataset_path: str = None):
    """Valida el modelo en el conjunto de test."""
    from ultralytics import YOLO
    
    # Buscar modelo
    if model_path is None:
        runs_dir = Path('runs/obb')
        best_models = list(runs_dir.rglob('best.pt'))
        model_path = str(best_models[-1]) if best_models else None
    
    if model_path is None:
        print("No se encontró modelo para validar")
        return
    
    print(f"\nValidando modelo: {model_path}")
    
    model = YOLO(model_path)
    
    # Validar
    metrics = model.val(
        data=dataset_path,
        split='test',
        imgsz=640,
        batch=16,
        conf=0.25,
        iou=0.5,
        device=0,
        plots=True,
    )
    
    print("\n" + "="*60)
    print("MÉTRICAS DE VALIDACIÓN")
    print("="*60)
    print(f"mAP50: {metrics.box.map50:.4f}")
    print(f"mAP50-95: {metrics.box.map:.4f}")
    print(f"Precision: {metrics.box.mp:.4f}")
    print(f"Recall: {metrics.box.mr:.4f}")
    
    return metrics


def main():
    """Función principal."""
    print("="*60)
    print("ENTRENAMIENTO YOLOv8-OBB - DETECCIÓN DE CAJAS")
    print("="*60)
    
    # 1. Instalar dependencias
    install_dependencies()
    
    # 2. Descargar dataset
    dataset_path = download_dataset()
    
    # 3. Entrenar modelo
    results = train_model(dataset_path, epochs=100, imgsz=640)
    
    # 4. Validar modelo
    validate_model(dataset_path=dataset_path)
    
    # 5. Exportar a ONNX
    export_model()
    
    print("\n" + "="*60)
    print("PROCESO COMPLETADO")
    print("="*60)
    print("\nModelos guardados en: runs/obb/kartonger_yolov8l/")
    print("  - best.pt   : Mejor modelo (para inferencia)")
    print("  - last.pt   : Último checkpoint")
    print("  - best.onnx : Modelo exportado ONNX")


if __name__ == '__main__':
    main()
