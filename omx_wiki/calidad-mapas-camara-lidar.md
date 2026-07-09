# Calidad de mapas cámara–LiDAR

Category: reference
Tags: mapping, calidad, realsense, lidar, fused, parametros
Updated: 2026-07-09

Esta página resume cómo usar las mejoras de calidad añadidas al pipeline de mapeo con LiDAR + RealSense D435. La idea central es no “limpiar por apariencia”: el perfil `baseline` conserva el comportamiento actual, mientras que `quality` activa filtros/gates reversibles y deja evidencia diagnóstica separada.

Relacionado: [[index]]

## Contrato de productos

El acumulador separa diagnóstico y producto final:

- `/accumulated_camera_diagnostic_points`: frames de cámara base-válidos tras rango/TF. Si un frame se rechaza por filtro o keyframe, sigue visible aquí.
- `/accumulated_camera_points`: cámara aceptada por los quality gates.
- `/accumulated_points`: fused de calidad, usando LiDAR + cámara aceptada.

Al guardar mapa se generan:

```text
*_lidar.pcd
*_camera.pcd
*_camera_diagnostic.pcd
*_fused.pcd
*_fused_quality.pcd
*_manifest.json
```

El manifiesto `*_manifest.json` se escribe al final e incluye `snapshot_id`, parámetros, contadores y hashes. Si falta el manifiesto, trata los PCDs como snapshot parcial.

## Uso rápido

Perfil estable actual:

```bash
./catkin_ws/scripts/start_lidar_mapping.sh
```

Perfil de calidad:

```bash
MAPPING_PROFILE=quality ./catkin_ws/scripts/start_lidar_mapping.sh
```

Guardar mapa:

```bash
./catkin_ws/scripts/save_accumulated_map.sh
```

Ver contadores y razones de rechazo:

```bash
tail -f agv_mapping/logs/accumulator.log
```

## Parámetros principales

| Parámetro | Default baseline | Default quality | Uso |
|---|---:|---:|---|
| `MAPPING_PROFILE` | `baseline` | `quality` | Selecciona perfil operativo. |
| `REALSENSE_DEPTH_WIDTH` | `640` | `848` | Resolución depth. |
| `REALSENSE_DEPTH_HEIGHT` | `480` | `480` | Resolución depth. |
| `REALSENSE_DEPTH_FPS` | `6` | `15` | FPS depth. Subir reduce desfase entre frames si USB/CPU aguanta. |
| `REALSENSE_COLOR_WIDTH` | `640` | `848` | Resolución RGB. |
| `REALSENSE_COLOR_HEIGHT` | `480` | `480` | Resolución RGB. |
| `REALSENSE_COLOR_FPS` | `6` | `15` | FPS RGB. |
| `REALSENSE_FILTERS` | vacío | `decimation,spatial` | Filtros librealsense conservadores. Temporal queda apagado por riesgo de estelas. |
| `CAMERA_MIN_RANGE` | `0.20` | `0.20` | Recorte mínimo de cámara en metros. |
| `CAMERA_MAX_RANGE` | `5.0` | `3.0` | Recorte máximo; D435 suele degradar mucho con rango. |
| `CAMERA_OUTLIER_FILTER` | `none` | `sor` | Filtro PCL por frame: `none`, `sor` o `ror`. |
| `CAMERA_SOR_MEAN_K` | `24` | `24` | Vecinos para Statistical Outlier Removal. |
| `CAMERA_SOR_STDDEV_MUL` | `1.0` | `1.0` | Umbral SOR. Más bajo filtra más agresivo. |
| `CAMERA_ROR_RADIUS` | `0.08` | `0.08` | Radio en metros para Radius Outlier Removal. |
| `CAMERA_ROR_MIN_NEIGHBORS` | `3` | `3` | Vecinos mínimos ROR. |
| `CAMERA_MIN_POINTS` | `30` | `30` | Rechaza frames demasiado pobres. |
| `CAMERA_KEYFRAME_MIN_TRANSLATION` | `0.0` | `0.05` | Distancia mínima entre keyframes aceptados. |
| `CAMERA_KEYFRAME_MIN_ROTATION_DEG` | `0.0` | `3.0` | Rotación mínima entre keyframes aceptados. |
| `CAMERA_SYNC_TOLERANCE` | `0.03` | `0.03` | Tolerancia en segundos para sincronización RGB-D aligned-depth. |
| `USE_LATEST_TF_ON_FAILURE` | `false` | `false` | Debe seguir en `false`; usar latest TF contamina geometría. |

Todos son reversibles por variable de entorno. Ejemplo:

```bash
MAPPING_PROFILE=quality \
CAMERA_MAX_RANGE=2.5 \
CAMERA_OUTLIER_FILTER=ror \
CAMERA_ROR_RADIUS=0.06 \
CAMERA_ROR_MIN_NEIGHBORS=4 \
./catkin_ws/scripts/start_lidar_mapping.sh
```

## Recomendaciones de uso

1. Empieza con `baseline` y guarda un mapa de referencia.
2. Ejecuta `quality` sin cambiar nada más.
3. Compara visualmente `*_camera_diagnostic.pcd` contra `*_camera.pcd`:
   - si diagnostic está sucio pero camera mejora, los gates están ayudando;
   - si ambos están desplazados, probablemente es TF/tiempo/extrínseca, no ruido;
   - si LiDAR también se ve duplicado, revisar LeGO-LOAM/pose antes de tocar cámara.
4. No actives filtro temporal de RealSense como default hasta probar giros; puede crear estelas.
5. No subas voxel solo para ocultar fantasmas: primero corrige timestamp, TF, extrínseca o keyframes.

## Captura reproducible B0

Para medir mejoras sin depender solo de RViz:

```bash
SCENARIO=S4_recta_pared \
SPLIT=training \
DURATION_SEC=60 \
./catkin_ws/scripts/capture_mapping_dataset.sh
```

El script crea un rosbag y un manifiesto con commit, tópicos, calibración, hash y configuración resuelta. Usa al menos tres capturas baseline por escenario para estimar variabilidad.

Escenarios mínimos recomendados:

- plano estático a ~1 m, ~2 m y ~3 m;
- recta lenta junto a pared;
- giro horario y antihorario;
- circuito cerrado corto.

## Evaluación offline

Evaluar un PCD con un plano congelado:

```bash
rosrun scout_pointcloud_accumulator mapping_quality_eval.py \
  /ruta/map_camera.pcd \
  --sensor-origin camera \
  --plane 0 0 1 -1 \
  --output-json /tmp/camera_eval.json \
  --output-csv /tmp/camera_eval.csv
```

Métricas útiles:

- `finite_points`: puntos válidos.
- `range_p50_m`, `range_p95_m`: distribución de rango.
- `plane_p95_m`: distancia P95 al plano.
- `plane_thickness_m`: grosor robusto del plano.

## Criterio práctico de promoción

Promueve `quality` solo si:

- baja el grosor/ruido de cámara más que la variabilidad baseline;
- no cae la cobertura útil de forma fuerte;
- no aparecen estelas en giros;
- `USE_LATEST_TF_ON_FAILURE=false`;
- el manifiesto existe y los contadores explican frames aceptados/rechazados.

Si la cámara no aporta geometría nueva frente a LiDAR, preferir LiDAR como geometría y usar cámara solo para color/diagnóstico.
