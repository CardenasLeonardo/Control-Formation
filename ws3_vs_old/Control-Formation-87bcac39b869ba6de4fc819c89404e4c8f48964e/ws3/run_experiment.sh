#!/bin/bash
set -e

# Uso: ./run_experiment.sh <nombre> [launch_file]
# Ejemplo: ./run_experiment.sh consenso_lider consenso_lider.launch.py

NAME=${1:-"experimento"}
LAUNCH=${2:-"consenso_lider.launch.py"}

WS_DIR="$(cd "$(dirname "$0")" && pwd)"
TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)
EXP_DIR="${WS_DIR}/experiments/${TIMESTAMP}_${NAME}"
FIG_DIR="${EXP_DIR}/figures"

mkdir -p "${FIG_DIR}"

# Guardar parámetros del experimento
cat > "${EXP_DIR}/config.yaml" << EOF
name: ${NAME}
timestamp: ${TIMESTAMP}
launch: ${LAUNCH}
extra_args: ${@:3}
EOF

# Crear notas vacías para llenar después
cat > "${EXP_DIR}/notes.md" << EOF
# ${NAME} — ${TIMESTAMP}

## Descripción


## Observaciones


## Conclusiones

EOF

echo ""
echo "============================================"
echo "  Experimento: ${NAME}"
echo "  Directorio:  ${EXP_DIR}"
echo "  Launch:      ${LAUNCH}"
echo "============================================"
echo "  Presiona Ctrl+C para terminar."
echo "  Las figuras se guardan automáticamente."
echo "============================================"
echo ""

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"

# Los plotters leen esta variable para saber dónde guardar
export EXP_SAVE_DIR="${FIG_DIR}"

# Lanzar simulación — pasa argumentos extra si los hay (ej: n_robots:=5)
ros2 launch simulador "${LAUNCH}" ${@:3}

echo ""
echo "Figuras guardadas en: ${FIG_DIR}"
echo "Notas en:             ${EXP_DIR}/notes.md"