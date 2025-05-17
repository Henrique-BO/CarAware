#!/bin/bash

# Default values
RUN_MAIN=false
MAIN_MODE=""
RUN_MODEL_SERVER=false
MODEL_PATH=""
WAIT_FOR_CARLA=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --run-main)
            RUN_MAIN=true
            MAIN_MODE="$2"
            shift 2
            ;;
        --run-model-server)
            RUN_MODEL_SERVER=true
            MODEL_PATH="$2"
            shift 2
            ;;
        --wait-for-carla)
            WAIT_FOR_CARLA=true
            shift
            ;;
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done

if $WAIT_FOR_CARLA; then
    echo "[TF Entrypoint] Waiting for CARLA to become available..."
    python3 wait_for_carla.py
fi

if $RUN_MODEL_SERVER; then
    echo "[TF Entrypoint] Launching model server with model: $MODEL_PATH"
    python3 rl/serve_model.py --model "$MODEL_PATH" 2>&1 | sed 's/^/[Model Server] /' &
    MODEL_SERVER_PID=$!
fi

if $RUN_MAIN; then
    echo "[TF Entrypoint] Running main.py in mode: $MAIN_MODE"
    python3 main.py "$MAIN_MODE" 2>&1 | sed 's/^/[CarAware] /' &
    MAIN_PID=$!
fi

# Wait for background processes to finish
if $RUN_MODEL_SERVER; then
    wait $MODEL_SERVER_PID
fi

if $RUN_MAIN; then
    wait $MAIN_PID
fi
