#!/bin/bash

# Script di build per il progetto ROS 2

# Configura l'ambiente del progetto
echo "Configurazione dell'ambiente..."
source /opt/ros/jazzy/setup.bash

# Verifica se esiste un workspace ROS 2 valido
WORKSPACE_DIR="$(pwd)"
SRC_DIR="$WORKSPACE_DIR/src"

if [ ! -d "$SRC_DIR" ]; then
    echo "Errore: Non è stato trovato un workspace ROS 2 valido. Assicurati che esista la directory 'src'."
    exit 1
fi

# Pulisce build, install e log (opzionale)
echo "Pulizia delle directory build, install e log..."
rm -rf "$WORKSPACE_DIR/build" "$WORKSPACE_DIR/install" "$WORKSPACE_DIR/log"

# Compila i pacchetti del progetto
echo "Compilazione dei pacchetti ROS 2..."
colcon build --symlink-install 

# Controlla il risultato della build
if [ $? -ne 0 ]; then
    echo "Errore durante la compilazione dei pacchetti."
    exit 1
fi

# Sorgente dell'ambiente di lavoro compilato
echo "Sorgente dell'ambiente di lavoro compilato..."
source "$WORKSPACE_DIR/install/setup.bash"

echo "Build completata con successo!"

