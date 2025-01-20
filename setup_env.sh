#!/bin/bash

echo "Creazione di un ambiente virtuale e installazione dei pacchetti richiesti..."

# Rimuovi e ricrea l'ambiente virtuale
rm -rf ~/sentinel_venv
python3 -m venv ~/sentinel_venv
source ~/sentinel_venv/bin/activate

# Aggiorna pip, setuptools e wheel per evitare problemi di compatibilità
echo "Aggiornamento di pip, setuptools e wheel..."
pip install --upgrade pip setuptools wheel

# Installa pacchetti necessari
echo "Installazione dei pacchetti Python richiesti..."
pip install pillow ttkwidgets empy==3.3.4 catkin_pkg pyyaml lark numpy

# Test installazioni
echo "Verifica dell'installazione dei moduli Python..."
if python3 -c "import catkin_pkg; import yaml; from PIL import Image, ImageTk; from ttkwidgets import Dial; import em; import numpy; from lark import Lark" &> /dev/null; then
    echo "Tutti i moduli Python richiesti sono stati installati correttamente."
else
    echo "Errore: problemi con l'importazione dei moduli Python."
    exit 1
fi

echo "Configurazione completata. Ora puoi eseguire il build del progetto con ./build.sh."

