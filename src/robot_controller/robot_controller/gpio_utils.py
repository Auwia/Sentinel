import os

def is_raspberry_pi():
    """Verifica se il sistema è un Raspberry Pi basandosi su /proc/device-tree/model."""
    try:
        if os.path.exists("/proc/device-tree/model"):
            with open("/proc/device-tree/model", "r") as f:
                return "raspberry pi" in f.read().lower()
        return False
    except Exception:
        return False

class MockGPIO:
    BCM = OUT = HIGH = LOW = None

    def __init__(self):
        self.pin_states = {}

    def setmode(self, *args): pass
    def setup(self, pin, mode, initial=None):
        self.pin_states[pin] = initial
    def output(self, pin, state):
        self.pin_states[pin] = state
    def input(self, pin):
        return self.pin_states.get(pin, None)
    def cleanup(self): pass

def load_gpio():
    """Carica RPi.GPIO se disponibile, altrimenti utilizza MockGPIO."""
    if is_raspberry_pi():
        try:
            import RPi.GPIO as GPIO
            print("RPi.GPIO correttamente caricato.")
            return GPIO
        except ImportError:
            pass
    print("Warning: Utilizzando MockGPIO (nessun controllo reale).")
    return MockGPIO()

