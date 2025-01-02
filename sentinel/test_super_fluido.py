import time
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# Configura il bus I2C
i2c = busio.I2C(SCL, SDA)

# Configura il PCA9685
pca = PCA9685(i2c, address=0x40)  # Usa l'indirizzo 0x40
pca.frequency = 50  # Frequenza PWM (50 Hz per i servo)

# Funzione per convertire il duty cycle in angoli
def duty_cycle_to_angle(duty_cycle, duty_min, duty_max):
    # Mappa il duty cycle in un range di angoli (0° - 180°)
    angle = (duty_cycle - duty_min) / (duty_max - duty_min) * 180
    return round(angle, 1)

try:
    print("Test dei gradi del servo sul canale 0...")
    
    # Range tipico di duty cycle per un servo (modifica se necessario):
    duty_min = 0x0666  # Circa 2.5% duty cycle (angolo minimo)
    duty_max = 0x2CCC  # Circa 12.5% duty cycle (angolo massimo)

    step = 0  # Contatore per il ciclo
    increment = 50  # Incrementi più piccoli per maggiore fluidità
    pause = 0.02  # Ridotto il tempo tra i passi per maggiore precisione

    # Movimento fluido: Incrementa il duty cycle gradualmente
    print("Movimento dal minimo al massimo...")
    for duty in range(duty_min, duty_max, increment):  # Incrementi di 50
        pca.channels[0].duty_cycle = duty
        angle = duty_cycle_to_angle(duty, duty_min, duty_max)
        step += 1
        if step % 20 == 0:  # Stampa ogni 20 passi per non rallentare
            print(f"Duty cycle: {hex(duty)}, Angolo: {angle}°")
        time.sleep(pause)  # Pausa tra i passi

    # Movimento fluido: Decrementa il duty cycle gradualmente
    print("Movimento dal massimo al minimo...")
    for duty in range(duty_max, duty_min, -increment):  # Decrementi di 50
        pca.channels[0].duty_cycle = duty
        angle = duty_cycle_to_angle(duty, duty_min, duty_max)
        step += 1
        if step % 20 == 0:  # Stampa ogni 20 passi per non rallentare
            print(f"Duty cycle: {hex(duty)}, Angolo: {angle}°")
        time.sleep(pause)  # Pausa tra i passi

    print("Test completato. Ritorno alla posizione centrale...")
    # Posiziona il servo al centro
    pca.channels[0].duty_cycle = 0x1999  # Circa 7.5% duty cycle (angolo centrale)
    print("Posizione centrale: 90°")
    time.sleep(1)

except KeyboardInterrupt:
    print("Test interrotto manualmente.")
finally:
    print("Spegnimento del canale 0...")
    pca.channels[0].duty_cycle = 0  # Spegni il canale 0
    print("Test completato.")
