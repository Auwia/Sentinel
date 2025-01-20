import rclpy
from rclpy.node import Node
from custom_interfaces.srv import SetServoAngle, PumpControl, ValveControl
import tkinter as tk
from tkinter import ttk
from threading import Thread
import json
import os

POSITIONS_FILE = "src/robot_controller/robot_controller/share/servo_positions.json"

class Cruscotto(Node):
    def __init__(self):
        super().__init__('cruscotto')
        self.get_logger().info("Cruscotto avviato.")

        # ROS 2 Service Clients
        self.servo_service_client = self.create_client(SetServoAngle, '/servo_control_service')
        self.pump_client = self.create_client(PumpControl, '/pump_control')
        self.valve_client = self.create_client(ValveControl, '/valve_control')

        # Flag di emergenza
        self.emergency_flag = False

        # Tkinter GUI setup
        self.root = tk.Tk()
        self.root.title("Cruscotto GUI - Service")

        # GUI Elements
        self.create_gui_elements()

        # Carica la posizione attuale dei motori
        self.get_logger().info("Caricamento della posizione attuale dei motori...")
        self.load_motor_positions()

    def create_gui_elements(self):
        # Frame per i motori
        motors_frame = ttk.LabelFrame(self.root, text="Motori")
        motors_frame.grid(row=0, column=0, padx=10, pady=10)

        self.motor_sliders = {}
        self.motor_labels = {}
        self.speed_sliders = {}
        self.speed_labels = {}

        for motor_id in range(3):
            lbl = ttk.Label(motors_frame, text=f"Motore {motor_id}")
            lbl.grid(row=motor_id * 2, column=0, padx=5, pady=5)

            slider = ttk.Scale(
                motors_frame,
                from_=0,
                to=180,
                orient="horizontal",
                command=lambda val, m=motor_id: self.update_motor_label(m, val)
            )
            slider.grid(row=motor_id * 2, column=1, padx=5, pady=5)
            self.motor_sliders[motor_id] = slider

            angle_label = ttk.Label(motors_frame, text="0°")
            angle_label.grid(row=motor_id * 2, column=2, padx=5, pady=5)
            self.motor_labels[motor_id] = angle_label

            speed_slider = ttk.Scale(
                motors_frame,
                from_=1,
                to=100,
                orient="horizontal",
                command=lambda val, m=motor_id: self.update_speed_label(m, val)
            )
            speed_slider.grid(row=motor_id * 2 + 1, column=1, padx=5, pady=5)
            self.speed_sliders[motor_id] = speed_slider

            speed_label = ttk.Label(motors_frame, text="1")
            speed_label.grid(row=motor_id * 2 + 1, column=2, padx=5, pady=5)
            self.speed_labels[motor_id] = speed_label

            btn = ttk.Button(motors_frame, text="Invia", command=lambda m=motor_id: Thread(target=self.send_motor_command, args=(m,)).start())
            btn.grid(row=motor_id * 2, column=3, rowspan=2, padx=5, pady=5)

        # Frame per pompa e valvola
        controls_frame = ttk.LabelFrame(self.root, text="Controlli")
        controls_frame.grid(row=1, column=0, padx=10, pady=10)

        # Bottone Pompa
        self.pump_btn = tk.Button(
            controls_frame,
            text="Pump",
            bg="red",
            fg="white",
            width=10,
            height=2,
            command=lambda: Thread(target=self.toggle_pump).start()
        )
        self.pump_btn.grid(row=0, column=0, padx=10, pady=10)

        # Bottone Valvola
        self.valve_btn = tk.Button(
            controls_frame,
            text="Valve",
            bg="red",
            fg="white",
            width=10,
            height=2,
            command=lambda: Thread(target=self.toggle_valve).start()
        )
        self.valve_btn.grid(row=0, column=1, padx=10, pady=10)

        # Pulsante di emergenza
        emergency_btn = tk.Button(
            controls_frame,
            text="EMERGENZA",
            bg="orange",
            fg="black",
            width=20,
            height=2,
            command=lambda: Thread(target=self.emergency_stop).start()
        )
        emergency_btn.grid(row=1, column=0, columnspan=2, pady=10)

        # Pulsante Prendi
        prendi_btn = tk.Button(
            controls_frame,
            text="Prendi",
            bg="blue",
            fg="white",
            width=20,
            height=2,
            command=lambda: Thread(target=self.execute_prendi).start()
        )
        prendi_btn.grid(row=2, column=0, columnspan=2, pady=10)

        # Console log
        log_frame = ttk.LabelFrame(self.root, text="Console")
        log_frame.grid(row=2, column=0, padx=10, pady=10)

        self.log_text = tk.Text(log_frame, height=10, width=50)
        self.log_text.grid(row=0, column=0, padx=5, pady=5)

    def update_motor_label(self, motor_id, value):
        angle = round(float(value))
        self.motor_labels[motor_id].config(text=f"{angle}°")

    def update_speed_label(self, motor_id, value):
        speed = round(float(value))
        self.speed_labels[motor_id].config(text=f"{speed}")

    def send_motor_command(self, motor_id):
        try:
            angle = round(float(self.motor_sliders[motor_id].get()))
            speed = round(float(self.speed_sliders[motor_id].get()))
            success = self.send_motor_request(motor_id, angle, speed)
            if success:
                self.log(f"Motore {motor_id} impostato su angolo {angle} con velocità {speed}")
                self.save_motor_position(motor_id, angle)  # Salva la posizione
            else:
                self.log(f"Errore durante l'impostazione del motore {motor_id}")
        except Exception as e:
            self.log(f"Errore durante l'invio del comando al motore {motor_id}: {e}")

    def send_motor_request(self, motor_id, target_angle, target_speed, servo_type=180):
        if self.emergency_flag:
            return False  # Interrompi se emergenza attiva

        # Preparazione della richiesta per il servizio
        req = SetServoAngle.Request()
        req.motor_id = motor_id
        req.target_angle = float(target_angle)
        req.target_speed = float(target_speed)
        req.servo_type = int(servo_type)

        if not self.servo_service_client.wait_for_service(timeout_sec=10.0):
            self.log("Servizio '/servo_control_service' non disponibile.")
            return False

        future = self.servo_service_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        if result and result.success:
            return True
        else:
            self.log(f"Errore motore {motor_id}: {result.status_message if result else 'Nessuna risposta'}")
            return False

    def execute_prendi(self):
        """
        Invoca la sequenza 'prendi' sul server dei motori.
        """
        self.log("Esecuzione della sequenza 'prendi' iniziata...")
        try:
            self.send_motor_request(0, 90, 10)  # Motore 0: posizione iniziale
            self.send_motor_request(1, 45, 10)  # Motore 1: posizione iniziale
            self.send_motor_request(2, 10, 10)  # Motore 2: posizione iniziale

            # Movimento intermedio
            for angle_1, angle_2 in zip(range(45, 35, -1), range(0, 70, +1)):
                self.send_motor_request(1, angle_1, 10)
                self.send_motor_request(2, angle_2, 10)

            # Posizioni finali
            self.send_motor_request(0, 90, 10)
            self.send_motor_request(1, 25, 10)
            self.send_motor_request(2, 91, 10)

            self.log("Sequenza 'prendi' completata.")
        except Exception as e:
            self.log(f"Errore durante l'esecuzione di 'prendi': {e}")

    def toggle_pump(self):
        # Determina lo stato attuale del pulsante (ON o OFF)
        current_state = self.pump_btn["bg"] == "green"
        new_state = not current_state  # Inverte lo stato
    
        # Prepara la richiesta per il servizio PumpControl
        req = PumpControl.Request()
        req.turn_on = new_state
    
        # Controlla se il servizio è disponibile
        if not self.pump_client.wait_for_service(timeout_sec=10.0):
            self.log("Servizio '/pump_control' non disponibile.")
            return
    
        # Invia la richiesta
        future = self.pump_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
    
        # Aggiorna lo stato del pulsante in base al risultato
        if result and result.success:
            self.update_pump_button(new_state)
            self.log(f"Pompa {'ON' if new_state else 'OFF'}: {result.message}")
        else:
            self.log(f"Errore pompa: {result.message if result else 'Nessuna risposta'}")
    
    def update_pump_button(self, state):
        # Cambia il colore del pulsante in base allo stato
        color = "green" if state else "red"
        self.pump_btn.config(bg=color)

    def toggle_valve(self):
        # Determina lo stato attuale del pulsante (ON o OFF)
        current_state = self.valve_btn["bg"] == "green"
        new_state = not current_state  # Inverte lo stato
    
        # Prepara la richiesta per il servizio ValveControl
        req = ValveControl.Request()
        req.turn_on = new_state
    
        # Controlla se il servizio è disponibile
        if not self.valve_client.wait_for_service(timeout_sec=10.0):
            self.log("Servizio '/valve_control' non disponibile.")
            return
    
        # Invia la richiesta
        future = self.valve_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
    
        # Aggiorna lo stato del pulsante in base al risultato
        if result and result.success:
            self.update_valve_button(new_state)
            self.log(f"Valvola {'ON' if new_state else 'OFF'}: {result.message}")
        else:
            self.log(f"Errore valvola: {result.message if result else 'Nessuna risposta'}")
    
    def update_valve_button(self, state):
        # Cambia il colore del pulsante in base allo stato
        color = "green" if state else "red"
        self.valve_btn.config(bg=color)

    def emergency_stop(self):
        self.log("EMERGENZA ATTIVATA: Arresto di tutti i movimenti.")
        self.emergency_flag = True  # Attiva la flag di emergenza
    
        # Arresta i motori
        for motor_id in range(3):
            try:
                self.log(f"Inviando comando di emergenza al motore {motor_id}: angolo=90, velocità=0")
                self.save_motor_position(motor_id, self.motor_sliders[motor_id].get())  # Salva la posizione attuale
                self.send_motor_request(motor_id, 0, 0)  # Posiziona i motori a 90° (posizione neutra)
            except Exception as e:
                self.log(f"Errore durante l'arresto del motore {motor_id}: {e}")
    
        # Arresta la pompa
        try:
            self.set_pump_state(False)
            self.update_pump_button(False)
        except Exception as e:
            self.log(f"Errore durante lo spegnimento della pompa: {e}")
    
        # Arresta la valvola
        try:
            self.set_valve_state(False)
            self.update_valve_button(False)
        except Exception as e:
            self.log(f"Errore durante la chiusura della valvola: {e}")
    
        self.log("Emergenza completata.")
        self.emergency_flag = False  # Resetta la flag di emergenza

    # Funzioni per salvare/caricare la posizione dei motori
    def save_motor_position(self, motor_id, angle):
        positions = self.load_motor_positions()
        positions[str(motor_id)] = angle
        with open(POSITIONS_FILE, 'w') as f:
            json.dump(positions, f)

    def load_motor_positions(self):
        if os.path.exists(POSITIONS_FILE):
            with open(POSITIONS_FILE, 'r') as f:
                positions = json.load(f)
                for motor_id, angle in positions.items():
                    if int(motor_id) in self.motor_sliders:
                        self.motor_sliders[int(motor_id)].set(angle)
                        self.update_motor_label(int(motor_id), angle)
                return positions
        else:
            return {str(i): 90 for i in range(3)}  # Default a 90°

    def log(self, message):
        self.log_text.insert(tk.END, f"{message}\n")
        self.log_text.see(tk.END)
        self.get_logger().info(message)

    def run(self):
        self.log("Cruscotto in esecuzione.")
        self.root.mainloop()

    def set_pump_state(self, state):
        req = PumpControl.Request()
        req.turn_on = state
    
        if not self.pump_client.wait_for_service(timeout_sec=10.0):
            self.log("Servizio '/pump_control' non disponibile.")
            return False
    
        future = self.pump_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
    
        if result and result.success:
            self.log(f"Pompa {'ON' if state else 'OFF'}: {result.message}")
            return True
        else:
            self.log(f"Errore pompa: {result.message if result else 'Nessuna risposta'}")
            return False
    
    def set_valve_state(self, state):
        req = ValveControl.Request()
        req.turn_on = state
    
        if not self.valve_client.wait_for_service(timeout_sec=10.0):
            self.log("Servizio '/valve_control' non disponibile.")
            return False
    
        future = self.valve_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
    
        if result and result.success:
            self.log(f"Valvola {'ON' if state else 'OFF'}: {result.message}")
            return True
        else:
            self.log(f"Errore valvola: {result.message if result else 'Nessuna risposta'}")
            return False

def main():
    rclpy.init()
    gui_node = Cruscotto()
    try:
        gui_node.run()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

