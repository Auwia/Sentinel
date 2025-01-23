import time
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import SetServoAngle, PumpControl, ValveControl
import tkinter as tk
from tkinter import ttk
from threading import Thread
import json
import os
from rclpy.executors import MultiThreadedExecutor

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
        self.log("Caricamento della posizione attuale dei motori...")

        # Carica la posizione attuale dei motori
        self.motor_positions = self.load_motor_positions()

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

    def execute_prendi(self):
        """
        Sequenza 'prendi' con step intermedi e finali.
        """
        self.log("Esecuzione della sequenza 'prendi' iniziata...")
        try:
            motor_positions = self.load_motor_positions()

            # Sincronizza i motori con posizioni iniziali
            self.log("Sincronizzazione posizioni dei motori...")
            self.move_motors_in_parallel(self.motor_positions, speed=10, servo_type=180)

            # Step iniziale
            step_1 = {0: 132, 1: 45, 2: 10}
            self.log("Step 1: movimento iniziale...")
            self.move_motors_in_parallel(step_1, speed=10, servo_type=180)

            # Step intermedi (movimento graduale)
            self.log("Step 2: movimenti intermedi...")
            for pos_1, pos_2 in zip(range(45, 35, -1), range(10, 20)):
                self.move_motors_in_parallel({1: pos_1, 2: pos_2}, speed=10, servo_type=180)
                time.sleep(0.1) 

            # Step finale
            step_3 = {0: 132, 1: 25, 2: 92}
            self.log("Step 3: posizione finale...")
            self.move_motors_in_parallel(step_3, speed=10, servo_type=180)

            self.log("Sequenza 'prendi' completata.")
        except Exception as e:
            self.log(f"Errore durante l'esecuzione di 'prendi': {e}")

    # Le altre funzioni rimangono invariate...

    def load_motor_positions(self):
        """
        Carica le posizioni dei motori dal file. Se il file non esiste,
        restituisce una posizione di default.
        """
        if os.path.exists(POSITIONS_FILE):
            with open(POSITIONS_FILE, 'r') as f:
                positions = json.load(f)
                self.log(f"Posizioni motori caricate: {positions}")
    
                # Aggiorna i cursori con le posizioni caricate
                for motor_id, angle in positions.items():
                    if int(motor_id) in self.motor_sliders:
                        self.motor_sliders[int(motor_id)].set(angle)
                        self.update_motor_label(int(motor_id), angle)
                return {int(k): v for k, v in positions.items()}
        else:
            default_positions = {i: 90 for i in range(3)}  # Default a 90°
            self.log(f"File posizioni non trovato, uso default: {default_positions}")
            return default_positions

    def log(self, message):
        """Log su ROS 2 e aggiorna la GUI."""
        self.get_logger().info(message)  # Log per ROS 2
        if hasattr(self, "log_text"):  # Evita di chiamare log_text prima che sia inizializzato
            self.root.after(0, self._log_to_gui, message)

    def _log_to_gui(self, message):
        """Aggiunge un messaggio alla GUI della console."""
        self.log_text.insert(tk.END, f"{message}\n")
        self.log_text.see(tk.END)

    def update_motor_label(self, motor_id, value):
        """Aggiorna l'etichetta dell'angolo del motore."""
        angle = round(float(value))
        self.motor_labels[motor_id].config(text=f"{angle}°")

    def update_speed_label(self, motor_id, value):
        """Aggiorna l'etichetta della velocità del motore."""
        speed = round(float(value))
        self.speed_labels[motor_id].config(text=f"{speed}")

    def run(self):
        """
        Avvia il ciclo principale di Tkinter per la GUI.
        """
        self.log("Cruscotto in esecuzione.")
        try:
            self.root.mainloop()  # Questo avvia la GUI
        except KeyboardInterrupt:
            self.log("Interruzione della GUI (KeyboardInterrupt).")

    def move_motors_in_parallel(self, target_positions, speed=10, servo_type=180):
        """
        Muove i motori verso le posizioni target in parallelo, con un tipo di servo specifico.
        
        :param target_positions: Dizionario con {motor_id: target_angle}.
        :param speed: Velocità del movimento (default: 10).
        :param servo_type: Tipo di servo (default: 180).
        """
        self.log("Inizio movimento parallelo dei motori...")
        threads = []
        for motor_id, target_angle in target_positions.items():
            current_angle = self.motor_positions.get(motor_id, 0)
            if abs(current_angle - target_angle) < 0.01:
                self.log(f"Motore {motor_id}: già all'angolo {target_angle}°, nessun movimento necessario.")
                continue
            self.log(f"Motore {motor_id}: movimento da {current_angle}° a {target_angle}° a velocità {speed}°/s.")
            thread = Thread(target=self._move_motor, args=(motor_id, target_angle, speed, servo_type))
            thread.start()
            threads.append(thread)
        
        self.log("Movimento parallelo dei motori completato.")

    def _move_motor(self, motor_id, target_angle, speed, servo_type=180):
        """
        Muove un singolo motore verso la posizione target.
        :param motor_id: ID del motore.
        :param target_angle: Angolo target.
        :param speed: Velocità del movimento.
        """
        current_position = self.motor_positions.get(motor_id, 0)
        if current_position == target_angle:
            self.log(f"Motore {motor_id}: già all'angolo {current_position}°, nessun movimento necessario.")
            return
    
        self.log(f"Inizio movimento del motore {motor_id}: angolo={target_angle}°, velocità={speed}°/s, tipo servo={servo_type}.")
        success = self.send_motor_request(motor_id, target_angle, speed, servo_type)
        if success:
            self.log(f"Motore {motor_id} raggiunto: {target_angle}°.")
            self.motor_positions[motor_id] = target_angle  # Aggiorna la posizione attuale
        else:
            self.log(f"Errore nel movimento del motore {motor_id}.")

    def send_motor_request(self, motor_id, target_angle, target_speed, servo_type=180):
        """
        Invia una richiesta al servizio per controllare il motore.
        
        :param motor_id: ID del motore.
        :param target_angle: Angolo target da raggiungere.
        :param target_speed: Velocità con cui raggiungere l'angolo.
        :return: True se il comando è stato inviato con successo, altrimenti False.
        """
        if self.emergency_flag:
            self.log(f"Comando motore {motor_id} interrotto: emergenza attivata.")
            return False  # Interrompe se l'emergenza è attivata
        
        # Prepara la richiesta per il servizio
        req = SetServoAngle.Request()
        req.motor_id = motor_id
        req.target_angle = float(target_angle)
        req.target_speed = float(target_speed)
        req.servo_type = int(servo_type)
        
        # Controlla se il servizio è disponibile
        if not self.servo_service_client.wait_for_service(timeout_sec=10.0):
            self.log("Servizio '/servo_control_service' non disponibile.")
            return False
        
        # Invia la richiesta in modo asincrono
        future = self.servo_service_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_motor_response(f, motor_id, target_angle, target_speed))
        return True

    def _handle_motor_response(self, future, motor_id, target_angle, target_speed):
        """
        Callback per gestire la risposta del servizio.
        """
        try:
            result = future.result()
            if result.success:
                self.log(f"Motore {motor_id} raggiunto: {target_angle}° con velocità {target_speed}°/s.")
            else:
                self.log(f"Errore motore {motor_id}: {result.status_message} (Codice: {result.error_code})")
        except Exception as e:
            self.log(f"Errore nella risposta del motore {motor_id}: {e}")

def main():
    rclpy.init()
    gui_node = Cruscotto()

    executor = MultiThreadedExecutor()
    executor.add_node(gui_node)

    def spin_executor():
        try:
            executor.spin()
        finally:
            executor.shutdown()

    # Thread per gestire l'executor
    executor_thread = Thread(target=spin_executor, daemon=True)
    executor_thread.start()

    try:
        gui_node.run()
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()
        executor_thread.join()


if __name__ == "__main__":
    main()

