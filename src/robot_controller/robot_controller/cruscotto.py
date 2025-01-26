import time
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import SetServoAngle, PumpControl, ValveControl, EmergencyStop, StopServo, GetServoPosition
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

        self.motor_positions = {}
        self.current_labels = {} 
        self.motor_logs = {i: [] for i in range(3)}
        self.motor_polling_flags = {motor_id: False for motor_id in range(3)} 

        # ROS 2 Service Clients
        self.servo_service_client = self.create_client(SetServoAngle, '/servo_control_service')
        self.pump_client = self.create_client(PumpControl, '/pump_control')
        self.valve_client = self.create_client(ValveControl, '/valve_control')
        self.stop_servo_client = self.create_client(StopServo, '/stop_servo')
        self.get_position_client = self.create_client(GetServoPosition, '/get_servo_position')

        # Flag di emergenza
        self.emergency_flag = False
        self.emergency_client = self.create_client(EmergencyStop, '/emergency_stop')  # Nuovo client

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
        # Frame per i motori (contenitore principale)
        motors_frame = ttk.LabelFrame(self.root, text="Motori")
        motors_frame.grid(row=0, column=0, padx=10, pady=10)
    
        self.motor_sliders = {}
        self.motor_labels = {}
        self.speed_sliders = {}
        self.speed_labels = {}
        self.motor_logs = {}
    
        for motor_id in range(3):  # Tre motori
            # Frame per ciascun motore
            motor_frame = ttk.LabelFrame(motors_frame, text=f"Motore {motor_id}")
            motor_frame.grid(row=0, column=motor_id, padx=10, pady=10)
    
            # Slider angolo
            angle_slider = ttk.Scale(
                motor_frame,
                from_=0,
                to=180,
                orient="horizontal"
            )
            angle_slider.set(self.motor_positions.get(motor_id, 0))  # Posizione iniziale
            angle_slider.grid(row=0, column=0, columnspan=2, padx=5, pady=5)
            self.motor_sliders[motor_id] = angle_slider  # Popola il dizionario
    
            # Etichetta angolo
            angle_label = ttk.Label(motor_frame, text="0\u00B0")
            angle_label.grid(row=1, column=0, columnspan=2, padx=5, pady=5)
            self.motor_labels[motor_id] = angle_label  # Popola il dizionario
    
            # Slider velocità
            speed_slider = ttk.Scale(
                motor_frame,
                from_=1,
                to=100,
                orient="horizontal"
            )
            speed_slider.set(10)  # Valore iniziale della velocità
            speed_slider.grid(row=2, column=0, columnspan=2, padx=5, pady=5)
            self.speed_sliders[motor_id] = speed_slider  # Popola il dizionario
    
            # Etichetta velocità
            speed_label = ttk.Label(motor_frame, text="10")
            speed_label.grid(row=3, column=0, columnspan=2, padx=5, pady=5)
            self.speed_labels[motor_id] = speed_label  # Popola il dizionario
    
            # Pulsante Invio
            btn_invia = ttk.Button(
                motor_frame,
                text="Invia",
                command=lambda m=motor_id: Thread(target=self.send_motor_command, args=(m,)).start()
            )
            btn_invia.grid(row=4, column=0, padx=5, pady=5)
    
            # Pulsante STOP
            btn_stop = ttk.Button(
                motor_frame,
                text="STOP",
                command=lambda m=motor_id: Thread(target=self.stop_motor, args=(m,)).start()
            )
            btn_stop.grid(row=4, column=1, padx=5, pady=5)
    
            # Log individuale per ciascun motore
            motor_log = tk.Text(motor_frame, height=5, width=30, state=tk.DISABLED, wrap=tk.WORD)
            motor_log.grid(row=5, column=0, columnspan=2, padx=5, pady=5)
            self.motor_logs[motor_id] = motor_log  # Popola il dizionario

            # Etichetta per mostrare la posizione corrente del motore
            current_label = ttk.Label(motor_frame, text="Corrente: 0°")
            current_label.grid(row=6, column=0, columnspan=2, padx=5, pady=5)
            self.current_labels[motor_id] = current_label  # Popola il dizionario delle label correnti
    
        # Aggiungere i callback solo dopo che i dizionari sono popolati
        for motor_id, slider in self.motor_sliders.items():
            slider.config(command=lambda val, m=motor_id: self.update_motor_label(m, val))
    
        for motor_id, slider in self.speed_sliders.items():
            slider.config(command=lambda val, m=motor_id: self.update_speed_label(m, val))
    
        # Frame per i controlli generali
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
    
        # Frame per il log generale
        log_frame = ttk.LabelFrame(self.root, text="Console")
        log_frame.grid(row=2, column=0, columnspan=3, padx=5, pady=5, sticky="ew")
    
        self.log_text = tk.Text(log_frame, height=15, width=100)
        self.log_text.grid(row=0, column=0, padx=5, pady=5)

    def stop_motor(self, motor_id):
        """Interrompe il movimento del motore."""
        self.log(f"Interruzione del motore {motor_id}.")

        # Prepara la richiesta per fermare il motore
        req = StopServo.Request()
        req.motor_id = motor_id

        # Verifica se il servizio è disponibile
        if not self.stop_servo_client.wait_for_service(timeout_sec=5.0):
            self.log(f"Servizio di stop per il motore {motor_id} non disponibile.")
            return False
    
        # Invia la richiesta al server
        future = self.stop_servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
    
        if result and result.success:
            self.log(f"Motore {motor_id} fermato con successo.")
            self.motor_positions[motor_id] = self.motor_positions[motor_id]  # Mantieni la posizione attuale
            self.update_current_label(motor_id, self.motor_positions[motor_id])
        else:
            self.log(f"Errore durante l'arresto del motore {motor_id}: {result.message if result else 'Nessuna risposta'}")

        # Interrompi il polling per il motore
        self.motor_polling_flags[motor_id] = False

    def execute_prendi(self):
        """
        Sequenza 'prendi' con step intermedi e finali.
        """

        if self.emergency_flag:
            self.log("Impossibile eseguire 'prendi': emergenza attiva.")
            return

        self.log("Esecuzione della sequenza 'prendi' iniziata...")
        try:
            motor_positions = self.load_motor_positions()

            # Sincronizza i motori con posizioni iniziali
            self.log("Sincronizzazione posizioni dei motori...")
            self.move_motors_in_parallel(self.motor_positions, speed=10, servo_type=180)

            # Step iniziale
            if not self.emergency_flag:
                step_1 = {0: 132, 1: 45, 2: 10}
                self.log("Step 1: movimento iniziale...")
                self.move_motors_in_parallel(step_1, speed=10, servo_type=180)

            # Step intermedi (movimento graduale)
            if not self.emergency_flag:
                self.log("Step 2: movimenti intermedi...")
                for pos_1, pos_2 in zip(range(45, 35, -2), range(10, 20, 2)):
                    start_time = time.time()
                    speed = 10 
                    distance = max(abs(pos_1 - self.motor_positions[1]), abs(pos_2 - self.motor_positions[2]))
                    pause = max(distance / speed if distance > 0 else 0, 0.1)  
                    if not self.emergency_flag:
                            self.move_motors_in_parallel({1: pos_1, 2: pos_2}, speed=speed, servo_type=180)
                            end_time = time.time()
                            self.log(f"Step completato in {end_time - start_time:.2f} secondi. Pausa di {pause:.2f} secondi.")
                    time.sleep(pause)

            # Step finale
            if not self.emergency_flag:
                step_3 = {0: 132, 1: 25, 2: 92}
                self.log("Step 3: posizione finale...")
                self.move_motors_in_parallel(step_3, speed=10, servo_type=180)

            self.log("Sequenza 'prendi' completata.")

        except Exception as e:
            self.log(f"Errore durante l'esecuzione di 'prendi': {e}")

    # Le altre funzioni rimangono invariate...

    def load_motor_positions(self):
        """
        Tenta di recuperare le posizioni reali dei motori dal servizio '/get_servo_position'.
        Se il servizio non è disponibile, carica le posizioni dal file.
        """
        positions = {}
        for motor_id in range(3):
            req = GetServoPosition.Request()
            req.motor_id = motor_id
    
            # Prova a recuperare la posizione dal servizio
            if self.get_position_client.wait_for_service(timeout_sec=5.0):
                self.log(f"Tentativo di recupero posizione reale per motore {motor_id}...")
                future = self.get_position_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                result = future.result()
                if result and result.success:
                    positions[motor_id] = result.position
                    self.log(f"Motore {motor_id}: posizione reale recuperata: {result.position}°")
                    continue
    
            # Se il servizio non è disponibile o fallisce, usa il file JSON
            self.log(f"Impossibile recuperare la posizione reale per motore {motor_id}, uso il file.")
            if os.path.exists(POSITIONS_FILE):
                with open(POSITIONS_FILE, 'r') as f:
                    file_positions = json.load(f)
                    positions[motor_id] = file_positions.get(str(motor_id), 90)
            else:
                self.log(f"File posizioni non trovato, uso default per motore {motor_id}: 90°")
                positions[motor_id] = 90
    
        # Aggiorna la GUI con le posizioni caricate
        for motor_id, angle in positions.items():
            if motor_id in self.current_labels:
                self.update_current_label(motor_id, angle)
    
        self.log(f"Posizioni motori finali caricate: {positions}")
        return positions

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
        self.motor_labels[motor_id].config(text=f"{angle}\u00B0")

    def update_speed_label(self, motor_id, value):
        """Aggiorna l'etichetta della velocità del motore."""
        if motor_id in self.speed_labels:
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

            if self.emergency_flag:
                self.log("Movimento parallelo interrotto per emergenza.")
                return

            current_angle = self.motor_positions.get(motor_id, 0)
            if abs(current_angle - target_angle) < 0.01:
                message = f"Motore {motor_id}: già all'angolo {current_angle}\u00B0, nessun movimento necessario."
                self.log(message)
                self.root.after(0, self._update_motor_log, motor_id, message)
                continue

            message = f"Motore {motor_id}: movimento da {current_angle}\u00B0 a {target_angle}\u00B0 a velocità {speed}\u00B0/s."
            self.log(message)
            self.root.after(0, self._update_motor_log, motor_id, message)  # Aggiungi al log privato del motore
   
            thread = Thread(target=self._move_motor, args=(motor_id, target_angle, speed, servo_type), name=f"Thread-Motor-{motor_id}")
            thread.start()
            threads.append(thread)
        
    def _move_motor(self, motor_id, target_angle, speed, servo_type=180):
        """
        Muove un singolo motore verso la posizione target.
        :param motor_id: ID del motore.
        :param target_angle: Angolo target.
        :param speed: Velocità del movimento.
        """
        if self.emergency_flag:
            self.log(f"Motore {motor_id}: movimento interrotto per emergenza.")
            return
    
        current_position = self.motor_positions.get(motor_id, 0)
        if abs(current_position - target_angle) < 0.01:
            self.log(f"Motore {motor_id}: già all'angolo {current_position}\u00B0, nessun movimento necessario.")
            self.root.after(0, self.update_current_label, motor_id, current_position)  # Sincronizza la label corrente
            return
    
        start_time = time.time()
        start_time_readable = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(start_time))
        self.log(f"Inizio movimento del motore {motor_id}: angolo={target_angle}\u00B0, velocità={speed}\u00B0/s, tipo servo={servo_type}. START_TIME: {start_time_readable}")
    
        success = self.send_motor_request(motor_id, target_angle, speed, servo_type)
        if success:
            end_time = time.time()
            end_time_readable = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(end_time))
            self.log(f"Motore {motor_id} raggiunto: {target_angle}\u00B0. END_TIME: {end_time_readable}")
            log_entry = f"Motore {motor_id} raggiunto: {target_angle}\u00B0. Velocità: {speed}\u00B0/s."
            self.log(log_entry)
            self.root.after(0, self._update_motor_log, motor_id, log_entry)
    
            self.motor_positions[motor_id] = target_angle  # Aggiorna la posizione attuale
            self.update_current_label(motor_id, target_angle)
            self.root.after(0, self.update_current_label, motor_id, target_angle)
        else:
            self.log(f"Errore nel movimento del motore {motor_id}.")

    def poll_motor_position(self, motor_id):
        """Esegue il polling della posizione corrente del motore."""
        req = GetServoPosition.Request()
        req.motor_id = motor_id
    
        self.motor_polling_flags[motor_id] = True  # Imposta il flag di polling per il motore
    
        while self.motor_polling_flags[motor_id]:  # Continua solo se il polling è attivo
            if self.emergency_flag:  # Interrompi il polling in caso di emergenza
                break
    
            # Controlla se il servizio è disponibile
            if not self.get_position_client.wait_for_service(timeout_sec=5.0):
                self.log(f"Servizio '/get_servo_position' non disponibile per il motore {motor_id}.")
                break
    
            # Invia la richiesta al servizio
            future = self.get_position_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
    
            if result and result.success:
                # Aggiorna la posizione corrente sulla GUI
                self.root.after(0, self.update_current_label, motor_id, result.position)
                self.log(f"Motore {motor_id}: posizione reale aggiornata: {result.position}°")
            else:
                self.log(f"Errore nel recupero della posizione del motore {motor_id}: {result.message if result else 'Nessuna risposta'}")
                break
    
            time.sleep(1)  # Riduci la frequenza del polling a 1 secondo
    
        self.log(f"Polling interrotto per il motore {motor_id}.")

    def _update_motor_log(self, motor_id, log_entry):
        """Aggiorna il log del motore specifico nel widget Text."""
        motor_log_widget = self.motor_logs.get(motor_id)
        if motor_log_widget:
            motor_log_widget.config(state=tk.NORMAL)  # Abilita modifiche
            motor_log_widget.insert(tk.END, f"{log_entry}\n")
            motor_log_widget.see(tk.END)  # Scorri fino alla fine
            motor_log_widget.config(state=tk.DISABLED)  # Disabilita modifiche

    def show_motor_log(self, motor_id):
        """Mostra il log del motore nella console generale."""
        motor_log_widget = self.motor_logs[motor_id]
        motor_log_widget.config(state=tk.NORMAL)
        log_content = motor_log_widget.get("1.0", tk.END)
        motor_log_widget.config(state=tk.DISABLED)
    
        # Pulire il log della console generale e copiarci il contenuto
        self.log_text.delete(1.0, tk.END)
        self.log_text.insert(tk.END, log_content)

    def update_current_label(self, motor_id, angle):
        self.log(f"Aggiornamento etichetta motore {motor_id} con angolo {angle}")
        """Aggiorna la posizione corrente nella GUI."""
        if motor_id in self.current_labels:
            self.current_labels[motor_id].config(text=f"Corrente: {angle}\u00B0")

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
                self.log(f"Motore {motor_id} raggiunto: {target_angle}\u00B0 con velocità {target_speed}\u00B0/s.")
                self.update_current_label(motor_id, target_angle)
                self.root.after(0, self.update_current_label, motor_id, target_angle)
        except Exception as e:
            self.log(f"Errore nella risposta del motore {motor_id}: {e}")

    def emergency_stop(self):
        """Gestisce la modalità di emergenza."""
        self.emergency_flag = True
        self.log("Emergenza attivata. Arresto di tutti i motori.")

        # Invoca il servizio di emergenza nel nodo servo_control_service
        if not self.emergency_client.wait_for_service(timeout_sec=5.0):
            self.log("Servizio '/emergency_stop' non disponibile.")
        else:
            request = EmergencyStop.Request()
            request.activate = True
            future = self.emergency_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
    
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
    
        # Arresta i motori impostando le loro posizioni attuali
        for motor_id in self.motor_positions:
            self.stop_motor(motor_id)
    
        # Aggiorna la GUI per riflettere lo stato di emergenza
        self.root.after(0, lambda: self.log_text.insert(tk.END, "Modalità di emergenza attivata.\n"))

        self.log("Emergenza completata.")
        self.emergency_flag = False  # Resetta la flag di emergenza

        self.log("Ripristino lo stato normale sul server...")
        request = EmergencyStop.Request()
        request.activate = False
        future = self.emergency_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.log("Ripristino completato: il server è pronto per nuovi comandi.")
            self.motor_positions = self.load_motor_positions()
            self.log("Posizioni motori ripristinate dopo l'emergenza.")
        else:
            self.log("Errore durante il ripristino dello stato sul server.")
    
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

    def send_motor_command(self, motor_id):
        try:
            angle = round(float(self.motor_sliders[motor_id].get()))
            speed = round(float(self.speed_sliders[motor_id].get()))
            success = self.send_motor_request(motor_id, angle, speed)
            if success:
                message = f"Motore {motor_id} impostato su angolo {angle} con velocità {speed}"
                self.log(message)
                self.root.after(0, self._update_motor_log, motor_id, message)  # Log per il motore specifico
                self.save_motor_position(motor_id, angle)  # Salva la posizione
                self.update_current_label(motor_id, angle)
                self.root.after(0, self.update_current_label, motor_id, angle)
                # Lancia il thread di polling per aggiornare la posizione corrente
                if not self.motor_polling_flags[motor_id]:
                    Thread(target=self.poll_motor_position, args=(motor_id,), daemon=True).start()
            else:
                message = f"Errore durante l'impostazione del motore {motor_id}"
                self.log(message)
                self.root.after(0, self._update_motor_log, motor_id, message)  # Log per il motore specifico

        except Exception as e:
            self.log(f"Errore durante l'invio del comando al motore {motor_id}: {e}")

    def save_motor_position(self, motor_id, angle):
        """
        Salva la posizione del motore nel file JSON.
        """
        try:
            # Carica le posizioni esistenti
            if os.path.exists(POSITIONS_FILE):
                with open(POSITIONS_FILE, 'r') as f:
                    positions = json.load(f)
            else:
                positions = {}
    
            # Aggiorna la posizione del motore
            positions[str(motor_id)] = angle
    
            # Salva le posizioni aggiornate nel file
            with open(POSITIONS_FILE, 'w') as f:
                json.dump(positions, f)
            self.log(f"Posizione del motore {motor_id} salvata: {angle}°")
        except Exception as e:
            self.log(f"Errore durante il salvataggio della posizione del motore {motor_id}: {e}")

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
