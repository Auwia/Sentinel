import rclpy
from rclpy.node import Node
from custom_interfaces.srv import PumpControl, ValveControl
from rclpy.action import ActionClient
from custom_interfaces.action import MoveMotors
import tkinter as tk
from tkinter import ttk
from threading import Thread

class CalibrationGUI(Node):
    def __init__(self):
        super().__init__('calibration_gui')
        self.get_logger().info("CalibrationGUI avviato.")

        # ROS 2 Clients
        self.motor_client = ActionClient(self, MoveMotors, '/servo_control/set_servo_angle')
        self.pump_client = self.create_client(PumpControl, '/pump_control')
        self.valve_client = self.create_client(ValveControl, '/valve_control')

        # Tkinter GUI setup
        self.root = tk.Tk()
        self.root.title("Calibration GUI")

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

        for motor_id in range(3):  # Supponiamo 3 motori
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
            success = self.send_motor_goal(motor_id, angle, speed)
            if success:
                self.log(f"Motore {motor_id} impostato su angolo {angle} con velocità {speed}")
            else:
                self.log(f"Errore durante l'impostazione del motore {motor_id}")
        except Exception as e:
            self.log(f"Errore durante l'invio del comando al motore {motor_id}: {e}")

    def send_motor_goal(self, motor_id, target_angle, target_speed, servo_type=180):
        target_angle = float(target_angle)
        target_speed = float(target_speed)
        servo_type = int(servo_type)

        goal_msg = MoveMotors.Goal()
        goal_msg.motor_id = motor_id
        goal_msg.target_angle = target_angle
        goal_msg.target_speed = target_speed
        goal_msg.servo_type = servo_type

        self.motor_client.wait_for_server()
        goal_future = self.motor_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()

        if not goal_handle.accepted:
            self.log(f"Comando al motore {motor_id} rifiutato.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.success:
            return True
        else:
            self.log(f"Errore motore {motor_id}: {result.status_message}")
            return False

    def toggle_pump(self):
        current_state = self.pump_btn["bg"] == "green"
        new_state = not current_state
        success = self.set_pump_state(new_state)
        if success:
            self.update_pump_button(new_state)

    def set_pump_state(self, state):
        req = PumpControl.Request()
        req.turn_on = state

        if not self.pump_client.wait_for_service(timeout_sec=10.0):
            self.log("Servizio 'pump_control' non disponibile.")
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

    def update_pump_button(self, state):
        color = "green" if state else "red"
        self.pump_btn.config(bg=color)

    def toggle_valve(self):
        current_state = self.valve_btn["bg"] == "green"
        new_state = not current_state
        success = self.set_valve_state(new_state)
        if success:
            self.update_valve_button(new_state)

    def set_valve_state(self, state):
        req = ValveControl.Request()
        req.turn_on = state

        if not self.valve_client.wait_for_service(timeout_sec=10.0):
            self.log("Servizio 'valve_control' non disponibile.")
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

    def update_valve_button(self, state):
        color = "green" if state else "red"
        self.valve_btn.config(bg=color)

    def emergency_stop(self):
        self.log("EMERGENZA ATTIVATA: Arresto di tutti i movimenti.")
        for motor_id in range(3):
            try:
                angle = round(float(self.motor_sliders[motor_id].get()))
                self.send_motor_goal(motor_id, angle, 0)  # Velocità a 0 per fermare il motore
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

    def load_motor_positions(self):
        for motor_id in range(3):
            initial_angle = 90  # Recuperare il valore reale se disponibile
            initial_speed = 10  # Recuperare il valore reale se disponibile

            self.motor_sliders[motor_id].set(initial_angle)
            self.update_motor_label(motor_id, initial_angle)

            self.speed_sliders[motor_id].set(initial_speed)
            self.update_speed_label(motor_id, initial_speed)

    def log(self, message):
        self.log_text.insert(tk.END, f"{message}\n")
        self.log_text.see(tk.END)
        self.get_logger().info(message)

    def run(self):
        self.log("CalibrationGUI in esecuzione.")
        self.root.mainloop()


def main():
    rclpy.init()
    gui_node = CalibrationGUI()
    gui_node.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

