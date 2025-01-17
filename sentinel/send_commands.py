import time
import rclpy
from rclpy.action import ActionClient
from custom_interfaces.action import MoveMotors
from custom_interfaces.srv import PumpControl 
from custom_interfaces.srv import ValveControl  

def send_motor_goal(node, action_client, motor_id, target_angle, target_speed=10.0, servo_type=180):
    goal_msg = MoveMotors.Goal()
    goal_msg.motor_id = motor_id
    goal_msg.target_angle = float(target_angle)
    goal_msg.target_speed = float(target_speed)
    goal_msg.servo_type = servo_type

    node.get_logger().info(f"Inviando comando al motore {motor_id}: angolo={target_angle}, velocità={target_speed}")
    
    # Attendere il server dell'azione
    if not action_client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error(f"Action server non disponibile per il motore {motor_id}.")
        return False

    # Inviare il goal
    goal_future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, goal_future)
    goal_handle = goal_future.result()

    if not goal_handle.accepted:
        node.get_logger().error(f"Il comando per il motore {motor_id} è stato rifiutato!")
        return False

    # Aspettare il risultato del goal
    result_future = goal_handle.get_result_async()

    # Spin con timeout esplicito per attendere il completamento del futuro
    timeout = 15.0  # Secondi
    start_time = time.time()
    while not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start_time > timeout:
            node.get_logger().error(f"Timeout durante l'attesa del risultato del motore {motor_id}.")
            return False

    # Ottenere il risultato
    result_response = result_future.result()
    result = result_response.result

    if not result.success:
        node.get_logger().error(f"Errore durante il movimento del motore {motor_id}: {result.status_message}")
        return False

    node.get_logger().info(f"Motore {motor_id} completato con successo!")
    return True

def set_pump_state(node, client, state):
    req = PumpControl.Request()
    req.turn_on = state

    node.get_logger().info(f"Tentativo di attivazione della pompa: {'ON' if state else 'OFF'}")

    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(f"Servizio 'pump_control' non disponibile.")
        return False

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        result = future.result()
        if result.success:
            node.get_logger().info(f"Pompa {'ON' if state else 'OFF'}: {result.success}")
            return True
        else:
            node.get_logger().error(f"Errore durante l'attivazione della pompa: {result}")
            return False
    else:
        node.get_logger().error(f"Errore: Nessuna risposta dal servizio 'pump_control'.")
        return False

def set_solenoid_state(node, client, state):
    req = ValveControl.Request()
    req.turn_on = state

    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(f"Servizio 'valve_control' non disponibile.")
        return False

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        result = future.result()
        if result.success:
            node.get_logger().info(f"Solenoide {'ON' if state else 'OFF'}: {result.success}")
            return True
        else:
            node.get_logger().error(f"Errore durante l'attivazione del solenoide: {result}")
            return False
    else:
        node.get_logger().error(f"Errore: Nessuna risposta dal servizio 'valve_control'.")
        return False

def execute_phase(node, motor_client, pump_client, solenoid_client, phase, motor_turn_ons, pump_state, solenoid_state, delay=1.0):
    node.get_logger().info(f"Esecuzione fase {phase} con {len(motor_turn_ons)} comandi motore.")
    
    # Invia i comandi ai motori
    for motor_id, (target_angle, target_speed, servo_type) in motor_turn_ons.items():
        node.get_logger().info(f"Fase {phase}: Inviando comando al motore {motor_id} (angolo={target_angle}, velocità={target_speed}, tipo={servo_type})")
        success = send_motor_goal(node, motor_client, motor_id, target_angle, target_speed, servo_type)
        if not success:
            node.get_logger().error(f"Fase {phase}: Errore durante il movimento del motore {motor_id}. Interruzione.")
            return False

    # Attivazione pompa e solenoide
    node.get_logger().info(f"Fase {phase}: Attivazione pompa: {pump_state}, solenoide: {solenoid_state}")

    if not set_pump_state(node, pump_client, pump_state):
        node.get_logger().error(f"Errore durante l'attivazione della pompa per la fase {phase}.")
        return False

    if not set_solenoid_state(node, solenoid_client, solenoid_state):
        node.get_logger().error(f"Errore durante l'attivazione del solenoide per la fase {phase}.")
        return False

    time.sleep(delay)
    node.get_logger().info(f"Fase {phase} completata.")
    return True

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('send_turn_ons')

    motor_client = ActionClient(node, MoveMotors, '/servo_control/set_servo_angle')
    pump_client = node.create_client(PumpControl, '/pump_control')
    solenoid_client = node.create_client(ValveControl, '/valve_control')

    try:
        phases = [
            {"phase": 1, "motor_turn_ons": {0: (0, 10.0, 180), 1: (45, 10.0, 180), 2: (0, 10.0, 180)}, "pump": False, "solenoid": False},
            {"phase": 2, "motor_turn_ons": {0: (40, 10.0, 180), 1: (35, 10.0, 180), 2: (60, 10.0, 180)}, "pump": False, "solenoid": False},
            {"phase": 3, "motor_turn_ons": {0: (40, 10.0, 180), 1: (33, 10.0, 180), 2: (80, 10.0, 180)}, "pump": True, "solenoid": False},
            {"phase": 4, "motor_turn_ons": {0: (40, 10.0, 180), 1: (45, 10.0, 180), 2: (0, 10.0, 180)}, "pump": True, "solenoid": True},
            {"phase": 5, "motor_turn_ons": {0: (0, 10.0, 180), 1: (45, 10.0, 180), 2: (0, 10.0, 180)}, "pump": False, "solenoid": False},
        ]

        for phase in phases:
            node.get_logger().info(f"Iniziando la fase {phase['phase']}")
            if not execute_phase(
                node,
                motor_client,
                pump_client,
                solenoid_client,
                phase["phase"],
                phase["motor_turn_ons"],
                phase["pump"],
                phase["solenoid"],
            ):
                node.get_logger().error(f"Errore nella fase {phase['phase']}. Interruzione del test.")
                break
            node.get_logger().info(f"Fase {phase['phase']} completata con successo.")

    except Exception as e:
        node.get_logger().error(f"Errore durante l'esecuzione del test: {e}")

    finally:
        motor_client.destroy()
        pump_client.destroy()
        solenoid_client.destroy()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

