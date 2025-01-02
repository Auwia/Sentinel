import paho.mqtt.client as mqtt
import ssl

# Configura i certificati
CA_CERT = "/home/pi/certs/ca.crt"
CLIENT_CERT = "/home/pi/certs/broker.crt"
CLIENT_KEY = "/home/pi/certs/broker.key"

# Callback MQTT
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe("gpio/control")

def on_message(client, userdata, msg):
    print(f"Message received on {msg.topic}: {msg.payload.decode()}")

# Configura il client MQTT
mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

# Configura la connessione TLS
mqtt_client.tls_set(ca_certs=CA_CERT,
                    certfile=CLIENT_CERT,
                    keyfile=CLIENT_KEY,
                    tls_version=ssl.PROTOCOL_TLS)

# Connetti al broker usando il nome host definito in /etc/hosts
mqtt_client.connect("sentinel-broker", 8883)

# Avvia il loop
mqtt_client.loop_forever()

