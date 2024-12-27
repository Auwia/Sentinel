from flask import Flask, request, jsonify
from adafruit_pca9685 import PCA9685
import busio
from board import SCL, SDA

# Inizializza PCA9685
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

app = Flask(__name__)

@app.route('/control', methods=['POST'])
def control_servo():
    try:
        data = request.json
        channel = int(data.get('channel', -1))
        angle = int(data.get('angle', -1))

        if 0 <= channel < 16 and 0 <= angle <= 180:
            # Calcola il duty cycle
            min_duty = 0x0666  # ~2.5% duty cycle
            max_duty = 0x2CCC  # ~12.5% duty cycle
            duty_cycle = int(min_duty + (angle / 180.0) * (max_duty - min_duty))
            pca.channels[channel].duty_cycle = duty_cycle
            return jsonify({"message": f"Channel {channel} set to {angle}°", "status": "success"})
        else:
            return jsonify({"message": "Invalid channel or angle range", "status": "error"}), 400
    except Exception as e:
        return jsonify({"message": str(e), "status": "error"}), 500

@app.route('/cleanup', methods=['POST'])
def cleanup():
    try:
        for channel in range(16):
            pca.channels[channel].duty_cycle = 0
        return jsonify({"message": "All channels cleaned up", "status": "success"})
    except Exception as e:
        return jsonify({"message": str(e), "status": "error"}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
