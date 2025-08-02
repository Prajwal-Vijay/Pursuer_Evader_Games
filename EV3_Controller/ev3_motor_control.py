"""
EV3 Motor Control via MQTT.

This script subscribes to MQTT messages and controls three EV3 motors accordingly.
"""

import paho.mqtt.client as mqtt
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, SpeedPercent
import time
import json

# Initialize motors
motor_a = LargeMotor(OUTPUT_A)
motor_b = LargeMotor(OUTPUT_B)
motor_c = LargeMotor(OUTPUT_C)

def on_connect(client, userdata, flags, rc):
	"""
	Callback for when the client receives a CONNECTION ACKNOWLEDGED response from the MQTT server.

	Subscribes to the /motor_commands topic.
	"""
	print("Connected to MQTT broker with result code " + str(rc))
	client.subscribe("/motor_commands")

def on_message(client, userdata, msg):
	"""
	Callback for when a PUBLISH message is received from the MQTT server.

	Decodes the message and sets the speeds of the three motors.
	Note the message must be a JSON string representing a list of three speed percentage values.
 	"""
	json_command = msg.payload.decode()
	command = json.loads(json_command) # The string message is now converted to a list
	#print(f"Received command: {command}")
	print("Received command: "+st(command))
	motor_a.on(SpeedPercent(command[0]))
	motor_b.on(SpeedPercent(command[1]))
	motor_c.on(SpeedPercent(command[2]))
	if command == [0,0,0]:
		motor_a.off()
		motor_b.off()
		motor_c.off()

# MQTT Setup
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set("esb201", "esb@201")  # Set MQTT username and password
# Replace with your PC's MQTT broker IP address(Same as the IP Address as PC the broker is setup on)
client.connect("192.168.43.177", 1883, 60)

#Start MQTT Loop
client.loop_start()

try:
	while True:
		time.sleep(0.01)
except KeyboardInterrupt:
    # Switch off the motors on exit
	motor_a.off()
	motor_b.off()
	motor_c.off()
	client.loop_stop()
