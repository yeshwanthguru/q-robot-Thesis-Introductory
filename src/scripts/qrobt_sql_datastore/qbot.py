import json
import time
import random
import sqlite3

from tkinter import *
from qrobot.models import AngularModel
import rospy
from qiskit_output_msgs.msg import qiskit_output


# Connect to the database
conn = sqlite3.connect('qiskit_output.db')
cursor = conn.cursor()

# Create a table to store the Qiskit output data
cursor.execute('''CREATE TABLE qiskit_output (data text)''')

Ts = 0.5  # sample every 500ms
# Initialize a AngularModel with single qubit:
n = 1  # single qubit
tau = 1  # 10 events for each temporal window (5 seconds in total)
model = AngularModel(n, tau)

# Start a loop
t_index = 1
def func():
    global t_index
    # Every sample period:
    time.sleep(Ts)
    # A switch ("ON/OFF" signal) is read by the python script as input
    if toggle_button.config('text')[-1] == 'ON':
        toggle_button.config(text='OFF')
        switch_signal=0
    else:
        toggle_button.config(text='ON')
        switch_signal = 1
    print("switch_signal =", switch_signal)
    # This input is then encoded in the AngularModel
    model.encode(switch_signal, dim=0)
    # If we are not at the end of the temporal window:
    if t_index < tau:
        t_index += 1
    # If we are at the end of the temporal window:
    else:
        # Print the final circuit
        print(model.circ)
        # Measure the module
        result = model.measure()
        # Output the result with print or in a log file
        print("result =", result)
        # Insert the data into the table
        output_data = json.dumps(result)
        cursor.execute("INSERT INTO qiskit_output (data) VALUES (?)", (output_data,))

        # Commit the changes
        conn.commit()
        # Initialize the ROS node
        rospy.init_node('qiskit_output_node', anonymous=True)

        # Create a publisher for the message
        pub = rospy.Publisher('qiskit_output', qiskit_output, queue_size=10)

        # Continuously publish the data from SQLite
        while not rospy.is_shutdown():
           cursor = conn.cursor()
           cursor.execute("SELECT * from qiskit_output")
           output_data = cursor.fetchall()
           data = output_data[0][0]
           msg = qiskit_output()
           msg.data = data
           pub.publish(msg)
           rospy.sleep(1)

ws = Tk()
ws.title("Q-robot Introductory Interface")
ws.geometry("200x100")


toggle_button = Button(text="OFF", width=10, command=func)
toggle_button.pack(pady=10)

ws.mainloop()
