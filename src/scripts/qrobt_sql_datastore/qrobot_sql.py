import pandas as pd
import sqlite3
conn = sqlite3.connect('qiskit_output.db')
data = pd.read_sql_query("SELECT * from qiskit_output", conn)
print(data)
