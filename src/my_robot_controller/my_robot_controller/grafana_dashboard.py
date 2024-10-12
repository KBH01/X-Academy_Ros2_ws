from datetime import datetime, timedelta
import random
from influxdb import InfluxDBClient

# InfluxDB settings
host = "localhost"
port = 8086
#url = "http://localhost:8086"
username = 'x-academy'
password = 'Gauss5050'
dbname = 'topside-rov'

# Initialize InfluxDB Client
client = InfluxDBClient(host=host, port=port)

# Create database if it doesn't exist
dbs = client.get_list_database()
if dbname not in [db['name'] for db in dbs]:
    client.create_database(dbname)
    print(f"Database {dbname} created")
client.switch_database(dbname)


# Function to generate fake data
def generate_data(current_time):
    temperature = random.uniform(20.0, 30.0)  # fake temperature
    pressure = random.uniform(970.0, 1030.0)  # fake pressure
    return current_time, temperature, pressure

# Function to write data to InfluxDB
def write_data(current_time):
    time, temperature, pressure = generate_data(current_time)
    json_body = [
        {
            "measurement": dbname,
            "tags": {
                "location": "office"
            },
            "time": time,
            "fields": {
                "temperature": temperature,
                "pressure": pressure
            }
        }
    ]
    client.write_points(json_body)

# Start time
start_time = datetime.utcnow()
# Write data to InfluxDB every second for 10 minutes
for i in range(10 * 60):  # 10 minutes * 60 seconds
    current_time = start_time + timedelta(seconds=i)
    write_data(current_time)
    print(f"Data for {current_time} written to InfluxDB")

# Close the client
client.close()