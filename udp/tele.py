from methods import *
import numpy as np
import cv2
import math
from time import sleep
from dronekit import connect, VehicleMode

username = "rnanthak13@gmail.com"
password = "Chaos_theory1234"

# Instruction
# 1. Run dronekit-sitl copter --home=13.44166667,80.23194444,0,0
# 2. python3 mavproxy.py --master tcp:127.0.0.1:5760 --out=127.0.0.1:14550 --out=127.0.0.1:14551
# 3. Connect mission planner to 14550 port
# 4. Run the tele.py code with port 14551

# Mav command: python3 mavproxy.py --master tcp:127.0.0.1:5760 --out=127.0.0.1:14550 --out=127.0.0.1:14551
vehicle = connect('127.0.0.1:14551',heartbeat_timeout=30)
def getTimestamp():
    d = datetime.now()
    return int(d.microsecond / 1000 + time.mktime(d.timetuple()) * 1000)
if __name__ == '__main__':
    # API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJjcmVkZW50aWFsX2lkIjoiY3JlZGVudGlhbHxMUU5vbExMaEFheTI0UklxR0pkd0Vmb1g0Z0pxIiwiYXBwbGljYXRpb25faWQiOiJhcHBsaWNhdGlvbnw5RTNYd2F2dDh5cXBKd2hFeVBQTXB1TXh4MlJBIiwib3JnYW5pemF0aW9uX2lkIjoiZGV2ZWxvcGVyfFh6R01YbEpJOXptbUJEZndwNmFLUElsUTkyM2QiLCJpYXQiOjE1OTY2OTY0NDB9.7wsWdRI4g8y1BZ8OBc-Lniz3zO90dzfMKtqifWYKQkk"
    # eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJjcmVkZW50aWFsX2lkIjoiY3JlZGVudGlhbHxMUU5vbExMaEFheTI0UklxR0pkd0Vmb1g0Z0pxIiwiYXBwbGljYXRpb25faWQiOiJhcHBsaWNhdGlvbnw5RTNYd2F2dDh5cXBKd2hFeVBQTXB1TXh4MlJBIiwib3JnYW5pemF0aW9uX2lkIjoiZGV2ZWxvcGVyfFh6R01YbEpJOXptbUJEZndwNmFLUElsUTkyM2QiLCJpYXQiOjE1OTY2OTY0NDB9.7wsWdRI4g8y1BZ8OBc-Lniz3zO90dzfMKtqifWYKQkk
    API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJjcmVkZW50aWFsX2lkIjoiY3JlZGVudGlhbHxwQUFNWlBxaEx2T2Q2cGZSR2JkMlhDQkdRcTdNIiwiYXBwbGljYXRpb25faWQiOiJhcHBsaWNhdGlvbnx3ZURHZ01oTldtek55c1A4S0xEdlRsQW5QTE0iLCJvcmdhbml6YXRpb25faWQiOiJkZXZlbG9wZXJ8MnpvYmI3eWh4ZVk0cWtDM1BSeDBaSEtNejIzOCIsImlhdCI6MTQ3MTM3OTc0Mn0.MeO0jt6holPt0jdPJvRJrTBi380WsbOPGCEO6u-tfSo"
    client_id = "87b7374e-5a0c-497a-96a0-37393f649fef"

    js = get_token_user(client_id, username, password)
    access_token = js['access_token']
    print("Access Token: ", access_token)
    print()
    refresh_token = js['refresh_token']
    refreshed = do_token_refresh(client_id, refresh_token)

    profile = get_pilot_profile(API_KEY, access_token)
    pilot_id = profile['data']['id']

    # print("%s : %s".format(p,profile[p]))

    # print("########## Creating FLight Plan ########")
    manufacturer_name = "AeroSense"
    model_name = "AS-MC02-P"
    print("Manufacture name:", manufacturer_name)
    print("model_name ", model_name)
    print("Obtaining aircraft ID")
    model_id = get_model_id(API_KEY, manufacturer_name, model_name)
    print("Obtained model id: ", model_id)

    # print("Creating flight plan")
    required = {}

    required["pilot_id"] = pilot_id
    required["token"] = access_token
    required["model_id"] = model_id
    required["nickname"] = "lightsaber"

    aircrafts = get_pilot_aircrafts(API_KEY, required)
    print("Aircrafts :", aircrafts)
    aircraft_id = aircrafts[0]["id"]
    print("Choosing aircraft: ", aircraft_id)

    data = {
        "takeoff_latitude": 13.44166667,
        "takeoff_longitude": 80.23194444,
        "pilot_id": pilot_id,
        "aircraft_id": aircraft_id,
        "start_time": "2020-08-31T13:11:00.730Z",
        "end_time":   "2020-08-31T14:10:42.000Z",
        "max_altitude_agl": 90,
        "rulesets": ["ind_airmap_rules",
                     "ind_notam"],
        "buffer": 1,
        "geometry": {"type": "Polygon", "coordinates": [[[80.23194444, 13.44166667],
                                                         [80.23194444, 13.44266667],
                                                         [80.23394444, 13.44266667],
                                                         [80.23394444, 13.44166667],
                                                         [80.23194444, 13.44166667]
                                                         ]]
                     }
    }
    print()
    print("\nCreating Flight plan for geometry", data["geometry"])
    created_flightplan = create_flight_plan(API_KEY, access_token, data)
    try:
        flight_plan_id = created_flightplan["data"]["id"]
        print("Flight plan id", flight_plan_id)
    except:
        print("Failed creating plan")
        exit()

    # print(API_KEY)

    # print(flights.keys())
    submit_plan = False
    flight_id = "flight|P58bXJB6eQWkqhg5YXgyA5Kyd0uoMK3ZZ3baXycQEXA49YxxP0"
    print()
    print("Submiting Flight Plan")
    if submit_plan:
        file = open("output.json", "w")
        out = get_fligh_brief(API_KEY, access_token, flight_plan_id)
        submission_result = submit_flight_plan(API_KEY, access_token, flight_plan_id)
        try:
            json.dump(submission_result, file)
            flight_id = submission_result["data"]["flight_id"]
            print("Flight ID: ", flight_id)
        except:
            print("Error in submitting plan")
            print(submission_result)
            file.close()
            exit()
        file.close()

    else:
        file = open("output.json", "r")
        submission_result = json.load(file)
        flight_id = submission_result["data"]["flight_id"]
        print("Flight ID: ", flight_id)
        file.close()
    # flight_id = "flight|G7BDmG6IgezoleUkWaAbzuJlP7pe"
    print("Starting communication")
    secret_key = start_comm(API_KEY, access_token, flight_id)
    print("Secret Key: ", secret_key)

    secretKey = base64.b64decode(secret_key)

    position = telemetry_pb2.Position()
    attitude = telemetry_pb2.Attitude()
    speed = telemetry_pb2.Speed()
    barometer = telemetry_pb2.Barometer()

    HOSTNAME = 'telemetry.airmap.com'
    IPADDR = socket.gethostbyname(HOSTNAME)
    PORTNUM = 16060
    print(IPADDR)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
    counter = 1

    try:
        # connect the socket
        s.connect((IPADDR, PORTNUM))
        print("Socket Connected")
        # send 100 messages at 5Hz
        while(True):

            # update messages
            timestamp = getTimestamp()
            position.timestamp = 1
            position.latitude = vehicle.location.global_relative_frame.lat
            print(dir(vehicle.location.global_relative_frame))
            position.longitude =vehicle.location.global_relative_frame.lon
            position.altitude_agl =vehicle.location.global_relative_frame.alt
            position.altitude_msl =vehicle.location.global_relative_frame.alt
            position.horizontal_accuracy = 0.0

            attitude.timestamp = timestamp
            attitude.yaw = vehicle.attitude.yaw
            attitude.pitch = vehicle.attitude.pitch
            attitude.roll =vehicle.attitude.roll

            speed.timestamp = timestamp
            speed.velocity_x =vehicle.velocity[0]
            speed.velocity_y =vehicle.velocity[1]
            speed.velocity_z =vehicle.velocity[2]

            barometer.timestamp = timestamp
            barometer.pressure =1012.0

            # build  payload

            # serialize  protobuf messages to string and pack to payload buffer
            bytestring = position.SerializeToString()
            format = '!HH' + str(len(bytestring)) + 's'
            payload = struct.pack(format, 1, len(bytestring), bytestring)

            bytestring = attitude.SerializeToString()
            format = '!HH' + str(len(bytestring)) + 's'
            payload += struct.pack(format, 2, len(bytestring), bytestring)

            bytestring = speed.SerializeToString()
            format = '!HH' + str(len(bytestring)) + 's'
            payload += struct.pack(format, 3, len(bytestring), bytestring)

            bytestring = barometer.SerializeToString()
            format = '!HH' + str(len(bytestring)) + 's'
            payload += struct.pack(format, 4, len(bytestring), bytestring)
            print(payload)
            # encrypt payload

            print(payload)


            # encrypt payload

            # use PKCS7 padding with block size 16
            def pad(data, BS):
                PS = (BS - len(data)) % BS
                if PS == 0:
                    PS = BS
                P = (chr(PS) * PS).encode()
                return data + P


            payload = pad(payload, 16)
            IV = Random.new().read(16)
            aes = AES.new(secretKey, AES.MODE_CBC, IV)
            encryptedPayload = aes.encrypt(payload)
            # send telemetry
            # packed data content of the UDP packet
            format = '!LB' + str(len(flight_id)) + 'sB16s' + str(len(encryptedPayload)) + 's'
            PACKETDATA = struct.pack(format, counter, len(flight_id), flight_id.encode(), 1, IV, encryptedPayload)

            # send the payload
            s.send(PACKETDATA)

            # print timestamp when payload was sent
            print("Sent payload messsage #", counter, "@", time.strftime("%H:%M:%S"))

            # increment sequence number
            counter += 1

            # 5 Hz
            sleep(0.5)
        k = input("Type soemthing to close")
    except KeyboardInterrupt:
        print("Error sending telemetry")
        exit(1)
    s.close()
    print("Socket closed")
    end_comm(API_KEY, access_token, flight_id)
    end_flight(API_KEY,access_token,flight_id)
    print("Communication Ended")



