from methods import *
import numpy as np
import math
from time import sleep
from dronekit import connect, VehicleMode, Command
from datetime import datetime, timedelta
import argparse  

username = "kumaranshanthi13@gmail.com"
password = "Dskumaran@1995"

# Instruction
# 1. Run dronekit-sitl copter --home=13.44166667,80.23194444,0,0
# 2. python3 mavproxy.py --master tcp:127.0.0.1:5760 --out=127.0.0.1:14550 --out=127.0.0.1:14551
# 3. Connect mission planner to 14550 port
# 4. Run the tele.py code with port 14551

# Mav command: python3 mavproxy.py --master tcp:127.0.0.1:5760 --out=127.0.0.1:14550 --out=127.0.0.1:14551
vehicle = connect('127.0.0.1:14551',heartbeat_timeout=30)

def readmission(aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

    This function is used by upload_mission().
    """
    print("\nReading mission from file: %s" % aFileName)
    cmds = vehicle.commands
    missionlist=[]
    missionlist0=[]
    missionlist1=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append([ln_param5,ln_param6])
                missionlist0.append(ln_param5) 
                missionlist1.append(ln_param6)      
   # print("missionlist0",missionlist0)
    #print("missionlist1",missionlist1)
    return missionlist

def download_mission():
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    print(" Download mission from vehicle")
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist

def save_mission(aFileName):
    """
    Save a mission in the Waypoint file format 
    (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    print("\nSave mission from Vehicle to file: %s" % aFileName)    
    #Download mission from vehicle
    missionlist = download_mission()
    #Add file-format information
    output='QGC WPL 110\n'
    #Add home location as 0th waypoint
    home = vehicle.home_location
    output+="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (0,1,0,16,0,0,0,0,home.lat,home.lon,home.alt,1)
    #Add commands
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        print(" Write mission to file")
        file_.write(output)
        


def getTimestamp():
    d = datetime.now()
    return int(d.microsecond / 1000 + time.mktime(d.timetuple()) * 1000)
if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
    parser.add_argument('--H', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
    
    parser.add_argument( '--flag',
                   help="vehicle connection target string.")
    args = parser.parse_args()
    hr = int(args.H)
    flag=args.flag
    print(flag)

    API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJjcmVkZW50aWFsX2lkIjoiY3JlZGVudGlhbHw5TGd3N21uU1lFZGw5QVVxem82eUp1eGJlQnlnIiwiYXBwbGljYXRpb25faWQiOiJhcHBsaWNhdGlvbnxBWDI4eG5rdDluYVdYWGNOYW03WmdUNjd6d3BBIiwib3JnYW5pemF0aW9uX2lkIjoiZGV2ZWxvcGVyfEx3RW5aeE1jeDBhOFlCVERQeDIyZ0NlRFF4V3kiLCJpYXQiOjE1OTgzNTczOTF9.s-W3dzKLfhDsRsr51mb8IilqkNRLeYLxj6PTlPrA134"
    client_id = "70c6a951-9643-4f10-8fde-c46aa126b708"

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
    start_time=datetime.utcnow()
   # start_time=start_time-5
    start_time = start_time.strftime('%Y-%m-%dT%H:%M:%S.%fZ')
    end_time=(datetime.utcnow()+timedelta(hours=hr)).strftime('%Y-%m-%dT%H:%M:%S.%fZ')
    print(start_time)
    print(end_time)


    import_mission_filename = 'mpmission.txt'
    export_mission_filename = 'exportedmission.txt'

    save_mission(export_mission_filename)
    missionlist=readmission(export_mission_filename)
  
    print(missionlist)

    take_off_lat= vehicle.location.global_relative_frame.lat
    take_off_lon= vehicle.location.global_relative_frame.lon
    data = {
        "takeoff_latitude": take_off_lat,
        "takeoff_longitude":take_off_lon,
        "pilot_id": pilot_id,
        "aircraft_id": aircraft_id,
        "start_time": start_time,
        "end_time":   end_time,
        "max_altitude_agl": 90,
        "rulesets": ["ind_airmap_rules",
                     "ind_notam"],
        "buffer": 1,
        "geometry": {"type": "Polygon", "coordinates": [missionlist]
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
    submit_plan = True
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



