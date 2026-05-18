import streamlit as st
import serial
import csv
import threading
import time

# If we do not use fragments then the data keeps re-running which is problematic with respect to listening to the port 24/7
st.title("Ground Station")


#Format for the data

#TEAM_ID, TIME_S, PKT_COUNT, ALT_M, PRES_PA, TEMP_C, VOLT_V, GNSS_TIME,
#GNSS_LAT, GNSS_LON, GNSS_ALT_M, GNSS_SATS, ACCEL_X, ACCEL_Y, ACCEL_Z,
#GYRO_SPIN_DEG_S, STATE, [OPTIONAL]

#States
#0 BOOT Power-on, sensor init
#1 STANDBY Checks passed, awaiting command
#2 TELEMETRY_ACTIVE Ground station commanded start
#3 DRONE_ASCENT Altitude increasing
#4 FREE_FALL Altitude decreasing + accel ≈0 g
#5 DEPLOYMENT 70 m detected, trigger fired
#6 CONTROLLED_DESCENT Descent rate ≤5 m/s
#7 LANDED


shared_data = { 
    "telemetry_active": False, 
    "set_gyro_zero": False, 
    "time": 0, 
    "altitude": 0, 
    "temperature": 0, 
    "accel_x": 0, 
    "accel_y": 0, 
    "accel_z": 0, 
    "gyro_spin": 0, 
    "gnss_lat": 0, 
    "gnss_lon": 0, 
    "current_state": 0
} 
data_lock = threading.Lock()

if "telemetry_active" not in st.session_state:
    st.session_state.telemetry_active = False    
if "set_gyro_zero" not in st.session_state:
    st.session_state.set_gyro_zero = False  

#Code for Enabling Rerunable fragments
def rerunable_fragment(func, **kwargs):
    frag = st.fragment(func, **kwargs)
    container_ref = []

    def wrapper():
        if not container_ref:
            container_ref.append(st.empty())

        container = container_ref[0]

        with container:
            frag()

    return wrapper


@st.fragment()
def current_state_ui():
    if st.session_state.current_state==0:
        st.write("Current State: BOOT")
    elif st.session_state.current_state==1:
        st.write("Current State: STANDBY")
    elif st.session_state.current_state==2:
        st.write("Current State: TELEMETRY_ACTIVE")
    elif st.session_state.current_state==3:
        st.write("Current State: DRONE_ASCENT")
    elif st.session_state.current_state==4:
        st.write("Current State: FREE_FALL")
    elif st.session_state.current_state==5:
        st.write("Current State: DEPLOYMENT")
    elif st.session_state.current_state==6:
        st.write("Current State: CONTROLLED_DESCENT")
    elif st.session_state.current_state==7:
        st.write("Current State: LANDED")

@st.fragment()
def time_since_poweron_ui():
    st.write("Current time since power-on:",st.session_state.time)

@st.fragment()
def current_altitude_ui():
    st.write("Current Above Ground Altitude:",st.session_state.altitude)

@st.fragment()
def sat_temp_ui():
    st.write("Current Temperature:",st.session_state.temperature)

@st.fragment()
def accel_x_ui():
    st.write("Current Acceleration X-Axis (m/s^2):",st.session_state.accel_x)

@st.fragment()
def accel_y_ui():
    st.write("Current Acceleration Y-Axis (m/s^2):",st.session_state.accel_y)

@st.fragment()
def accel_z_ui():
    st.write("Current Acceleration Z-Axis (m/s^2):",st.session_state.accel_z)

@st.fragment()
def gyro_state_ui():
    st.write("Current Gyro Spin Rate (deg/s^2):", st.session_state.gyro_spin)

@st.fragment()
def gnss_lat_ui():
    st.write("Current Latitude:", st.session_state.gnss_lat)

@st.fragment()
def gnss_lon_ui():
    st.write("Current Longitude:",st.session_state.gnss_lon)
print("hello2 - main app rerender")


#Setting up the command part
@st.fragment()
def rf_on_off():
    st.write("Is Telemetry Active:", st.session_state.telemetry_active)
    confirmation_text = st.text_input("Type Yes/No to confirm your change", key="rf_confirm_text")
    rf_onoff_button = st.button("Toggle") 


    if rf_onoff_button:
        if confirmation_text.lower()=="yes":
            st.session_state.telemetry_active = not st.session_state.telemetry_active
            print("confirm pressed")
            st.rerun(scope="fragment")

@st.fragment()
def set_gyro_zero():
    st.write("Set Gyro Zero")
    gyro_confirm_text = st.text_input("Type Yes/No to confirm your change", key="gyro_confirm_text")
    gyro_set_button = st.button("Send Zero Command") 


    if gyro_set_button:
        if gyro_confirm_text.lower()=="yes":
            st.session_state.set_gyro_zero = True
    
@st.fragment(run_every="100ms")
def updateStateFromDataLock():
    with data_lock:
        st.session_state.current_state = shared_data["current_state"]
        st.session_state.time = shared_data["time"]
        st.session_state.altitude = shared_data["altitude"]
        st.session_state.temperature = shared_data["temperature"]
        st.session_state.accel_x = shared_data["accel_x"]
        st.session_state.accel_y = shared_data["accel_y"]
        st.session_state.accel_z = shared_data["accel_z"]
        st.session_state.gyro_spin = shared_data["gyro_spin"]
        st.session_state.gnss_lat = shared_data["gnss_lat"]
        st.session_state.gnss_lon = shared_data["gnss_lon"]

        #These depend on user commands
        shared_data["telemetry_active"] = st.session_state.telemetry_active
        shared_data["set_gyro_zero"] = st.session_state.set_gyro_zero
    
    #Update the UI components
    current_state_ui()
    time_since_poweron_ui()
    current_altitude_ui()
    sat_temp_ui()
    accel_x_ui()
    accel_y_ui()
    accel_z_ui()
    gyro_state_ui()
    gnss_lat_ui()
    gnss_lon_ui()
    
updateStateFromDataLock()
rf_on_off()
set_gyro_zero()


#Background thread logic
def serial_listener():
    team_id = "FOO"
    #TEAM_ID, TIME_S, PKT_COUNT, ALT_M, PRES_PA, TEMP_C, VOLT_V, GNSS_TIME, GNSS_LAT, GNSS_LON, GNSS_ALT_M, GNSS_SATS, ACCEL_X, ACCEL_Y, ACCEL_Z,GYRO_SPIN_DEG_S, STATE
    # Serial Port Communication Section
    ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200)
    with open("Flight_"+team_id+".csv", 'a+') as csvfile:
        csvwriter = csv.writer(csvfile)
        isTelemetryOn = False
        while True:
            try:
                data = ser.readline().decode('utf-8', errors="ignore").strip()
                arrayData = data.split(",")
                if data=="testTelemetryMessage":
                    print("telemetryMessage was successful")
                print(data)

                if not data.startswith(team_id):
                    continue
                    
                if len(arrayData)>16:
                    with data_lock:
                        shared_data["time"] = arrayData[1]
                        shared_data["altitude"] = arrayData[3]
                        shared_data["current_state"] = int(arrayData[16])
                        shared_data["temperature"] = arrayData[5]
                        shared_data["accel_x"] = arrayData[12]
                        shared_data["accel_y"] = arrayData[13]
                        shared_data["accel_z"] = arrayData[14]
                        shared_data["gyro_spin"] = arrayData[15]
                        shared_data["gnss_lat"] = arrayData[8]
                        shared_data["gnss_lon"] = arrayData[9]


                    #break the loop if session state is 7 (Landed)
                    if int(arrayData[16]) == 7:
                        print("broken the chain")
                        break

                    #Writing the data to CSV
                    csvwriter.writerow(arrayData)
                with data_lock:
                    #Code for the telemetry on/off command
                    if isTelemetryOn != shared_data["telemetry_active"]:
                        #Code to send the command
                        ser.write("telemetry-"+shared_data["telemetry_active"]+"\n".encode("utf-8"))

                    isTelemetryOn = shared_data["telemetry_active"]

                    #Code for sending gyro zero command
                    if shared_data["set_gyro_zero"] == True:
                        #Code to send the command
                        ser.write("setgyrozero\n".encode("utf-8"))

                    shared_data["set_gyro_zero"] = False
            except KeyboardInterrupt:
                break
            except Exception as e:
                print("Serial thread error:", repr(e))
if "thread_started" not in st.session_state:

    thread = threading.Thread(
        target=serial_listener,
        daemon=True
    )

    thread.start()

    st.session_state.thread_started = True




# sudo chmod 666 /dev/ttyACM0  #Run this each time you replug your esp32