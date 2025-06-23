from Phidget22.Phidget import *
from Phidget22.Devices.Stepper import *
from Phidget22.Devices.VoltageRatioInput import *
from Phidget22.Devices.VoltageInput import *
from PgmController import PgmControllerQBleakClient
from bleak import BleakScanner
import asyncio
import time
import pandas as pd
import os

#===============================================================================================================================================================
# Input constant values
hub_serial_number = 766651
abs_path = "C:/Users/bakeratta/Documents/SmartAssist/"

target_perssure = float(input("target pressure [MPa] : "))
PGM_length = int(input("PGM length [mm] : "))  # The length of PGM used in this experiment ([mm])
PGM_number = int(input("PGM number (1 ~ 3) : "))

CALIBRATION_GAIN = -32732.2065488163  # Calibrated by using Phidget Controll Panel
path_offset = abs_path + "Code/PGM/"  # OFFSET is loaded in main function
file_force_offset = "Force_Offset_exp.txt"
file_PGMdata = "PGM_Length_data.txt"
file_pressure_offset = "Pressure_Offset.txt"

save = True
path_savefolder = abs_path + f"Data/PGM/Force-Displacement/Length_{PGM_length}/PGM_number_{PGM_number}/exp2/"

gravity = 9.80665  # Gravitational accelation ([m//s^2])
RESCALE_FACTOR = 0.0003125  # Rescale factor for stepper motor ([rotation/steps])
pitch = 5  # Pitch of ball screw ([mm/rotation])
miss_count = []
is_experiment = False
#===============================================================================================================================================================

def load_PGM_data(length):
    with open(path_offset + file_PGMdata, "r") as file:
        data = []

        for line in file:
            values = line.strip().split(",")
            data.append((int(values[0]), float(values[1]), float(values[2]), float(values[3])))

    for entry in data:
        if entry[0] == length:
            max_displacement = entry[0] - entry[PGM_number]
            return max_displacement

    raise ValueError(f"There is no data of the PGM length : {length}")

def load_force_offset(length):
    with open(path_offset + file_force_offset, "r") as file:
        data = []

        for line in file:
            values = line.strip().split(",")
            # data.append((int(values[0]), float(values[1])))
            data.append((int(values[0]), float(values[1]), float(values[2]), float(values[3])))

    for entry in data:
        if entry[0] == length:
            return entry[PGM_number]

    raise ValueError(f"There is no data of the PGM length : {length}")

def load_pressure_offset():
    with open(path_offset + file_pressure_offset, "r") as file:
        atmospheric_pressure = float(file.read())

    return atmospheric_pressure

def onVoltageChange(self, voltage):
    global pressure
    pressure = ((voltage / 4.5) * 150 * 6.8947448) - atmospheric_pressure

def onVoltageRatioChange(self, voltage_ratio):
    global force
    force = -(voltage_ratio - OFFSET)* CALIBRATION_GAIN * gravity  # Change negative value to positive value

def onPositionChange(self, position):
    global results, current_position_mm

    current_position_mm =abs(position * RESCALE_FACTOR * pitch)  # Transrate measured position from steps to mm
    current_time = time.time()

    if force == 0:
        print(f"Pressure: {pressure:.2f} kPa, Position: {current_position_mm:.2f} mm, Force: Has not starated measurement")
    else:
        print(f"Pressure: {pressure:.2f} kPa, Position: {current_position_mm:.2f} mm, Force: {force:.2f} N")

    if initialized:
        elapsed_time = current_time - start_time
        results.append([elapsed_time, current_position_mm, force, pressure])

class Init_Phidget():
    def __init__(self):
        return
    
    def init_stepper(self):
        global stepper

        stepper = Stepper()
        stepper.setHubPort(1)
        stepper.setDeviceSerialNumber(hub_serial_number)

        try:
            stepper.openWaitForAttachment(5000)
        except PhidgetException as e:
            raise Exception(f"Error opening device: {e}")
        
        stepper.setDataRate(50)

    def init_ForceSensor(self):
        global ForceSensor

        ForceSensor = VoltageRatioInput()
        ForceSensor.setHubPort(0)
        ForceSensor.setChannel(0)
        ForceSensor.setDeviceSerialNumber(hub_serial_number)

        try:
            ForceSensor.openWaitForAttachment(5000)
        except PhidgetException as e:
            raise Exception(f"Error opening device: {e}")
        
        ForceSensor.setDataRate(50)

    def init_PressureSensor(self):
        global PressureSensor

        PressureSensor = VoltageInput()
        PressureSensor.setIsHubPortDevice(True)
        PressureSensor.setHubPort(2)
        PressureSensor.setDeviceSerialNumber(hub_serial_number)

        # PressureSensor.setOnVoltageChangeHandler(onVoltageChange)

        try:
            PressureSensor.openWaitForAttachment(5000)
        except PhidgetException as e:
            raise Exception(f"Error opening device: {e}")
        
        PressureSensor.setSensorType(VoltageSensorType.SENSOR_TYPE_VOLTAGE)
        PressureSensor.setDataRate(50)

def set_target_position(TARGET_POSITION, velocity):
    global target_steps

    # Calcurate target steps from target position (input)
    target_steps = TARGET_POSITION / pitch / RESCALE_FACTOR  # Calcurate steps to reach target position
    velocity_steps = int(velocity / pitch / RESCALE_FACTOR)
    stepper.setTargetPosition(target_steps)
    stepper.setVelocityLimit(velocity_steps)
    stepper.setAcceleration(40000)

def zero_position():
    global force, initialized

    # Bool value to control main loop
    running = True
    is_target_position = False

    displacement = -2  # Initial displacement to get initial force
    additional_displacement = 0.1
    threshold = 0.02  # Threshold to check wether slider reached zero force position or not

    print("\nStart to set zero positoin.")
    try:
        # Attach the position change handler and voltage ratio change handler
        stepper.setEngaged(True)
        stepper.setOnPositionChangeHandler(onPositionChange)
        ForceSensor.setOnVoltageRatioChangeHandler(onVoltageRatioChange)

        # Move stage to get initial force value
        set_target_position(displacement, 40)

        while running:
            # When movement of slider finish and absolute force is smaller than threshold
            if is_target_position and abs(force) <= threshold:
                running = False

            elif is_target_position:
                is_target_position = False
                displacement += additional_displacement if force > 0 else -additional_displacement
        
                set_target_position(displacement, 40)

            # Get current position (steps) 
            current_position_steps = stepper.getPosition()

            # When target position has been reached
            if not is_target_position and abs(target_steps - current_position_steps) < 1:
                is_target_position = True

    except Exception as e:
        print(f"An error occurred: {e}")

    finally: 
        print("Zero position has been reached.")

        try:
            stepper.setOnPositionChangeHandler(None)
            ForceSensor.setOnVoltageRatioChangeHandler(None)
            stepper.setEngaged(False)
            stepper.addPositionOffset(-current_position_steps)
            initialized = True
            print("Connection with Phidget has been successfully completed.\n")

        except Exception as e:
            print(f"Error during cleanup: {e}")

async def connect_to_PGM():
    global pgmController, is_connected
    devices = await BleakScanner.discover(timeout=3.0)
    for device in devices:
        # if device.name == 'PGM Controller' and device.address == '30:C6:F7:25:63:C2':
        if device.name == 'PGM Controller' and device.address == '30:C6:F7:1D:26:7A':
            # print(device.name, device.address)
            # sys.exit()
            pgmController = PgmControllerQBleakClient(device, 'PgmController')
            await pgmController.start()
            is_connected = True

def check_target_position():
    running = True
    while running:
        # Get the current position of the stepper motor (in steps)
        current_position_steps = stepper.getPosition()

        # When target position has been reached
        if abs(current_position_steps - target_steps) < 1:
            print("target position reached")
            running = False

def check_dir(folder_path):
    if not os.path.isdir(folder_path):
        os.makedirs(folder_path)
    else:
        pass

def Collecting_data(folder_path, velocity):
    csv_files = [f for f in os.listdir(folder_path) if f.endswith('.csv')]

    output_file = f"Analysis_vel{velocity}.xlsx"
    all_data = []

    for file in csv_files:
        df = pd.read_csv(os.path.join(folder_path, file))
        
        all_data.append(df)
        all_data.append(pd.DataFrame({'': [None] * len(df)}))

    result = pd.concat(all_data, axis=1)
    result.to_excel(os.path.join(folder_path, output_file), index=False)

    print(f"Experiment data saved to {output_file}")

def save_results(TARGET_POSITION, velocity):
    # path_savefolder_vel = os.path.join(path_savefolder, f"velocity_{velocity}")
    check_dir(path_savefolder)
    results_path = os.path.join(path_savefolder, f"pressure_{target_perssure}_pos{abs(TARGET_POSITION)}_vel{velocity}.csv")
    # results_path = os.path.join(path_savefolder, f"pos{abs(TARGET_POSITION)}_vel{velocity}.csv")
    results_df = pd.DataFrame(results, columns=['Time [sec]', 'Position [mm]', 'Force [N]', 'Pressure [kPa]'])
    results_df.to_csv(results_path, index=None)
    print("Experiment data was saved to csv.")

#===============================================================================================================================================================

async def main():
    global start_time, force, results, OFFSET, atmospheric_pressure, velocity, results, initialized, is_connected

    ## PREPARATION ##
    OFFSET = load_force_offset(PGM_length)  # Load offset from text file
    max_displacement = load_PGM_data(PGM_length)
    atmospheric_pressure = load_pressure_offset()

    init = Init_Phidget()
    init.init_stepper()
    init.init_ForceSensor()
    init.init_PressureSensor()

    is_connected = False
    time.sleep(5)
    await connect_to_PGM()
    while not is_connected:
        time.sleep(0.1)
    await pgmController.setPgmParameters(0, 2, 0, 0, 0, 100)  # Channel, ContractionMode ,CycleTime, DelayTime, ContractionTime, DutyRaio


    ## EXPERIMENT ##
    print("\nStart experiment.")
    initialized = False  # To avoid recording during initialization
    PressureSensor.setOnVoltageChangeHandler(onVoltageChange)
    time.sleep(0.5)
    zero_position()

    results = []
    force = 0  # Temporary value until force sensor returns some outputs

    TARGET_POSITION = -int(max_displacement)
    velocity = 1
    print(f"\ntarget position : {-TARGET_POSITION}")
    time.sleep(1)
    
    start_time = time.time()
    try:
        stepper.setEngaged(True)
        stepper.setOnPositionChangeHandler(onPositionChange)
        ForceSensor.setOnVoltageRatioChangeHandler(onVoltageRatioChange)


        set_target_position(TARGET_POSITION, velocity)
        check_target_position()
        time.sleep(0.5)

        print("\nReturn to the zero position")
        set_target_position(0, velocity)
        check_target_position()

    except Exception as e:
        print(f"An error occurred: {e}")

    ## SAVE ##
    finally: 
        print("\nExpperiment finished.")
        stepper.setEngaged(False)
        stepper.setOnPositionChangeHandler(None)
        ForceSensor.setOnVoltageRatioChangeHandler(None)
        PressureSensor.setOnVoltageChangeHandler(None)
        
        
        if save:
            try:
                save_results(TARGET_POSITION, velocity)
            except:
                print("Some error occured durinig save.")


    ## CLOSING ##
    try:
        stepper.setEngaged(False)  # Safely disengage the motor
        stepper.close()
        ForceSensor.close()
        PressureSensor.close()
        await pgmController.setPgmParameters(0, 0, 0, 0, 0, 100)
        await pgmController.stop()
        print("Connection with Phidget has been successfully completed.")

        if miss_count:
            print("Data lacked in below parameters")
            print(miss_count)

    except Exception as e:
        print(f"Error during cleanup: {e}")

asyncio.run(main())