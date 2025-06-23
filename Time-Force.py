from Phidget22.Phidget import *
from Phidget22.Devices.Stepper import *
from Phidget22.Devices.VoltageRatioInput import *
from Phidget22.Devices.VoltageInput import *
from PgmController import PgmControllerQBleakClient
from bleak import BleakScanner
import asyncio
import time
import os
import pandas as pd
import numpy as np
from itertools import zip_longest

########################################################################################################################
### INPUT VARIABLES ###
hub_serial_number = 766651

PGM_length = int(input("PGM length [mm]: "))
PGM_number = int(input("PGM number (1 ~ 3): "))
target_pressure = int(input("target pressure [kPa]: "))
target_position = int(input("target position [mm]: "))

save = True
abs_path = "C:/Users/bakeratta/Documents/SmartAssist/"
# path_savefolder = abs_path + f"Data/PGM/Time-Force/exp2/Length_{PGM_length}/PGM_number_{PGM_number}/"
path_savefolder = abs_path + f"Data/PGM/Time-Force/exp2/test/"
path_offset = abs_path + "Code/Texts/"

time_Before_Contraction = 1.0
time_During_Contraction = 2.0
time_After_Contraction = 1.0

filename = f"pressure_{target_pressure}_pos_{int(target_position)}"

gravity = 9.80665
RESCALE_FACTOR = 0.0003125
pitch = 5

force_list = []
pressure_list = []
position_list = []
command_list = []

is_experimenting = False
########################################################################################################################

def onVoltageRatioChange(self, voltage_ratio):
    if is_experimenting:
        force = -(voltage_ratio + OFFSET)* GAIN * gravity  # Change negative value to positive value
        force_list.append([time.perf_counter(), force])

def onVoltageChange(self, voltage):
    if is_experimenting:
        pressure = ((voltage / 4.5) * 150 * 6.8947448) - atmospheric_pressure
        pressure_list.append([time.perf_counter(), pressure])

def onPositionChange(self, position):
    if is_experimenting:
        current_position_mm = -position * RESCALE_FACTOR * pitch  # Transrate measured position from steps to mm
        position_list.append([time.perf_counter(), current_position_mm])

class Phidget():
    def __init__(self):
        self.init_stepper()
        self.init_ForceSensor()
        self.init_PressureSensor()
    
    def init_stepper(self):
        global stepper

        stepper = Stepper()
        stepper.setHubPort(1)
        stepper.setDeviceSerialNumber(hub_serial_number)

        try:
            stepper.openWaitForAttachment(5000)
        except PhidgetException as e:
            raise Exception(f"Error opening device: {e}")
        
        stepper.setDataRate(100)  # frequency range: 0.016667 - 100.000000  [Hz]

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
        
        ForceSensor.setDataRate(50)  # frequency range: 0.016667 - 50.000000  [Hz]

    def init_PressureSensor(self):
        global PressureSensor

        PressureSensor = VoltageInput()
        PressureSensor.setIsHubPortDevice(True)
        PressureSensor.setHubPort(2)
        PressureSensor.setDeviceSerialNumber(hub_serial_number)

        try:
            PressureSensor.openWaitForAttachment(5000)
        except PhidgetException as e:
            raise Exception(f"Error opening device: {e}")
        
        PressureSensor.setSensorType(VoltageSensorType.SENSOR_TYPE_VOLTAGE)
        PressureSensor.setDataRate(1000)  # frequency range: 0.016667 - 1000.000000  [Hz]

    def engage_Sensors(self):
        stepper.setEngaged(True)
        stepper.setOnPositionChangeHandler(onPositionChange)
        ForceSensor.setOnVoltageRatioChangeHandler(onVoltageRatioChange)
        PressureSensor.setOnVoltageChangeHandler(onVoltageChange)

    def disengage_Sensors(self):
        stepper.setEngaged(False)
        stepper.setOnPositionChangeHandler(None)
        ForceSensor.setOnVoltageRatioChangeHandler(None)
        PressureSensor.setOnVoltageChangeHandler(None)

    def close_Sensors(self):
        stepper.setEngaged(False)  # Safely disengage the motor
        stepper.close()
        ForceSensor.close()
        PressureSensor.close()
        
    def set_target_position(self, TARGET_POSITION, velocity, acceleration):
        global target_steps

        # Calcurate target steps from target position (input)
        target_steps = TARGET_POSITION / pitch / RESCALE_FACTOR  # Calcurate steps to reach target position
        velocity_steps = int(velocity / pitch / RESCALE_FACTOR)
        acceleration_steps = int(acceleration / pitch / RESCALE_FACTOR)
        stepper.setTargetPosition(target_steps)
        stepper.setVelocityLimit(velocity_steps)
        stepper.setAcceleration(acceleration_steps)

class Load_data():
    def __init__(self):
        return None
    
    def main(self, length, num):
        OFFSET = self.load_force_offset(length, num)
        GAIN = self.load_force_gain(length, num)
        atomosphere = self.load_pressure_offset()
        # target_position = self.load_target_position(length)

        return OFFSET, GAIN, atomosphere
    
    def load_force_offset(self, length, num):
        with open(path_offset + "Force_Offset.txt", "r") as file:
            data = []

            for line in file:
                values = line.strip().split(",")
                data.append((int(values[0]), float(values[1]), float(values[2]), float(values[3])))

        for entry in data:
            if entry[0] == length:
                return entry[num]

        raise ValueError(f"There is no data of the PGM length : {length}")
    
    def load_force_gain(self, length, num):
        with open(path_offset + "Force_Gain.txt", "r") as file:
            data = []

            for line in file:
                values = line.strip().split(",")
                data.append((int(values[0]), float(values[1]), float(values[2]), float(values[3])))

        for entry in data:
            if entry[0] == length:
                return entry[num]

        raise ValueError(f"There is no data of the PGM length : {length}")
    
    def load_pressure_offset(self):
        with open(path_offset + "Pressure_Offset.txt", "r") as file:
            atmospheric_pressure = float(file.read())

        return atmospheric_pressure
    
    def load_target_position(self, length):
        with open(path_offset + "Target_Displacement.txt", "r") as file:
            data = []

            for line in file:
                values = line.strip().split(",")
                data.append((int(values[0]), float(values[1])))

        for entry in data:
            if entry[0] == length:
                return entry[1]

        raise ValueError(f"There is no data of the PGM length : {length}")

def zero_position(init_displacement, add_displacement, force_threshold):
    print("\nStart to set zero positoin.")
    try:
        phidget.engage_Sensors()  # Attach the position change handler and voltage ratio change handler
        is_zero = True
        is_target_position = False
        phidget.set_target_position(init_displacement, velocity=40, acceleration=60)

        while is_zero:
            force = -(ForceSensor.getVoltageRatio() + OFFSET)* GAIN * gravity
            print(f"force: {force:.2f} [N]")

            if is_target_position and abs(force) <= force_threshold:  # When movement of slider finish and absolute force is smaller than threshold
                is_zero = False

            elif is_target_position:  # When movement of slider finish and absolute force is NOT smaller than threshold
                is_target_position = False
                init_displacement += add_displacement if force > 0 else -add_displacement
                phidget.set_target_position(init_displacement, velocity=40, acceleration=60)
 
            current_position_steps = stepper.getPosition()  # Get current position (steps)

            if not is_target_position and abs(target_steps - current_position_steps) < 1:  # When target position reach
                is_target_position = True

    except Exception as e:
        print(f"An error occurred: {e}")

    finally: 
        print("Zero position reached.")

        try:
            phidget.disengage_Sensors()
            stepper.addPositionOffset(-current_position_steps)
            print("Connection with Phidget are successfully completed.")

        except Exception as e:
            print(f"Error during cleanup: {e}")

def check_target_position():
    running = True
    while running:
        current_position_steps = stepper.getPosition()  # Get the current position of the stepper motor (in steps)

        if abs(current_position_steps - target_steps) < 1:  # When target position has been reached
            print("target position reached")
            running = False

async def connect_to_PGM():
    global pgmController, is_connected
    devices = await BleakScanner.discover(timeout=3.0)

    for device in devices:
        if device.name == 'PGM Controller' and device.address == '08:F9:E0:D2:63:8A':
            pgmController = PgmControllerQBleakClient(device, 'PgmController')
            await pgmController.start()
            is_connected = True

    # for device in devices:
    #     print(device.name, device.address)
    # sys.exit()

async def Control_PGM(start_time, time_Before_Contraction, time_During_Contraction, time_After_Contraction):
    global is_experimenting

    command_sent = 0
    interval = 0.005  # [s]
    is_contracted = False
    is_end = False

    while is_experimenting:
        elapsed_time = time.perf_counter() - start_time
        print(f"elapsed time: {round(elapsed_time, 3)} [s]")

        # Contraction
        if (elapsed_time >= time_Before_Contraction) and not is_contracted:
            await pgmController.setPgmParameters(
                channel=0,
                contractionMode=pgmController.PERMANENT_CONTRACTION_MODE,
                cycleTime=0,
                contractionTime=0,
                delayTime=0,
                dutyRatio=100
            )
            is_contracted = True
            command_sent = 1

        elif (elapsed_time >= (time_Before_Contraction + time_During_Contraction)) and not is_end:
            await pgmController.setPgmParameters(
                channel=0,
                contractionMode=pgmController.EMERGENCY_STOP,
                cycleTime=0,
                contractionTime=0,
                delayTime=0,
                dutyRatio=100
            )
            is_end = True
            command_sent = 0

        elif elapsed_time > (time_Before_Contraction + time_During_Contraction + time_After_Contraction):
            is_experimenting = False

        command_list.append([time.perf_counter(), command_sent])
        time.sleep(interval)

def data_collection(start_time):
    data_lines = []  # A list for save all results

    # Combine all experimental results in dat_lines
    for data_list in [command_list, force_list, pressure_list]:
        time_col = [round(row[0] - start_time, 3) for row in data_list]
        value_col = [row[1] for row in data_list]
        data_lines.append(time_col)
        data_lines.append(value_col)

    combined = list(zip_longest(*data_lines, fillvalue=np.nan))  # Transpose data_lines

    columns = ['Command_time', 'Command_value',
               'Force_time', 'Force_value',
               'Pressure_time', 'Pressure_value']

    results = pd.DataFrame(combined, columns=columns)
    return results

def data_synchronize(results):  # Assume that the input "results" converted into relative time
    data_frequency = 50
    time_record = np.linspace(0, 4, 4*data_frequency+1)

    results_interp = []
    results_interp.append(time_record)

    nearest_command_values = []
    for t in time_record:
        idx = np.abs(results['Command_time'] - t).argmin()
        nearest_command_values.append(results['Command_value'][idx])
    results_interp.append(nearest_command_values)

    for col in [['Force_time', 'Force_value'], ['Pressure_time', 'Pressure_value']]:
        results_interp.append(np.interp(time_record, results[col[0]], results[col[1]]))
    
    results_df = pd.DataFrame(list(zip_longest(*results_interp, fillvalue="")),
                              columns=["Time [s]", 'Command [-]', 'Force [N]', 'Pressure [kPa]'])
    return results_df

def save_results(filename, results):
    if not os.path.isdir(path_savefolder):
        os.makedirs(path_savefolder)
    else:
        pass

    try:
        results_path = os.path.join(path_savefolder, filename)
        results.to_csv(results_path, index=None)
        print(f"Experiment's result is saved to {results_path}.")

    except Exception as e:
        print(f'An error occured during save: {e}')


########################################################################################################################

async def main():
    global is_connected, is_experimenting
    global OFFSET, GAIN, atmospheric_pressure
    global phidget

    ####################################################################################################################
    ### PREPARATION ###
    print('\n===== Start preparation =====')

    try:  # load offsets
        load = Load_data()
        OFFSET, GAIN, atmospheric_pressure = load.main(PGM_length, PGM_number)
    except Exception as e:
        print(f"An error occured during offset loading: {e}")

    try:  # Preparation of Phidgets
        phidget = Phidget()
        print('Phidgets initialised.')
    except Exception as e:
        print(f"An error occured during phidget connection: {e}")

    try:  # Connect to PGM
        is_connected = False
        await connect_to_PGM()
        while not is_connected:
            time.sleep(0.1)
        print('PGM connected.'), time.sleep(1)
    except Exception as e:
        print(f"An error occured during PGM connection: {e}")

    try:  # Zero point setting
        await pgmController.setPgmParameters(
                    channel=0,
                    contractionMode=pgmController.PERMANENT_CONTRACTION_MODE,
                    cycleTime=0,
                    contractionTime=0,
                    delayTime=0,
                    dutyRatio=100
                )
        time.sleep(0.5)
        zero_position(init_displacement=-2, add_displacement=0.1, force_threshold=0.02), time.sleep(0.5)
        await pgmController.setPgmParameters(
                    channel=0,
                    contractionMode=pgmController.EMERGENCY_STOP,
                    cycleTime=0,
                    contractionTime=0,
                    delayTime=0,
                    dutyRatio=100
                )
        time.sleep(1)
    except Exception as e:
        print(f"An error occured during zero position setting: {e}")

    try:  # Move to initial displlacement
        print(f"\nMove to target position: {target_position} mm")
        phidget.engage_Sensors()
        phidget.set_target_position(-target_position, velocity=5, acceleration=60)
        check_target_position()
        time.sleep(1)
    except Exception as e:
        print(f"An error occured during target position setting: {e}")

    ####################################################################################################################
    ### EXPERIMENT ###
    print('\n===== Start experiment ====='), time.sleep(0.5)
    is_experimenting = True
    start_time = time.perf_counter()

    try:
        await Control_PGM(
                    start_time = start_time,
                    time_Before_Contraction = time_Before_Contraction,
                    time_During_Contraction = time_During_Contraction,
                    time_After_Contraction  = time_During_Contraction
                )
        
    except Exception as e:
        print(f'An error occured: {e}')
        pass


    ####################################################################################################################
    ### CLOSING ###
    finally:
        time.sleep(0.5)
        print('\n===== Closing ====='), time.sleep(1)
        print(f"\nMove to the initial position")
        phidget.set_target_position(0, velocity=5, acceleration=60)
        check_target_position()
        time.sleep(1)

        phidget.close_Sensors(), print("Phidgets disconnected.")
        await pgmController.stop(), print("PGM disconnected.")

        if save:
            results = data_collection(start_time)
            save_results(filename + "_original.csv", results)

            results = data_synchronize(results)
            save_results(filename + "_synchronized.csv", results)


if __name__ == "__main__":
    asyncio.run(main())