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
from itertools import zip_longest
import numpy as np

##################################################################################################################################
# Input constant values
hub_serial_number = 766651
abs_path = "C:/Users/bakeratta/Documents/SmartAssist/"

PGM_length = int(input("PGM length [mm] : "))  # The length of PGM used in this experiment ([mm])
PGM_number = int(input("PGM number (1 ~ 3) : "))
target_pressure = int(input("target pressure [kPa] : "))

save = True
# path_savefolder = abs_path + f"Data/PGM/Velocity-Force/exp2/Length_{PGM_length}/PGM_number_{PGM_number}/pressure_{target_pressure}/"
path_savefolder = abs_path + f"Data/PGM/Velocity-Force/exp2/test/"
path_offset = abs_path + "Code/Texts/"

acceleration = 70  # [mm/s^2]

gravity = 9.80665  # Gravitational accelation ([m//s^2])
RESCALE_FACTOR = 0.0003125  # Rescale factor for stepper motor ([rotation/steps])
pitch = 5  # Pitch of ball screw ([mm/rotation])

errors = []
results = []

is_experimenting = False
##################################################################################################################################

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
        current_position_mm = - position * RESCALE_FACTOR * pitch  # Transrate measured position from steps to mm
        position_list.append([time.perf_counter(), current_position_mm])

        force = -(ForceSensor.getVoltageRatio() + OFFSET)* GAIN * gravity
        print(f"Position: {current_position_mm:.2f} mm, Force: {force:.2f} N")

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

        try:
            PressureSensor.openWaitForAttachment(5000)
        except PhidgetException as e:
            raise Exception(f"Error opening device: {e}")
        
        PressureSensor.setSensorType(VoltageSensorType.SENSOR_TYPE_VOLTAGE)
        PressureSensor.setDataRate(50)

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

def zero_position(init_displacement, add_displacement, force_threshold):
    global is_initialized

    print("\nStart to set zero positoin.")
    try:
        phidget.engage_Sensors()  # Attach the position change handler and voltage ratio change handler
        is_zero = True
        is_target_position = False
        phidget.set_target_position(init_displacement, 40, acceleration)

        while is_zero:
            force = -(ForceSensor.getVoltageRatio() + OFFSET)* GAIN * gravity
            # print(f"force: {force:.2f} [N]")

            if is_target_position and abs(force) <= force_threshold:  # When movement of slider finish and absolute force is smaller than threshold
                is_zero = False

            elif is_target_position:  # When movement of slider finish and absolute force is NOT smaller than threshold
                is_target_position = False
                init_displacement += add_displacement if force > 0 else -add_displacement
                phidget.set_target_position(init_displacement, 40, acceleration)
 
            current_position_steps = stepper.getPosition()  # Get current position (steps)

            if not is_target_position and abs(target_steps - current_position_steps) < 1:  # When target position reach
                is_target_position = True

    except Exception as e:
        print(f"An error occurred: {e}")

    finally: 
        print("Zero position has been reached.")

        try:
            phidget.disengage_Sensors()
            stepper.addPositionOffset(-current_position_steps)
            is_initialized = True
            print("Connection with Phidget has been successfully completed.\n")

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

def data_collection(start_time):
    data_lines = []  # A list for save all results

    # 1列ずつ取り出してdata_linesに格納
    for data_list in [position_list, force_list, pressure_list]:
        time_col = [round(row[0] - start_time, 3) for row in data_list]
        value_col = [row[1] for row in data_list]
        data_lines.append(time_col)
        data_lines.append(value_col)

    # data_linesを転置
    combined = list(zip_longest(*data_lines, fillvalue=np.nan))

    columns = ['Position_time', 'Position_value',
               'Force_time', 'Force_value',
               'Pressure_time', 'Pressure_value']

    results = pd.DataFrame(combined, columns=columns)
    return results

def data_synchronize(results):  # 相対時刻に変換し、DataFrameとして保存されていることを前提とする
    results_interp = []
    for col in ['Position_time', 'Position_value']:
        results_interp.append(results[col])

    # time_recoordのときのデータを取得・線形補完
    for col in [['Force_time', 'Force_value'], ['Pressure_time', 'Pressure_value']]:
        results_interp.append(np.interp(results['Position_time'], results[col[0]], results[col[1]]))
    
    # 結果を出力
    results_df = pd.DataFrame(list(zip_longest(*results_interp, fillvalue="")),
                              columns=["Time [s]", 'Position [mm]', 'Force [N]', 'Pressure [kPa]'])
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

##################################################################################################################################

async def main():
    global start_time, phidget
    global OFFSET, GAIN, atmospheric_pressure
    global is_initialized, is_connected, is_experimenting
    global position_list, force_list, pressure_list

    ##################################################################################################################################
    ## PREPARATION ##
    print('\n===== Start preparation =====')
    try:  # オフセットの読み込み
        load = Load_data()
        OFFSET, GAIN, atmospheric_pressure = load.main(PGM_length, PGM_number)
    except Exception as e:
        print(f"An error occured during offset loading: {e}")

    try:  # Phidgetの準備
        phidget = Phidget()
        print('Phidgets initialised.')
    except Exception as e:
        print(f"An error occured during phidget connection: {e}")

    try:  # PGMの接続
        is_connected = False
        await connect_to_PGM()
        while not is_connected:
            time.sleep(0.1)
        print('PGM connected.'), time.sleep(1)
    except Exception as e:
        print(f"An error occured during PGM connection: {e}")


    ##################################################################################################################################
    ## EXPERIMENT ##
    print('\n===== Start experiment =====')
    await pgmController.setPgmParameters(
         channel=0,
         contractionMode=pgmController.PERMANENT_CONTRACTION_MODE,
         cycleTime=0,
         contractionTime=0,
         delayTime=0,
         dutyRatio=100
        )
    time.sleep(2)
    
    for i in range(5):
        velocity = i+1 if i<4 else 5*(i-3)
        target_position = 2*velocity if i<6 else 30

        force_list = []
        pressure_list = []
        position_list = []

        is_initialized = False  # To avoid recording during initialization
        zero_position(init_displacement=-2, add_displacement=0.1, force_threshold=0.02)

        print(f"\n===== Target velocity: {velocity} [mm/s] =====\n"), time.sleep(1)
        start_time = time.perf_counter()
        try:
            phidget.engage_Sensors()

            print("Preparing for experiment")
            phidget.set_target_position(-target_position, 5, acceleration)
            check_target_position()
            time.sleep(0.5)

            print("\nMeasure damping force during NEGATIVE velocity")
            is_experimenting = True
            phidget.set_target_position(target_position, velocity, acceleration)
            check_target_position()
            time.sleep(0.5)

            print("\nMeasure damping force during POSITIVE velocity")
            phidget.set_target_position(-target_position, velocity, acceleration)
            check_target_position()
            time.sleep(0.5)

            print("\nReturn to the zero position")
            is_experimenting = False
            phidget.set_target_position(0, 5, acceleration)
            check_target_position()

        except Exception as e:
            print(f"An error occurred at velocity {velocity}")
            errors.append((velocity, "Experiment", str(e)))  # store the error message and continue experiments


        ##################################################################################################################################
        ## SAVE ##
        finally: 
            print("\nExperiment finished.")
            phidget.disengage_Sensors()
            
            if save:
                try:
                    filename = f"pressure_{target_pressure}_vel_{int(velocity)}"

                    results = data_collection(start_time)
                    save_results(filename + "_original.csv", results)

                    results = data_synchronize(results)
                    save_results(filename + "_synchronized.csv", results)
                except Exception as e:
                    print(f"Some error occured durinig save at velocity {velocity}")
                    errors.append((velocity, "Save", str(e)))

    if errors:
        print("\n===== Summary of errors =====")
        for vel, stage, msg in errors:
            print(f"velocity {vel} - Stage: {stage} - Error: {msg}")
    else:
        print("\nAll experiments completed successfully.")


    ##################################################################################################################################
    ## CLOSING ##
    try:
        phidget.close_Sensors(), print("Phidgets disconnected.")
        await pgmController.setPgmParameters(
         channel=0,
         contractionMode=pgmController.EMERGENCY_STOP,
         cycleTime=0,
         contractionTime=0,
         delayTime=0,
         dutyRatio=100
        )
        await pgmController.stop(), print("PGM disconnected.")

    except Exception as e:
        print(f"Error during cleanup: {e}")


##################################################################################################################################
asyncio.run(main())