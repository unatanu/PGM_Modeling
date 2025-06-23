import time
import os
import pandas as pd
import asyncio
import datetime
from bleak import BleakScanner
from PgmController import PgmControllerQBleakClient
from Phidget22.Phidget import *
from Phidget22.Devices.VoltageInput import *
from Phidget22.Devices.Stepper import *
from Phidget22.Devices.VoltageRatioInput import *

#===============================================================================================================================================================
# Constants
hub_serial_number = 766651
abs_path = "C:/Users/bakeratta/Documents/SmartAssist/"

target_pressure = float(input("target_pressure [MPa] : "))
PGM_length = int(input("PGM length [mm] : "))  # The length of PGM used in this experiment ([mm])
PGM_number = int(input("PGM number (1 ~ 3) : "))

CALIBRATION_GAIN = -3.1114E+004  # Calibrated by using Phidget Control Panel
path_offset = abs_path + "Code/PGM/"  # OFFSET is loaded in main function
offset_file = "Force_Offset_exp.txt"

save = True
path_savefolder = abs_path + f"Data/PGM/Pressure-Time/Length_{PGM_length}/PGM_number_{PGM_number}/pressure_{target_pressure}"

RESCALE_FACTOR = 0.0003125  # Steps to mm conversion factor
pitch = 5
gravity = 9.80665
is_experiment = False
preparation = True

time_Before_Contraction = 1.0
time_Contraction = 2.0
time_After_Contraction = 1.0
#===============================================================================================================================================================

def load_force_offset(length):
    with open(path_offset + offset_file, "r") as file:
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
    with open(path_offset + "Pressure_Offset.txt", "r") as file:
        atmospheric_pressure = float(file.read())

    return atmospheric_pressure

def onVoltageChange(self, voltage):
    global results, start_time, force

    pressure = ((voltage / 4.5) * 150 * 6.8947448) - atmospheric_pressure

    if preparation:
        print(f"pressure: {pressure:.2f} kPa")

    if is_experiment:
        print(f"pressure: {pressure:.2f} kPa, Force: {force:.2f} N")

        current_time = time.perf_counter() - start_time
        force = -(ForceSensor.getVoltageRatio() - OFFSET)* CALIBRATION_GAIN * gravity 
        results.append([current_time, pressure, commandSent, force])  

def onVoltageRatioChange(self, voltage_ratio):
    global force
    force = -(voltage_ratio - OFFSET)* CALIBRATION_GAIN * gravity 

def onPositionChange(self, position):
    # Transrate measured position from steps to mm
    current_position_mm = position * RESCALE_FACTOR * pitch

    if not is_experiment:
        if force == None:
            print(f"Current position: {current_position_mm:.2f} mm, Force: Has not starated measurement")
        else:
            print(f"Current position: {current_position_mm:.2f} mm, Force: {force:.2f} N")

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
        stepper.setEngaged(True)

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

def set_target_position(target_position):
    global target_steps

    target_steps = target_position / pitch / RESCALE_FACTOR  # Calcurate target steps [steps] from target position [mm]
    stepper.setTargetPosition(target_steps)
    stepper.setVelocityLimit(20000)  # Set velocity limit
    stepper.setAcceleration(40000)  # Set acceleration

def zero_position():
    global force

    # Bool value to control main loop
    running = True
    is_target_position = False

    force = None

    displacement = -2  # Initial displacement to get initial force
    additional_displacement = 0.1
    threshold = 0.02  # Threshold to check wether slider reached zero force position or not

    print("\nStart to set zero positoin.")
    try:
        # Attach the position change handler and voltage ratio change handler
        stepper.setOnPositionChangeHandler(onPositionChange)
        ForceSensor.setOnVoltageRatioChangeHandler(onVoltageRatioChange)

        # Move stage to get initial force value
        set_target_position(displacement)

        while running:
            # When movement of slider finish and absolute force is smaller than threshold
            if is_target_position and abs(force) <= threshold:
                running = False

            elif is_target_position:
                is_target_position = False
                displacement += additional_displacement if force > 0 else -additional_displacement
        
                set_target_position(displacement)

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
            stepper.setEngaged(False)  # Safely disengage the motor
            print("Connection with Phidget has been successfully completed.\n")

        except Exception as e:
            print(f"Error during cleanup: {e}")

async def pressure_preparation():
    global preparation

    PressureSensor.setOnVoltageChangeHandler(onVoltageChange)
    running = True
    while running:
        await pgmController.setPgmParameters(0, 2, 0, 0, 0, 100)
        running = await check_keyboard_input()
    preparation = False
    await pgmController.setPgmParameters(0, 0, 0, 0, 0, 100)
    PressureSensor.setOnVoltageChangeHandler(None)

async def connect_PGM():
    global pgmController
    devices = await BleakScanner.discover(timeout=3.0)
    for device in devices:
        if device.name == 'PGM Controller' and device.address == '30:C6:F7:1D:26:7A':
            # print(device.address)
            # sys.exit()
            pgmController = PgmControllerQBleakClient(device, 'PgmController')
            await pgmController.start()

async def Control_PGM():
    global commandSent, current_time, finish_Contraction

    commandSent = 0
    is_Contracted = False
    finish_Contraction = False

    while True:
        current_time = time.perf_counter() - start_time
        if (current_time > time_Before_Contraction) and not is_Contracted:
            commandSent = 1000 * target_pressure
            await pgmController.setPgmParameters(0, 2, 0, 0, 0, 100)  # Channel, ContractionMode, CycleTime, DelayTime, ContractionTime, DutyRaio
            is_Contracted = True

        elif (current_time > (time_Before_Contraction + time_Contraction)) and not finish_Contraction:
            commandSent = 0
            await pgmController.setPgmParameters(0, 0, 0, 0, 0, 100)
            finish_Contraction = True

        elif current_time > (time_Before_Contraction + time_Contraction + time_After_Contraction):
            break

async def check_keyboard_input():
    try:
        input()
    except (Exception, KeyboardInterrupt):
        running = False
        return running

def check_dir(folder_path):
    if not os.path.isdir(folder_path):
        os.makedirs(folder_path)
        print("created new folder")
    else:
        pass

def save_results(filename):
    results_df = pd.DataFrame(results, columns=['Time [sec]', 'Pressure [kPa]', 'Command Sent [-]', 'Force [N]'])
    path_results = os.path.join(path_savefolder, filename)
    results_df.to_csv(path_results, index=None)

    print("Experiment result saved.")

#===============================================================================================================================================================

async def main():
    global results, force, OFFSET, atmospheric_pressure, start_time, current_time, is_experiment, preparation

    ## PREPARATION ##
    OFFSET = load_force_offset(PGM_length)
    atmospheric_pressure = load_pressure_offset()

    init = Init_Phidget()
    init.init_stepper()
    init.init_ForceSensor()
    init.init_PressureSensor()  # Initialize voltage input sensor

    check_dir(path_savefolder)

    await connect_PGM()
    zero_position()

    stepper.setEngaged(True)  # Set stepper motor again

    stepper.setVelocityLimit(0)  # Keep stopping stepper motor
    stepper.setTargetPosition(1)
 
    await pressure_preparation()
    while preparation:
        time.sleep(0.05)
    time.sleep(2)


    ## EXPERIMENT ##
    is_experiment = True
    for i in range(3):
        results = []
        print("\nStart Experiment.")
        time.sleep(0.5)
        start_time = time.perf_counter()

        PressureSensor.setOnVoltageChangeHandler(onVoltageChange)
        await Control_PGM()

        while not finish_Contraction:
            time.sleep(0.05)
        PressureSensor.setOnVoltageChangeHandler(None)


        ## SAVE ##
        if save:
            filename = f"pressure_{target_pressure}_exp{i+1}.csv"

            results_df = pd.DataFrame(results, columns=['Time [sec]', 'Pressure [kPa]', 'Command Sent [-]', 'Force [N]'])
            path_results = os.path.join(path_savefolder, filename)
            results_df.to_csv(path_results, index=None)
            print(f"Experiment result saved to {filename}.")

        print()
        time.sleep(2)


    ## CLOSING ##
    print("Finished Experiment.")
    await pgmController.stop()
    PressureSensor.close()  # Close voltage input when done
    ForceSensor.close()
    stepper.setEngaged(False)

asyncio.run(main())