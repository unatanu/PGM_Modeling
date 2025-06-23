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

TARGET_PRESSURE = float(input("taget pressure [MPa] : "))
PGM_length = int(input("PGM length [mm] : "))  # The length of PGM used in this experiment ([mm])
PGM_number = int(input("PGM number (1 ~ 3) : "))

CALIBRATION_GAIN = -3.1114E+004  # Calibrated by using Phidget Controll Panel
path_offset = abs_path + "Code/PGM/"  # OFFSET is loaded in main function
offset_file = "Force_Offset_exp.txt"

# Save the results to excel
save = True
path_savefolder = abs_path + f"Data/PGM/Velocity-Force/Length_{PGM_length}/PGM_number_{PGM_number}/pressure_{TARGET_PRESSURE}"
analysis_file = "Comparison.csv"

RESCALE_FACTOR = 0.0003125  # Rescale factor for stepper motor ([rotation/steps])
pitch = 5  # Pitch of ball screw ([mm/rotation])
gravity = 9.80665  # Gravitational accelation ([m//s^2])

is_experiment = False  # Is the slider set to the initial position
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
    global pressure
    pressure = ((voltage / 4.5) * 150 * 6.8947448) - atmospheric_pressure

def onVoltageRatioChange(self, voltage_ratio):
    global force
    force = -(voltage_ratio - OFFSET)* CALIBRATION_GAIN * gravity  # Change negative value to positive value

def onPositionChange(self, position):
    global results, current_position_mm, prev_position, prev_time
    current_position_mm = -position * RESCALE_FACTOR * pitch  # Transrate measured position from steps to mm
    current_time = time.perf_counter()

    if force == 0:
        print(f"Position: {current_position_mm:.2f} mm, Force: Has not starated measurement")
    else:
        print(f"Position: {current_position_mm:.2f} mm, Force: {force:.2f} N")

    if is_experiment:
        if (current_time - prev_time) != 0:
            velocity = (current_position_mm - prev_position) / (current_time - prev_time)
        results.append([current_time - start_time, current_position_mm, velocity, force, pressure])
        prev_position = current_position_mm
        prev_time = current_time

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
    acceleration_steps = int(80 / pitch / RESCALE_FACTOR)

    stepper.setTargetPosition(target_steps)
    stepper.setVelocityLimit(velocity_steps)
    stepper.setAcceleration(acceleration_steps)

def zero_position():
    global force

    # Bool value to control main loop
    running = True
    is_target_position = False

    displacement = -2  # Initial displacement to get initial force
    additional_displacement = 0.1
    threshold = 0.02  # Threshold to check wether slider reached zero force position or not

    print("\nStart to set zero positoin.")
    try:
        # Attach the position change handler and voltage ratio change handler
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

        except Exception as e:
            print(f"Error during cleanup: {e}")

async def connect_to_PGM():
    global pgmController
    devices = await BleakScanner.discover(timeout=3.0)
    for device in devices:
        if device.name == 'PGM Controller' and device.address == '30:C6:F7:1D:26:7A':
            # print(device.address)
            # sys.exit()
            pgmController = PgmControllerQBleakClient(device, 'PgmController')
            await pgmController.start()

def check_target_position_reached():
    running = True
    # is_target_position = False

    while running:
        # Get the current position of the stepper motor (in steps)
        current_position_steps = stepper.getPosition()

        # When target position has been reached
        if abs(current_position_steps - target_steps) < 1:
            print("target position reached")
            running = False

def linear_interpolate(x0, y0, x1, y1, x):
    return y0 + (y1 - y0) * ((x - x0) / (x1 - x0))

def check_dir(folderpath):
    if not os.path.isdir(folderpath):
        os.makedirs(folderpath)
    else:
        pass

def check_existance_savefile(file_path):
    if not os.path.exists(file_path):
        velocity = []
        for i in range(-60, 61):
            if abs(i) < 5:  # 絶対値が5未満の場合、1刻み
                if i not in velocity:  # 重複防止
                    velocity.append(i)
            elif i % 5 == 0:  # それ以外の場合、5刻み
                velocity.append(i)

        velocity_range = {
            "velocity": velocity,
        }

        for i in range(-5, 6):
            velocity_range[f"{i}"] = [None] * len(velocity)

        df = pd.DataFrame(velocity_range)

        df.to_csv(file_path, index=False)
        print(f"Created new analysis file: {analysis_file}")
    else:
        pass

def save_results():
    check_dir(path_savefolder)
    results_path = os.path.join(path_savefolder, f"velocity_{VELOCITY}.csv")
    results_df = pd.DataFrame(results, columns=['Time [sec]', 'Position [mm]', 'Velocity [mm/s]', 'Force [N]', 'Pressure [kPa]'])
    results_df.to_csv(results_path, index=None)

    positions = [row[1] for row in results]
    velocities = [row[2] for row in results]
    forces = [row[3] for row in results]

    analysis_path = os.path.join(path_savefolder, analysis_file)
    check_existance_savefile(analysis_path)
    df = pd.read_csv(analysis_path)

    # Iterate for positive and negative velocity ranges
    for i in range(2):
        # Determine target velocity based on sign (positive or negative)
        target_velocity = (-1)**i * VELOCITY
        row_index = df.index[df['velocity'] == target_velocity].tolist()[0]

        # Retrieve or interpolate `force` based on `position` in steps of 5
        for target_pos in range(-5, 6, 1):
            col_name = str(target_pos)
            target_force = None  # Default to None in case no match is found
            
            # Find indices where position matches target_pos and velocity has the correct sign
            indices = [j for j in range(len(positions)) if positions[j] == target_pos and (velocities[j] > 0 if i == 0 else velocities[j] < 0)]
            
            if indices:
                # Direct match found; retrieve corresponding force
                target_force = forces[indices[0]]
            else:
                # Linear interpolation for the closest positions around target_pos
                for j in range(len(positions) - 1):
                    if ((positions[j] < target_pos < positions[j + 1]) and (velocities[j] > 0 and i == 0)):
                        target_force = linear_interpolate(positions[j], forces[j], positions[j + 1], forces[j + 1], target_pos)
                        break

                    if ((positions[j] > target_pos > positions[j + 1]) and (velocities[j] < 0 and i == 1)):
                        target_force = linear_interpolate(positions[j], forces[j], positions[j + 1], forces[j + 1], target_pos)
                        break
            
            df.at[row_index, col_name] = target_force

    df.to_csv(analysis_path, index=False)
    print("Experiment data was saved to CSV.")

#===============================================================================================================================================================

async def main():
    global start_time, force, results, pgmController, prev_position , prev_time, is_experiment, OFFSET, atmospheric_pressure, VELOCITY

    ## PREPARATION ##
    OFFSET = load_force_offset(PGM_length)
    atmospheric_pressure = load_pressure_offset()

    init = Init_Phidget()
    init.init_stepper()
    init.init_ForceSensor()
    init.init_PressureSensor()

    # await connect_to_PGM()
    await connect_to_PGM()
    await pgmController.setPgmParameters(0, 2, 0, 0, 0, 100)  # Channel, CycleTime, DelayTime, ContractionTime, DutyRaio
    time.sleep(0.5)


    ## EXPERIMENT ##
    for i in range(12+4):
        zero_position()
        
        VELOCITY = (i+1) if i < 4 else 5*(i-3)
        TARGET_POSITION = -VELOCITY*2 if VELOCITY <= 5 else -30
        results = []  # Store position and force for later output
        force = 0

        print(f"\nVELOCITY : {VELOCITY}")
        time.sleep(1)
        print("Start experiment.")

        try:
            stepper.setEngaged(True)
            stepper.setOnPositionChangeHandler(onPositionChange)
            ForceSensor.setOnVoltageRatioChangeHandler(onVoltageRatioChange)
            PressureSensor.setOnVoltageChangeHandler(onVoltageChange)

            start_time = time.perf_counter()
            prev_time = time.perf_counter()
            prev_position = stepper.getPosition()

            # Make distance from zero position to reach target velocity
            set_target_position(TARGET_POSITION, VELOCITY)
            check_target_position_reached()

            # Record zero position force when the sign of velocity is negative
            is_experiment = True
            print("\n")
            set_target_position(-TARGET_POSITION, VELOCITY)
            check_target_position_reached()

            # Record zero position force when the sign of velocity is positive
            print("\n")
            set_target_position(TARGET_POSITION, VELOCITY)
            check_target_position_reached()
            is_experiment = False

            # Return to zero position
            print("\n")
            set_target_position(0, VELOCITY)
            check_target_position_reached()

        except Exception as e:
            print(f"An error occurred: {e}")


        ## SAVE ##
        finally: 
            if save:
                save_results()
            print()
            time.sleep(0.5)


    ## CLOSING ##
    try:
        print("\nExpperiment finished.")
        stepper.setEngaged(False)  # Safely disengage the motor
        stepper.close()
        ForceSensor.close()
        PressureSensor.close()
        await pgmController.setPgmParameters(0, 0, 0, 0, 0, 100)
        await pgmController.stop()
        print("Connection with Phidget has been successfully completed.")

    except Exception as e:
        print(f"Error during cleanup: {e}")

asyncio.run(main())