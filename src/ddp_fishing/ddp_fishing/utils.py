import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
import pandas as pd
import shutil
from termcolor import colored

def get_csv_filepath(parent_folder: str, file_name: str) -> str:
    """Search for a file within a specified parent folder and its subdirectories."""
    # folder install 
    directory = f"{get_package_share_directory('ddp_controller_publisher')}/../../../../tasks/"
    full_path = os.path.join(directory, parent_folder)
    # print(f'full_path is {full_path}')
    
    for root, dirs, files in os.walk(full_path):
        if file_name in files:
            return os.path.join(root, file_name)
    
    raise FileNotFoundError(f"No file named {file_name} found in {full_path}.")

def dataCallBacks(solver, 
                robot,
                name="test", 
                nA=1,
                WANNA_SAVE=False,
                dtDDP=1e-4, 
                fROS=1000):
    
    """Probably best ot put as a method of the controlling node

    Args:
        solver (_type_): _description_
        robot (_type_): _description_
        K (_type_): _description_
        dtDDP (_type_, optional): _description_. Defaults to 2e-3.
        name (str, optional): _description_. Defaults to "mulinex".
        nA (int, optional): _description_. Defaults to 8.
        nCart (int, optional): _description_. Defaults to 7.
        nState (int, optional): _description_. Defaults to 15.
        WANNA_SAVE (bool, optional): _description_. Defaults to False.
        dtDDP (_type_, optional): _description_. Defaults to 1e-2.
        fROS (int, optional): _description_. Defaults to 500.

    Returns:
        _type_: _description_
    """
    
    if WANNA_SAVE:
        from datetime import datetime 
        current_date = datetime.now()
        day = current_date.day
        hour = current_date.hour
        minute = current_date.minute
        month = current_date.month
        formatted_date = f"{day:02d}_{month:02d}_{hour:02d}_{minute:02d}"
        path_to_save_tasks = os.path.join(os.getcwd(), f"{name}_{formatted_date}")
        
        if os.path.isdir(path_to_save_tasks):
            path_to_save_tasks = path_to_save_tasks + '_' + str(int(np.floor(np.random.rand(1) * 100))) 
        os.mkdir(path_to_save_tasks)
        
    q0 = robot.q0
    u = solver.us.tolist()
    xs = solver.xs.tolist()
    K_fb = solver.K.tolist()
    K_fb_all, big_data = [], []

    uContrlOut = np.zeros((len(u), nA))
    stateOut = np.zeros((len(xs), len(q0)))
    velOut = np.zeros((len(xs), len(q0)))
    
    for i in range(len(u)):
        appControl = u[i] if i != 0 else np.zeros((nA, 1))
        if len(appControl) == 0:
            continue
        for j in range(nA):
            try:
                uContrlOut[i, j] = appControl[j]
            except IndexError:
                break

    for i in range(0, len(xs)):
        appState = xs[:][i]
        for j in range(0, len(q0)):
            stateOut[i, j] = appState[j]
        for jj in range(0, len(q0)):
            velOut[i, jj] = appState[jj + len(q0)]
            
    for i in range(len(K_fb)):
        if np.shape(K_fb[i])[0] > 0 :
            K_fb_all.append(np.array(K_fb[i][0,:]))
            
    N_new = int(fROS * dtDDP * len(stateOut))
    # print(colored(f'[INFO]:\t From {dtDDP} to dt = 2e-3 sec, data len {len(data_q_new)} --> {N_new}', 'cyan'))
    data_q_new = interpolate_matrix(data_q_new, N_new).tolist()
    data_ff_new = interpolate_matrix(uContrlOut, N_new).tolist()
    data_fb_new = interpolate_matrix(K_fb_all, N_new).tolist()
    data_q_vel_new = interpolate_matrix(velOut, N_new).tolist()

   
    data_q_new = np.asanyarray(data_q_new)
    data_q_vel_new = np.asanyarray(data_q_vel_new)
    data_ff_new = np.asanyarray(data_ff_new)
    
    for i in range(len(data_q_new)):
        for j in range(0, nA):
            big_data.append(data_q_new[i, j])
        for j in range(0, nA):
            big_data.append(data_q_vel_new[i, j])
        for j in range(0, nA):
            big_data.append(data_ff_new[i, j])
    
    return big_data, data_q_new, data_q_vel_new, data_ff_new


def reshape_data(data, n_colums_1=12, n_coloums_2=8):
    """Assume the dimension to comprehend just the joints not the base information"""
    _, cols = data.shape if data.shape[0] > data.shape[1] else data.T.shape
    
    if cols == n_colums_1 or cols == n_coloums_2:
        return data if data.shape[1] == cols else data.T
    else:
        raise ValueError(f"Input data must have {n_colums_1} or {n_coloums_2} columns")
    
def interpolate_matrix(matrix: np.array, N_new: int) -> np.array:
    """Interpolate a matrix Nxm
    Args:
        matrix (np.array): _description_
        N_new (np.array): _description_

    Returns:
        np.array: _description_
    """
    matrix = np.asanyarray(matrix)
    N, num_columns = matrix.shape
    interpolated_matrix = np.zeros((N_new, num_columns))
    for i in range(num_columns):
        interpolated_matrix[:, i] = np.interp(np.linspace(0, 1, N_new), np.linspace(0, 1, N), matrix[:, i])
        
    # print(f'[INFO]:\tOriginal matrix shape     : {matrix.shape}')
    # print(f'[INFO]:\tInterpolated matrix shape : {interpolated_matrix.shape}')
    return interpolated_matrix