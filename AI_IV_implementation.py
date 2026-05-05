import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
from math import atan2, acos, sqrt, sin, cos
import csv
import pandas as pd
import os
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
import keras

#INVERSE KINEMATIC
def inverse_k(x, y, z):   
    # Link Dimensions
    ped_offset = 0.1
    a1 = 0.4
    a2 = 0.3
    a3 = 0.4

    # Base rotation (link 1)
    theta1 = atan2(y, x)

    # Reduce to 2D Plane
    hor_distance = sqrt(x^2 + y^2)
    vert_distance = z - ped_offset - a1

    # Distance
    diagonal_distance = (hor_distance^2 + vert_distance^2 - a2^2 - a3^2) / (2 * a2 * a3)
    diagonal_distance = max(-1, min(1, diagonal_distance))


    # Elbow (link 3)
    theta3 = acos(diagonal_distance)    

    # Shoulder (link 2)
    theta2 = atan2(hor_distance, vert_distance) - atan2(a3 * sin(theta3), a2 + a3 * cos(theta3))
    
    return theta1, theta2, theta3

rows = []
size = 10000

#Create Dataset
output_file_raw = "inverse_kinematics_data.csv"
if not os.path.exists(output_file_raw):
    while len(rows) < size:
        x = np.random.uniform(-1, 1)
        y = np.random.uniform(-1, 1)
        z = np.random.uniform(0, 1)
        
        try:
            theta1, theta2, theta3 = inverse_k(x, y, z)
            rows.append({
                "x": x,
                "y": y,
                "z": z,
                "theta1": theta1,
                "theta2": theta2,
                "theta3": theta3
            })

        except ValueError as e:
            print(f"{e}")
        
        
    df = pd.DataFrame(rows)
    df.to_csv(output_file_raw, index=False)

else:
    df = pd.read_csv(output_file_raw)

#Create Normalised Data
output_file_normalised = "inverse_kinematics_data_normalised.csv"
rows = []
if not os.path.exists(output_file_normalised):
    for index, row in df.iterrows():
        x = (row["x"] + 1) / 2
        y = (row["y"] + 1) / 2
        z = row["z"]

        theta1 = (row["theta1"] + np.pi) / (2 * np.pi)
        theta2 = (row["theta2"] + np.pi/2) / np.pi
        theta3 = row["theta3"] / np.pi

        rows.append({
            "x": x,
            "y": y,
            "z": z,
            "theta1": theta1,
            "theta2": theta2,
            "theta3": theta3
        })

    df = pd.DataFrame(rows)
    df.to_csv(output_file_normalised, index=False)

else:
    df = pd.read_csv(output_file_normalised)


    

#Create or Load Neural Network
model_file = "ik_model.keras"
if not os.path.exists(model_file):

    #Create Training and Testing Data
    x = df[["x", "y", "z"]].values
    y = df[["theta1", "theta2", "theta3"]].values
    x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.2, random_state=42)

    #Build Model
    model = keras.Sequential([
        keras.layers.Input(shape=(3,)),
        keras.layers.Dense(64, activation='relu'),
        keras.layers.Dense(64, activation='relu'),
        keras.layers.Dense(64, activation='relu'),
        keras.layers.Dense(3, activation='tanh')
    ])

    #Compile
    model.compile(
        optimizer='adam',
        loss='mse',
        metrics=['mae']
    )

    #Train
    history = model.fit(
        x_train, y_train,
        validation_data=(x_test, y_test),
        epochs=100,
        batch_size=32,
        verbose=1
    )

    loss, mae = model.evaluate(x_test, y_test)
    print("Test Loss:", loss)
    print("Test MAE:", mae)

    #Save Model
    model.save("ik_model.keras")

else:
    model = keras.models.load_model(model_file)

