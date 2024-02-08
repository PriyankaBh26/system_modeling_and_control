import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import argparse

def main():
    # Create argument parser
    parser = argparse.ArgumentParser(description="This program plots various solution csv.")

    # Add necessary argument
    parser.add_argument("directory", help="Location of solution csv")
    parser.add_argument("problem", help="problem name for solution csv")

    # Add optional argument
    parser.add_argument("-ctrl", "--control_history", help="If control history csv is present type 1")
    parser.add_argument("-meas", "--measurement_history", help="If measurement history csv is present type 1")
    parser.add_argument("-est", "--estimated_state_history", help="If estimated state history csv is present type 1")

    # Parse arguments
    args = parser.parse_args()

    # Access and process necessary argument
    directory = args.directory
    print("solution data directory:", directory)

    problem = args.problem
    print("problem:", problem)

    y = pd.read_csv(f"examples/{problem}_solution.csv", sep="\s+")
    states = y.columns
    y = y.to_numpy()
    t = pd.read_csv(f"examples/{problem}_time.csv", sep=" ")
    t = t.to_numpy()
    num_states = y.shape[1] # number of columns of df

    # plot pos, vel
    fig1, ax1 = plt.subplots(num_states, figsize=(10, 8))
    fig1.suptitle(f"{problem}")
    for j in range(num_states):
        print(f"plotting {states[j]}")
        ax1[j].plot(t, y[:,j], label = states[j])
        ax1[j].grid(True)
        ax1[j].legend()

    # Access and process optional argument if provided
    control_history = args.control_history
    measurement_history = args.measurement_history
    estimated_state_history = args.estimated_state_history

    if control_history:
        print("Control history is present")
        u = pd.read_csv(f"examples/{problem}_control_history.csv", sep="\s+")
        control_ips = u.columns
        num_control_ips = control_ips.size
        u = u.to_numpy()
        # Create a figure and axis
        fig2, ax2 = plt.subplots(num_control_ips)
        fig2.suptitle(f"{problem} Control Inputs")
        for k in range(num_control_ips):
            ax2[k].plot(t, u[:,k], label = states[k])
            ax2[k].grid(True)
            ax2[k].legend()

    if measurement_history:
        print("Measurement history is present")
        y_meas = pd.read_csv(f"examples/{problem}_meas_history.csv", sep="\s+")
        y_meas = y_meas.to_numpy()
        for j in range(num_states):
            ax1[j].plot(t, y_meas[:,j], label = "meas_" + states[j])
            ax1[j].grid(True)
            ax1[j].legend()

    if estimated_state_history:
        print("Estimated state history is present")
        y_est = pd.read_csv(f"examples/{problem}_est_history.csv", sep="\s+")
        y_est = y_est.to_numpy()
        for j in range(num_states):
            ax1[j].plot(t, y_est[:,j], label = "est_" + states[j])
            ax1[j].grid(True)
            ax1[j].legend()
    
    # Adjust layout to prevent overlapping
    plt.tight_layout()

    plt.show()

if __name__ == "__main__":
    main()
