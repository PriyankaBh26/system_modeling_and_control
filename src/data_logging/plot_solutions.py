import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import argparse

def legend_without_duplicate_labels(ax):
    handles, labels = ax.get_legend_handles_labels()
    unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) if l not in labels[:i]]
    ax.legend(*zip(*unique))

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
    parser.add_argument("-ref", "--reference_state_history", help="If reference state history csv is present type 1")

    # Parse arguments
    args = parser.parse_args()

    # Access and process necessary argument
    directory = args.directory
    print("solution data directory:", directory)

    problem = args.problem
    print("problem:", problem)

    y = pd.read_csv(f"{directory}/{problem}_state_history.csv", sep="\s+")
    states = y.columns
    y = y.to_numpy()
    t = pd.read_csv(f"{directory}/{problem}_time.csv", sep=" ")
    t = t.to_numpy()
    num_states = y.shape[1] # number of columns of df

    # plot pos, vel
    fig1, ax1 = plt.subplots(num_states, figsize=(10, 8))
    fig1.suptitle(f"{problem}")
    for j in range(num_states):
        print(f"plotting {states[j]}")
        if (num_states == 1):
            ax = ax1
        else:
            ax = ax1[j]
        ax.plot(t, y[:,j], label = states[j])
        ax.grid(True)
        legend_without_duplicate_labels(ax)

    # Access and process optional argument if provided
    control_history = args.control_history
    measurement_history = args.measurement_history
    estimated_state_history = args.estimated_state_history
    reference_state_history = args.reference_state_history

    if control_history:
        print("Control history and error history is present")
        u = pd.read_csv(f"{directory}/{problem}_control_history.csv", sep="\s+")
        control_ips = u.columns
        num_control_ips = control_ips.size
        u = u.to_numpy()

        error = pd.read_csv(f"{directory}/{problem}_err_history.csv", sep="\s+")
        err_ips = error.columns
        num_err_ips = err_ips.size
        error = error.to_numpy()

        # Create a figure and axis
        fig2, ax2 = plt.subplots(num_control_ips, figsize=(10, 8))
        fig2.suptitle(f"{problem} Control Inputs")
        for k in range(num_control_ips):
            if (num_control_ips == 1):
                ax = ax2
            else:
                ax = ax2[k]
            ax.plot(t, u[:,k], label = control_ips[k])
            ax.grid(True)
            legend_without_duplicate_labels(ax)

        fig3, ax3 = plt.subplots(num_err_ips, figsize=(10, 8))
        fig3.suptitle(f"{problem} Error")
        for k in range(num_err_ips):
            if (num_err_ips == 1):
                ax = ax3
            else:
                ax = ax3[k]
            ax.plot(t, error[:,k], label = "err_" + err_ips[k])
            ax.grid(True)
            legend_without_duplicate_labels(ax)

    if measurement_history:
        print("Measurement history is present")
        y_meas = pd.read_csv(f"{directory}/{problem}_meas_history.csv", sep="\s+")
        y_meas = y_meas.to_numpy()
        num_meas_states = y_meas.shape[1]
        for j in range(num_meas_states):
            if (num_states == 1):
                ax = ax1
            else:
                ax = ax1[j]
            ax.plot(t, y_meas[:,j], label = "meas_" + states[j])
            ax.grid(True)
            legend_without_duplicate_labels(ax)

    if estimated_state_history:
        print("Estimated state history is present")
        y_est = pd.read_csv(f"{directory}/{problem}_est_history.csv", sep="\s+")
        y_est = y_est.to_numpy()
        num_est_states = y_est.shape[1]
        for j in range(num_est_states):
            if (num_states == 1):
                ax = ax1
            else:
                ax = ax1[j]
            ax.plot(t, y_est[:,j], label = "est_" + states[j])
            ax.grid(True)
            legend_without_duplicate_labels(ax)

    if reference_state_history:
        print("Reference state history is present")
        y_ref = pd.read_csv(f"{directory}/{problem}_ref_history.csv", sep="\s+")
        y_ref = y_ref.to_numpy()
        num_ref_states = y_ref.shape[1]
        for j in range(num_ref_states):
            if (num_states == 1):
                ax = ax1
            else:
                ax = ax1[j]
            ax.plot(t, y_ref[:,j], label = "ref_" + states[j])
            ax.grid(True)
            legend_without_duplicate_labels(ax)
    
    # Adjust layout to prevent overlapping
    plt.tight_layout()

    plt.show()

if __name__ == "__main__":
    main()
