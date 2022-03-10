import csv
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


def load_csv(file_results_path):
    with open(file_results_path, 'r') as csvfile:
        plots = csv.reader(csvfile, delimiter=',')
        data = []
        for row in plots:
            data.append(row)
    return data


def plot_column(planners, data, id, viz_number, title):
    # plt.subplot(2, 2, id)
    plt.figure()
    plt.title(title)
    plt.bar(planners[:viz_number], data[:viz_number],
            width=0.5,  align='center')
    plt.grid(axis='y', alpha=0.75)
    plt.xticks(rotation=60)


def main():
    file_results_path = "/home/marios/thesis_ws/src/drone_path_planning/resources/planners_results.csv"
    data = load_csv(file_results_path)
    planners = data[0]

    data = np.array(data[1:]).astype(np.float64).T
    print(data.shape)

    planners_number = -1  # number of planners to visualize

    # rename big names
    planners = np.array(planners)
    planners = np.where(planners == 'NearestNeighbors', 'NNs', planners)
    planners = np.where(
        planners == 'NearestNeighborsLinear', 'NNLinear', planners)

    # remove unwanted columns
    cols_to_delete = ['NumNeighborsFn',
                      'KStarStrategy', 'KStrategy', 'NNs', 'NNLinear', 'QRRT', 'SPARS', 'SPARStwo']

    for col in cols_to_delete:
        data, planners = delete_col(data, planners, col)

    print(planners)
    time = data[:, 0]
    states_tried = data[:, 1]
    avrg_time_taken_for_each_state = data[:, 2]
    path_cost = data[:, 3]

    plot_column(planners, time, 1, planners_number, "Time")
    plot_column(planners, states_tried, 2, planners_number, "States tried")
    plot_column(planners, avrg_time_taken_for_each_state*1e3, 3,
                planners_number, "Average time taken for each state (msec)")
    plot_column(planners, path_cost, 4, planners_number, "Path cost")

    plt.show()


def delete_col(data, planners, col_name):
    id = np.where(planners == col_name)[0]
    planners = np.delete(planners, id)
    data = np.delete(data, id, axis=0)
    return data, planners


if __name__ == """__main__""":
    main()
    # pandas load csv
    # file_results_path = "/home/marios/thesis_ws/src/drone_path_planning/resources/planners_results.csv"
    # df = pd.read_csv(file_results_path, sep=',').T

    # # save as csv
    # processed_file_path = "/home/marios/thesis_ws/src/drone_path_planning/resources/planners_results_pandas.csv"
    # df.to_csv(processed_file_path, index=True)

    # df = pd.read_csv(processed_file_path, sep=',')
    # df.columns = ["planner", "time", "states_tried",
    #               "avrg_time_taken_for_each_state", "path_cost"]
    # print(df.head())
    # ax = df.plot.bar(x="planner", y="time", rot=0)
    # plt.show()
    # # ax = df.plot(x="X", y="A", kind="bar")
