import argparse
import yaml

import numpy as np
import matplotlib.pyplot as plt

class Mission:
    def __init__(self, data):
        self.data = data

    def analyze(self):
        raise NotImplementedError
    def merge(self):
        raise NotImplementedError

# class CovarianceEstimationExperiment(Experiment):
#     def print_comparison(self):
#         print(f"Mission: {self.data['mission_definition']['nazev']}")
#         for requested, real in zip(self.data['requested_postions'], self.data['real_positions']):
#             print(f"Requested Position: ", end="")
#             for num in requested:
#                 print(f"{num:7.3f}", end=", ")
#             print(f"\n     Real Position: ", end="")
#             for num in real:
#                 print(f"{num:7.3f}", end=", ")
#             print("\n")

class CovarianceEstimationMission(Mission):
    def __init__(self, data):
        super().__init__(data)
        self.requested_positions = np.array(data['requested_postions'])
        self.real_positions = np.array(data['real_positions'])
        self.differences = self.real_positions - self.requested_positions
        self.differences[:,3] = (self.differences[:,3] + 180)%360-180
        self.variables = ["x", "y", "z", "yaw"]

    def merge(self, other):
        new = CovarianceEstimationMission(self.data)
        new.requested_positions = np.concatenate((self.requested_positions, other.requested_positions))
        new.real_positions = np.concatenate((self.real_positions, other.real_positions))
        new.differences = np.concatenate((self.differences, other.differences))
        return new

    def print_comparison(self):
        print(f"Mission: {self.data['mission_definition']['nazev']}")
        for requested, real, difference in zip(self.requested_positions, self.real_positions, self.differences):
            print("Requested Position: ", end="")
            self._print_numbers(requested)
            print("     Real Position: ", end="")
            self._print_numbers(real)
            print("        Difference: ", end="")
            self._print_numbers(difference)
            print()

    def _print_numbers(self, numbers):
        for num in numbers:
            print(f"{num:7.3f}", end=", ")
        print()

    def plot_histograms(self):
        num_variables = self.differences.shape[1]
        fig, axs = plt.subplots(1, num_variables, figsize=(15, 5))
        for i in range(num_variables):
            axs[i].hist(self.differences[:, i], weights=np.zeros_like(self.differences[:, i]) + 1. / self.differences[:, i].size, bins=10, edgecolor='black', )
            axs[i].set_title(self.variables[i])
            axs[i].set_xlabel('Difference')
            axs[i].set_ylabel('Frequency')
        plt.show()

    def print_statistics(self):
        print(f"Samples: {self.differences.shape[0]}")
        mean = np.mean(self.differences, axis=0)
        covariance_matrix = np.cov(self.differences, rowvar=False)
        
        print("Mean of Differences:")
        self._print_numbers(mean)
        
        print("\nCross-Covariance Matrix of Differences:")
        print(covariance_matrix)

    def analyze(self):
        self.print_comparison()
        self.print_statistics()
        self.plot_histograms()


class MeasurementCovarianceEstimationMission(Mission):
    def __init__(self, data):
        super().__init__(data)
        self.measured_positions = np.array(data['measured_postions'])
        self.real_positions = np.array(data['real_positions'])
        self.differences = self.real_positions - self.measured_positions
        self.differences[:,3] = (self.differences[:,3] + 180)%360-180
        self.variables = ["x", "y", "z", "yaw"]

    def merge(self, other):
        new = MeasurementCovarianceEstimationMission(self.data)
        new.measured_positions = np.concatenate((self.measured_positions, other.measured_positions))
        new.real_positions = np.concatenate((self.real_positions, other.real_positions))
        new.differences = np.concatenate((self.differences, other.differences))
        return new

    def print_comparison(self):
        print(f"Mission: {self.data['mission_definition']['nazev']}")
        for requested, real, difference in zip(self.measured_positions, self.real_positions, self.differences):
            print("Measured Position: ", end="")
            self._print_numbers(requested)
            print("    Real Position: ", end="")
            self._print_numbers(real)
            print("       Difference: ", end="")
            self._print_numbers(difference)
            print()

    def _print_numbers(self, numbers):
        for num in numbers:
            print(f"{num:7.3f}", end=", ")
        print()

    def plot_histograms(self):
        num_variables = self.differences.shape[1]
        fig, axs = plt.subplots(1, num_variables, figsize=(15, 5))
        for i in range(num_variables):
            axs[i].hist(self.differences[:, i], weights=np.zeros_like(self.differences[:, i]) + 1. / self.differences[:, i].size, bins=10, edgecolor='black', )
            axs[i].set_title(self.variables[i])
            axs[i].set_xlabel('Difference')
            axs[i].set_ylabel('Frequency')
        plt.show()

    def plot_measurements_real(self):
        num_variables = self.differences.shape[1]
        fig, axs = plt.subplots(1, num_variables, figsize=(15, 5))
        for i in range(num_variables):
            axs[i].plot(self.measured_positions[:,i])
            axs[i].plot(self.real_positions[:,i])
            axs[i].set_title(self.variables[i])
            axs[i].set_xlabel('Difference')
            axs[i].set_ylabel('Frequency')
        plt.show()

    def print_statistics(self):
        print(f"Samples: {self.differences.shape[0]}")
        mean = np.mean(self.differences, axis=0)
        covariance_matrix = np.cov(self.differences, rowvar=False)
        
        print("Mean of Differences:")
        self._print_numbers(mean)
        
        print("\nCross-Covariance Matrix of Differences:")
        print(covariance_matrix)

    def analyze(self):
        self.print_comparison()
        self.print_statistics()
        self.plot_histograms()
        self.plot_measurements_real()

def main():
    parser = argparse.ArgumentParser(description='Process experiments stored in YAML files')
    parser.add_argument('yaml_files', type=str, nargs='+', help='Paths to the YAML files containing experiment data')
    args = parser.parse_args()

    experiment_data = []

    for yaml_file in args.yaml_files:
        with open(yaml_file, 'r') as file:
            experiment_data.extend(yaml.full_load(file))

    missions_by_name = {}
    missions_by_type = {}

    for mission in experiment_data:
        name = mission['mission_definition']['nazev']
        if 'covx_0' in mission['mission_definition']['nazev']:
            cov_mission = CovarianceEstimationMission(mission)
        elif 'covx_k' in mission['mission_definition']['nazev'] or 'oprava' in mission['mission_definition']['nazev']:
            cov_mission = MeasurementCovarianceEstimationMission(mission)
        if name not in missions_by_name:
            missions_by_name[name] = []
        if type(cov_mission) not in missions_by_type:
            missions_by_type[type(cov_mission)] = []
        missions_by_name[name].append(cov_mission)
        missions_by_type[type(cov_mission)].append(cov_mission)

    merged_missions = []
    for name, missions in missions_by_name.items():
        if len(missions) > 1:
            merged_mission = missions[0]
            for exp in missions[1:]:
                merged_mission = merged_mission.merge(exp)
            merged_missions.append(merged_mission)
        else:
            merged_missions.append(missions[0])

    merged_missions_by_type = []
    for mission_type, missions in missions_by_type.items():
        if len(missions) > 1:
            merged_mission = missions[0]
            for exp in missions[1:]:
                merged_mission = merged_mission.merge(exp)
            merged_missions_by_type.append(merged_mission)
        else:
            merged_missions_by_type.append(missions[0])

    for mission in merged_missions:
        mission.analyze()

    print("\n\nMerged all by type: \n")
    for mission in merged_missions_by_type:
        mission.analyze()

if __name__ == "__main__":
    main()