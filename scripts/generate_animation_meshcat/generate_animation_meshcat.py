from idyntree.visualize import MeshcatVisualizer
import resolve_robotics_uri_py
import h5py
from pathlib import Path
import numpy as np

import idyntree.bindings as idyn

from rich.progress import track

timestamps = []
initial_time = 0
end_time = 0


class Index:
    def __init__(self, index=0, is_plot=False):
        self.index = index
        self.is_plot = is_plot


def find_non_zero_chunks_with_threshold(input_vector, threshold):
    # Initialize variables to store the indices of chunks
    chunk_indices = []
    current_start_index = 0

    # Iterate through the vector
    for i in range(len(input_vector)):
        if abs(input_vector[i]) > threshold:
            if current_start_index == 0:
                current_start_index = i
        else:
            if current_start_index != 0:
                chunk_indices.append([current_start_index, i - 1])
                current_start_index = 0

    # Add the last chunk if it's not empty
    if current_start_index != 0:
        chunk_indices.append([current_start_index, len(input_vector) - 1])

    return np.array(chunk_indices)


def find_chunks_with_diff_greater_than_threshold(input_vector, threshold):
    # Initialize variables to store the indices of chunks
    chunk_indices = []
    current_start_index = 0

    # Iterate through the vector
    for i in range(len(input_vector) - 1):
        if abs(input_vector[i + 1] - input_vector[i]) > threshold:
            if current_start_index == 0:
                current_start_index = i
        else:
            if current_start_index != 0:
                chunk_indices.append([current_start_index, i])
                current_start_index = 0

    # Add the last chunk if it's not empty
    if current_start_index != 0:
        chunk_indices.append([current_start_index, len(input_vector) - 1])

    return np.array(chunk_indices)


def __populate_numerical_data(file_object):
    global timestamps
    global initial_time
    global end_time
    data = {}
    for key, value in file_object.items():
        if not isinstance(value, h5py._hl.group.Group):
            continue
        if key == "#refs#":
            continue
        if key == "log":
            continue
        if "data" in value.keys():
            data[key] = {}
            data[key]["data"] = np.squeeze(np.array(value["data"]))
            data[key]["timestamps"] = np.squeeze(np.array(value["timestamps"]))

            # if the initial or end time has been updated we can also update the entire timestamps dataset
            if data[key]["timestamps"][0] < initial_time:
                timestamps = data[key]["timestamps"]
                initial_time = timestamps[0]

            if data[key]["timestamps"][-1] > end_time:
                timestamps = data[key]["timestamps"]
                end_time = timestamps[-1]

            # In yarp telemetry v0.4.0 the elements_names was saved.
            if "elements_names" in value.keys():
                elements_names_ref = value["elements_names"]
                data[key]["elements_names"] = [
                    "".join(chr(c[0]) for c in value[ref])
                    for ref in elements_names_ref[0]
                ]
        else:
            data[key] = __populate_numerical_data(file_object=value)

    return data


def generate_website(dataset_path: Path, website_path: Path):
    far_position = np.array([1e5, 1e5, 1e5])

    file_name = str(dataset_path)
    absolute_path = resolve_robotics_uri_py.resolve_robotics_uri(
        "package://ergoCub/robots/ergoCubSN001/model.urdf"
    )

    with h5py.File(file_name, "r") as file:
        root_variable = file["robot_logger_device"]
        data = __populate_numerical_data(root_variable)
        joint_ref = root_variable["description_list"]
        joints_name = ["".join(chr(c[0]) for c in file[ref]) for ref in joint_ref[0]]

        joints_state = data["joints_state"]["positions"]["data"]
        base_orientation = data["cmw"]["root_link"]["orientation"]["measured"]["data"]
        base_position = data["cmw"]["root_link"]["position"]["measured"]["data"]
        time_joints = data["joints_state"]["positions"]["timestamps"]
        time_base = data["cmw"]["root_link"]["position"]["measured"]["timestamps"]
        external_wrench = data["cmw"]["external_wrench"]["filtered"]["data"]
        left_foot = data["cmw"]["left_foot"]["position"]["desired"]["data"]
        left_foot_orientation = data["cmw"]["left_foot"]["orientation"]["desired"][
            "data"
        ]
        left_foot_nominal = data["cmw"]["contact"]["left_foot"]["position"]["nominal"][
            "data"
        ]
        left_wrench = data["cmw"]["left_foot"]["wrench"]["desired_mpc"]["data"]
        right_wrench = data["cmw"]["right_foot"]["wrench"]["desired_mpc"]["data"]
        right_foot = data["cmw"]["right_foot"]["position"]["desired"]["data"]
        right_foot_orientation = data["cmw"]["right_foot"]["orientation"]["desired"][
            "data"
        ]
        right_foot_nominal = data["cmw"]["contact"]["right_foot"]["position"][
            "nominal"
        ]["data"]
        zmp_desired = data["cmw"]["zmp"]["desired"]["data"]
        zmp_measured = data["cmw"]["zmp"]["measured"]["data"]
        dt_base = np.average(np.diff(time_base))

    chunk_index_left = [
        Index(index=i)
        for i in find_non_zero_chunks_with_threshold(left_foot[:, 2], 0.00001)
    ]
    chunk_index_right = [
        Index(index=i)
        for i in find_non_zero_chunks_with_threshold(right_foot[:, 2], 0.00001)
    ]

    chunk_index_nominal_left = [
        Index(index=i)
        for i in find_chunks_with_diff_greater_than_threshold(
            left_foot_nominal[:, 1], 0.01
        )
    ]

    chunk_index_nominal_right = [
        Index(index=i)
        for i in find_chunks_with_diff_greater_than_threshold(
            right_foot_nominal[:, 1], 0.01
        )
    ]

    viz = MeshcatVisualizer()
    viz.load_model_from_file(absolute_path, joints_name, "robot")
    viz.load_sphere(0.02, "desired_zmp", color=[66 / 255.0, 133 / 255.0, 244 / 255.0])
    viz.load_sphere(0.02, "measured_zmp", color=[219 / 255.0, 68 / 255.0, 55 / 255.0])

    yellow_google = [244 / 255.0, 194 / 255.0, 13 / 255.0]
    purple_google = [66 / 255.0, 133 / 255.0, 244 / 255.0]
    green_google = [15 / 255.0, 157 / 255.0, 88 / 255.0]

    for i in range(len(chunk_index_left)):
        viz.load_box(0.2, 0.1, 0.0001, f"left_foot_{i}", color=yellow_google)
        viz.load_box(0.2, 0.1, 0.0001, f"nominal_left_foot_{i}", color=green_google)
        # viz.set_primitive_geometry_transform(far_position, idyn.Rotation.Identity().toNumPy(), f'left_foot_{i}')

    for i in range(len(chunk_index_right)):
        viz.load_box(0.2, 0.1, 0.0001, f"right_foot_{i}", color=yellow_google)
        viz.load_box(0.2, 0.1, 0.0001, f"nominal_right_foot_{i}", color=green_google)
        # viz.set_primitive_geometry_transform(far_position, idyn.Rotation.Identity().toNumPy(), f'right_foot_{i}')

    # find the index in which the heigh of the foot is equal to zero and the value of the z at the previous index is different from zero
    left_contact_idx = np.where(left_foot[:, 2] == 0)[0]

    viz.start_recording_animation()
    index = 0

    # move all the feet to a far position
    viz.set_animation_frame(0)

    for i in range(len(chunk_index_left)):
        viz.set_primitive_geometry_transform(
            far_position, idyn.Rotation.Identity().toNumPy(), f"left_foot_{i}"
        )
        viz.set_primitive_geometry_transform(
            far_position, idyn.Rotation.Identity().toNumPy(), f"nominal_left_foot_{i}"
        )

    for i in range(len(chunk_index_right)):
        viz.set_primitive_geometry_transform(
            far_position, idyn.Rotation.Identity().toNumPy(), f"right_foot_{i}"
        )
        viz.set_primitive_geometry_transform(
            far_position, idyn.Rotation.Identity().toNumPy(), f"nominal_right_foot_{i}"
        )

    time_scaling = 5
    for i in track(
        range(0, time_base.shape[0], int(1 / (dt_base * (30 / time_scaling))))
    ):

        t = time_base[i]

        viz.set_animation_frame(index)

        # check if the left foot is in contact
        left_foot_index = -1
        for c in chunk_index_left:
            left_foot_index += 1

            if i < c.index[1]:
                viz.set_primitive_geometry_transform(
                    far_position,
                    idyn.Rotation.Identity().toNumPy(),
                    f"left_foot_{left_foot_index}",
                )

            if i < c.index[1] or c.is_plot:
                continue

            left_foot_rpy = idyn.Rotation.RPY(
                left_foot_orientation[i, 0],
                left_foot_orientation[i, 1],
                left_foot_orientation[i, 2],
            ).toNumPy()
            viz.set_primitive_geometry_transform(
                left_foot[i, :], left_foot_rpy, f"left_foot_{left_foot_index}"
            )

            c.is_plot = True

        right_foot_index = -1
        for c in chunk_index_right:
            right_foot_index += 1

            if i < c.index[1]:
                viz.set_primitive_geometry_transform(
                    far_position,
                    idyn.Rotation.Identity().toNumPy(),
                    f"right_foot_{right_foot_index}",
                )

            if i < c.index[1] or c.is_plot:
                continue

            right_foot_rpy = idyn.Rotation.RPY(
                right_foot_orientation[i, 0],
                right_foot_orientation[i, 1],
                right_foot_orientation[i, 2],
            ).toNumPy()
            viz.set_primitive_geometry_transform(
                right_foot[i, :], right_foot_rpy, f"right_foot_{right_foot_index}"
            )

            c.is_plot = True

        left_foot_nominal_index = -1
        for c in chunk_index_nominal_left:
            left_foot_nominal_index += 1

            if i < c.index[1]:
                viz.set_primitive_geometry_transform(
                    far_position,
                    idyn.Rotation.Identity().toNumPy(),
                    f"nominal_left_foot_{left_foot_nominal_index}",
                )

            if i < c.index[1] or c.is_plot:
                continue

            left_foot_rpy = idyn.Rotation.RPY(
                left_foot_orientation[i, 0],
                left_foot_orientation[i, 1],
                left_foot_orientation[i, 2],
            ).toNumPy()

            viz.set_primitive_geometry_transform(
                left_foot_nominal[i, :],
                left_foot_rpy,
                f"nominal_left_foot_{left_foot_nominal_index}",
            )

            c.is_plot = True

        right_foot_nominal_index = -1
        for c in chunk_index_nominal_right:
            right_foot_nominal_index += 1

            if i < c.index[1]:
                viz.set_primitive_geometry_transform(
                    far_position,
                    idyn.Rotation.Identity().toNumPy(),
                    f"nominal_right_foot_{right_foot_nominal_index}",
                )

            if i < c.index[1] or c.is_plot:
                continue

            right_foot_rpy = idyn.Rotation.RPY(
                right_foot_orientation[i, 0],
                right_foot_orientation[i, 1],
                right_foot_orientation[i, 2],
            ).toNumPy()

            viz.set_primitive_geometry_transform(
                right_foot_nominal[i, :],
                right_foot_rpy,
                f"nominal_right_foot_{right_foot_nominal_index}",
            )

            c.is_plot = True

        # find the closest timestamp in the joints state
        idx = np.argmin(np.abs(time_joints - t))

        orientation = idyn.Rotation.RPY(
            base_orientation[i, 0],
            base_orientation[i, 1],
            base_orientation[i, 2],
        ).toNumPy()

        viz.set_multibody_system_state(
            base_position=base_position[i, :],
            base_rotation=orientation,
            joint_value=joints_state[idx, :],
            model_name="robot",
        )

        zmp_desired_3d = np.array([zmp_desired[i, 0], zmp_desired[i, 1], 0])
        zmp_measured_3d = np.array([zmp_measured[i, 0], zmp_measured[i, 1], 0])
        viz.set_primitive_geometry_transform(zmp_desired_3d, orientation, "desired_zmp")
        viz.set_primitive_geometry_transform(
            zmp_measured_3d, orientation, "measured_zmp"
        )

        index += time_scaling
    viz.publish_animation()

    # save
    open(website_path, "w").write(viz.viewer.static_html())


def main():
    current_file_path = Path(__file__).resolve()
    dataset_path = current_file_path.parent.parent.parent / "dataset" / "paper_experiments"
    website_path = current_file_path.parent / "website"

    # check if the website folder exists if not create it
    if not website_path.exists():
        website_path.mkdir()

    # generate the website for the trajectory generation used as MPC
    generate_website(
        dataset_path / "robot_logger_device_2024_07_09_12_48_52_c.mat",
        website_path / "website_c.html",
    )
    generate_website(
        dataset_path / "robot_logger_device_2024_07_03_15_08_33_p.mat",
        website_path / "website_p.html",
    )


if __name__ == "__main__":
    main()
