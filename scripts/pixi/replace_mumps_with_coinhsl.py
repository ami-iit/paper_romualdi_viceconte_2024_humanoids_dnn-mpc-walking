import os

def replace_mumps_with_ma97(file_path):
    # Read the file
    with open(file_path, 'r') as file:
        content = file.read()

    # Replace occurrences of 'mumps' with 'ma97'
    updated_content = content.replace("mumps", "ma97")

    # Write the updated content back to the file
    with open(file_path, 'w') as file:
        file.write(updated_content)

    print(f"Replaced all occurrences of 'mumps' with 'ma97' in {file_path}")

# Get the CONDA_PREFIX environment variable
conda_prefix = os.getenv("CONDA_PREFIX")

if conda_prefix:
    # Construct the file path
    file_path = os.path.join(conda_prefix, "share/dnn_mpc/robots/ergoCubGazeboV1_1/dnn-mpc/centroidal_mpc.ini")

    if os.path.exists(file_path):
        replace_mumps_with_ma97(file_path)
    else:
        print(f"File does not exist: {file_path}")
        os.exit(1)
else:
    print("CONDA_PREFIX environment variable is not set.")
    os.exit(1)
