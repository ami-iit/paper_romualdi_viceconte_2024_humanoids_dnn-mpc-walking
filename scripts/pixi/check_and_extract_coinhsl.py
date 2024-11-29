import os
import zipfile
import sys

# Define file and directories
file_path = './coinhsl_src/coinhsl-2023.11.17.zip'
output_dir = './.build-coinhsl'

def main():
    if os.path.exists(file_path):
        print(f"File '{file_path}' found. Proceeding to unzip...")

        # Ensure the output directory exists
        os.makedirs(output_dir, exist_ok=True)

        # Unzipping the file
        try:
            with zipfile.ZipFile(file_path, 'r') as zip_ref:
                zip_ref.extractall(output_dir)
            print(f"File unzipped successfully to '{output_dir}'.")
        except zipfile.BadZipFile:
            print("Error: The file is not a valid ZIP archive.")
            sys.exit(1)
    else:
        print(f"Error: File '{file_path}' does not exist.")
        print("""
Please follow these steps to resolve the issue:
1. Go to https://licences.stfc.ac.uk/product/coin-hsl, and:
   - If you already have a license for Coin-HSL:
     - Go to https://licences.stfc.ac.uk/account/orders and find the coin-hsl order.
     - Download the coinhsl-2023.11.17.zip file and place it in the './coinhsl_src' folder of this repository.
   - If you do not have a license for Coin-HSL:
     - If you are an academic, request a license at https://licences.stfc.ac.uk/product/coin-hsl.
     - If you are not an academic, purchase a license at https://licences.stfc.ac.uk/product/coin-hsl.
     - Once your order is approved, download the coinhsl-2023.11.17.zip file and place it in the './coinhsl_src' folder.
""")
        sys.exit(1)

if __name__ == "__main__":
    main()
