import argparse
import sys
import os

import keras

from tensorflow_model_optimization.quantization.keras import vitis_inspect

# ==========================================================================================
# Get Input Arguments
# ==========================================================================================
def get_arguments():
    parser = argparse.ArgumentParser(description="Vitis AI TF2 Quantization of ResNet18")

    # model config
    parser.add_argument("--keras_model", type=str, default="build/yolov3_ts.h5",
                        help="h5 floating point model file full path name")

    return parser.parse_args()

args = get_arguments()

# ==========================================================================================
# Global Variables
# ==========================================================================================
cwd = os.getcwd()
print("\n[INFO_M] : current working directory =>", cwd)

FLOAT_HDF5_FILE = os.path.join(cwd,  args.keras_model)
print("[INFO_M] : path to the model file =>", FLOAT_HDF5_FILE)
print("\n")

# ==========================================================================================
# Get the trained floating point model
# ==========================================================================================

if os.path.isfile(FLOAT_HDF5_FILE):
    print("[INFO_M] : Opening H5 video file: {}".format(FLOAT_HDF5_FILE))
    model = keras.models.load_model(FLOAT_HDF5_FILE)
else:
    print("[INFO_M] : The file {} does not exist! The program will be terminated.".format(FLOAT_HDF5_FILE))
    sys.exit()

# ==========================================================================================
# Vitis AI Model Inspector
# ==========================================================================================
print("\n[INFO_M] Vitis AI Model Inspector...\n")

inspector = vitis_inspect.VitisInspector(target="/opt/vitis_ai/compiler/arch/DPUCZDX8G/KV260/arch.json")
filename_dump = os.path.join(cwd,  "build/inspect_results.txt")
filename_svg  = os.path.join(cwd,  "build/model.svg")
inspector.inspect_model(model,
                        input_shape=[1,  416, 416, 3],
                        plot=True,
                        plot_file=filename_svg,
                        dump_results=True,
                        dump_results_file=filename_dump,
                        verbose=0)

print("\n")
