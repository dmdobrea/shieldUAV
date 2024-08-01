import argparse
import os
import cv2
import sys
import numpy as np

import keras
from tensorflow_model_optimization.quantization.keras import vitis_quantize
from imutils import paths

# ==========================================================================================
# Load and preprocess data 
# ==========================================================================================
def load(image_paths, verbose=-1):
    data = []

    for i, image_path in enumerate(image_paths):
        image = cv2.imread(image_path)
        
        image = pre_process(image, (416, 416))
        data.append(image)
        
        if verbose > 0 and i > 0 and (i+1) % verbose == 0:
            print('[INFO]: Processed {}/{}'.format(i+1, len(image_paths)))

    return (np.array(data))
        
def pre_process(image, model_image_size):
    image = image[...,::-1]
    image_h, image_w, _ = image.shape

    if model_image_size != (None, None):
        assert model_image_size[0]%32 == 0, 'Multiples of 32 required'
        assert model_image_size[1]%32 == 0, 'Multiples of 32 required'
        boxed_image = letterbox_image(image, tuple(reversed(model_image_size)))
    else:
        new_image_size = (image_w - (image_w % 32), image_h - (image_h % 32))
        boxed_image = letterbox_image(image, new_image_size)

    # print ("AAA: boxwd = ", boxed_image.shape)
    image_data = np.array(boxed_image, dtype='float32')
    # print ("BBB: array", image_data.shape)
    image_data /= 255.

    return image_data

'''resize image with unchanged aspect ratio using padding'''
def letterbox_image(image, size):
    ih, iw, _ = image.shape
    w, h = size
    scale = min(w/iw, h/ih)
    #print(scale)

    nw = int(iw*scale)
    nh = int(ih*scale)
    #print(nw)
    #print(nh)

    image = cv2.resize(image, (nw,nh), interpolation=cv2.INTER_LINEAR)
    new_image = np.ones((h,w,3), np.uint8) * 128
    h_start = (h-nh)//2
    w_start = (w-nw)//2
    new_image[h_start:h_start+nh, w_start:w_start+nw, :] = image
    
    return new_image

# ==========================================================================================
# Get Input Arguments
# ==========================================================================================
def get_arguments():
    parser = argparse.ArgumentParser(description="Vitis AI Quantization of any DNN model")

    # data path set
    parser.add_argument('-d', '--dataset', required=True, help='Path to input dataset')

    # model config
    parser.add_argument("--keras_model", type=str, default="./build/float/train2_resnet18_cifar10.h5",
                        help="h5 floating point file full path name")
 
    # quantization config
    parser.add_argument("--quant_file", type=str, default="./build/quantized/q_train2_resnet18_cifar10.h5",
                        help="quantized model file full path ename ")

    return parser.parse_args()


#main() function start here:
args = get_arguments()

# ==========================================================================================
# Grab the list of images
# ==========================================================================================

print('[INFO_Q] : Loading images....')
my_image_paths = list(paths.list_images(args.dataset))

# print (my_image_paths)

data_calib = load(my_image_paths, 10)
print ("[Info_Q] : data shape", data_calib.shape)

# ==========================================================================================
# Paths variables
# ==========================================================================================
cwd = os.getcwd()

FLOAT_HDF5_FILE = os.path.join(cwd,  args.keras_model)
QUANT_HDF5_FILE = os.path.join(cwd,  args.quant_file)

# ==========================================================================================
# Get the trained floating point model
# ==========================================================================================
if os.path.isfile(FLOAT_HDF5_FILE):
    print("[INFO_Q] : Opening H5 video file: {}".format(FLOAT_HDF5_FILE))
    model = keras.models.load_model(FLOAT_HDF5_FILE)
else:
    print("[INFO_Q] : The file {} does not exist! The program will be terminated.".format(FLOAT_HDF5_FILE))
    sys.exit()

# ==========================================================================================
# Vitis AI Quantization
# ==========================================================================================
print("\n[INFO_Q] Vitis AI Quantization...\n")

quantizer = vitis_quantize.VitisQuantizer(model)
q_model = quantizer.quantize_model(calib_dataset=data_calib)
q_model.save(QUANT_HDF5_FILE)














