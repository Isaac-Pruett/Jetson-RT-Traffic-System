import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit  # automatically initializes CUDA driver
import numpy as np
import cv2

# Paths
ENGINE_PATH = "/home/nvidia/Jetson-RT-Traffic-System/models/Secondary_VehicleTypeNet/resnet18_pruned.engine"
IMAGE_PATH = "test_image.jpg"  # replace with your image path

# TensorRT logger
TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

def load_engine(engine_path):
    with open(engine_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
        return runtime.deserialize_cuda_engine(f.read())

def preprocess_image(image_path):
    # Load and resize to 224x224
    img = cv2.imread(image_path)
    img = cv2.resize(img, (224, 224))
    # Convert to float32, normalize to [0,1]
    img = img.astype(np.float32) / 255.0
    # HWC -> CHW
    img = np.transpose(img, (2, 0, 1))
    # Add batch dimension
    img = np.expand_dims(img, axis=0)
    return img

def allocate_buffers(engine):
    inputs = []
    outputs = []
    bindings = []
    stream = cuda.Stream()

    for binding in engine:
        size = trt.volume(engine.get_binding_shape(binding))
        dtype = trt.nptype(engine.get_binding_dtype(binding))
        # Allocate device memory
        device_mem = cuda.mem_alloc(size * dtype().nbytes)
        bindings.append(int(device_mem))
        if engine.binding_is_input(binding):
            inputs.append(device_mem)
        else:
            outputs.append(device_mem)
    return inputs, outputs, bindings, stream

def infer(engine, context, inputs, outputs, bindings, stream, image):
    # Transfer input data to device
    cuda.memcpy_htod_async(inputs[0], image, stream)
    # Run inference
    context.execute_async_v2(bindings=bindings, stream_handle=stream.handle)
    # Transfer predictions back
    output_shape = engine.get_binding_shape(1)  # assuming output is binding 1
    output = np.empty(trt.volume(output_shape), dtype=np.float32)
    cuda.memcpy_dtoh_async(output, outputs[0], stream)
    stream.synchronize()
    return output.reshape(output_shape)

def main():
    engine = load_engine(ENGINE_PATH)
    context = engine.create_execution_context()

    image = preprocess_image(IMAGE_PATH).ravel()  # flatten for device
    inputs, outputs, bindings, stream = allocate_buffers(engine)

    output = infer(engine, context, inputs, outputs, bindings, stream, image)
    print("Model output:", output)

if __name__ == "__main__":
    main()


