import onnx
m = onnx.load("/home/nvidia/Jetson-RT-Traffic-System/models/Primary_Detector/resnet50_trafficcamnet_rtdetr.fp16.onnx")
print("Inputs:")
for i in m.graph.input:
    t = i.type.tensor_type
    shape = [d.dim]
