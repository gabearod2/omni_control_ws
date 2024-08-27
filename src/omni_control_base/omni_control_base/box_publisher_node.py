#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from msg import Box
import numpy as np
import jetson_inference
import jetson_utils


class BoxPublisherNode(Node):
    def __init__(self):
        super().__init__("pid_control")

        # Creating publisher
        self.publisher = self.create_publisher(
            Box,
            'detections',
            10
        )

    def detection_publisher(self):
        # Setting up detection and display
        net = jetson_inference.detectNet(network="models/summer-project-v3/ssd-mobilenet.onnx",
                                threshold=0.4,
                                labels="models/summer-project-v3/labels.txt",
                                input_blob="input_0",
                                output_cvg="scores",
                                output_bbox="boxes")
        camera = jetson_utils.videoSource("csi://0")
        display = jetson_utils.videoOutput("rtsp://@:1234/my_output")

        # Capture image from camera
        img = camera.Capture()

        # Detect the ball
        detections = net.Detect(img)
        detection = detections[0]
        display.Render(img)
        display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))

        # Find the current centroid of the ball from the detection bounding box
        self.msg = Box()
        self.msg.center = [detection.Center(0), detection.Center(1)]
        self.msg.width = detection.Width
        self.msg.height = detection.Height
        self.publisher.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = BoxPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()