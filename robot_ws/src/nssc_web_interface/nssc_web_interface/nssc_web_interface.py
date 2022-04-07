import os
from flask import Flask, render_template, request, redirect
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from camera_ingest.msg import ColorFilterParams, CameraSettings
import threading
from dataclasses import dataclass

package_share_directory = get_package_share_directory('nssc_web_interface')
print(package_share_directory)

template_dir = os.path.abspath(package_share_directory + '/templates')
app = Flask(__name__, template_folder=template_dir)

web_command_publisher = None

@dataclass
class FilterStruct:
    low_h: int
    low_s: int
    low_v: int
    high_h: int
    high_s: int
    high_v: int
    dilation_element: int
    dilation_size: int

current_pos = FilterStruct(0, 0, 0, 180, 255, 255, 0, 2)

@dataclass
class CameraStruct:
    gain: int
    exposure: int

current_cam = CameraStruct(15, 50000)

class WebCommandPublisher(Node):

    def __init__(self):
        super().__init__('nssc_web_interface')
        self.CFpublisher = self.create_publisher(ColorFilterParams, 'color_filter_params', 10)
        self.CSpublisher = self.create_publisher(CameraSettings, 'camera_settings', 10)

    def sendColorFilter(self, current_pos):
        msg = ColorFilterParams()
        msg.low_h = current_pos.low_h
        msg.low_s = current_pos.low_s
        msg.low_v = current_pos.low_v
        msg.high_h = current_pos.high_h
        msg.high_s = current_pos.high_s
        msg.high_v = current_pos.high_v
        msg.dilation_element = current_pos.dilation_element
        msg.dilation_size = current_pos.dilation_size
        self.CFpublisher.publish(msg)
        self.get_logger().info('Publishing color filter settings')
    
    def sendCameraSettings(self, current_cam):
        msg = CameraSettings()
        msg.gain = current_cam.gain
        msg.exposure = current_cam.exposure
        self.CSpublisher.publish(msg)
        self.get_logger().info('Publishing camera settings')

@app.route('/')
def index():
    return render_template('index.html', current_pos=current_pos, current_cam=current_cam)

@app.route("/send", methods=["POST", "GET"])
def send():
    global web_command_publisher
    global current_pos
    global current_cam

    if request.method == 'POST':

        current_pos.low_h = int(request.form["low_h"])
        current_pos.low_s = int(request.form["low_s"])
        current_pos.low_v = int(request.form["low_v"])
        current_pos.high_h = int(request.form["high_h"])
        current_pos.high_s = int(request.form["high_s"])
        current_pos.high_v = int(request.form["high_v"])
        current_pos.dilation_element = int(request.form["dilation_element"])
        current_pos.dilation_size = int(request.form["dilation_size"])

        web_command_publisher.sendColorFilter(current_pos)

        return redirect('/')

    else:
        return render_template('index.html', current_pos=current_pos, current_cam=current_cam)

@app.route('/update', methods=['GET', 'POST'])
def update():
    global current_pos
    return render_template('index.html', current_pos=current_pos, current_cam=current_cam)

@app.route('/camera', methods=['GET', 'POST'])
def camera():
    global web_command_publisher
    global current_pos
    global current_cam

    if request.method == 'POST':

        current_cam.gain = int(request.form["gain"])
        current_cam.exposure = int(request.form["exposure"])

        web_command_publisher.sendCameraSettings(current_cam)

        return redirect('/')

    else:
        return render_template('index.html', current_pos=current_pos, current_cam=current_cam)


def run_page():
    app.run(host="0.0.0.0")

def main(args=None):
    global web_command_publisher

    rclpy.init(args=args)

    web_command_publisher = WebCommandPublisher()
    
    t = threading.Thread(target=run_page)
    t.start()

    rclpy.spin(web_command_publisher)

    web_command_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()