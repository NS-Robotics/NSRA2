import os
from flask import Flask, render_template, request, redirect
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from camera_ingest.msg import ColorFilterParams
import threading
from dataclasses import dataclass

package_share_directory = get_package_share_directory('nssc_web_interface')
print(package_share_directory)

template_dir = os.path.abspath(package_share_directory + '/templates')
app = Flask(__name__, template_folder=template_dir)

color_filter_params = None

@dataclass
class FilterStruct:
    low_h: int
    low_s: int
    low_v: int
    high_h: int
    high_s: int
    high_v: int

current_pos = FilterStruct(0, 0, 0, 180, 255, 255)

class WebCommandPublisher(Node):

    def __init__(self):
        super().__init__('nssc_web_interface')
        self.publisher_ = self.create_publisher(ColorFilterParams, 'color_filter_params', 10)

    def sendCommand(self, current_pos):
        msg = ColorFilterParams()
        msg.low_h = current_pos.low_h
        msg.low_s = current_pos.low_s
        msg.low_v = current_pos.low_v
        msg.high_h = current_pos.high_h
        msg.high_s = current_pos.high_s
        msg.high_v = current_pos.high_v
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing command')

@app.route('/')
def index():
    return render_template('index.html', current_pos=current_pos)

@app.route("/test", methods=["POST", "GET"])
def test():
    global color_filter_params
    global current_pos

    if request.method == 'POST':

        current_pos.low_h = int(request.form["low_h"])
        current_pos.low_s = int(request.form["low_s"])
        current_pos.low_v = int(request.form["low_v"])
        current_pos.high_h = int(request.form["high_h"])
        current_pos.high_s = int(request.form["high_s"])
        current_pos.high_v = int(request.form["high_v"])

        color_filter_params.sendCommand(current_pos)

        return redirect('/')

    else:
        return render_template('index.html', current_pos=current_pos)


def run_page():
    app.run(host="0.0.0.0")

def main(args=None):
    global color_filter_params

    rclpy.init(args=args)

    color_filter_params = WebCommandPublisher()
    
    t = threading.Thread(target=run_page)
    t.start()

    rclpy.spin(color_filter_params)

    color_filter_params.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()