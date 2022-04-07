import os
from flask import Flask, render_template, request, redirect
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from camera_ingest.msg import ColorFilterParams
import threading

package_share_directory = get_package_share_directory('nssc_web_interface')
print(package_share_directory)

template_dir = os.path.abspath(package_share_directory + '/templates')
app = Flask(__name__, template_folder=template_dir)

color_filter_params = None

class WebCommandPublisher(Node):

    def __init__(self):
        super().__init__('nssc_web_interface')
        self.publisher_ = self.create_publisher(ColorFilterParams, 'color_filter_params', 10)

    def sendCommand(self, low_h, low_s, low_v, high_h, high_s, high_v):
        msg = ColorFilterParams()
        msg.low_h = low_h
        msg.low_s = low_s
        msg.low_v = low_v
        msg.high_h = high_h
        msg.high_s = high_s
        msg.high_v = high_v
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing command')

@app.route('/')
def index():
    return render_template('index.html')

@app.route("/test", methods=["POST"])
def test():
    global color_filter_params

    low_h = request.form["low_h"]
    low_s = request.form["low_s"]
    low_v = request.form["low_v"]
    high_h = request.form["high_h"]
    high_s = request.form["high_s"]
    high_v = request.form["high_v"]

    color_filter_params.sendCommand(int(low_h), int(low_s), int(low_v), int(high_h), int(high_s), int(high_v))

    return redirect('/')

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