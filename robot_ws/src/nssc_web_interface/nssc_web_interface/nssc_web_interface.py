import os
from flask import Flask, render_template, request
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from camera_ingest.msg import ColorFilterParams

package_share_directory = get_package_share_directory('nssc_web_interface')
print(package_share_directory)

template_dir = os.path.abspath(package_share_directory + '/templates')
app = Flask(__name__, template_folder=template_dir)

@app.route('/')
def index():
    return render_template('index.html')

@app.route("/test", methods=["POST"])
def test():
    lower_y = request.form["lower_y"]
    lower_u = request.form["lower_u"]
    lower_v = request.form["lower_v"]
    return str(lower_y) + " - " + str(lower_u) + " - " + str(lower_v)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(ColorFilterParams, 'topic', 10)

    def timer_callback(self):
        msg = ColorFilterParams()
        msg.low_h = 10
        #self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1

def main():
    app.run(host="0.0.0.0")

if __name__ == "__main__":
    main()