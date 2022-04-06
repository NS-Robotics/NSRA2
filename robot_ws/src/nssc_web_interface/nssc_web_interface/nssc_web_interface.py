from flask import Flask, render_template, request

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route("/test", methods=["POST"])
def test():
    lower_y = request.form["lower_y"]
    lower_u = request.form["lower_u"]
    lower_v = request.form["lower_v"]
    return str(lower_y) + " - " + str(lower_u) + " - " + str(lower_v)

if __name__ == "__main__":
    app.run(debug=True)