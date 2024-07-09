from flask import Flask, Response
from tele_camera import tele_camera

app = Flask(__name__)

@app.route('/')
def home():
    return app.send_static_file('index.html')

@app.route('/video_feed')
def video_feed():
    # 使用生成器函数 tele_camera 获取视频流帧
    return Response(tele_camera(target="your_target_ip_here", width=640, height=480),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host="0.0.0.0", debug=True)
