import os
import subprocess
from flask import Flask, request, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app, resources={r"/move": {"origins": "*"}})

# ✅ 원격 실행할 로봇의 정보
ROBOT_IP = "192.168.0.6"  # 로봇 IP
ROBOT_USER = "storagy"  # SSH 계정
LAUNCH_CMD = "ros2 launch circle circle_navigation.launch.py"  # ✅ 실행할 ROS2 런치 명령

@app.route('/move', methods=['POST'])
def move_robot():
    data = request.get_json()
    zone = data.get("zone")  # 'Home' 구역만 처리

    if zone != "Home":
        return jsonify({"status": "error", "message": "잘못된 요청"}), 400

    print(f"📌 [Home] 호출 요청 수신")  # 🛠️ 터미널 로그


    # ✅ 쉼표로 구분된 문자열을 직접 전달
    ssh_command = f"ssh {ROBOT_USER}@{ROBOT_IP} 'export ROS_DOMAIN_ID=13 && source /opt/ros/humble/setup.sh && source /home/storagy/storagy-repo-1/circle/install/local_setup.zsh && ros2 launch circle circle_navigation.launch.py'"
    
    print(f"🛠️ 실행 명령어: {ssh_command}")  # 🔹 디버깅용 출력
    subprocess.Popen(ssh_command, shell=True)

    return jsonify({"status": "success", "message": "Home 실행 시작됨."})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5050, debug=True)
