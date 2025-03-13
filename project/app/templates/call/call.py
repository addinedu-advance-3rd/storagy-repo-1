import os
import subprocess
from flask import Flask, request, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app, resources={r"/move": {"origins": "*"}})

# ✅ 원격 실행할 로봇의 정보
ROBOT_IP = "192.168.0.6"  # 로봇 IP
ROBOT_USER = "storagy"  # SSH 계정
MOVE_SCRIPT_PATH = "/home/storagy/storagy-repo-1/circle/src/circle/circle/move_to_call.py"  # ✅ 이동된 move_to_call.py 절대경로

@app.route('/move', methods=['POST'])
def move_robot():
    data = request.get_json()
    zone = data.get("zone")  # 'A' 또는 'B'

    if not zone:
        return jsonify({"status": "error", "message": "구역 정보가 없습니다."}), 400

    print(f"📌 호출된 구역: {zone}")  # 터미널 로그 확인

    # ✅ 이동할 목표 위치 설정 (쉼표로 구분된 문자열로 변환)
    if zone == "A":
        target_pose = "-0.046,-1.063,0.0,-0.565,0.825"  # x, y, z, orientation_z, orientation_w
    elif zone == "B":
        target_pose = "0.371,0.539,0.0,0.643,0.765"
    else:
        return jsonify({"status": "error", "message": "잘못된 구역"}), 400

    # ✅ 쉼표로 구분된 문자열을 직접 전달
    ssh_command = f"ssh {ROBOT_USER}@{ROBOT_IP} 'export ROS_DOMAIN_ID=13 && source /opt/ros/humble/setup.sh && python3 {MOVE_SCRIPT_PATH} {target_pose}'"
    
    print(f"🛠️ 실행 명령어: {ssh_command}")  # 🔹 디버깅용 출력
    subprocess.Popen(ssh_command, shell=True)

    return jsonify({"status": "success", "message": f"로봇이 {zone}구역으로 이동 중입니다."})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5050, debug=True)
