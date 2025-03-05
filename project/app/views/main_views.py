from flask import Blueprint, url_for, jsonify
from werkzeug.utils import redirect
from .. import main_manager

bp = Blueprint('main', __name__, url_prefix='/')

@bp.route('/')
def index():
    return redirect(url_for('tool._list'))

@bp.route('/start_processes', methods=['POST'])
def start_processes():
    if main_manager:
        main_manager.start_processes()
        return jsonify({"status": "success", "message": "프로세스가 시작되었습니다."})
    return jsonify({"status": "error", "message": "MainManager가 초기화되지 않았습니다."})

@bp.route('/stop_processes', methods=['POST'])
def stop_processes():
    if main_manager:
        main_manager.stop_processes()
        return jsonify({"status": "success", "message": "프로세스가 중지되었습니다."})
    return jsonify({"status": "error", "message": "MainManager가 초기화되지 않았습니다."})

@bp.route('/process_status', methods=['GET'])
def process_status():
    if main_manager:
        face_alive = main_manager.face_process and main_manager.face_process.is_alive()
        object_alive = main_manager.object_process and main_manager.object_process.is_alive()
        latest_worker = main_manager.latest_worker.value
        return jsonify({
            "face_process": "실행 중" if face_alive else "중지됨",
            "object_process": "실행 중" if object_alive else "중지됨",
            "latest_worker": latest_worker
        })
    return jsonify({"status": "error", "message": "MainManager가 초기화되지 않았습니다."})
