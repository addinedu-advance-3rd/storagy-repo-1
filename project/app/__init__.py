from flask import Flask, current_app
from flask_migrate import Migrate
from flask_sqlalchemy import SQLAlchemy
from flask_socketio import SocketIO
import config
import json

db = SQLAlchemy()
migrate = Migrate()
socketio = SocketIO()
main_manager = None  # MainManager 인스턴스를 저장할 전역 변수

tools_json_path = "/home/addinedu/dev_ws/ftud_branch/storagy-repo-1/project/cv/db/tools.json"

def create_app():
    app = Flask(__name__)
    app.config.from_object(config)

    # ORM
    db.init_app(app)
    migrate.init_app(app, db)
    from .models import Tool, Log

    # DB 세팅
    def is_Tool_empty():
        return db.session.query(Tool).count() == 0

    def Tool_init():
        if is_Tool_empty():
            tools = load_json(tools_json_path)
            for tool in tools:
                new_tool = Tool(name=tool['name'], avail=tool['avail'])
                db.session.add(new_tool)
            db.session.commit()
    
    with app.app_context():
        Tool_init()

    # Blueprint
    from .views import main_views, tool_views, log_views, call_views, map_views
    app.register_blueprint(main_views.bp)
    app.register_blueprint(tool_views.bp)
    app.register_blueprint(log_views.bp)
    app.register_blueprint(call_views.bp)
    app.register_blueprint(map_views.bp)
    
    # Websocket
    socketio.init_app(app)

    # 멀티프로세스 관리자 초기화
    from cv.main import MainManager
    global main_manager
    main_manager = MainManager()
    
    # 애플리케이션 종료 시 프로세스 정리
    @app.teardown_appcontext
    def cleanup_processes(exception=None):
        global main_manager
        if main_manager:
            main_manager.stop_processes()

    # Filter
    from .filter import format_datetime
    app.jinja_env.filters['datetime'] = format_datetime

    # 소켓 이벤트 핸들러 추가
    @socketio.on('test_message')
    def handle_test_message(data):
        print('테스트 메시지 수신:', data)
        socketio.emit('test_response', {'response': '서버에서 응답합니다', 'received': data})

    # 지도 서버와 통합 (before_first_request 대신 첫 번째 요청 시 실행되는 함수 등록)
    def setup_map_integration():
        try:
            from circle.circle.map_to_web import set_robot_pose_callback
            from app.views.map_views import forward_robot_pose
            
            # 지도 서버의 로봇 위치 업데이트를 Flask-SocketIO로 전달하는 콜백 설정
            success = set_robot_pose_callback(forward_robot_pose)
            if success:
                print("지도 서버와 Flask-SocketIO 통합 성공")
            else:
                print("지도 서버와 Flask-SocketIO 통합 실패: 지도 서버가 아직 초기화되지 않았습니다")
        except ImportError as e:
            print(f"지도 서버 모듈 로드 실패: {e}")
        except Exception as e:
            print(f"지도 서버 통합 중 오류 발생: {e}")
    
    # 첫 번째 요청 시 실행될 함수 등록
    @app.route('/setup-map', methods=['GET'])
    def trigger_setup_map():
        setup_map_integration()
        return "지도 서버 통합 설정 완료"
    
    # 또는 앱 시작 시 직접 호출
    with app.app_context():
        setup_map_integration()

    return app

def load_json(path):
    """ JSON 파일 읽기 """
    try:
        with open(path, "r") as file:
            return json.load(file)
    except FileNotFoundError:
        print(f"Error: {path} 파일을 찾을 수 없습니다.")
        return []
    except json.JSONDecodeError:
        print(f"Error: {path} 파일의 JSON 형식이 잘못되었습니다.")
        return []