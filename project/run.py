import os
import sys
import threading
import time
from app import create_app, socketio
from cv import main  # main 모듈 내에 MainManager 클래스가 있다고 가정
import signal
import importlib.util

# 프로젝트 루트 경로 설정
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)

# circle 패키지 경로 설정 (필요한 경우 경로 조정)
CIRCLE_PATH = os.path.join(os.path.dirname(BASE_DIR), 'circle/src')
sys.path.append(CIRCLE_PATH)

# 전역 변수로 백그라운드 스레드를 저장
manager_thread = None
map_thread = None
manager = None  # MainManager 인스턴스

def start_manager():
    global manager
    try:
        manager.start_processes()
        # 작업이 완료될 때까지 대기 (여기서 필요하다면 주기적으로 상태를 확인)
        while manager.is_running():
            time.sleep(1)
    except KeyboardInterrupt:
        manager.stop_processes()
    except Exception as e:
        print(f"프로세스 실행 중 오류 발생: {e}")
        manager.stop_processes()

def start_map_server():
    # map_to_web 모듈 동적 로드
    try:
        from circle.circle.map_to_web import main as map_main
        map_main()
    except ImportError as e:
        print(f"지도 서버 모듈 로드 실패: {e}")
    except Exception as e:
        print(f"지도 서버 실행 중 오류 발생: {e}")

def signal_handler(sig, frame):
    global manager, manager_thread, map_thread
    print("신호를 받아 종료합니다...")
    if manager:
        manager.stop_processes()
    if manager_thread:
        manager_thread.join(timeout=5)
    if map_thread:
        map_thread.join(timeout=5)
    sys.exit(0)

if __name__ == "__main__":
    # 신호 핸들러 등록 (Ctrl+C 등)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        app = create_app()
        with app.app_context():
            # socketio 객체를 MainManager에 전달
            manager = main.MainManager(socketio=socketio)
            # 백그라운드 작업을 별도 스레드로 실행
            manager_thread = threading.Thread(target=start_manager)
            manager_thread.start()
            
            # 지도 서버 시작
            map_thread = threading.Thread(target=start_map_server)
            map_thread.start()
            
            # Flask-SocketIO 서버 실행
            socketio.run(app, host='0.0.0.0', port=5000, debug=False)
    except Exception as e:
        print(f"애플리케이션 시작 중 오류 발생: {e}")
        if manager:
            manager.stop_processes()
        sys.exit(1)
