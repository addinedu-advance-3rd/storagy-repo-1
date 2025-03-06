import os
import sys
import threading
import time
from app import create_app, socketio
from cv import main  # main 모듈 내에 MainManager 클래스가 있다고 가정
import signal
import importlib.util
import atexit

# 프로젝트 루트 경로 설정
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CIRCLE_PATH = os.path.join(os.path.dirname(BASE_DIR), 'circle/src')

print(f"BASE_DIR: {BASE_DIR}")  # 디버깅 로그 추가
print(f"CIRCLE_PATH: {CIRCLE_PATH}")  # 디버깅 로그 추가

sys.path.append(BASE_DIR)
sys.path.append(CIRCLE_PATH)

# 전역 변수로 백그라운드 스레드를 저장
manager_thread = None
map_thread = None
manager = None  # MainManager 인스턴스
map_visualizer = None  # MapVisualizer 인스턴스

# 전역 변수로 ROS 관련 상태 관리
ros_initialized = False

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
    try:
        import rclpy
        from circle.circle.map_to_web import MapVisualizer
        
        print("ROS2 초기화 시작...")
        global ros_initialized
        if not ros_initialized:
            if not rclpy.ok():
                rclpy.init()
            ros_initialized = True
        print("ROS2 초기화 완료")
        
        print("MapVisualizer 노드 생성 시작...")
        global map_visualizer
        map_visualizer = MapVisualizer()
        print("MapVisualizer 노드 생성 완료")
        
        # 단일 스핀 대신 executor 사용
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()
        executor.add_node(map_visualizer)
        
        try:
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
                time.sleep(0.01)  # CPU 사용량 감소
        except Exception as e:
            print(f"Executor 실행 중 오류: {e}")
    except ImportError as e:
        print(f"지도 서버 모듈 로드 실패: {e}")
        print(f"PYTHONPATH: {sys.path}")
    except Exception as e:
        print(f"지도 서버 실행 중 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if map_visualizer:
            try:
                map_visualizer.destroy_node()
            except Exception as e:
                print(f"노드 종료 중 오류: {e}")

def signal_handler(sig, frame):
    print("\n[INFO] 종료 신호를 받았습니다. 프로세스를 정리합니다...")
    try:
        if manager:
            manager.stop_processes()
        
        # ROS 종료 처리 개선
        global ros_initialized
        if ros_initialized and rclpy.ok():
            try:
                rclpy.shutdown()
                ros_initialized = False
                print("[INFO] ROS2 종료 완료")
            except Exception as e:
                print(f"[ERROR] ROS2 종료 중 오류: {e}")
    except Exception as e:
        print(f"[ERROR] 프로세스 종료 중 오류 발생: {e}")
    finally:
        print("[INFO] 프로그램을 종료합니다.")
        os._exit(0)  # sys.exit 대신 os._exit 사용

if __name__ == "__main__":
    # 신호 핸들러 등록 (Ctrl+C 등)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # ROS_DOMAIN_ID 설정 (다른 컴퓨터와 동일하게)
    if not os.environ.get('ROS_DOMAIN_ID'):
        os.environ['ROS_DOMAIN_ID'] = '13'  # 다른 컴퓨터와 동일한 값으로 설정
    
    # 프로세스 종료 시 정리 작업 등록
    def cleanup():
        print("[INFO] 프로그램 종료 중...")
        if manager:
            try:
                manager.stop_processes()
            except:
                pass
    atexit.register(cleanup)
    
    try:
        app = create_app()
        with app.app_context():
            # socketio 객체를 MainManager에 전달
            manager = main.MainManager(socketio=socketio)
            # 백그라운드 작업을 별도 스레드로 실행
            manager_thread = threading.Thread(target=start_manager, daemon=True)
            manager_thread.start()
            
            # 지도 서버 시작 (daemon=True로 설정하여 메인 스레드 종료 시 자동 종료)
            map_thread = threading.Thread(target=start_map_server, daemon=True)
            map_thread.start()
            
            # ROS2 환경 확인
            ros_distro = os.environ.get('ROS_DISTRO')
            ros_domain_id = os.environ.get('ROS_DOMAIN_ID')
            print(f"ROS_DISTRO: {ros_distro}")  # 디버깅 로그 추가
            print(f"ROS_DOMAIN_ID: {ros_domain_id}")  # 디버깅 로그 추가
            
            # Flask-SocketIO 서버 실행
            socketio.run(app, host='0.0.0.0', port=5000, debug=False)
    except Exception as e:
        print(f"애플리케이션 시작 중 오류 발생: {e}")
        if manager:
            manager.stop_processes()
        sys.exit(1)
