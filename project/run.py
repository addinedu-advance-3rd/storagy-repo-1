import eventlet
eventlet.monkey_patch()

from app import create_app
from cv import main
import threading
import signal
import atexit
import os
from flask_socketio import SocketIO

socketio = SocketIO(message_queue='redis://')
app = create_app()
manager = None
shutdown_event = threading.Event()

def start_manager():
    global manager
    manager = main.MainManager()
    manager.start_processes()
    try:
        manager.face_process.join()
        manager.object_process.join()
    except Exception as e:
        print(f"[ERROR] Error in processes: {e}")
    finally:
        print("[INFO] Manager processes completed")

def start_socketio_server():
    socketio.run(app)

def cleanup_resources():
    print("[INFO] Cleaning up resources...")
    
    # manager 프로세스 정리
    if manager:
        manager.stop_processes()
    
    # 종료 이벤트 설정
    shutdown_event.set()
    
    # socketio.stop()을 직접 호출하지 않고 프로세스 종료
    print("[INFO] Resources cleaned up successfully")

def signal_handler(sig, frame):
    print("[INFO] Shutdown signal received! Stopping all processes...")
    cleanup_resources()
    
    # 강제 종료가 필요한 경우 (최후의 수단)
    if sig == signal.SIGINT:
        print("[INFO] Force terminating process...")
        os._exit(0)  # 강제 종료

@socketio.on('connect')
def connect():
    print('외부 소켓 연결됨')

@socketio.on('disconnect')
def disconnect(reason):
    print('외부 소켓 끊김, reason:', reason)

if __name__ == "__main__":
    # 종료 시그널 핸들러 등록
    signal.signal(signal.SIGINT, signal_handler)
    #signal.signal(signal.SIGTERM, signal_handler)
    
    # 정상 종료 시 리소스 정리를 위한 atexit 핸들러 등록
    atexit.register(cleanup_resources)
    
    # 매니저 시작
    manager_thread = threading.Thread(target=start_manager, daemon=True)
    manager_thread.start()
    
    try:
        # 메인 스레드에서 SocketIO 서버 실행
        start_socketio_server()
    except KeyboardInterrupt:
        print("[INFO] KeyboardInterrupt detected in main thread")
    except Exception as e:
        print(f"[ERROR] Error in socketio server: {e}")
    finally:
        print("[INFO] Application shutting down...")
        cleanup_resources()
        
    # 스레드가 정상적으로 종료될 때까지 짧게 대기
    if manager_thread.is_alive():
        manager_thread.join(timeout=1.0)
        
    print("[INFO] Application shutdown complete")