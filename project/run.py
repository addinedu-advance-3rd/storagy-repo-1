from app import create_app, socketio
from cv import main
import threading

def start_manager():
    try:
        manager.start_processes()
        manager.face_process.join()
        manager.object_process.join()
    except KeyboardInterrupt:
        manager.stop_processes()

if __name__ == "__main__":
    app = create_app()
    with app.app_context():
        manager = main.MainManager()
        threading.Thread(target=start_manager, daemon=True).start()
        socketio.run(app)
