from app import create_app, socketio
from cv import main
import threading

app = create_app()

manager = main.MainManager()
def start_manager():
    with app.app_context():
        try:
            manager.start_processes()
            manager.face_process.join()
            manager.object_process.join()
        except KeyboardInterrupt:
            manager.stop_processes()

if __name__ == "__main__":
    threading.Thread(target=start_manager, daemon=True).start()
    socketio.run(app)


