from flask import Flask
from flask_migrate import Migrate
from flask_sqlalchemy import SQLAlchemy

import config

from cv import main
import threading

db = SQLAlchemy()
migrate = Migrate()

def create_app():
    app = Flask(__name__)
    app.config.from_object(config)

    # ORM
    db.init_app(app)
    migrate.init_app(app, db)
    from . import models

    # Blueprint
    from .views import main_views, tool_views, log_views, call_views
    app.register_blueprint(main_views.bp)
    app.register_blueprint(tool_views.bp)
    app.register_blueprint(log_views.bp)
    app.register_blueprint(call_views.bp)
    
    # Filter
    from .filter import format_datetime
    app.jinja_env.filters['datetime'] = format_datetime

    # CV
    manager = main.MainManager()

    def start_manager():
        manager.start_processes()
        manager.face_process.join()
        manager.object_process.join()

    # Start the manager in a separate thread
    threading.Thread(target=start_manager, daemon=True).start()
    
    return app