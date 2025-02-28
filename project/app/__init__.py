from flask import Flask
from flask_migrate import Migrate
from flask_sqlalchemy import SQLAlchemy

import config

import threading
from cv import main
import json

db = SQLAlchemy()
migrate = Migrate()
tools_json_path = "/home/addinedu/dev_ws/ftud_branch/storagy-repo-1/project/cv/db/tools.json"

def create_app():
    app = Flask(__name__)
    app.config.from_object(config)

    # ORM
    db.init_app(app)
    migrate.init_app(app, db)
    from .models import Tool, Log

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
        Tool_init() # 위치 고민

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
    manager = main.MainManager(app)
    def start_manager():
        manager.start_processes()
        manager.face_process.join()
        manager.object_process.join()
    threading.Thread(target=start_manager, daemon=True).start()
    
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