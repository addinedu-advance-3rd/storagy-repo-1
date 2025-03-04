from flask import Flask
from flask_migrate import Migrate
from flask_sqlalchemy import SQLAlchemy
from flask_socketio import SocketIO
import config
import json

db = SQLAlchemy()
migrate = Migrate()
socketio = SocketIO()

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
    from .views import main_views, tool_views, log_views, call_views
    app.register_blueprint(main_views.bp)
    app.register_blueprint(tool_views.bp)
    app.register_blueprint(log_views.bp)
    app.register_blueprint(call_views.bp)
    
    # Websocket
    socketio.init_app(app)

    # Filter
    from .filter import format_datetime
    app.jinja_env.filters['datetime'] = format_datetime

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