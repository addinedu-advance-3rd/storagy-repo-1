import os

BASE_DIR = os.path.dirname(__file__)

SQLALCHEMY_DATABASE_URI = 'sqlite:///{}'.format(os.path.join(BASE_DIR, 'app.db'))
SQLALCHEMY_TRACK_MODIFICATIONS = False

#REDIS_URL = 'redis://localhost:6379/0'
#SOCKETIO_MESSAGE_QUEUE = REDIS_URL