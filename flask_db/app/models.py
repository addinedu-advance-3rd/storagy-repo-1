from app import db

class Tool(db.Model):
    id = db.Column(db.Integer, primary_key=True) # 공구 코드
    name = db.Column(db.String(100), nullable=False)
    avail = db.Column(db.Boolean, nullable=False)

class Log(db.Model):
    id = db.Column(db.Integer, primary_key=True) # 식별 코드
    tool_id = db.Column(db.Integer, db.ForeignKey('tool.id'), nullable=False)
    tool = db.relationship('Tool', backref=db.backref('Log_set'))
    user_name = db.Column(db.String(100), nullable=False)
    rental_date = db.Column(db.DateTime(), nullable=False)
    return_date = db.Column(db.DateTime(), nullable=True)
