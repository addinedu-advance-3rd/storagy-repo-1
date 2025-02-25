from flask import Blueprint, render_template, request, jsonify

from app import db
from app.models import Log, Tool

bp = Blueprint('log', __name__, url_prefix='/log')

@bp.route('/list/')
def _list():
    page = request.args.get('page', type=int, default=1)
    kw = request.args.get('kw', type=str, default='')
    # 기본 로그 목록 쿼리
    log_query = Log.query.order_by(Log.rental_date.desc())
    if kw:
        tool = Tool.query.filter_by(id=kw).first()
        if tool is None:
            # tool이 존재하지 않을 경우
            return "그런 도구는 존재하지 않습니다.", 404
        # tool에 해당하는 로그 목록 쿼리
        log_query = log_query.filter(Log.tool_id == tool.id)
    log_list = log_query.paginate(page=page, per_page=10)
    return render_template('log/log_list.html', log_list=log_list, page=page, kw=kw)

# CREATE: 새로운 Log 추가
@bp.route('/', methods=['POST'])
def create_log():
    data = request.json
    new_log = Log(
        tool_id=data['tool_id'],
        user_name=data['user_name'],
        rental_date=data['rental_date'],
        return_date=data.get('return_date')  # return_date는 선택적
    )
    db.session.add(new_log)
    db.session.commit()
    return jsonify({'message': 'Log created successfully', 'log': {'id': new_log.id, 'tool_id': new_log.tool_id, 'user_name': new_log.user_name, 'rental_date': new_log.rental_date, 'return_date': new_log.return_date}}), 201

# READ: 모든 Log 조회
@bp.route('/', methods=['GET'])
def get_logs():
    logs = Log.query.all()
    return jsonify([{'id': log.id, 'tool_id': log.tool_id, 'user_name': log.user_name, 'rental_date': log.rental_date, 'return_date': log.return_date} for log in logs]), 200

# READ: 특정 Log 조회
@bp.route('/<int:log_id>/', methods=['GET'])
def get_log(log_id):
    log = Log.query.get(log_id)
    if log:
        return jsonify({'id': log.id, 'tool_id': log.tool_id, 'user_name': log.user_name, 'rental_date': log.rental_date, 'return_date': log.return_date}), 200
    else:
        return jsonify({'message': 'Log not found'}), 404

# UPDATE: Log 수정
@bp.route('/<int:log_id>/', methods=['PUT'])
def update_log(log_id):
    data = request.json
    log = Log.query.get(log_id)
    if log:
        log.tool_id = data['tool_id']
        log.user_name = data['user_name']
        log.rental_date = data['rental_date']
        log.return_date = data.get('return_date')  # 선택적
        db.session.commit()
        return jsonify({'message': 'Log updated successfully', 'log': {'id': log.id, 'tool_id': log.tool_id, 'user_name': log.user_name, 'rental_date': log.rental_date, 'return_date': log.return_date}}), 200
    else:
        return jsonify({'message': 'Log not found'}), 404

# DELETE: Log 삭제
@bp.route('/<int:log_id>/', methods=['DELETE'])
def delete_log(log_id):
    log = Log.query.get(log_id)
    if log:
        db.session.delete(log)
        db.session.commit()
        return jsonify({'message': 'Log deleted successfully'}), 200
    else:
        return jsonify({'message': 'Log not found'}), 404