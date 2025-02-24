from flask import Blueprint, render_template, request
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