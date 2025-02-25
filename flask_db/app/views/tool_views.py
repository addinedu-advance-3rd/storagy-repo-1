from flask import Blueprint, render_template, request
from app.models import Tool

bp = Blueprint('tool', __name__, url_prefix='/tool')

@bp.route('/list/')
def _list():
    tool_list = Tool.query.order_by(Tool.id.asc()).all() or [] # 고민
    return render_template('tool/tool_list.html', tool_list=tool_list)

