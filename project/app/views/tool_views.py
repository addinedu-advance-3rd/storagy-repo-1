from flask import Blueprint, render_template, request, jsonify

from app import db
from app.models import Tool

bp = Blueprint('tool', __name__, url_prefix='/tool')

@bp.route('/list/')
def _list():
    tool_list = Tool.query.order_by(Tool.id.asc()).all()
    return render_template('tool/tool_list.html', tool_list=tool_list)

# CREATE: 새로운 Tool 추가
@bp.route('/', methods=['POST'])
def create_tool():
    data = request.json
    new_tool = Tool(name=data['name'], avail=data['avail'])
    db.session.add(new_tool)
    db.session.commit()
    return jsonify({'message': 'Tool created successfully', 'tool': {'id': new_tool.id, 'name': new_tool.name, 'avail': new_tool.avail}}), 201

# READ: 모든 Tool 조회
@bp.route('/', methods=['GET'])
def get_tools():
    tools = Tool.query.all()
    return jsonify([{'id': tool.id, 'name': tool.name, 'avail': tool.avail} for tool in tools]), 200

# READ: 특정 Tool 조회
@bp.route('/<int:tool_id>/', methods=['GET'])
def get_tool(tool_id):
    tool = Tool.query.get(tool_id)
    if tool:
        return jsonify({'id': tool.id, 'name': tool.name, 'avail': tool.avail}), 200
    else:
        return jsonify({'message': 'Tool not found'}), 404

# UPDATE: Tool 수정
@bp.route('/<int:tool_id>/', methods=['PUT'])
def update_tool(tool_id):
    data = request.json
    tool = Tool.query.get(tool_id)
    if tool:
        tool.name = data['name']
        tool.avail = data['avail']
        db.session.commit()
        return jsonify({'message': 'Tool updated successfully', 'tool': {'id': tool.id, 'name': tool.name, 'avail': tool.avail}}), 200
    else:
        return jsonify({'message': 'Tool not found'}), 404

# DELETE: Tool 삭제
@bp.route('/<int:tool_id>/', methods=['DELETE'])
def delete_tool(tool_id):
    tool = Tool.query.get(tool_id)
    if tool:
        db.session.delete(tool)
        db.session.commit()
        return jsonify({'message': 'Tool deleted successfully'}), 200
    else:
        return jsonify({'message': 'Tool not found'}), 404