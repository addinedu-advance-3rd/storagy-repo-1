from flask import Blueprint, render_template, request, jsonify
from datetime import datetime
import sys, os

from app import db
from app.models import Tool

bp = Blueprint('tool', __name__, url_prefix='/tool')

@bp.route('/list/')
def _list():
    """도구 목록 페이지"""
    # 데이터베이스 상태 확인
    tool_list = Tool.query.all()
    
    # 디버깅 정보 출력
    print("\n===== 도구 상태 확인 =====")
    for tool in tool_list:
        print(f"도구: {tool.name}, ID: {tool.id}, 상태: {'사용 가능' if tool.avail else '사용 중'}")
    print("==========================\n")
    
    # 공유 메모리 접근 시도는 제거하고 대신 상태 파일 확인
    try:
        # 상태 파일 경로
        status_file = os.path.join(os.path.dirname(__file__), '..', '..', 'cv', 'tool_status.json')
        
        # 파일이 존재하는지 확인
        if os.path.exists(status_file):
            import json
            with open(status_file, 'r') as f:
                try:
                    tool_status = json.load(f)
                    print("\n===== 상태 파일 확인 =====")
                    for tool_name, avail in tool_status.items():
                        print(f"도구: {tool_name}, 상태: {'사용 가능' if avail else '사용 중'}")
                        
                        # 데이터베이스와 파일 상태가 다른 경우 데이터베이스 업데이트
                        db_tool = next((t for t in tool_list if t.name == tool_name), None)
                        if db_tool and db_tool.avail != avail:
                            print(f"상태 불일치 감지: {tool_name} - DB: {db_tool.avail}, 파일: {avail}")
                            db_tool.avail = avail
                            db.session.commit()
                            print(f"데이터베이스 상태 업데이트: {tool_name} -> {avail}")
                    print("==========================\n")
                except json.JSONDecodeError:
                    print("상태 파일 형식 오류")
        else:
            print("상태 파일이 존재하지 않습니다.")
    except Exception as e:
        print(f"상태 파일 확인 중 오류: {e}")
    
    # 최신 상태로 다시 조회
    tool_list = Tool.query.all()
    now = datetime.now()
    
    return render_template('tool/tool_list.html', tool_list=tool_list, now=now)

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