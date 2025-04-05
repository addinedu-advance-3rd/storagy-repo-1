from flask import Blueprint, request, jsonify
from app.models import Tool
from app import db

bp = Blueprint('api', __name__, url_prefix='/api')

@bp.route('/tool/update', methods=['POST'])
def update_tool():
    """도구 상태를 업데이트하는 API 엔드포인트"""
    try:
        # 요청 데이터 가져오기
        data = request.get_json()
        
        # 필수 필드 확인
        if not data or 'tool_name' not in data or 'avail' not in data:
            return jsonify({'error': '필수 필드가 누락되었습니다.'}), 400
        
        # API 키 확인 (보안)
        if data.get('api_key') != 'your_secret_key':
            return jsonify({'error': '인증 실패'}), 401
        
        # 도구 이름과 상태 가져오기
        tool_name = data['tool_name']
        avail = data['avail']
        
        # 도구 찾기
        tool = Tool.query.filter_by(name=tool_name).first()
        
        if not tool:
            return jsonify({'error': f'도구를 찾을 수 없습니다: {tool_name}'}), 404
        
        # 상태 업데이트
        tool.avail = avail
        db.session.commit()
        
        # 응답
        return jsonify({
            'success': True,
            'tool_name': tool_name,
            'avail': avail,
            'id': tool.id
        })
    except Exception as e:
        db.session.rollback()
        return jsonify({'error': str(e)}), 500

@bp.route('/tool/force-update', methods=['GET'])
def force_update_tools():
    """모든 도구 상태를 강제로 업데이트하는 API 엔드포인트"""
    try:
        # 모든 도구 가져오기
        tools = Tool.query.all()
        
        # 공유 메모리에서 상태 가져오기
        try:
            from project.cv.main import main_manager
            memory_status = main_manager.tools_status if hasattr(main_manager, 'tools_status') else {}
        except Exception as e:
            print(f"공유 메모리 접근 오류: {e}")
            memory_status = {}
        
        # 업데이트 결과
        results = []
        
        # 각 도구 상태 업데이트
        for tool in tools:
            # 메모리에 상태가 있으면 해당 상태로 업데이트
            if tool.name in memory_status:
                memory_avail = memory_status[tool.name]
                if tool.avail != memory_avail:
                    tool.avail = memory_avail
                    results.append({
                        'name': tool.name,
                        'id': tool.id,
                        'old_status': not memory_avail,
                        'new_status': memory_avail
                    })
        
        # 변경사항 저장
        if results:
            db.session.commit()
        
        # 응답
        return jsonify({
            'success': True,
            'updated_tools': results,
            'total_tools': len(tools)
        })
    except Exception as e:
        db.session.rollback()
        return jsonify({'error': str(e)}), 500

@bp.route('/tool/direct-update/<tool_name>/<int:status>', methods=['GET'])
def direct_update_tool(tool_name, status):
    """도구 상태를 직접 업데이트하는 API 엔드포인트"""
    try:
        # 도구 찾기
        tool = Tool.query.filter_by(name=tool_name).first()
        
        if not tool:
            return jsonify({'error': f'도구를 찾을 수 없습니다: {tool_name}'}), 404
        
        # 상태 변환 (0 = 사용 중, 1 = 사용 가능)
        avail = bool(status)
        
        # 현재 상태 확인
        current_status = tool.avail
        
        # 상태 업데이트
        tool.avail = avail
        db.session.commit()
        
        # 응답
        return jsonify({
            'success': True,
            'tool_name': tool_name,
            'old_status': current_status,
            'new_status': avail,
            'id': tool.id
        })
    except Exception as e:
        db.session.rollback()
        return jsonify({'error': str(e)}), 500 