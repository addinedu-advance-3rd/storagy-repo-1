from flask import Blueprint, render_template

bp = Blueprint('call', __name__, url_prefix='/call')

@bp.route('/')
def main():
    return render_template('call/call_main.html')