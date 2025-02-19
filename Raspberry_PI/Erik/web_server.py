from flask import Flask, request, jsonify
from flask_socketio import SocketIO
import threading

app = Flask(__name__)
socketio = SocketIO(app)

@socketio.on('request_update')
def handle_update():
    """Sender siste kart- og posisjonsdata til webgrensesnittet"""
    socketio.emit('update', {'position': robot_position, 'map': latest_scan_data})

def run_flask():
    """ Starter Flask-server med WebSocket-støtte """
    socketio.run(app, host='0.0.0.0', port=5000)