#!/usr/bin/env python3
"""
Web-based 3D Grasp Viewer
Access from browser: http://robot_ip:8080
"""

import os
os.environ['LD_PRELOAD'] = '/usr/lib/aarch64-linux-gnu/libffi.so.7'

import rospy
from perception.msg import GraspDetectionArray3D
from flask import Flask, render_template_string, jsonify
import threading
import json

app = Flask(__name__)
latest_data = {"detections": [], "timestamp": 0}
lock = threading.Lock()

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>3D Grasp Viewer</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .container { max-width: 1200px; margin: 0 auto; }
        .header { background: #2196F3; color: white; padding: 20px; border-radius: 5px; }
        .detection { background: white; margin: 10px 0; padding: 15px; border-radius: 5px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
        .grasp { background: #f9f9f9; margin: 5px 0; padding: 10px; border-left: 3px solid #4CAF50; }
        .label { font-weight: bold; color: #333; }
        .value { color: #666; }
        .refresh { background: #4CAF50; color: white; border: none; padding: 10px 20px; border-radius: 5px; cursor: pointer; }
        .status { padding: 10px; margin: 10px 0; border-radius: 5px; }
        .status.active { background: #c8f7c5; }
        .status.inactive { background: #ffcdd2; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🤖 3D Grasp Visualization</h1>
            <button class="refresh" onclick="refreshData()">Refresh</button>
        </div>
        
        <div id="status" class="status inactive">
            <span class="label">Status:</span> <span id="status-text">Waiting for data...</span>
        </div>
        
        <div id="detections"></div>
    </div>
    
    <script>
        function refreshData() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    const container = document.getElementById('detections');
                    const statusDiv = document.getElementById('status');
                    const statusText = document.getElementById('status-text');
                    
                    if (data.detections.length > 0) {
                        statusDiv.className = 'status active';
                        statusText.textContent = `Active - ${data.detections.length} objects detected`;
                        
                        container.innerHTML = data.detections.map((det, i) => `
                            <div class="detection">
                                <h3>📦 Object ${i + 1}</h3>
                                <p><span class="label">Score:</span> <span class="value">${det.score.toFixed(3)}</span></p>
                                <p><span class="label">3D Position:</span> <span class="value">
                                    X: ${det.x.toFixed(3)}m, Y: ${det.y.toFixed(3)}m, Z: ${det.z.toFixed(3)}m
                                </span></p>
                                <p><span class="label">Dimensions:</span> <span class="value">
                                    ${det.width.toFixed(3)}m × ${det.height.toFixed(3)}m × ${det.depth.toFixed(3)}m
                                </span></p>
                                <p><span class="label">Grasp Poses:</span> <span class="value">${det.grasps} candidates</span></p>
                                ${det.grasp_details.map((g, j) => `
                                    <div class="grasp">
                                        Grasp ${j + 1}: Confidence ${g.confidence.toFixed(2)}, Width ${g.width.toFixed(3)}m
                                    </div>
                                `).join('')}
                            </div>
                        `).join('');
                    } else {
                        statusDiv.className = 'status inactive';
                        statusText.textContent = 'No detections';
                        container.innerHTML = '<p>Place objects in front of camera...</p>';
                    }
                });
        }
        
        // Auto refresh every second
        setInterval(refreshData, 1000);
        refreshData();
    </script>
</body>
</html>
"""

class WebViewer:
    def __init__(self):
        rospy.init_node('web_3d_viewer', anonymous=True)
        rospy.Subscriber('/perception/hand/grasps_3d', GraspDetectionArray3D, self.grasp_callback)
        
    def grasp_callback(self, msg):
        global latest_data
        with lock:
            latest_data["timestamp"] = msg.header.stamp.to_sec()
            latest_data["detections"] = []
            
            for det in msg.detections:
                detection = {
                    "score": det.score,
                    "x": det.center_3d.x if det.center_3d else 0,
                    "y": det.center_3d.y if det.center_3d else 0,
                    "z": det.center_3d.z if det.center_3d else 0,
                    "width": det.dimensions.x if det.dimensions else 0,
                    "height": det.dimensions.y if det.dimensions else 0,
                    "depth": det.dimensions.z if det.dimensions else 0,
                    "grasps": len(det.grasp_poses_3d),
                    "grasp_details": []
                }
                
                for grasp in det.grasp_poses_3d[:3]:  # Top 3 grasps
                    detection["grasp_details"].append({
                        "confidence": grasp.confidence,
                        "width": grasp.width
                    })
                
                latest_data["detections"].append(detection)

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/data')
def get_data():
    with lock:
        return jsonify(latest_data)

def run_flask():
    app.run(host='0.0.0.0', port=8080, debug=False)

if __name__ == '__main__':
    viewer = WebViewer()
    
    # Start Flask in separate thread
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()
    
    print("=" * 60)
    print("Web 3D Viewer Started!")
    print("=" * 60)
    print(f"Open browser: http://localhost:8080")
    print("Or from network: http://<robot_ip>:8080")
    print("=" * 60)
    
    rospy.spin()