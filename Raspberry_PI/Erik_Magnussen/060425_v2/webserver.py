from flask import Flask, send_file, render_template_string, request
import os
import time

app = Flask(__name__)
BASE_DIR = "/home/admin/GitHub/SelfDriving/Raspberry_PI/Erik_Magnussen/060425_v2"
MAP_IMAGE_PATH = os.path.join(BASE_DIR, "occupancy_grid.png")

@app.route("/")
def index():
    # Legger til et tidsstempel for � tvinge en oppdatering
    timestamp = int(time.time())
    html = f"""
    <!DOCTYPE html>
    <html>
      <head>
        <title>Occupancy Grid Map</title>
      </head>
      <body>
        <h1>Occupancy Grid Map</h1>
        <img src="/map?ts={timestamp}" alt="Occupancy Grid Map" style="max-width:100%;">
      </body>
    </html>
    """
    return render_template_string(html)

@app.route("/map")
def map_image():
    if os.path.exists(MAP_IMAGE_PATH):
        response = send_file(MAP_IMAGE_PATH, mimetype="image/png")
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Pragma'] = 'no-cache'
        response.headers['Expires'] = '0'
        return response
    else:
        return "Map image not found", 404

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001, debug=True)
