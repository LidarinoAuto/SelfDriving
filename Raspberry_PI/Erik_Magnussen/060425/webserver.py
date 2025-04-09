from flask import Flask, send_file, render_template_string
import os

app = Flask(__name__)
MAP_IMAGE_PATH = "occupancy_grid.png"

@app.route("/")
def index():
    html = """
    <!DOCTYPE html>
    <html>
      <head>
        <title>Occupancy Grid Map</title>
      </head>
      <body>
        <h1>Occupancy Grid Map</h1>
        <img src="/map" alt="Occupancy Grid Map" style="max-width:100%;">
      </body>
    </html>
    """
    return render_template_string(html)

@app.route("/map")
def map_image():
    if os.path.exists(MAP_IMAGE_PATH):
        return send_file(MAP_IMAGE_PATH, mimetype="image/png")
    else:
        return "Map image not found", 404

if __name__ == "__main__":
    # Appen vil kj�re p� alle grensesnitt (0.0.0.0) p� port 5000
    app.run(host="0.0.0.0", port=5000, debug=True)

