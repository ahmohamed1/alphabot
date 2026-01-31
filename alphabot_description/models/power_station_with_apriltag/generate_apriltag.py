"""Generate a simple AprilTag-like PNG for use with the Gazebo model.

This produces a square black/white tag with a thick outer border and an inner pattern
that visually resembles an AprilTag (for testing/visual detection only).

Requires Pillow: pip install Pillow
Run: python3 generate_apriltag.py
"""
from PIL import Image, ImageDraw, ImageFont

OUT = "materials/textures/apriltag.png"
SIZE = 400
BORDER = 32
INNER = 48

img = Image.new("RGB", (SIZE, SIZE), "white")
d = ImageDraw.Draw(img)

# outer border
d.rectangle([0,0,SIZE-1,SIZE-1], fill="black")
# inner white square
d.rectangle([BORDER, BORDER, SIZE-BORDER-1, SIZE-BORDER-1], fill="white")
# inner black pattern (simple nested squares)
for i in range(4):
    off = BORDER + i*(INNER//2)
    d.rectangle([off, off, SIZE-off-1, SIZE-off-1], outline="black", width=INNER//8)

# add a central number as ID
try:
    font = ImageFont.truetype("DejaVuSans-Bold.ttf", 40)
except Exception:
    font = ImageFont.load_default()
text = "ID0"
w, h = d.textsize(text, font=font)
d.text(((SIZE-w)/2, (SIZE-h)/2), text, fill="black", font=font)

img.save(OUT)
print(f"Wrote {OUT}. Copy this file into the model materials/textures folder if needed.")
