from PIL import Image, ImageDraw, ImageFont
import math


# --- Configuration ---
image_path = 'image.png'           # input (your uploaded track image)
output_path = 'annotated_track.png'  # output

# Segment lengths ("Length of each section" column), in meters
segment_lengths = [
    13.41, 10.11, 6.73, 19.25, 6.30, 4.76, 9.56, 3.76, 3.05, 8.78,
    16.62, 20.96, 18.86, 52.90, 18.75, 13.06, 16.48, 5.20, 14.11, 4.42,
    6.98, 11.17, 5.46, 11.58, 10.58, 26.38, 8.78, 18.92, 5.42, 16.04,
    15.62, 12.18, 8.85, 6.13, 8.17, 8.97, 12.61, 18.16, 4.92, 12.87,
    4.48, 18.74, 2.60, 10.10, 10.15, 12.41, 7.44, 9.63, 5.25, 69.54,
    10.52, 14.25, 8.37, 8.61, 17.08, 8.48, 10.26, 6.40, 8.30, 9.64,
    10.37, 10.22, 9.85, 7.54, 9.96, 10.01, 10.93, 4.24, 30.43, 6.36,
    11.72, 11.09, 8.30, 11.52, 4.73, 9.87, 0.89, 35.34, 8.93, 17.87,
    10.72, 6.72, 12.57, 9.77, 10.32, 7.88, 8.91, 13.12, 10.98, 7.69,
    11.03, 5.85, 6.65, 7.48, 4.26, 13.06, 4.89, 10.26, 7.67, 8.93,
    9.33, 8.82, 6.13, 5.44
]
assert len(segment_lengths) == 104

# Flag bends vs straights from the table (1-based indices of straights)
straight_indices = {2,4,7,10,14,19,22,28,30,34,40,42,50,55,60,69,76,78,87,89,93,96}
is_bend = [False if (i+1) in straight_indices else True for i in range(104)]

# Load image
img = Image.open(image_path).convert('RGBA')
W, H = img.size

# Transparent overlay for labels
overlay = Image.new('RGBA', (W, H), (0, 0, 0, 0))
draw = ImageDraw.Draw(overlay)

# Fonts
try:
    font_main = ImageFont.truetype('DejaVuSans.ttf', size=max(10, int(0.018*max(W,H))))
    font_total = ImageFont.truetype('DejaVuSans-Bold.ttf', size=max(12, int(0.03*max(W,H))))
except Exception:
    font_main = ImageFont.load_default()
    font_total = ImageFont.load_default()

# Approximate centerline polyline (relative positions, tweak to your track)
anchors_rel = [
    (0.10, 0.80), (0.18, 0.68), (0.22, 0.55), (0.28, 0.42), (0.38, 0.30),
    (0.52, 0.22), (0.70, 0.20), (0.85, 0.28), (0.86, 0.40), (0.78, 0.52),
    (0.64, 0.60), (0.48, 0.66), (0.34, 0.70), (0.22, 0.74), (0.12, 0.82)
]
anchors = [(int(x*W), int(y*H)) for (x, y) in anchors_rel]

def dist(a, b): return math.hypot(b[0]-a[0], b[1]-a[1])
seg_lengths_poly = [dist(anchors[i], anchors[i+1]) for i in range(len(anchors)-1)]
total_poly_len = sum(seg_lengths_poly)

# Evenly resample the polyline to N points (N segments)
N = len(segment_lengths)
step = total_poly_len / (N+1)
resampled_points = []
for k in range(N):
    target = (k+1) * step
    cumulative = 0.0
    for i in range(len(anchors)-1):
        a, b = anchors[i], anchors[i+1]
        seg_len = dist(a, b)
        if cumulative + seg_len >= target:
            t = max(0.0, min(1.0, (target - cumulative) / seg_len))
            x = int(a[0] + t*(b[0]-a[0]))
            y = int(a[1] + t*(b[1]-a[1]))
            resampled_points.append((x, y))
            break
        cumulative += seg_len
    else:
        resampled_points.append(anchors[-1])

def local_angle(points, idx):
    if idx <= 0:
        p0, p1 = points[idx], points[min(idx+1, len(points)-1)]
    elif idx >= len(points)-1:
        p0, p1 = points[max(idx-1, 0)], points[idx]
    else:
        p0, p1 = points[idx-1], points[idx+1]
    dx, dy = (p1[0]-p0[0]), (p1[1]-p0[1])
    return math.degrees(math.atan2(dy, dx))

COLOR_STRAIGHT = (0, 200, 255, 255)  # cyan
COLOR_BEND     = (255, 140, 0, 255)  # orange

# Draw all labels
for i, (pt, L, bend) in enumerate(zip(resampled_points, segment_lengths, is_bend), start=1):
    label = f"S{i}: {L:.2f} m"
    color = COLOR_BEND if bend else COLOR_STRAIGHT

    txt_img = Image.new('RGBA', (600, 100), (0, 0, 0, 0))
    txt_draw = ImageDraw.Draw(txt_img)
    txt_draw.text((10, 10), label, font=font_main, fill=color, stroke_width=2, stroke_fill=(0,0,0,200))

    angle = local_angle(resampled_points, i-1)
    rot_txt = txt_img.rotate(angle, expand=True)

    # Offset slightly perpendicular to the path for readability
    rad = math.radians(angle + 90)
    offset = (int(15*math.cos(rad)), int(15*math.sin(rad)))
    overlay.alpha_composite(rot_txt, (pt[0] + offset[0], pt[1] + offset[1]))

    # optional marker dot at the centerline point
    draw.ellipse([pt[0]-2, pt[1]-2, pt[0]+2, pt[1]+2], fill=(255,255,255,200), outline=(0,0,0,180))

# Totals
single_lap = sum(segment_lengths)
endurance_18_km = single_lap * 18 / 1000.0
total_text = f"Single Lap: {single_lap:.2f} m\n18 Laps: {endurance_18_km:.2f} km"

# place totals top-right
tmp = ImageDraw.Draw(Image.new('RGBA', (1,1))).textbbox((0,0), total_text, font=font_total)
bbox_w, bbox_h = tmp[2], tmp[3]
margin = 10
box_x = W - bbox_w - margin
box_y = margin
box_pad = 8
draw.rectangle([box_x - box_pad, box_y - box_pad, box_x + bbox_w + box_pad, box_y + bbox_h + box_pad], fill=(0,0,0,120))
draw.text((box_x, box_y), total_text, font=font_total, fill=(255,255,255,255))

# legend (top-left)
legend_text = "Legend:\nStraight segment\nBend segment"
lg_x, lg_y = margin, margin
tmp = ImageDraw.Draw(Image.new('RGBA', (1,1))).textbbox((0,0), legend_text, font=font_main)
lg_w, lg_h = tmp[2], tmp[3]
draw.rectangle([lg_x-6, lg_y-6, lg_x+lg_w+6, lg_y+lg_h+6], fill=(0,0,0,120))
draw.text((lg_x, lg_y), "Legend:", font=font_main, fill=(255,255,255,255))
swatch_h, swatch_w = 16, 26
draw.rectangle([lg_x, lg_y+26, lg_x+swatch_w, lg_y+26+swatch_h], fill=COLOR_STRAIGHT)
draw.text((lg_x+swatch_w+8, lg_y+24), "Straight segment", font=font_main, fill=(255,255,255,255))
draw.rectangle([lg_x, lg_y+26+swatch_h+8, lg_x+swatch_w, lg_y+26+2*swatch_h+8], fill=COLOR_BEND)
draw.text((lg_x+swatch_w+8, lg_y+26+swatch_h+6), "Bend segment", font=font_main, fill=(255,255,255,255))

# Compose and save
annotated = Image.alpha_composite(img, overlay)
annotated.convert('RGB').save(output_path, quality=95)

print(f"Saved annotated image to {output_path}")
print(f"Computed single lap distance: {single_lap:.2f} m")
print(f"Computed endurance (18 laps): {endurance_18_km:.2f} km")

# --- Graphs ---
import matplotlib.pyplot as plt
import numpy as np

# Reuse your variables
lengths = np.array(segment_lengths)
segments = np.arange(1, len(segment_lengths)+1)
is_bend_arr = np.array([False if (i+1) in straight_indices else True for i in range(104)])

cum = np.cumsum(lengths)
max_len = lengths.max()
max_idx = int(np.argmax(lengths)) + 1
min_len = lengths.min()
min_idx = int(np.argmin(lengths)) + 1

# 1) Cumulative distance
plt.figure(figsize=(14, 5))
plt.title('Cumulative Distance Along Lap')
plt.plot(segments, cum, color='#333', lw=2)
plt.xlabel('Segment Number')
plt.ylabel('Cumulative Distance (m)')
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('cumulative_distance.png', dpi=160)



print(f"Graphs saved: cumulative_distance.png")