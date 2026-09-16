import math

width = 1280
height = 720
hfov = 0.27

focal_length = width / (2 * math.tan(hfov / 2))
print(f"True focal length for {width}x{height} with {hfov} HFoV is: {focal_length}")
