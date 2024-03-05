import math

coord = [(0, 0), (-3.73, 2.64), (3.75, 2.64), (3.75, 0.54), (2.327, 0.54), (2.327, 1.36)]
distances = []
speed = 0.5
angular_speed = 0.5
time = []
angles = []
angular_times = []
angle_rad_list =[]
for i in range(len(coord) - 1):
    x1, y1 = coord[i]
    x2, y2 = coord[i + 1]

    # Calculate distance
    distance = round(math.sqrt((x2 - x1)**2 + (y2 - y1)**2), 4)
    distances.append(distance)

    # Calculate time
    time.append(round(distance / speed, 4))

    # Calculate angle with x-axis
    angle_rad = math.atan2(y2 - y1, x2 - x1)
    angle_deg = round(math.degrees(angle_rad),4)
    if i == 0:
        angle_deg -= 0
    else:
        angle_deg -= round(sum(angles[:i]),4) 
    angles.append(angle_deg)

    # Convert angle_deg back to radians for angular time calculation
    angle_rad = math.radians(angle_deg)
    angle_rad_list.append(angle_rad)
    # Calculate angular time
    angular_time = angle_rad / angular_speed
    angular_times.append(round(angular_time/10, 4))


print("Distances between consecutive points:")
print(distances)

print("Time taken for each segment:")
print(time)

print("Angles between each line segment and x-axis (in degrees):")
print(angles)

print("Angular time for each segment:")
print(angular_times)
print(angle_rad_list)