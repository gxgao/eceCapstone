z = 150

back_camera_array = [
                        (105, -48, -31),
                        (85, -35, -27),
                        (67, -31, -25),
                        (56, -26, -24),
                    ]
                    # (40, -21, -16), (31, -18, -13)
calibrate = None
sleep_extra = False

for i in range(len(back_camera_array)):
                        
    if z > back_camera_array[i][0]:
        calibrate = back_camera_array[i]
        if i == len(back_camera_array) - 1:
            sleep_extra = True
        break

print(f"picked {calibrate} and sleep_extra={sleep_extra}")
                    
                    
def lerp(x1, y1, x2, y2, x):
    return y1 + (y2 - y1) / (x2 - x1) * (x - x1)

if calibrate is not None and z > 60:
    cali_z, l_x, r_x = calibrate
    # if not last range, we look at prev indx and do linear 
                        # interpolation to find what range we should be within 
    if i > 0 :
        prev_z, prev_lx, prev_rx = back_camera_array[i - 1] 
                            
        l_x = lerp(prev_z, prev_lx, cali_z, l_x, z) 
        r_x = lerp(prev_z, prev_rx, cali_z, r_x, z) 
    else: 
        # i == 0
        prev_z, prev_lx, prev_rx = back_camera_array[0]
        cali_z, l_x, r_x = back_camera_array[1]

        l_x = lerp(prev_z, prev_lx, cali_z, l_x, z) 
        r_x = lerp(prev_z, prev_rx, cali_z, r_x, z) 

print(prev_z, l_x, r_x, z)