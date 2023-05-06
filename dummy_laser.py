import rospy
from geometry_msgs.msg import PoseStamped 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler
import time
import pdb
import pandas as pd
import numpy as np
from numpy.linalg import norm
from enum import Enum


cnt = 0
bin_window = None

def clean(data):
    def remove_15(df):
        # pdb.set_trace()
        for i in range(df.size):
            l = i - 1
            r = i + 1 if i + 1 < df.size else -1

            if df.iloc[i, 0] == np.inf and df.iloc[l, 0] != np.inf and df.iloc[r, 0] != np.inf:
                df.iloc[i, 0] = (df.iloc[l, 0] + df.iloc[r, 0]) / 2
        return df

    return (
             pd.DataFrame({"ranges": data.ranges})
             .pipe(remove_15)
             .replace([np.inf], 15)
           )

def identify_bin_window(df, aruco_dist_guess):
    # todo change this, diff dist don't work next to wall
    # idk...
    

    find_df = (
            df.diff()
            .abs()
            .pipe(lambda df: df.where(df > .20))
            .dropna()
            )
    
    def get_window(L, R):
        if L <= R:
            return df[L:R]
        else:
            return pd.concat([df[L:], df[:R]])
    
    closest_dist = np.inf
    closest = 0 # window is [closest-1:closest]
    for i in range(find_df.size):
        cur_l = find_df.index[i-1]
        cur_r = find_df.index[i]
        mean_dist = get_window(cur_l, cur_r).mean()[0]

        if abs(mean_dist - aruco_dist_guess) < abs(closest_dist - aruco_dist_guess):
            closest = i
            closest_dist = mean_dist

    # pdb.set_trace()
    return get_window(find_df.index[closest-1], find_df.index[closest])

def identify_bin(df, bin_window):
    bin_window_s = bin_window["ranges"].squeeze().reset_index(drop=True)
    def similarity(window):
        # return np.dot(window, bin_window) / (norm(window) * norm(bin_window))
        return norm(window.reset_index(drop=True) - bin_window_s, ord=1)

    s = df["ranges"].squeeze()
    return (
            pd.concat([s.iloc[-len(bin_window)+1:], s, s.iloc[:len(bin_window)-1]])
            .rolling(len(bin_window), center=True)
            .apply(similarity)
            .idxmin()
        )

first = True
def callback(data):
    global cnt
    global bin_window
    global first
    
    cnt += 1
    if cnt % 15 != 0:
        return
    
    print("min dist", min(data.ranges[500:650]))
    if False:
        df = clean(data)
        print("min_dist", min(data.ranges[500:650]))
        def get_window(L, R):
            L = L % df.shape[0]
            R = R % df.shape[0]
            if L <= R:
                return df[L:R]
            else:
                return pd.concat([df[L:], df[:R]])
        # todo guess angle
        if first:
            bin_window = identify_bin_window(df, .44)
            first = False
            print(bin_window)
        # pdb.set_trace()
    
        # print(get_window(center_of_bin - bin_window.shape[0] // 2, center_of_bin + bin_window.shape[0] // 2))
        print("normal identify", identify_bin(df, bin_window))
        print("filtered identify", identify_bin(df.iloc[400:800], bin_window))



rospy.init_node('testing_laser')
rate = rospy.Rate(.1)
sub = rospy.Subscriber('/scan', LaserScan, callback)


while 1:
    time.sleep(2)
    rate.sleep() 
  
