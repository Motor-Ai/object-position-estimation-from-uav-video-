# Name:        Getting frames from video
# Purpose:
#
# Author:      güray
# Additional
# source:      https://stackoverflow.com/questions/33311153/python-extracting-and-saving-video-frames
# Created:     09/08/2022
# Copyright:   (c) guray 2022
# Licence:     <your licence>
#-------------------------------------------------------------------------------

import cv2
import time
import os
from pathlib import Path 

def video_to_frames(input_loc, output_loc):
    """Function to extract frames from input video file
    and save them as separate frames in an output directory.
    Args:
        input_loc: Input video file.
        output_loc: Output directory to save the frames.
    Returns:
        None
    """
    try:
        os.mkdir(output_loc)
    except OSError:
        pass
    # Log the time
    time_start = time.time()
    # Start capturing the feed
    cap = cv2.VideoCapture(input_loc)
    # Find the number of frames
    video_length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) - 1
    print ("Number of frames: ", video_length)
    count = 0
    print ("Converting video..\n")
    # Start converting the video
    while cap.isOpened():
        # Extract the frame
        ret, frame = cap.read()
        if not ret:
            continue
        # Write the results back to output location.
        cv2.imwrite(output_loc + "/%#05d.jpg" % (count+1), frame)
        count = count + 1
        # If there are no more frames left
        if (count > (video_length-1)):
            # Log the time again
            time_end = time.time()
            # Release the feed
            cap.release()
            # Print stats
            print ("Done extracting frames.\n%d frames extracted" % count)
            print ("It took %d seconds for conversion." % (time_end-time_start))
            break

if __name__=="__main__":
    

    dir_path=str(Path.home())+r'/Documents/Drone_Fly/Flug_21_08_22'

    input_loc = dir_path+'/flight_video.MP4'
    output_loc = dir_path+'/frames'
    video_to_frames(input_loc, output_loc)
    