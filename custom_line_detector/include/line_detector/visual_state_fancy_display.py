import cv2

from duckietown_msgs.msg import Segment, SegmentList
import numpy as np

import duckietown_utils as dtu
AA = cv2.LINE_AA  # @UndefinedVariable

@dtu.contract(bgr='array', segment_list=SegmentList, width='int,>=1')
def vs_fancy_display(bgr, segment_list, width=2):
    """
         Writes on a bgr image.
         
         Returns a new image.
    """
    
    ground = np.copy(bgr)
    
    ground = ground / 8 + 80
     
    draw_segment_list_on_image(ground, segment_list, width)
    return ground
    
def draw_segment_list_on_image(bgr, segment_list, width):
    colors2bgr = {Segment.WHITE: dtu.ColorConstants.BGR_WHITE,
              Segment.RED: dtu.ColorConstants.BGR_RED,
              Segment.YELLOW: dtu.ColorConstants.BGR_YELLOW}
    shape = bgr.shape[:2]
    
    for segment in segment_list.segments:
        
        p1 = segment.pixels_normalized[0]
        p2 = segment.pixels_normalized[1]
        
        P1 = normalized_to_image(p1, shape)
        P2 = normalized_to_image(p2, shape) 
        
        paint = colors2bgr[segment.color]
        
        cv2.line(bgr, P1, P2, paint, width, lineType=AA)
        

def normalized_to_image(p,shape):
    x, y = p.x, p.y
    H, W = shape
    X = x * W
    Y = y * H
    return int(X), int(Y)

