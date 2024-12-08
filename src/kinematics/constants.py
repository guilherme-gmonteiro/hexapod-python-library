LEG_POINT_TYPES_LIST = [
    "bodyContactPoint",
    "coxiaPoint",
    "femurPoint",
    "footTipPoint",
]

POSITION_NAME_TO_ID_MAP = {
    "rightMiddle": 0,
    "rightFront": 1,
    "leftFront": 2,
    "leftMiddle": 3,
    "leftBack": 4,
    "rightBack": 5,
}

POSITION_NAMES_LIST = [
    "rightMiddle",
    "rightFront",
    "leftFront",
    "leftMiddle",
    "leftBack",
    "rightBack",
]

ANGLE_NAMES_LIST = ["alpha", "beta", "gamma"]

MAX_ANGLES = {
    "alpha": 90,
    "beta": 180,
    "gamma": 180,
}

# 
#   hexapodYaxis
#       ^  
#       |  
#       |  
#       *-----> hexapodXaxis  
#      / (cog)  
#     /  
#  hexapodZaxis  
#  
#  Relative x-axis, for each attached linkage  
#  
#  (+135)  x2          x1 (+45)  
#           \   head  /  
#            *---*---*  
#           /    |    \  
#          /     |     \  
# (+180)  /      |      \  
#   x3 --*------cog------*-- x0 (+0)  
#         \      |      /  
#          \     |     /  
#           \    |    /  
#            *---*---*  
#           /         \  
#         x4           x5  
#      (+225)        (+315)
#
POSITION_NAME_TO_AXIS_ANGLE_MAP = {
    "rightMiddle": 0,
    "rightFront": 45,
    "leftFront": 135,
    "leftMiddle": 180,
    "leftBack": 225,
    "rightBack": 315,
}

POSITION_NAME_TO_IS_LEFT_MAP = {
    "rightMiddle": False,
    "rightFront": False,
    "leftFront": True,
    "leftMiddle": True,
    "leftBack": True,
    "rightBack": False,
}

NUMBER_OF_LEGS = 6
