import numpy as np
import math

# Method 1
# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta):
    R_x = np.array([[1,         0,                  0                   ],

                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],

                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]

                    ])

    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],

                    [0,                     1,      0                   ],

                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]

                    ])

    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],

                    [math.sin(theta[2]),    math.cos(theta[2]),     0],

                    [0,                     0,                      1]

                    ])

    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R

#####################################################################################################################
# Method 2
