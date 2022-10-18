import numpy as np




if __name__ == "__main__":

    a = np.array([1,1,0])
    a = a.reshape(3,1)
    b = np.identity(3)

    d1 = 1.0

    d2 = 1.2
    
    b = [
        [1,1,0],
        [0,0,1],
        [d1,-d1, d2]
        ]

    
    C = np.array([2, 2, 0])

    C = C.reshape(3,1)

    b_inv = np.linalg.inv(b)


    # print(b_inv@C)

    motors = b_inv@C
    pass



    a = np.array([1,1,0, 0])
    a = a.reshape(4,1)
    b = np.identity(4)

    d1 = 1.0

    d2 = 1.2

    d3 = 1.0
    
    b = [
        [1,1,0, 0],
        [0,0,1, -1],
        [d1,-d1, d2, -d2]
        ]

    b_inv_2 = np.linalg.pinv(b)


    motors = b_inv_2@C
    print(motors)
    # print(a)
    # print(b)

    print(b@a)

    import math

    f_x_list = []
    f_y_list = []
    T_z_list = []
    for f1 in range(-100, 250, 10):
        for f2 in range(-100, 250, 10):
            for f3 in range(-100, 250, 10):
                for f4 in range(-100, 250, 10):

                    theta = math.radians(120)
                    thrust_matrix = np.array(
                            [
                                [1, 1, math.cos(theta), math.cos(theta)],
                                [0, 0, math.sin(theta), -math.sin(theta)],
                                [-d1, d1, d2*math.sin(theta) - d3*math.cos(theta), -d2*math.sin(theta) + d3*math.cos(theta)]
                            ])

                    
                    thrusts = np.array(
                        [[f1],
                        [f2],
                        [f3],
                        [f4]]
                    )

                    f_t = thrust_matrix @ thrusts

                    F_x = f_t[0,0]
                    F_y = f_t[1,0]
                    T_z = f_t[2,0]
                    f_x_list.append(F_x)
                    f_y_list.append(F_y)

                    T_z_list.append(T_z)

                
            print(f1, f2)
    
    print(f"Max X: {max(f_x_list)}")
    print(f"Min X: {min(f_x_list)}")
    print(f"Max Y: {max(f_y_list)}")
    print(f"Min Y: {min(f_y_list)}")
    print(f"Max Z: {max(T_z_list)}")
    print(f"Min Y: {min(T_z_list)}")