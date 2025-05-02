import math

class CoordHandler:
    
    #Точки погрузки

    def __init__(self):
        storages = iter([S_point_1(), S_point_2(), S_point_3(), 
                         S_point_4(), S_point_5(), S_point_6(),
                         S_point_5(), S_point_6(), S_point_7(), 
                         S_point_8(), S_point_9(), S_point_10(),
                         S_point_11(), S_point_12()])

        goals = iter([B_b_point_1(), B_b_point_2(), B_b_point_3(), 
                      B_b_point_4(), B_b_point_5(), B_b_point_6(),
                      B_b_point_5(), B_b_point_6(), B_b_point_7(), 
                      B_b_point_8(), B_b_point_9(), B_b_point_10()])

    def get_storage(self):
        return next(self.storages)

    def get_goal(self):
        return next(self.goals)

    @staticmethod
    def S_point_1():
        return (0.687, -1.074, math.radians(0)) 
        # return (0.400, -0.215, math.radians(90)) 
    
    @staticmethod
    def S_point_2():
        return (0.390, -0.775, math.radians(180)) 

    @staticmethod
    def S_point_3():
        return (0.390, -2.225, math.radians(180)) 
    
    @staticmethod
    def S_point_4():
        return (0.400, -2.775, math.radians(-90)) 

    @staticmethod
    def S_point_5():
        return (0.810, -1.100, math.radians(0)) 
    
    @staticmethod
    def S_point_6():
        return (0.810, -1.900, math.radians(0)) 

    @staticmethod
    def S_point_7():
        return (1.090, -1.100, math.radians(180)) 
    
    @staticmethod
    def S_point_8():
        return (1.090, -1.900, math.radians(180)) 

    @staticmethod
    def S_point_9():
        return (1.325, -0.215, math.radians(90)) 
    
    @staticmethod
    def S_point_10():
        return (1.325, -2.775, math.radians(-90)) 

    @staticmethod
    def S_point_11():
        return (1.585, -0.825, math.radians(0)) 
    
    @staticmethod
    def S_point_12():
        return (1.585, -2.175, math.radians(0)) 

    #Точки выгрузки b - синие, y - жёлтые

    @staticmethod
    def B_b_point_1():
        return (0.910, -0.200, math.radians(0)) 
    
    @staticmethod
    def B_b_point_2():
        return (0.810, -0.200, math.radians(0)) 

    def B_b_point_3():
        return (0.900, -0.190, math.radians(90)) 
    @staticmethod
    
    @staticmethod
    def B_b_point_4():
        return (0.900, -0.290, math.radians(90)) 

    @staticmethod
    def B_b_point_5():
        return (0.190, -0.200, math.radians(180)) 
    
    @staticmethod
    def B_b_point_6():
        return (0.190, -1.800, math.radians(180)) 

    @staticmethod
    def B_b_point_7():
        return (0.290, -1.800, math.radians(180)) 
    
    @staticmethod
    def B_b_point_8():
        return (0.200, -1.810, math.radians(-90)) 

    @staticmethod
    def B_b_point_9():
        return (0.200, -1.710, math.radians(-90)) 
    
    @staticmethod
    def B_b_point_10():
        return (0.190, -2.200, math.radians(180))


    @staticmethod
    def B_y_point_1():
        return (0.910, -2.800, math.radians(0)) 
    
    @staticmethod
    def B_y_point_2():
        return (0.810, -2.800, math.radians(0)) 

    def B_y_point_3():
        return (0.900, -2.810, math.radians(-90)) 
    @staticmethod
    
    @staticmethod
    def B_y_point_4():
        return (0.900, -2.710, math.radians(-90)) 

    @staticmethod
    def B_y_point_5():
        return (0.190, -2.800, math.radians(180)) 
    
    @staticmethod
    def B_y_point_6():
        return (0.190, -1.200, math.radians(180)) 

    @staticmethod
    def B_y_point_7():
        return (0.290, -1.200, math.radians(180)) 
    
    @staticmethod
    def B_y_point_8():
        return (0.200, -1.190, math.radians(90)) 

    @staticmethod
    def B_y_point_9():
        return (0.200, -1.290, math.radians(90)) 
    
    @staticmethod
    def B_y_point_10():
        return (0.190, -0.800, math.radians(180)) 