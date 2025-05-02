import math
from collections import deque

class CoordHandler:

    def __init__(self):
        self.storages = deque([self.S_point_1(), self.S_point_2(), self.S_point_3(), 
                         self.S_point_4(), self.S_point_5(), self.S_point_6(),
                         self.S_point_5(), self.S_point_6(), self.S_point_7(), 
                         self.S_point_8(), self.S_point_9(), self.S_point_10(),
                         self.S_point_11(), self.S_point_12()])

        self.goals_blue = deque([self.B_b_point_1(), self.B_b_point_2(), self.B_b_point_3(), 
                           self.B_b_point_4(), self.B_b_point_5(), self.B_b_point_6(),
                           self.B_b_point_5(), self.B_b_point_6(), self.B_b_point_7(), 
                           self.B_b_point_8(), self.B_b_point_9(), self.B_b_point_10()])

        self.goals_yellow = deque([self.B_y_point_1(), self.B_y_point_2(), self.B_y_point_3(), 
                             self.B_y_point_4(), self.B_y_point_5(), self.B_y_point_6(),
                             self.B_y_point_5(), self.B_y_point_6(), self.B_y_point_7(), 
                             self.B_y_point_8(), self.B_y_point_9(), self.B_y_point_10()])

    def get_goal(self, goal_type = "storage"):
        try: 
            if goal_type == "storage":
                return self.storages.pop()
            elif goal_type == "blue":
                return self.goals_blue.pop()
            else:
                return self.goals_yellow.pop()

        except IndexError:
            return None


    def add_goal(self, goal, goal_type = "storage"):        
        if goal_type == "storage":
            return self.storages.appendleft(goal)
        elif goal_type == "blue":
            return self.goals_blue.appendleft(goal)
        else:
            return self.goals_yellow.appendleft(goal)

    
    #Точки погрузки

    def S_point_1(self):
        return (0.400, -0.215, math.radians(90)) 
    
    def S_point_2(self):
        return (0.390, -0.775, math.radians(180)) 

    def S_point_3(self):
        return (0.390, -2.225, math.radians(180)) 
    
    def S_point_4(self):
        return (0.400, -2.775, math.radians(-90)) 

    def S_point_5(self):
        return (0.810, -1.100, math.radians(0)) 
    
    
    def S_point_6(self):
        return (0.810, -1.900, math.radians(0)) 

    
    def S_point_7(self):
        return (1.090, -1.100, math.radians(180)) 
    
    
    def S_point_8(self):
        return (1.090, -1.900, math.radians(180)) 

    
    def S_point_9(self):
        return (1.325, -0.215, math.radians(90)) 
    
    
    def S_point_10(self):
        return (1.325, -2.775, math.radians(-90)) 

    
    def S_point_11(self):
        return (1.585, -0.825, math.radians(0)) 
    
    
    def S_point_12(self):
        return (1.585, -2.175, math.radians(0)) 

    #Точки выгрузки b - синие, y - жёлтые

    def B_b_point_1(self):
        return (0.910, -0.200, math.radians(0)) 
    
    def B_b_point_2(self):
        return (0.810, -0.200, math.radians(0)) 

    def B_b_point_3(self):
        return (0.900, -0.190, math.radians(90)) 
    
    def B_b_point_4(self):
        return (0.900, -0.290, math.radians(90)) 

    def B_b_point_5(self):
        return (0.190, -0.200, math.radians(180)) 
    
    def B_b_point_6(self):
        return (0.190, -1.800, math.radians(180)) 

    def B_b_point_7(self):
        return (0.290, -1.800, math.radians(180)) 
    
    
    def B_b_point_8(self):
        return (0.200, -1.810, math.radians(-90)) 

    
    def B_b_point_9(self):
        return (0.200, -1.710, math.radians(-90)) 
    
    
    def B_b_point_10(self):
        return (0.190, -2.200, math.radians(180))


    
    def B_y_point_1(self):
        return (0.910, -2.800, math.radians(0)) 
    
    
    def B_y_point_2(self):
        return (0.810, -2.800, math.radians(-90))


    def B_y_point_3(self):
        return (0.900, -2.810, math.radians(-90)) 
    
    
    def B_y_point_4(self):
        return (0.900, -2.710, math.radians(-90)) 

    
    def B_y_point_5(self):
        return (0.190, -2.800, math.radians(180)) 
    
    
    def B_y_point_6(self):
        return (0.190, -1.200, math.radians(180)) 

    
    def B_y_point_7(self):
        return (0.290, -1.200, math.radians(180)) 
    
    
    def B_y_point_8(self):
        return (0.200, -1.190, math.radians(90)) 

    
    def B_y_point_9(self):
        return (0.200, -1.290, math.radians(90)) 
    
    
    def B_y_point_10(self):
        return (0.190, -0.800, math.radians(180)) 