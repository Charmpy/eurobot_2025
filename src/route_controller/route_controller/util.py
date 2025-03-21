class Gripper:
    def __init__(self, name):
        self.name = name
        self.status = '0'
    
    def open(self):
        self.status = "0"
    
    def close(self):
        self.status = "1"
    
    def get(self):
        return self.status

    def __str__(self):
        s = f"Gripper {self.name} position: {self.status}\n"
        return s


class Servo:
    def __init__(self, default, name):
        self.name = name
        self.pose = default
        self.zero = default
    
    def set_pose(self, pose):
        self.pose = pose
    
    def set_default(self):
        self.pose = self.zero
    
    def get(self):
        return str(self.pose)
    
    def __str__(self):
        s = f"Servo {self.name} position: {self.pose}\n"
        return s
    

class ServoControl:
    UD_CONST = 100
    FB_CONST = 150
    def __init__(self, up_grip, down_grip, ud_ser, fb_ser, publisher):
        self.up_grip = up_grip
        self.down_grip = down_grip
        self.ud_ser = ud_ser
        self.fb_ser = fb_ser
        self.publisher = publisher
    
    def set2zero(self):
        for i in self.up_grip:
            i.open()
        for i in self.down_grip:
            i.open()  
        
        self.ud_ser.set_default()
        self.fb_ser.set_default()
        self.publish()
    
    def open_up(self):
        for i in self.up_grip:
            i.open()
        self.publish()        

    def close_up(self):
        for i in self.up_grip:
            i.close()   
        self.publish()

    def open_down(self):
        for i in self.down_grip:
            i.open()
        self.publish()

    def close_down(self):
        for i in self.down_grip:
            i.close()   
        self.publish()
    
    def move_up_grippers(self):
        self.ud_ser.set_pose(self.UD_CONST) 
        self.publish()

    def move_down_grippers(self):
        self.ud_ser.set_default(self.UD_CONST) 
        self.publish()

    def move_forward_grippers(self):
        self.fb_ser.set_pose(self.UD_CONST) 
        self.publish()
        
    def move_backward_grippers(self):
        self.fb_ser.set_default(self.FB_CONST) 
        self.publish()        

    def get_command(self):
        command = ' '.join([i.get() for i in [*self.up_grip, *self.down_grip, self.ud_ser, self.fb_ser]])
    
        return command

    def publish(self):
        print(*[str(i) for i in [*self.up_grip, *self.down_grip, self.ud_ser, self.fb_ser]])

        self.publisher.publish(self.get_command())
    




