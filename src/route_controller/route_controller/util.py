# В этом файле содержатся вспомогательные файлы для управления роботом

from std_msgs.msg import String, Empty

# Это класс для управления захватами из главной ветки программы
# Таких будет два, один для симуляции, другой для реала

class Gripper:
    # gripper*_prefix - префикс топика захватов(часть до attach/detach)
    # s - side, c - center
    def __init__(self, node, grippers_prefix = ['detachable_jointsl', 'detachable_jointcl', 
                                                'detachable_jointcr', 'detachable_jointsr',
                                                'detachable_jointbd'],
                  model = ''):
        self.node = node

        self.grippers_full_prefix=[]
        self.publishers_attach = []
        self.publishers_detach = []
        self.subscriptions_status = []

        for i in grippers_prefix:
            self.grippers_full_prefix.append(f'{model}/{i}')

            self.publishers_attach.append(self.node.create_publisher(String, self.grippers_full_prefix[-1]+'/attach', 10))
            self.publishers_detach.append(self.node.create_publisher(Empty, self.grippers_full_prefix[-1]+'/detach', 10))
            self.subscriptions_status.append(self.node.create_subscription(String, self.grippers_full_prefix[-1]+'/output',self.on_status_msg,10))

        self.status = []

        self.msg = String()

    
    def grip_sides(self, namel='', namer=''):
        if namel != '':
            self.msg.data = namel
            self.publishers_attach[0].publish(self.msg)
        if namer != '':
            self.msg.data = namer
            self.publishers_attach[3].publish(self.msg)
    
    def grip_center(self, namel='', namer=''):
        if namel != '':
            self.msg.data = namel
            self.publishers_attach[1].publish(self.msg)
        if namer != '':
            self.msg.data = namer
            self.publishers_attach[2].publish(self.msg)
    
    def release_sides(self):
        self.publishers_detach[0].publish(Empty())
        self.publishers_detach[3].publish(Empty())
    
    def release_center(self):
        self.publishers_detach[1].publish(Empty())
        self.publishers_detach[2].publish(Empty())

    def grip_board(self, name=''):
        if name != '':
            self.msg.data = name
            self.publishers_attach[4].publish(self.msg)
    
    def release_board(self):
        self.publishers_detach[4].publish(Empty())

    def on_status_msg(self, data):
        self.status = data.data

    def get(self):
        return self.status

    def __str__(self):
        s = f"Gripper {self.name} position: {self.status}\n"
        return s


# Это класс для управления конкретной сервой

class Servo:
    def __init__(self, default, name):
        self.name = name
        self.pose = default
        self.zero = default
    
    def set_pose(self, pose):
        self.pose = pose
        # Вот тут будет паблишер, двигающий актуатор
    
    def set_default(self):
        self.pose = self.zero
        # Вот тут будет паблишер, двигающий актуатор в стандартное положение
    
    def get(self):
        return str(self.pose)
    
    def __str__(self):
        s = f"Servo {self.name} position: {self.pose}\n"
        return s
    



# Этот класс служит для написания типа "макросов" для движений группами захватов 
# и актуаторов одна функция клааса должна выполнять полноценное действите по захвату
# по сути, это просто более высокий уровень организации, сделан для практичности   

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
    




