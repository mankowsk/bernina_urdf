# better: put everything in instance of simulation class or visualization class
class Joint():
    def __init__(
        self,
        name = None,
        parent = None,
        number = None,
    ):
        self.name = name
        self.number = number
        
    def mv(self, value, duration=10):
        q1 = rob.q
        q2 = q1.copy()
        q2[number] = value
        self.parent.move_joints(q1,q2,duration)

    def mvr(self, value, duration=10):
        q1 = rob.q
        q2 = q1.copy()
        q2[number] += value
        self.parent.move_joints(q1,q2,duration)

def motion_simulation(Obj):
    # add motion simulation methods

    def show(self):
        self.sim.launch(realtime=True)
        self.add_self_to_sim()

        
    def add_self_to_sim(self):
        if not self in self.sim.swift_objects:
            self._swift_id = self.sim.add(self)

    def remove_self_from_sim(self):
        pass

    def move_joints(self, j1, j2, duration=10):
        self.add_self_to_sim()
        """
        j1: starting joint positions in rad
        j2: end joint positions in rad
        Visualizes motion of the robot in the Swift instance sim from j1 to j2 in <duration> seconds. 
        The device has to be added to sim before the motion is initialized
        """
        j1 = j1
        j2 = j2
        self.q = j1 #Start position
        self.sim.step(0) #Initialize visualization
        self.qd = (j2-j1)/duration #Set speed
        for n in range(int(1/0.05*duration)):
            self.sim.step(0.05)
        
    Obj.show = show    
    Obj.move_joints = move_joints
    Obj.add_self_to_sim = add_self_to_sim
    #for link in list(Obj.links):
    #    if link.hasdynamics:
    #        Obj.__dict__[link.name] = Joint(link.name, self, link.number)
    return Obj