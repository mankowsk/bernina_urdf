from swift import Swift
import numpy as np

class Joint():
    def __init__(
        self,
        name = None,
        sim = None,
        jindex = None,
    ):
        self.name = name
        self.sim = sim
        self.jindex = jindex
        
    def mv(self, value, duration=10):
        j1 = self.sim.pos.copy()
        j1[self.jindex] = value
        self.sim.move(j1,duration=duration)

    def mvr(self, value, duration=10):
        j1 = self.sim.pos.copy()
        j1[self.jindex] += value
        self.sim.move(j1,duration=duration)

class Motion_Visualization():
    def __init__(
        self,
        vis = None,
        robot = None,
        links = None,
        camera_views = None,
    ):
        self.vis = vis if (vis != None) else Swift()
        self.robot = robot
        self.camera_views = camera_views

        if links is not None:
            for link in links:
                if link.hasdynamics:
                    self.__dict__[link.name] = Joint(link.name, self, link.jindex)
    @property
    def pos(self):
        return self.robot.todegrees(self.robot.q)
    @pos.setter
    def pos(self, value):
        self.robot.q = self.robot.toradians(value)

    @property
    def velocity(self):
        return self.robot.todegrees(self.robot.qd)
    @velocity.setter
    def velocity(self, value):
        self.robot.qd = self.robot.toradians(value)

    def _add_self_to_vis(self):
        if not self.robot in self.vis.swift_objects:
            self._swift_id = self.vis.add(self.robot)
    
    def _ensure_vis_running(self):
        if not hasattr(self.vis, "server"):
            self.show()
    
    def show(self, camera_view=0, *args, **kwargs):
        if hasattr(self.vis, "server"):
            self.vis.reset()
        else:
            self.vis.launch(realtime=True, *args, **kwargs)
        self._add_self_to_vis()
        if self.camera_views is not None:
            self.vis.set_camera_pose(*self.camera_views[camera_view])

    def remove_self_from_vis(self):
        pass

    def move(self, j1, j0 = None, duration=10, init = True):
        """
        j1: end joint positions.
        j0: start joint positions, defaults to the current simulation joint positions.
        duration: time of the simulation in s.
        Visualizes motion of the robot in the Swift instance sim from j0 to j1 in <duration> seconds. 
        """
        self._ensure_vis_running()
        self._add_self_to_vis()

        if j0 is None:
            j0 = self.pos #Start position
        if init:
            self.vis.step(0) #Initialize visualization
        j0, j1 = np.asarray(j0), np.asarray(j1)
        self.velocity = (j1-j0)/duration #Set joint velocities
        for n in range(int(1/0.05*duration)):
            self.vis.step(0.05)

    def move_trajectory(self, js=[], duration=10, init = True, start_at_current_value = False):
        """
        js: joint positions in rad along the path. 
        j0: start joint positions, defaults to the current simulation joint positions.
        duration: time of the simulation in s.
        Visualizes motion of the robot in the Swift instance sim from j0 along js in <duration> seconds. 
        """
        self._ensure_vis_running()
        self._add_self_to_vis()
        js = np.asarray(js)
        if start_at_current_value:
            js = np.vstack([self.pos, js]) #Start position
        
        if init:
            self.vis.step(0) #Initialize visualization
        
        duration_section = duration / (len(js)-1)
        if duration_section < 0.05:
            duration_section = 0.05

        js_rs = np.hstack([js[:-1], js[1:]]).reshape((int(js.shape[0]-1/2),2,js.shape[1]))
        for j0, j1 in js_rs:
            self.move(j1, j0, duration=duration_section, init = False)

    
    