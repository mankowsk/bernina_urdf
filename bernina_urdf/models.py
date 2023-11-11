from roboticstoolbox import Robot
from swift import Swift
from .utilities import motion_simulation
from pathlib import Path
basepath = Path(__file__).parent

@motion_simulation
class Tx200_Ceiling(Robot):
    def __init__(
        self, 
        sim=None,
    ):
        self.sim = sim if (sim != None) else Swift()
        links, name, urdf_string, urdf_filepath = self.URDF_read(
            basepath.joinpath("tx200_ceiling.urdf"),
            tld = basepath.as_posix(),
        )

        super().__init__(
            links,
            name=name,
            manufacturer="Staeubli",
            urdf_string=urdf_string,
            urdf_filepath=None,
        )
        

@motion_simulation
class Tx200_Floor(Robot):
    def __init__(self):
        links, name, urdf_string, urdf_filepath = self.URDF_read(
            basepath.joinpath("tx200_floor.urdf"),
            tld = basepath.as_posix(),
        )
                
        super().__init__(
            links,
            name=name,
            manufacturer="Staeubli",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )
      