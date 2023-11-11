from roboticstoolbox import Robot
from pathlib import Path
basepath = Path(__file__).parent

class Tx200_Ceiling(Robot):
    def __init__(self):
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
      