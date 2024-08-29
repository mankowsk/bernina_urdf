from roboticstoolbox import Robot
from .utilities import Motion_Visualization, conversions
try:
    import matplotlib
    matplotlib.rcdefaults()
except:
    pass
from pathlib import Path
basepath = Path(__file__).parent

@conversions
class Tx200_Ceiling(Robot):
    def __init__(
        self,
        vis=None,
        jf_id = None,
        ):
        if "07" in jf_id:
            jf_id = "_16M"
        else:
            jf_id = ""
        links, name, urdf_string, urdf_filepath = self.URDF_read(
            basepath.joinpath(f"tx200_ceiling{jf_id}.urdf"),
            tld = basepath.as_posix(),
            )

        super().__init__(
            links,
            name=name,
            manufacturer="Staeubli",
            urdf_string=urdf_string,
            urdf_filepath=None,
            )

        self.sim = Motion_Visualization(
            vis = vis,
            robot = self,
            camera_views = [
                [[-3.000,0,1.500], [0,0,1.500]],
            ],
            links = self.links
            )

@conversions
class GPS(Robot):
    def __init__(
        self,
        vis=None,
        jf_id = "",
        ):
        if "07" in jf_id:
            jf_id = "_16M"
        else:
            jf_id = ""
        links, name, urdf_string, urdf_filepath = self.URDF_read(
            basepath.joinpath(f"gps_phi_stage_hex.urdf"),
            tld = basepath.as_posix(),
            )

        super().__init__(
            links,
            name=name,
            manufacturer="Staeubli",
            urdf_string=urdf_string,
            urdf_filepath=None,
            )

        self.sim = Motion_Visualization(
            vis = vis,
            robot = self,
            camera_views = [
                [[-3.000,0,1.500], [0,0,1.500]],
            ],
            links = self.links
            )


@conversions
class Tx200_Floor(Robot):
    def __init__(
            self,
            vis=None,
            ):
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
        self.sim = Motion_Visualization(
            vis = vis,
            robot = self,
            )
