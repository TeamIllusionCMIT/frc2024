from typing import Optional
from commands2.subsystem import Subsystem
from photonlibpy.photonCamera import PhotonCamera, PhotonPipelineResult
from photonlibpy.photonTrackedTarget import PhotonTrackedTarget
from photonlibpy.photonPoseEstimator import (
    PhotonPoseEstimator,
    PoseStrategy,
)
from wpimath.geometry import Transform3d, Pose3d
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpimath.controller import PIDController
from constants import DrivePID


class Vision(Subsystem):
    __slots__ = ("camera", "pose_estimator")

    def __init__(self, camera_name: str) -> None:
        super().__init__()

        self.camera = PhotonCamera(camera_name)
        self.pose_estimator = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.k2024Crescendo),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.camera,
            Transform3d(Pose3d(), Pose3d()),
        )  # TODO: make this actually work

        # * less computation, the computer gets to chill
        # * also gives streams higher fps
        self.driver_mode = True

    @property
    def driver_mode(self) -> bool:
        return self.camera.getDriverMode()

    @driver_mode.setter
    def driver_mode(self, enable: bool) -> None:
        """set the state of the driver mode.
        driver mode is a state in which the camera won't process images and will instead just stream them directly to the driver station.
        this results in less computation and higher fps.

        args:
            enable (bool): _description_
        """
        self.camera.setDriverMode(enable)

    def best_target(self) -> PhotonTrackedTarget:
        """gets the best target from the camera

        returns:
            PhotonTrackedTarget: the best target
        """
        if self.driver_mode:
            self.driver_mode = False

        # gets the current best target
        targets = self.latest_result().getTargets()
        return targets[0]

        # self.drive_pid = PhotonPIDController(0.1, 0, 0)
        # self.forward_pid = PhotonPIDController(0.1, 0, 0)

    def latest_result(self) -> PhotonPipelineResult:
        """gets the latest result from the camera

        returns:
            PhotonPipelineResult: the latest pipeline result
        """
        if self.driver_mode:
            self.driver_mode = False

        return self.camera.getLatestResult()

    def estimate_pose(self) -> Optional[Pose3d]:
        """estimates the pose of the robot using the camera data

        returns:
            Optional[Pose3d]: the estimated pose of the robot
        """
        if self.driver_mode:
            self.driver_mode = False

        # gets the current best target
        pose_result = self.pose_estimator.update(self.latest_result())
        return pose_result.estimatedPose if pose_result else None

    def calculate_distance(
        self, target: Optional[PhotonTrackedTarget] = None
    ) -> Optional[float]:
        """estimates the distance from the nearest target based on it's size in the frame.

        returns:
            Optional[float]: the distance in meters
        """
        target = target or self.best_target()

        # ! calculated for our 100deg global shutter camera
        # ! will likely be inaccurate for others
        return (-0.2326 * self.best_target().getArea()) + 2.1867

    def align_to_april_tag(self) -> tuple[float]:
        """calculates the pid output required to align the robot with the best apriltag target.

        returns:
            tuple[float]: the forward, side, and rotate motion, respectively
        """        
        target = self.best_target()
        pid_controller = PIDController(DrivePID.K_P, DrivePID.K_I, DrivePID.K_D)
        return (
            pid_controller.calculate(self.calculate_distance(target.getArea()), 0),
            pid_controller.calculate(target.getYaw(), 0),
            pid_controller.calculate(target.getSkew(), 0),
        )
