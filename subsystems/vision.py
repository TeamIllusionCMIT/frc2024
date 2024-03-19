from typing import Optional
from commands2.subsystem import Subsystem
from photonlibpy.photonCamera import PhotonCamera, PhotonPipelineResult
from photonlibpy.photonTrackedTarget import PhotonTrackedTarget
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy, EstimatedRobotPose
from wpimath.geometry import Transform3d, Pose3d
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField


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
        self.camera.setDriverMode(enable)

    def best_target(self) -> PhotonTrackedTarget:
        if self.driver_mode:
            self.driver_mode = False

        # gets the current best target
        targets = self.latest_result().getTargets()
        return targets[0]

        # self.drive_pid = PhotonPIDController(0.1, 0, 0)
        # self.forward_pid = PhotonPIDController(0.1, 0, 0)

    def latest_result(self) -> PhotonPipelineResult:
        if self.driver_mode:
            self.driver_mode = False

        return self.camera.getLatestResult()

    def estimate_pose(self) -> Optional[Pose3d]:
        if self.driver_mode:
            self.driver_mode = False

        # gets the current best target
        pose_result = self.pose_estimator.update(self.latest_result())
        return pose_result.estimatedPose if pose_result else None