from commands2.subsystem import Subsystem
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonTrackedTarget import PhotonTrackedTarget
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
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

    def best_target(self) -> PhotonTrackedTarget:
        # gets the current best target
        targets = self.camera.getLatestResult().getTargets()
        return targets[0].getBestCameraToTarget

        # self.drive_pid = PhotonPIDController(0.1, 0, 0)
        # self.forward_pid = PhotonPIDController(0.1, 0, 0)
