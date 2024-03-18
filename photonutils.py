from wpimath.geometry import (
    Pose2d,
    Translation2d,
    Rotation2d,
    Transform2d,
    Pose3d,
    Transform3d,
)
from math import tan


class PhotonUtils:

    @classmethod
    def calculateDisanceToTargetMeters(
        cls,
        cameraHeightMeters: float,
        targetHeightMeters: float,
        cameraPitchRadians: float,
        targetPitchRadians: float,
    ) -> float:
        """Algorithm from https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
        Estimates range to a target using the target's elevation. This method can produce more stable results
        than SolvePNP when well tuned, if the full 6d robot pose is not required. Note that this method
        requires the camera to have 0 roll (not be skewed clockwise or CCW relative to the floor), and
        for there to exist a height differential between goal and camera. The larger this differential,
        the more accurate the distance estimate will be.

        Args:
            cameraHeightMeters (float): The physical height of the camera off the floor in meters.
            targetHeightMeters (float): The physical height of the target off the floor in meters.
            cameraPitchRadians (float): The pitch of the camera from the horizontal plane in radians. Positive values up.
            targetPitchRadians (float): The pitch of the target in the camera's lens in radians. Positive values up.

        Returns:
            float: _description_
        """
        return (targetHeightMeters - cameraHeightMeters) / tan(
            cameraPitchRadians + targetPitchRadians
        )

    @classmethod
    def estimateCameraToTargetTranslation(
        cls, targetDistanceMeters: float, yaw: Rotation2d
    ) -> Translation2d:
        """Estimates the Translation2d of the target relative to the camera.

        Args:
            targetDistanceMeters (float): The distance to the target in meters.
            yaw (Rotation2d): The observed yaw of the target.

        Returns:
            Translation2d: The target's camera-relative translation.
        """
        return Translation2d(
            yaw.cos() * targetDistanceMeters, yaw.sin() * targetDistanceMeters
        )

    @classmethod
    def estimateFieldToRobot(
        cls,
        cameraToTarget: Transform2d,
        fieldToTarget: Pose2d,
        cameraToRobot: Transform2d,
    ) -> Pose2d:
        """Estimates the robot's pose relative to the field.

        Args:
            cameraToTarget (Transform2d): The position of the target relative to the camera.
            fieldToTarget (Pose2d): The position of the target in the field.
            cameraToRobot (Transform2d): The position of the robot relative to the camera. If the camera was mounted 3 inches behind the "origin" (usually physical center) of the robot, this would be Transform2d(3 inches, 0 inches, 0 degrees).

        Returns:
            Pose2d: The robot's pose relative to the field.
        """
        return cls.estimateFieldToCamera(cameraToTarget, fieldToTarget).transformBy(
            cameraToRobot
        )

    @classmethod
    def estimateFieldToCamera(
        cameraToTarget: Transform2d, fieldToTarget: Pose2d
    ) -> Pose2d:
        """Estimates the robot's pose relative to the field.

        Args:
            cameraToTarget (Transform2d): The position of the target relative to the camera.
            fieldToTarget (Pose2d): The position of the target in the field.

        Returns:
            Pose2d: The position of the camera in the field.
        """
        return fieldToTarget.transformBy(cameraToTarget.inverse())

    @classmethod
    def estimateCameraToTarget(
        cls,
        cameraToTargetTranslation: Translation2d,
        fieldToTarget: Translation2d,
        gyroAngle: Rotation2d,
    ) -> Transform2d:
        return Transform2d(
            cameraToTargetTranslation, -gyroAngle - fieldToTarget.getRotation()
        )

    @classmethod
    def estimateFieldToRobotAprilTag(
        cls,
        cameraToTarget: Transform3d,
        fieldRelativeTagPose: Pose3d,
        cameraToRobot: Transform3d,
    ) -> Pose3d:
        return fieldRelativeTagPose + cameraToTarget.inverse() + cameraToRobot
