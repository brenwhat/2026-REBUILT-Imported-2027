package frc.robot.Subsystems.Vision;

public record VisionMeasurement(
    org.wpilib.math.geometry.Pose2d pose,
    double timestampSeconds
){}