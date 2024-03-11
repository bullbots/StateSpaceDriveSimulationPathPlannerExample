package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;


public class RamseteAutoCommand extends SequentialCommandGroup {

    public RamseteAutoCommand(DriveSubsystem robotDrive) {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(
                                Constants.DriveConstants.ksVolts,
                                Constants.DriveConstants.kvVoltSecondsPerMeter,
                                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                        Constants.DriveConstants.kDriveKinematics,
                        7);

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                        // Start at (1, 2) facing the +X direction
                        new Pose2d(1, 2, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(2, 3), new Translation2d(3, 1)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(4, 2, new Rotation2d(0)),
                        // Pass config
                        config);

        RamseteCommand ramseteCommand =
                new RamseteCommand(
                        exampleTrajectory,
                        robotDrive::getPose,
                        new RamseteController(
                                Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                        new SimpleMotorFeedforward(
                                Constants.DriveConstants.ksVolts,
                                Constants.DriveConstants.kvVoltSecondsPerMeter,
                                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                        Constants.DriveConstants.kDriveKinematics,
                        robotDrive::getWheelSpeeds,
                        new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                        new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                        // RamseteCommand passes volts to the callback
                        robotDrive::tankDriveVolts,
                        robotDrive);

        addCommands(
                Commands.runOnce(() -> robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
                ramseteCommand,
                Commands.runOnce(() -> robotDrive.tankDriveVolts(0, 0))
        );
    }
}
