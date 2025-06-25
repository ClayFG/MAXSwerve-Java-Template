package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.controller.PIDController;

import java.util.List;

public class FollowWaypointsCommand extends Command {
    private final DriveSubsystem drive;
    private final List<Pose2d> waypoints;
    private final double positionTolerance;
    private final double angleToleranceRad;

    private int currentWaypointIndex = 0;

    // PID controllers for x, y, and theta control This is just declaring equivalent of the Holomonic Controller

    //Trapezoidal profile variables
    private final TrapezoidProfile.Constraints velocityConstraints = new Constraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    private final TrapezoidProfile.Constraints rotationConstraints = new Constraints(AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);
    TrapezoidProfile profileX = new TrapezoidProfile(velocityConstraints);
    TrapezoidProfile profileY = new TrapezoidProfile(velocityConstraints);
    TrapezoidProfile profileRot = new TrapezoidProfile(rotationConstraints);

    public FollowWaypointsCommand(
        DriveSubsystem drive,
        List<Pose2d> waypoints,
        double positionToleranceMeters,
        double angleToleranceDegrees
    ) {
        this.drive = drive;
        this.waypoints = waypoints;
        this.positionTolerance = positionToleranceMeters;
        this.angleToleranceRad = Math.toRadians(angleToleranceDegrees);
    }

    @Override
    public void initialize() {
        currentWaypointIndex = 0;
    }

    @Override
    public void execute() {
        // Compute current pose components
        Pose2d currentPose = drive.getPose();
        Translation2d currentTranslation = currentPose.getTranslation();

        // Set goalPose to the current waypoint
        Pose2d goalPose = waypoints.get(currentWaypointIndex);
        Translation2d goalTranslation = goalPose.getTranslation();

        double dx = goalTranslation.getX() - currentTranslation.getX();
        double dy = goalTranslation.getY() - currentTranslation.getY();

        double distanceToGoal = Math.hypot(dx, dy);
        double direction = Math.atan2(dy, dx);

        // Compute errors
        double xError = goalPose.getX() - currentPose.getX();
        double yError = goalPose.getY() - currentPose.getY();
        double thetaError = goalPose.getRotation().minus(currentPose.getRotation()).getRadians();

        // Create trapezoidal profiles for X, Y, and rotation
        TrapezoidProfile.State currentStateX = new TrapezoidProfile.State(currentPose.getX(), 0); // Initial state for X
        TrapezoidProfile.State goalStateX = new TrapezoidProfile.State(goalPose.getX(), 0); // Goal state for X
        var nextstatex = profileX.calculate(0.02, currentStateX, goalStateX); // Calculate next state for X

        TrapezoidProfile.State currentStateY = new TrapezoidProfile.State(currentPose.getY(), 0); // Initial state for Y
        TrapezoidProfile.State goalStateY = new TrapezoidProfile.State(goalPose.getY(), 0); // Goal state for Y
        var nextstatey = profileY.calculate(0.02, currentStateY, goalStateY); // Calculate next state for Y

        TrapezoidProfile.State currentStateRot = new TrapezoidProfile.State(currentPose.getRotation().getRadians(), 0); // Initial state for rotation
        TrapezoidProfile.State goalStateRot = new TrapezoidProfile.State(goalPose.getRotation().getRadians(), 0); // Goal state for rotation
        var nextstaterot = profileRot.calculate(0.02, currentStateRot, goalStateRot); // Calculate next state for rotation

        // Use PID controllers to calculate control outputs based on trapezoidal profile positions

        // Drive the robot using the calculated control outputs
        drive.driveSpeed(nextstatex.velocity, nextstatey.velocity, nextstaterot.velocity, true, false);

        // Check if the robot is within tolerance of the current waypoint
        if (distanceToGoal < positionTolerance && Math.abs(thetaError) < angleToleranceRad) {
            currentWaypointIndex++; // Move to the next waypoint
        }
    }

    @Override
    public boolean isFinished() {
        return currentWaypointIndex >= waypoints.size();
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveSpeed(0, 0, 0,true, false);
    }
}
