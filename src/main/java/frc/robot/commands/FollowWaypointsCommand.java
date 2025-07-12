package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import java.util.List;

public class FollowWaypointsCommand extends Command {
    private final DriveSubsystem drive;
    private final List<Pose2d> waypoints;
    private final double positionTolerance;
    private final double angleToleranceRad;

    private int currentWaypointIndex = 0;

    private double elapsedTime = 0;
    private double timeSinceLastUpdate = 0;
    private final double profileUpdateInterval = 0.2; // update every 0.1s

    private final TrapezoidProfile.Constraints velocityConstraints =
        new Constraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    private final TrapezoidProfile.Constraints rotationConstraints =
        new Constraints(AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);

    private TrapezoidProfile.State nextStateX = new TrapezoidProfile.State();
    private TrapezoidProfile.State nextStateY = new TrapezoidProfile.State();
    private TrapezoidProfile.State nextStateRot = new TrapezoidProfile.State();

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
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.zeroHeading();
        drive.resetOdometry(new Pose2d(0,0,new Rotation2d(0)));
        currentWaypointIndex = 0;
        elapsedTime = 0;
        timeSinceLastUpdate = profileUpdateInterval; // force profile gen at start
    }

    @Override
    public void execute() {

        elapsedTime += 0.02;
        timeSinceLastUpdate += 0.02;

        Pose2d currentPose = drive.getPose();
        Pose2d goalPose = waypoints.get(currentWaypointIndex);

        // Only regenerate trapezoid profiles every 0.1s
        if (timeSinceLastUpdate >= profileUpdateInterval) {
            timeSinceLastUpdate = 0;
            System.out.printf("Waypoint %d | xVel: %.2f, yVel: %.2f, rotVel: %.2f\n",
            currentWaypointIndex, nextStateX.velocity, nextStateY.velocity, nextStateRot.velocity);

            // X profile
            TrapezoidProfile profileX = new TrapezoidProfile(velocityConstraints);
            TrapezoidProfile.State currentStateX = new TrapezoidProfile.State(currentPose.getX(), 0);
            TrapezoidProfile.State goalStateX = new TrapezoidProfile.State(goalPose.getX(), 0);
            nextStateX = profileX.calculate(profileUpdateInterval, currentStateX, goalStateX);

            // Y profile
            TrapezoidProfile profileY = new TrapezoidProfile(velocityConstraints);
            TrapezoidProfile.State currentStateY = new TrapezoidProfile.State(currentPose.getY(), 0);
            TrapezoidProfile.State goalStateY = new TrapezoidProfile.State(goalPose.getY(), 0);
            nextStateY = profileY.calculate(profileUpdateInterval, currentStateY, goalStateY);

            // Rotation profile
            TrapezoidProfile profileRot = new TrapezoidProfile(rotationConstraints);
            TrapezoidProfile.State currentStateRot = new TrapezoidProfile.State(currentPose.getRotation().getRadians(), 0);
            TrapezoidProfile.State goalStateRot = new TrapezoidProfile.State(goalPose.getRotation().getRadians(), 0);
            nextStateRot = profileRot.calculate(profileUpdateInterval, currentStateRot, goalStateRot);
        }

        // Use velocities from profile outputs
        double xVel = MathUtil.applyDeadband(nextStateX.velocity, 0.05);
        double yVel = MathUtil.applyDeadband(nextStateY.velocity, 0.05);
        double rotVel = MathUtil.applyDeadband(nextStateRot.velocity, 0.05);

        drive.driveSpeed(xVel, yVel, rotVel, true, false);

        // Check if the robot is within tolerance of the current waypoint
        double distanceToGoal = currentPose.getTranslation().getDistance(goalPose.getTranslation());
        double thetaError = goalPose.getRotation().minus(currentPose.getRotation()).getRadians();

        if (distanceToGoal < positionTolerance && Math.abs(thetaError) < angleToleranceRad) {
            currentWaypointIndex++;
            if (currentWaypointIndex < waypoints.size()) {
                timeSinceLastUpdate = profileUpdateInterval; // trigger profile regen
            }
        }
    }

    @Override
    public boolean isFinished() {
        return currentWaypointIndex >= waypoints.size();
    }

    @Override
    public void end(boolean interrupted) {
        drive.driveSpeed(0, 0, 0, true, false);
    }
}
