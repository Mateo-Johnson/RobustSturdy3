package frc.robot.auto;


import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivetrain.DriveSubsystem;

import java.util.List;


public class TabletFollow extends Command {
    private final DriveSubsystem driveSubsystem;
    private Command pathFollowingCommand;

    public TabletFollow(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Assuming SerialPortListener.xValue and SerialPortListener.yValue are updated elsewhere
        int x = SerialPortListener.xValue;
        int y = SerialPortListener.yValue;

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(x, y, Rotation2d.fromDegrees(0))
        );

        PathPlannerPath followPath = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI),
            new GoalEndState(0.0, Rotation2d.fromDegrees(0))
        );

        followPath.preventFlipping = true;

        // Create a trajectory from the path
        PathPlannerTrajectory trajectory = followPath.getTrajectory(null, null);

        // Create a command to follow the trajectory


        // Schedule the path following command
        pathFollowingCommand.schedule();
    }

    @Override
    public void execute() {
        // The path following logic is handled by the scheduled command
    }

    @Override
    public boolean isFinished() {
        // Check if the path following command is finished
        return pathFollowingCommand != null && pathFollowingCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (pathFollowingCommand != null) {
            pathFollowingCommand.cancel();
        }
    }
}
