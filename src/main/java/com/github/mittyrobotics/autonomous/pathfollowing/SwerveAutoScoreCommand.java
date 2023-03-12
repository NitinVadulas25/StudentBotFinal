package com.github.mittyrobotics.autonomous.pathfollowing;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Angle;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.drivetrain.commands.JoystickThrottleCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveAutoScoreCommand extends SequentialCommandGroup {
    public SwerveAutoScoreCommand(Pose target) {
        super();
        Pose init = Odometry.getInstance().getState();
        Pose mid = new Pose(Point.add(target.getPosition(), new Point((Odometry.getInstance().FIELD_LEFT_SIDE ? 1 : -1) * 30, 0)), target.getHeading());
        addCommands(
                new SwerveAutoDriveToScoreCommand(2, 0.02, false,
                        new SwervePath(new QuinticHermiteSpline(init, mid),
                                init.getHeading(), target.getHeading(),
                                0, 0, 0, 0, 0, 0.1, 0.4, 1, 0, 0.02, 0.5))
                ,
                new SwerveAutoDriveToScoreCommand(2, 0.02, true,
                        new SwervePath(new QuinticHermiteSpline(mid, target),
                                mid.getHeading(), target.getHeading(),
                                0, 0, 0, 0, 0, 0.1, 0.4, 1, 0, 0.02, 0.5))
        );
    }
}
