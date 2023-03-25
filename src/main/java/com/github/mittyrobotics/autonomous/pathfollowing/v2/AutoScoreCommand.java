package com.github.mittyrobotics.autonomous.pathfollowing.v2;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Point;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.autonomous.pathfollowing.math.QuinticHermiteSpline;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoScoreCommand extends SequentialCommandGroup {
    public AutoScoreCommand(int tag, int index, double maxvel, double maxaccel, double maxdecel, double startvel, double endvel) {
        super();

        addRequirements(SwerveSubsystem.getInstance());

        boolean left = tag > 4;

        Pose init = Odometry.getInstance().getState();
        // 0 is left from driver perspective
        Pose target = Odometry.getInstance().getScoringZone(tag)[left ? index : 2 - index];
        Pose score = new Pose(Point.add(target.getPosition(), new Point(left ? 32 : -32, 0)), target.getHeading());

        QuinticHermiteSpline spline = new QuinticHermiteSpline(
                init.getPosition(),
                new Vector(0, 0),
                new Vector(0, 0),
                score.getPosition(),
                new Vector(left ? -150 : 150, 0),
                new Vector(0, 0)
        );

        addCommands(
                new PathFollowingCommand(
                        new SwervePath(spline, 3,
                                maxvel, maxaccel, maxdecel,
                                startvel, endvel, true
                        ),
                        left ? Math.PI : 0, 3, 0.02,
                        0, 0.75, 2.5, 0, 0
                )
        );
    }
}
