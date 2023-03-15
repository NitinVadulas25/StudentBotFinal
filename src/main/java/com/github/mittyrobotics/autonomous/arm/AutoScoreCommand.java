package com.github.mittyrobotics.autonomous.arm;

import com.github.mittyrobotics.autonomous.pathfollowing.SwerveAutoScoreCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.math.Pose;
import com.github.mittyrobotics.intake.StateMachine;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoScoreCommand extends SequentialCommandGroup {

    public AutoScoreCommand(Pose target, StateMachine.RobotState level, StateMachine.PieceState piece) {
        super();
        addCommands(
                new SwerveAutoScoreCommand(target, true),
                new AutoArmScoreCommand(level, piece, true)
        );
    }
}
