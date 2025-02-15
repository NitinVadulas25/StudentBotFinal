package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.autonomous.Odometry;
import com.github.mittyrobotics.autonomous.arm.AutoArmScoreCommand;
import com.github.mittyrobotics.autonomous.arm.AutoScoreCommandGroup;
import com.github.mittyrobotics.autonomous.pathfollowing.math.*;
import com.github.mittyrobotics.autonomous.pathfollowing.v2.PathFollowingCommand;
import com.github.mittyrobotics.autonomous.pathfollowing.v2.SwervePath;
import com.github.mittyrobotics.intake.StateMachine;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PPTwoAuto extends SequentialCommandGroup {
    public PPTwoAuto(boolean low, boolean leftSide) {
        super();

        int tag_id = leftSide ? (low ? 8 : 6) : (low ? 1 : 3);
        int second_index = 1;
        int third_index = leftSide ? (low ? 0 : 2) : (low ? 2 : 0);

        Pose scoring = Odometry.getInstance().getScoringZone(tag_id)[low ? 2 : 0];

        Pose starting = new Pose(Point.add(scoring.getPosition(), new Point(leftSide ? 32 : -32, 0)),
                new Angle(leftSide ? 0 : Math.PI));

        Pose firstCone = new Pose(Point.add(starting.getPosition(), new Point(leftSide ? 198 : -198, low ? 9 : -9)),
                starting.getHeading());

        Pose secondCone = new Pose(Point.add(firstCone.getPosition(), new Point(leftSide ? 17 : -17, low ? 58 : -58)),
                new Angle(leftSide ? (low ? 1 : -1) : (low ? Math.PI - 1 : -(Math.PI - 1))));

        Pose beforeSecondCone = new Pose(Point.add(firstCone.getPosition(), new Point(leftSide ? -60 : 60, low ? -9 : 9)),
                firstCone.getHeading());

        Pose beforeFirstCone = new Pose(Point.add(firstCone.getPosition(), new Point(leftSide ? -100 : 100, low ? -9 : 9)),
                starting.getHeading());

        Pose beforeAutoScore = new Pose(Point.add(starting.getPosition(), new Point(leftSide ? 30 : -30, 0)),
                starting.getHeading());

        Point starting_second = Point.add(
                Odometry.getInstance().getScoringZone(tag_id)[second_index].getPosition(),
                new Point(leftSide ? 32 : -32, 0));

        Pose startingSecondPose = new Pose(starting_second,
                new Vector(starting_second, Point.add(beforeAutoScore.getPosition(), new Point(leftSide ? 30 : -30, 0))).getAngle());

        double scoreHeading = leftSide ? (low ? 2.85 : Math.PI) : (low ? 0 : -0.3);



        addCommands(
                // FIRST CONE
                new InstantCommand(() -> Odometry.getInstance().setCustomCam(
                        Odometry.getInstance().FIELD_LEFT_SIDE ? (low ? 3 : 0) : (low ? 0 : 3) //left vs right BACK cam
                )),

                new InitAutoCommand(new Pose(starting.getPosition(), new Angle(leftSide ? Math.PI : 0))),
                new InstantCommand(() -> StateMachine.getInstance().setIntakeStowing()),

                new AutoArmScoreCommand(StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE),


                new PathFollowingCommand(
                        new SwervePath(
                                new QuinticHermiteSpline(starting, beforeFirstCone),
                                10, 2, 5, 5, 0, 2, true
                        ), leftSide ? 0 : Math.PI, 9, 3,
                        0.2, 0.8, 3.25, 0, 0.01, true
                ),

                // INTAKE
                new InstantCommand(() -> StateMachine.getInstance().setState(StateMachine.PieceState.CUBE)),
                new InstantCommand(() -> OI.getInstance().handleGround()),

                new PathFollowingCommand(
                        new SwervePath(
                                new QuinticHermiteSpline(beforeFirstCone, firstCone),
                                10, 2.5, 5, 2, 2, 0, true
                        ), leftSide ? 0 : Math.PI, 3, 0.05,
                        0, 0.6, 3.75, 0, 0.02, true
                ),


                new InstantCommand(() -> Odometry.getInstance().setCustomCam(
                        Odometry.getInstance().FIELD_LEFT_SIDE ? (low ? 2 : 1) : (low ? 1 : 2) //left vs right FRONT cam
                )),
                new InstantCommand(() -> {
                    OI.getInstance().zeroAll();
                    StateMachine.getInstance().setIntakeStowing();
                    Odometry.getInstance().setScoringCam(true);
                }),

                new PathFollowingCommand(
                        new SwervePath(
                                new QuinticHermiteSpline(
                                        new Pose(firstCone.getPosition(), new Angle(leftSide ? Math.PI : 0)),
                                        new Pose(beforeAutoScore.getPosition(), new Angle(leftSide ? Math.PI : 0))),
                                10, 4, 5, 2, 0, 1, true
                        ), scoreHeading, 6, 1,
                        0.1, 0.6, 3, 0, 0.02, true
                ),
//
//
                new AutoScoreCommandGroup(tag_id, second_index, StateMachine.RobotState.HIGH, StateMachine.PieceState.CUBE,
                        2, 5, 1.5, 1, 0),


                new PathFollowingCommand(
                        new SwervePath(
                                new QuinticHermiteSpline(
                                        startingSecondPose,
                                        beforeSecondCone),
                                10, 4, 5, 5, 0, 2, true
                        ), secondCone.getHeading().getRadians(), 6, 3,
                        0.1, 0.6, 2, 0, 0.02, true
                )

                , new InstantCommand(() -> StateMachine.getInstance().setState(StateMachine.PieceState.CONE))
               , new InstantCommand(() -> OI.getInstance().handleGround())

                , new PathFollowingCommand(
                        new SwervePath(
                                new QuinticHermiteSpline(beforeSecondCone, secondCone),
                                10, 3.5, 5, 2, 2, 0, true
                        ), secondCone.getHeading().getRadians(), 6, 3,
                        0, 1, 1.5, 0, 0.01, false
                )

                ,new InstantCommand(() -> {
                    OI.getInstance().zeroAll();
                    StateMachine.getInstance().setIntakeStowing();
                    Odometry.getInstance().setScoringCam(true);
                })

//                ,new PathFollowingCommand(
//                        new SwervePath(
//                                new QuinticHermiteSpline(
//                                        new Pose(secondCone.getPosition(), new Angle(
//                                                SwerveSubsystem.standardize(secondCone.getHeading().getRadians() + Math.PI))),
//                                        new Pose(beforeSecondCone.getPosition(), new Angle(
//                                                SwerveSubsystem.standardize(beforeSecondCone.getHeading().getRadians() + Math.PI)))),
//                                10, 3, 5, 2, 0, 2.5, true
//                        ), scoreHeading, 6, 3,
//                        0.1, 0.6, 3, 0, 0.01, true
//                )



//
//                ,new PathFollowingCommand(
//                        new SwervePath(
//                                new QuinticHermiteSpline(
//                                        new Pose(beforeSecondCone.getPosition(), new Angle(leftSide ? Math.PI : 0)),
//                                        new Pose(beforeAutoScore.getPosition(), new Angle(leftSide ? Math.PI : 0))),
//                                10, 4, 5, 2, 2.5, 1, true
//                        ), scoreHeading, 6, 3,
//                        0.1, 0.6, 3, 0, 0.01, true
//                ),
//
//                new AutoScoreCommandGroup(tag_id, third_index, StateMachine.RobotState.HIGH, StateMachine.PieceState.CONE,
//                        2, 5, 1.5, 1, 0)
        );
    }
}
