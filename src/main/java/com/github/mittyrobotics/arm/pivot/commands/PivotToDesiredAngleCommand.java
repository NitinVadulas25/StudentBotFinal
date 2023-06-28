package com.github.mittyrobotics.arm.pivot.commands;

import com.github.mittyrobotics.arm.ArmKinematics;
import com.github.mittyrobotics.arm.pivot.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PivotToDesiredAngleCommand extends CommandBase {
    public PivotToDesiredAngleCommand() {
        setName("Pivot To Desired Angle");
        addRequirements(PivotSubsystem.getInstance());
    }

    private double desiredAngle, desiredRPM, ff;

    @Override
    public void initialize() {
        desiredAngle = 0;
    }

    @Override
    public void execute() {
        desiredAngle = ArmKinematics.getDesiredAngle().getRadians();

        // TODO: 6/27/2023 ADD MP STUFF

        // TODO: 6/27/2023 ADD FF STUFF

        PivotSubsystem.getInstance().setRaw(desiredRPM, ff);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
