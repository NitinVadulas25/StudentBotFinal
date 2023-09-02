package com.github.mittyrobotics.drivetrain.commands;

import com.github.mittyrobotics.drivetrain.SwerveSubsystemDhruv;
import com.github.mittyrobotics.util.OI;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveCommandDhruv extends CommandBase {

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        double xInput = OI.getInstance().getDriverController().getLeftX();
        double yInput = OI.getInstance().getDriverController().getLeftY();
        double omega = OI.getInstance().getDriverController().getRightY();
        SwerveSubsystemDhruv.getInstance().setWheelVelocity(omega, xInput, yInput);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
