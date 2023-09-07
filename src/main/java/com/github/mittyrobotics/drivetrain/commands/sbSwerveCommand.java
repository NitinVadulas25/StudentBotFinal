package com.github.mittyrobotics.drivetrain.commands;

import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.sbSwerveSubsystem;
import com.github.mittyrobotics.util.OI;
import com.github.mittyrobotics.util.math.Vector;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class sbSwerveCommand extends CommandBase {

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        double throttlex = OI.getInstance().getDriverController().getLeftX();
        double throttley = OI.getInstance().getDriverController().getLeftY();
        double throttleangular = OI.getInstance().getDriverController().getRightY();
        sbSwerveSubsystem.getInstance().setWheelVelocity(SwerveConstants.MAX_ANGULAR_SPEED * throttleangular,
                new Vector(SwerveConstants.MAX_LINEAR_SPEED_INCHES_PER_SECOND * Math.abs(throttlex) * throttlex, SwerveConstants.MAX_LINEAR_SPEED_INCHES_PER_SECOND * Math.abs(throttley) * throttley));
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
