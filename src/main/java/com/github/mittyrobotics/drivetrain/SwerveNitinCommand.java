package com.github.mittyrobotics.drivetrain;

import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.OI;
import com.github.mittyrobotics.util.math.Vector;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveNitinCommand extends CommandBase {
    private double throttleX, throttleY, throttleAngular, joystickDeadzone;
    public SwerveNitinCommand() {
        setName("Swerve Default Command");
//        addRequirements(SwerveNitin.getInstance());
    }

    @Override
    public void initialize() {throttleX = 0; throttleY = 0; throttleAngular = 0;}

    @Override
    public void execute() {
        throttleX = -OI.getInstance().getDriverController().getLeftX();
        throttleY = -OI.getInstance().getDriverController().getLeftY();
        throttleAngular = -OI.getInstance().getDriverController().getRightX();

        SwerveNitin.getInstance().calculateInputs(
                new Vector(SwerveConstants.MAX_LINEAR_SPEED_INCHES_PER_SECOND * Math.abs(throttleX) * throttleX, SwerveConstants.MAX_LINEAR_SPEED_INCHES_PER_SECOND * Math.abs(throttleY) * throttleY), SwerveConstants.MAX_ANGULAR_SPEED * throttleAngular
        );
        SwerveNitin.getInstance().applyCalculatedInputs();
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
