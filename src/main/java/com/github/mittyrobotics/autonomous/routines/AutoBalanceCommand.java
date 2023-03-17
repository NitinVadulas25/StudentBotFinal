package com.github.mittyrobotics.autonomous.routines;

import com.github.mittyrobotics.autonomous.pathfollowing.math.Vector;
import com.github.mittyrobotics.drivetrain.SwerveConstants;
import com.github.mittyrobotics.drivetrain.SwerveSubsystem;
import com.github.mittyrobotics.util.Gyro;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalanceCommand extends CommandBase {
    private double maxVelStart, maxVelBalance;
    private boolean onScale = false;
    private boolean docking = false;

    private final double STOP_ANGLE = 10;
    private final double ON_ANGLE = 14;
    private final double START_ANGLE = 6;
    private final boolean pos;

    public AutoBalanceCommand(double maxVelStart, double maxVelBalance, boolean pos) {
        this.maxVelBalance = maxVelBalance;
        this.maxVelStart = maxVelStart;
        this.pos = pos;

        addRequirements(SwerveSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        onScale = false;
        docking = false;
    }

    @Override
    public void execute() {
        double pitch = Math.abs(Gyro.getInstance().getPitch());
        double speed;


        if (!onScale) {
            if (pitch > START_ANGLE) onScale = true;
            speed = maxVelStart;
        } else {
            if(!docking) {
                if (pitch > ON_ANGLE) docking = true;
            }
            speed = maxVelBalance;
        }
        SwerveSubsystem.getInstance().setSwerveInvKinematics(new Vector(
                pos ? speed : -speed, 0), 0);

        SwerveSubsystem.getInstance().setSwerveVelocity(SwerveSubsystem.getInstance().desiredVelocities());
        SwerveSubsystem.getInstance().setSwerveAngle(SwerveSubsystem.getInstance().desiredAngles());
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().setZero();
        SwerveSubsystem.getInstance().fortyFiveAngle();
    }

    @Override
    public boolean isFinished() {
        return docking && Gyro.getInstance().getPitch() < STOP_ANGLE;
    }
}
