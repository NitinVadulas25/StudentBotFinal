package com.github.mittyrobotics.arm.pivot;

import com.github.mittyrobotics.arm.pivot.commands.PivotToDesiredAngleCommand;
import com.github.mittyrobotics.arm.televator.commands.MoveToDesiredExtensionCommand;
import com.github.mittyrobotics.util.math.Angle;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.github.mittyrobotics.arm.pivot.PivotConstants.*;

public class PivotSubsystem extends SubsystemBase {
    private static PivotSubsystem instance;

    public static PivotSubsystem getInstance() {
        if (instance == null) instance = new PivotSubsystem();
        return instance;
    }

    private CANSparkMax[] motor;

    public void initHardware() {
        motor = new CANSparkMax[2];

        for (int i = 0; i < 2; i++) {
            motor[i] = new CANSparkMax(SPARK_ID[i], CANSparkMaxLowLevel.MotorType.kBrushless);
            motor[i].restoreFactoryDefaults();
            motor[i].setInverted(INVERTED[i]);
            motor[i].getPIDController().setP(PID[0]);
            motor[i].getPIDController().setI(PID[1]);
            motor[i].getPIDController().setD(PID[2]);
            motor[i].getEncoder().setPositionConversionFactor(RADIANS_PER_REV);
            motor[i].getEncoder().setVelocityConversionFactor(RADIANS_PER_REV / 60);
            motor[i].getEncoder().setPosition(0);
        }

        setDefaultCommand(new PivotToDesiredAngleCommand());
    }

    public void setRaw(double rpm, double ff) {
        motor[0].getPIDController().setReference(
                rpm, CANSparkMax.ControlType.kVelocity, 0, ff, SparkMaxPIDController.ArbFFUnits.kPercentOut);
        motor[1].getPIDController().setReference(
                rpm, CANSparkMax.ControlType.kVelocity, 0, ff, SparkMaxPIDController.ArbFFUnits.kPercentOut);
//        motor[0].getPIDController().setReference(2, CANSparkMax.ControlType.kPosition);
    }

    public Angle getCurrentAngle() {
        return new Angle(motor[0].getEncoder().getPosition(), true);
    }

    public double getCurrentVelocity() {
        return motor[0].getEncoder().getVelocity();
    }
}
