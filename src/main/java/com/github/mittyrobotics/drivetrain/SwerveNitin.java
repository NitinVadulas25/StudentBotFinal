package com.github.mittyrobotics.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.github.mittyrobotics.drivetrain.commands.SwerveDefaultCommand;
import com.github.mittyrobotics.util.math.*;
import com.github.mittyrobotics.util.Pair;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.github.mittyrobotics.util.math.Angle;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.math.Vector;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import static java.lang.Math.*;
import static com.github.mittyrobotics.drivetrain.SwerveConstants.*;

public class SwerveNitin {
    private static SwerveNitin instance;

    public static SwerveNitin getInstance() {
        if (instance == null) instance = new SwerveNitin();
        return instance;
    }
    public final SwerveSubsystem.InverseKinematics inverseKinematics = new SwerveSubsystem.InverseKinematics(); //inverse kinematics object
    private WPI_TalonFX[] drivemotors = new WPI_TalonFX[4]; //drive motors
    private WPI_TalonFX[] anglemotors = new WPI_TalonFX[4]; //angle motors


    public void setAngleMotors(double[] values){
        for (int i = 0; i < 4; i++) {
            anglemotors[i].set(ControlMode.Position, values[i] * TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO);
        }
    }

    public void setDriveMotors(double[] values) {
        for (int i = 0; i < 4; i++) {
            drivemotors[i].set(ControlMode.Velocity, values[i] * TICKS_PER_INCH / 10);
        }
    }

    public static class InverseKinematics {
        private double[] angles;
        private double[] magnitudes;

        private final double length = 10, width = 10;

        private Vector rvector;

        public InverseKinematics() {
            angles = new double[4];
            magnitudes = new double[4];

            rvector = new Vector(length, width);
        }

        public void calculateInputs(Vector linearVel, double angularVel) {
            linearVel = new Vector(new Angle(
                            linearVel.getAngle().getRadians() - Gyro.getInstance().getHeadingRadians(), true),
                    linearVel.getMagnitude()
            );

            for (int i = 0; i < 4; i++) {
                Vector wheelVector = Vector.add(linearVel, Vector.multiply(angularVel, getAngularVector(i))); //get green
                angles[i] = -wheelVector.getAngle().getRadians();
                magnitudes[i] = wheelVector.getMagnitude();
            }
        }
        public Vector getAngularVector(int i) {
            return new Vector(rvector.getY() * (i == 0 || i == 1 ? -1 : 1), rvector.getX() * (i == 0 || i == 3 ? 1 : -1));
        }

        public double[] getAngles() {
            return angles;
        }

        public double[] getMagnitudes() {
            return magnitudes;
        }

    }
    public void calculateInputs(Vector linearVel, double angularVel) { //return angle speeds and wheel speeds
        inverseKinematics.calculateInputs(linearVel, angularVel);
    }

    public void applyCalculatedInputs() {
        setAngleMotors(inverseKinematics.getAngles());
        setDriveMotors(inverseKinematics.getMagnitudes());
    }
}
