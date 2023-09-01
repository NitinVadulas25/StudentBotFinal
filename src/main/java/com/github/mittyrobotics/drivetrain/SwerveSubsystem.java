package com.github.mittyrobotics.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.github.mittyrobotics.drivetrain.commands.SwerveDefaultCommand;
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

public class SwerveSubsystem extends SubsystemBase {
    private static SwerveSubsystem instance;
    public boolean flipped[] = new boolean[4]; //boolean array flipped to apply to motors

    public static SwerveSubsystem getInstance() {
        if (instance == null) instance = new SwerveSubsystem();
        return instance;
    }

    public final InverseKinematics inverseKinematics = new InverseKinematics();
    double robotLengthMeters;
    double robotWidthMeters;
    private WPI_TalonFX[] drivemotors = new WPI_TalonFX[4];
    private WPI_TalonFX[] anglemotors = new WPI_TalonFX[4];

    public void calculateInputs(Vector linearVel, double angularVel) {
        inverseKinematics.calculateInputs(linearVel, angularVel);

    }

    public void applyCalculatedInputs() {
        setAmotors(inverseKinematics.getAngles());
    }

    public double getEncoderPosition(int i) {
        return anglemotors[i].getSelectedSensorPosition();
    }

    public double getStandardizedModuleAngle(int i) {
        return Angle.standardize(getEncoderPosition(i));
    }

    public void setAmotors(double[] values) {
        for (int i = 0; i < 4; i++) {
            double currentangle = Angle.standardize(getStandardizedModuleAngle(i));
            values[i] = Angle.standardize(values[i]);
            boolean clock = (values[i] - currentangle < PI && values[i] - currentangle > 0 || values[i] - currentangle < -PI);
            double distance = Angle.getRealAngleDistance(currentangle, values[i], clock);
            boolean flip = distance > PI / 2;
            flipped[i] = flip;

            values[i] = getEncoderPosition(i) + (clock ? 1 : -1) * distance;

            if (flip) {
                values[i] += (clock ? -1 : 1) * PI;
            }

            anglemotors[i].set(ControlMode.Position, values[i] * TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO); //change constants
        }
    }

    public void setDmotors(double[] values) {
        for (int i = 0; i < 4; i++) {
            drivemotors[i].set(ControlMode.Velocity, (flipped[i] ? -1 : 1) * values[i] * TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO);
        }
    }


    public void initHardware() {


    }


    static class InverseKinematics {

        private double[] angles;
        private double[] magnitudes;

        private final double length = 25, width = 25;

        private Vector r;

        public InverseKinematics() {
            angles = new double[4];
            magnitudes = new double[4];

            r = new Vector(length, width);
        }

        public void calculateInputs(Vector linearVel, double angularVel) {

        }

        public Vector getAngularVector(int i) {
            return new Vector(r.getY() * (i == 0 || i == 1 ? -1 : 1), r.getX() * (i == 0 || i == 3 ? 1 : -1));
        }

        public double[] getAngles() {
            return angles;
        }

        public double[] getMagnitudes() {
            return magnitudes;
        }
    }

    public double[] wheelAngle(double omega, double vx, double vy, int wheelNum) {
        Vector angularVelocity1, angularVelocity2, angularVelocity3, angularVelocity4;
        Vector v = new Vector(vx, vy);
        double desiredAngle1 = 0;
        double desiredAngle2 = 0;
        double desiredAngle3 = 0;
        double desiredAngle4 = 0;
        for (int i = 0; i < 4; i++) {
            if (i == 0) {
                angularVelocity1 = new Vector(-omega * cos(PI / 2 - atan(robotLengthMeters / robotWidthMeters)),
                        omega * sin(PI / 2 - atan(robotLengthMeters / robotWidthMeters)));
                Vector v1 = Vector.add(angularVelocity1, v);
                double wheel1Speed = v1.getMagnitude();
                desiredAngle1 = atan2(v1.getY(), v1.getX());
            } else if (i == 1) {
                angularVelocity2 = new Vector(-omega * cos(PI / 2 - atan(robotLengthMeters / robotWidthMeters)),
                        omega * sin(PI / 2 - atan(robotLengthMeters / robotWidthMeters)));
                Vector v2 = Vector.add(angularVelocity2, v);
                double wheel2Speed = v2.getMagnitude();
                desiredAngle2 = atan2(v2.getY(), v2.getX());
            } else if (i == 2) {
                angularVelocity3 = new Vector(-omega * cos(PI / 2 - atan(robotLengthMeters / robotWidthMeters)),
                        omega * sin(PI / 2 - atan(robotLengthMeters / robotWidthMeters)));
                Vector v3 = Vector.add(angularVelocity3, v);
                double wheel3Speed = v3.getMagnitude();
                desiredAngle3 = atan2(v3.getY(), v3.getX());
            } else {
                angularVelocity4 = new Vector(-omega * cos(PI / 2 - atan(robotLengthMeters / robotWidthMeters)),
                        omega * sin(PI / 2 - atan(robotLengthMeters / robotWidthMeters)));
                Vector v4 = Vector.add(angularVelocity4, v);
                double wheel4Speed = v4.getMagnitude();
                desiredAngle4 = atan2(v4.getY(), v4.getX());
            }
        }
        return new double[]{desiredAngle1, desiredAngle2, desiredAngle3, desiredAngle4};
    }

    public double[] wheelSpeed(double omega, double vx, double vy, int wheelNum) {
        Vector angularVelocity1, angularVelocity2, angularVelocity3, angularVelocity4;
        Vector v = new Vector(vx, vy);
        double desiredAngle1 = 0;
        double desiredAngle2 = 0;
        double desiredAngle3 = 0;
        double desiredAngle4 = 0;
        double wheel1Speed = 0;
        double wheel2Speed = 0;
        double wheel3Speed = 0;
        double wheel4Speed = 0;
        for (int i = 0; i < 4; i++) {
            if (i == 0) {
                angularVelocity1 = new Vector(-omega * cos(PI / 2 - atan(robotLengthMeters / robotWidthMeters)),
                        omega * sin(PI / 2 - atan(robotLengthMeters / robotWidthMeters)));
                Vector v1 = Vector.add(angularVelocity1, v);
                wheel1Speed = v1.getMagnitude();
                desiredAngle1 = atan2(v1.getY(), v1.getX());
            } else if (i == 1) {
                angularVelocity2 = new Vector(-omega * cos(PI / 2 - atan(robotLengthMeters / robotWidthMeters)),
                        omega * sin(PI / 2 - atan(robotLengthMeters / robotWidthMeters)));
                Vector v2 = Vector.add(angularVelocity2, v);
                wheel2Speed = v2.getMagnitude();
                desiredAngle2 = atan2(v2.getY(), v2.getX());
            } else if (i == 2) {
                angularVelocity3 = new Vector(-omega * cos(PI / 2 - atan(robotLengthMeters / robotWidthMeters)),
                        omega * sin(PI / 2 - atan(robotLengthMeters / robotWidthMeters)));
                Vector v3 = Vector.add(angularVelocity3, v);
                wheel3Speed = v3.getMagnitude();
                desiredAngle3 = atan2(v3.getY(), v3.getX());
            } else {
                angularVelocity4 = new Vector(-omega * cos(PI / 2 - atan(robotLengthMeters / robotWidthMeters)),
                        omega * sin(PI / 2 - atan(robotLengthMeters / robotWidthMeters)));
                Vector v4 = Vector.add(angularVelocity4, v);
                wheel4Speed = v4.getMagnitude();
                desiredAngle4 = atan2(v4.getY(), v4.getX());
            }
        }
        return new double[]{wheel1Speed, wheel2Speed, wheel3Speed, wheel4Speed};
    }
}



