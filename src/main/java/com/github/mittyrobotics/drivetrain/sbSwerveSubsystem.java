package com.github.mittyrobotics.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.math.Angle;
import com.github.mittyrobotics.util.math.Vector;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class sbSwerveSubsystem extends SubsystemBase {
    WPI_TalonFX[] angleMotors;
    double robotLength, robotWidth;
    WPI_TalonFX[] modules;

    private static sbSwerveSubsystem instance;

    public static sbSwerveSubsystem getInstance() {
        if (instance == null) {
            instance = new sbSwerveSubsystem();
        }
        return instance;
    }

    public void initHardware() {
        modules = new WPI_TalonFX[4];
        angleMotors = new WPI_TalonFX[4];
    }

    // calculates each independent module's linear velocity
    public Vector[] getWheelLinearVelocity(double angularVel, Vector linearvel) {
        final double robotLengths[] = {1, 1, -1, -1};
        final double robotWidths[] = {1, -1, -1, 1};

        linearvel = new Vector(
                new Angle(linearvel.getAngle().getRadians() - Gyro.getInstance().getHeadingRadians(), true
                ), linearvel.getMagnitude()); // gets true angle change needed and converts to polar form
        Vector[] wheelVector = new Vector[4];
        for (int i = 0; i < 4; i ++) {
            double robotLength = robotLengths[i];
            double robotWidth = robotWidths[i];
            Vector r = new Vector(robotWidth, robotLength); // r vector
            wheelVector[i] = Vector.add(linearvel, Vector.multiply(angularVel, r));
        }
        return wheelVector; // returns array of module vectors
    }

    public void setWheelVelocity(double angularVel, Vector linearvel) {
        Vector[] moduleVector = getWheelLinearVelocity(angularVel, linearvel);
        for (int i = 0; i < 4; i ++) {
            angleMotors[i].set(ControlMode.Position, moduleVector[i].getAngle().getRadians() * SwerveConstants.TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO);
            modules[i].set(ControlMode.Velocity, moduleVector[i].getMagnitude() * SwerveConstants.TICKS_PER_INCH / 10);
        }
    }
}