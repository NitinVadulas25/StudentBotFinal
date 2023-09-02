package com.github.mittyrobotics.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.github.mittyrobotics.util.Gyro;
import com.github.mittyrobotics.util.math.Angle;
import com.github.mittyrobotics.util.math.Vector;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystemDhruv extends SubsystemBase {
    WPI_TalonFX[] angleMotors;
    double robotLength, robotWidth;
    WPI_TalonFX[] modules;

    private static SwerveSubsystemDhruv instance;

    public static SwerveSubsystemDhruv getInstance() {
        if (instance == null) {
            instance = new SwerveSubsystemDhruv();
        }
        return instance;
    }

    public void initHardware() {
        modules = new WPI_TalonFX[4];
        angleMotors = new WPI_TalonFX[4];
    }

    // translates (x,y) from controller joystick to vector of robot
    public Vector getRobotLinearVelocity(double xInput, double yInput) {
        Vector linearvel = new Vector(
                SwerveConstants.MAX_LINEAR_SPEED_INCHES_PER_SECOND * Math.abs(xInput) * xInput,
                SwerveConstants.MAX_LINEAR_SPEED_INCHES_PER_SECOND * Math.abs(yInput) * yInput
        );
        return linearvel;
    }

    // calculates actual angular velocity from controller input
    public double getAngularVelocity(double angularVel) {
        angularVel = angularVel * SwerveConstants.MAX_ANGULAR_SPEED;
        return angularVel;
    }

    // calculates each independent module's linear velocity
    public Vector[] getWheelLinearVelocity(double angularVel, double xInput, double yInput) {
        angularVel = getAngularVelocity(angularVel); // gets real angular velocity from controller input
        Vector linearvel = getRobotLinearVelocity(xInput, yInput); // gets robot linear velocity (x,y) from controller input
        linearvel = new Vector(
                new Angle(linearvel.getAngle().getRadians() - Gyro.getInstance().getHeadingRadians(), true
                ), linearvel.getMagnitude()); // gets true angle change needed and converts to polar form
        Vector[] wheelLinearVelocities = new Vector[4];
        for (int i = 0; i < 4; i ++) {
            robotLength = (i == 0 || i == 1) ? -1:1; // if module is 1st or 2nd, multiply robotlength *1. Else, * -1
            robotWidth = (i == 0 || i == 3) ? -1:1; // vice versa
            Vector r = new Vector(robotWidth, robotLength); // r vector
            Vector angularVector = Vector.multiply(angularVel, r); // wr vector
            wheelLinearVelocities[i] = Vector.add(linearvel, angularVector); // gets module vector
        }
        return wheelLinearVelocities; // returns array of module vectors
    }

    public void setWheelVelocity(double angularVel, double xInput, double yInput) {
        Vector[] moduleVector = getWheelLinearVelocity(angularVel, xInput, yInput);
        for (int i = 0; i < 4; i ++) {
            angleMotors[i].set(ControlMode.Position, moduleVector[i].getAngle().getRadians() *
                    SwerveConstants.TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO);
            modules[i].set(ControlMode.Velocity, moduleVector[i].getMagnitude() * SwerveConstants.TICKS_PER_INCH / 10);
        }
    }
}