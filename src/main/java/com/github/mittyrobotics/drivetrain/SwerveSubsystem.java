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

    public void calculateInputs(Vector linearVel, double angularVel){
        inverseKinematics.calculateInputs(linearVel, angularVel);

    }
    public double getEncoderPosition(int i){
        return anglemotors[i].getSelectedSensorPosition();
    }
    public double getStandardizedModuleAngle(int i){
        return Angle.standardize(getEncoderPosition(i));
    }
    public void setAmotors(double[] values){
        for (int i = 0; i < 4; i++){
            double currentangle = Angle.standardize(getStandardizedModuleAngle(i));
            values[i] = Angle.standardize(values[i]);
            boolean clock = (values[i] - currentangle < PI && values[i] - currentangle > 0 || values[i] - currentangle < -PI);
            double distance = Angle.getRealAngleDistance(currentangle, values[i], clock);
            boolean flip = distance > PI /2;
            flipped[i] = flip;

            values[i] = getEncoderPosition(i) + (clock ? 1 : -1) * distance;

            if (flip) {
                values[i] += (clock ? -1 : 1) * PI;
            }

            anglemotors[i].set(ControlMode.Position, values[i] * TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO); //change constants
        }
    }
    public void setDmotors(double[] values){
        for (int i = 0; i < 4; i++){
            drivemotors[i].set(ControlMode.Velocity, (flipped[i] ? -1 : 1) * values[i] * TICKS_PER_RADIAN_FALCON_WITH_GEAR_RATIO);
        }
    }




    public void initHardware() {


    }


    static class InverseKinematics {
        public void calculateInputs(Vector linearVel, double angularVel){

        }
    }

    public double[] wheelSpeedAngle(double omega, double vx, double vy, int wheelNum) {
        double wheel1Speed;
        double angle1;
        Vector angularVelocity;
        if (wheelNum == 1) {
            angularVelocity = new Vector(-omega*cos(PI/2-atan(robotLengthMeters/robotWidthMeters)),
                    omega*sin(PI/2-atan(robotLengthMeters/robotWidthMeters)));
        }
        else if (wheelNum == 2) {
            angularVelocity = new Vector(-omega*cos(PI/2-atan(robotLengthMeters/robotWidthMeters)),
                    -omega*sin(PI/2-atan(robotLengthMeters/robotWidthMeters)));
        }
        else if (wheelNum == 3) {
            angularVelocity = new Vector(omega*cos(PI/2-atan(robotLengthMeters/robotWidthMeters)),
                    -omega*sin(PI/2-atan(robotLengthMeters/robotWidthMeters)));
        }
        else {
            angularVelocity = new Vector(omega*cos(PI/2-atan(robotLengthMeters/robotWidthMeters)),
                    omega*sin(PI/2-atan(robotLengthMeters/robotWidthMeters)));
        }
        Vector v = new Vector(vx, vy);
        Vector v1 = Vector.add(angularVelocity, v);
        wheel1Speed = v1.getMagnitude();
        angle1 = atan2(v1.getY(), v1.getX());
        return new double[]{wheel1Speed, angle1};
    }

    public void rotateWheel(double angle) {

    }

}