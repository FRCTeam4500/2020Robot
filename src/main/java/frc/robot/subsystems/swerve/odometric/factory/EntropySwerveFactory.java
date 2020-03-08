/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.odometric.factory;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.components.hardware.AHRSAngleGetterComponent;
import frc.robot.components.hardware.TalonFXComponent;
import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.OdometricWheelModule;

/**
 * Add your docs here.
 */
public class EntropySwerveFactory {

    private static final double DRIVE_ROTATIONS_PER_MOTOR_ROTATIONS = 1/8.33333333333333333333333333333333333333333333;
    private static final double ANGLE_ROTATIONS_PER_MOTOR_ROTATIONS = 4096.0/5851.0 ;
    private static final double MAX_SURFACE_SPEED_METERS_PER_SECOND = 4.8;
    private static final int BR_DRIVE_PORT = 10; //10
    private static final int BL_DRIVE_PORT = 1; //1
    private static final int FR_DRIVE_PORT = 7; //7
    private static final int FL_DRIVE_PORT = 4; //4
    
    private static final int BR_ANGLE_PORT = 11; //11
    private static final int BL_ANGLE_PORT = 2; //2
    private static final int FR_ANGLE_PORT = 6; //6
    private static final int FL_ANGLE_PORT = 3; //3
    public double WHEEL_DIAMETER_METERS = Units.inchesToMeters(6.0);
    public double DRIVE_LEFTWARD = Units.inchesToMeters(24.5 - 1.75);
    public double DRIVE_FORWARD = Units.inchesToMeters(24.5 - 1.75);
    public OdometricSwerve makeSwerve(){

        var fl = makeWheelModule(FL_ANGLE_PORT, FL_DRIVE_PORT, new Translation2d(DRIVE_FORWARD / 2, DRIVE_LEFTWARD/2), true, true,true);
        var fr = makeWheelModule(FR_ANGLE_PORT, FR_DRIVE_PORT, new Translation2d(DRIVE_FORWARD / 2, -DRIVE_LEFTWARD / 2), true, true,false);
        var bl = makeWheelModule(BL_ANGLE_PORT, BL_DRIVE_PORT, new Translation2d(-DRIVE_FORWARD / 2, DRIVE_LEFTWARD / 2), false, false,true);
        var br = makeWheelModule(BR_ANGLE_PORT, BR_DRIVE_PORT, new Translation2d(-DRIVE_FORWARD / 2, -DRIVE_LEFTWARD / 2), true, true,false);

        return new OdometricSwerve(
            new AHRSAngleGetterComponent(Port.kMXP),
                fl, 
                fr, 
                bl,
                br
                );
            
    }
    public OdometricWheelModule makeWheelModule(int angleId, int driveId,Translation2d translationFromSwerveCenter, boolean invertSensorPhase, boolean invertAngle, boolean invertSpeed){
        var srx = new TalonSRXComponent(angleId);
        srx.setSensorPhase(invertSensorPhase);
        srx.setInverted(invertAngle);
        srx.config_kP(0, 2);
        srx.config_kF(0,0);
        srx.configMotionCruiseVelocity(5500);
        srx.configMotionAcceleration(5500);
        srx.configAllowableClosedloopError(0, 0);

        var falcon = new TalonFXComponent(driveId);
        falcon.config_kP(0, 0.03);
        falcon.config_kI(0, 0);
        falcon.config_kD(0,0);
        falcon.config_kF(0, 0.047);
        falcon.config_IntegralZone(0, 0);
        falcon.setInverted(invertSpeed);
        return new OdometricWheelModule(
            srx, 
            falcon, 
            translationFromSwerveCenter, 
            MAX_SURFACE_SPEED_METERS_PER_SECOND,  
            WHEEL_DIAMETER_METERS,
            ANGLE_ROTATIONS_PER_MOTOR_ROTATIONS,
            DRIVE_ROTATIONS_PER_MOTOR_ROTATIONS);
    }

}
