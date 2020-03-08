/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.odometric.factory;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.components.dashboard.GyroDashboardDecorator;
import frc.robot.components.hardware.AHRSAngleGetterComponent;
import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.OdometricWheelModule;

/**
 * @deprecated Gertrude's front left drive encoder is currently broken and this whole thing goes to hell. Do not use.
 */
@Deprecated
public class GertrudeOdometricSwerveFactory {
    public static final int TIMEOUT = 0;
    public static final double ANGLE_P = 1.03858;
    public static final double ANGLE_I = 0.004;
    public static final double ANGLE_D = 8;
    public static final double ANGLE_F = 0.51;
    public static final int ANGLE_V = 4012;
    public static final int ANGLE_A = 4012;
    public static final int BL_ANGLE_PORT = 8;
    public static final int BR_ANGLE_PORT = 2;
    public static final int FL_ANGLE_PORT = 6;
    public static final int FR_ANGLE_PORT = 3;
    public static final int BL_SPEED_PORT = 9;
    public static final int BR_SPEED_PORT = 1;
    public static final int FL_SPEED_PORT = 7;
    public static final int FR_SPEED_PORT = 4;
    public static final double WHEEL_DIAMETER = 0.0635;
    public static final double MAX_SURFACE_SPEED = 1.0;
    public static final double SWERVE_WHEELBASE_WIDTH = 0.6096;
    public static final double SWERVE_WHEELBASE_LENGTH = 0.6096;

    

    public OdometricSwerve makeSwerve() {
        return new OdometricSwerve(
            new GyroDashboardDecorator(
                "Gyro", 
                "Swerve", 
                new AHRSAngleGetterComponent(Port.kMXP)
            ), 
            makeWheelModule(
                FL_ANGLE_PORT, 
                FL_SPEED_PORT, 
                new Translation2d(
                    SWERVE_WHEELBASE_LENGTH / 2, 
                    SWERVE_WHEELBASE_WIDTH / 2), 
                false, 
                false, 
                "FL Wheel Module"
            ),
            makeWheelModule(
                FR_ANGLE_PORT, 
                FR_SPEED_PORT, 
                new Translation2d(
                    SWERVE_WHEELBASE_LENGTH/2,
                    -SWERVE_WHEELBASE_WIDTH/2), 
                false, 
                true, 
                "FR Wheel Module"
            ),
            makeWheelModule(
                BL_ANGLE_PORT, 
                BL_SPEED_PORT, 
                new Translation2d(
                    -SWERVE_WHEELBASE_LENGTH/2,
                    SWERVE_WHEELBASE_WIDTH/2), 
                false, 
                false, 
                "BL Wheel Module"
            ),
            makeWheelModule(
                BR_ANGLE_PORT, 
                BR_SPEED_PORT, 
                new Translation2d(
                    -SWERVE_WHEELBASE_LENGTH/2,
                    -SWERVE_WHEELBASE_WIDTH/2), 
                false, 
                true, 
                "BR Wheel Module"
            )
        );
    }

    private OdometricWheelModule makeWheelModule(int anglePort, int speedPort, Translation2d translationFromSwerveCenter, boolean angleInverted,
            boolean speedInverted, String subsystem) {

        var angleMotor = new TalonSRXComponent(anglePort);
        var speedMotor = new TalonSRXComponent(speedPort);

        speedMotor.configPeakOutputForward(1);
        speedMotor.configPeakOutputReverse(-1);
        speedMotor.setInverted(speedInverted);

        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TIMEOUT);

        angleMotor.setSensorPhase(false);
        angleMotor.configAllowableClosedloopError(0, 0, TIMEOUT);
        angleMotor.config_kP(0, ANGLE_P, TIMEOUT); // 0.8
        angleMotor.config_kI(0, ANGLE_I, TIMEOUT);
        angleMotor.config_kD(0, ANGLE_D, TIMEOUT); // 80
        angleMotor.config_kF(0, ANGLE_F, TIMEOUT);
        angleMotor.config_IntegralZone(0, 50, TIMEOUT);
        angleMotor.configMotionCruiseVelocity(ANGLE_V, TIMEOUT);
        angleMotor.configMotionAcceleration(ANGLE_A, TIMEOUT); // 1800
        angleMotor.setInverted(angleInverted);

        return new OdometricWheelModule(angleMotor, speedMotor, translationFromSwerveCenter, MAX_SURFACE_SPEED,WHEEL_DIAMETER,1,1);
    }
}
