/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.factory;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.WheelModule;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.subsystems.swerve.SwerveMap;
import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.components.dashboard.AngleGetterDashboardDecorator;
import frc.robot.components.dashboard.AngleSetterDashboardDecorator;
import frc.robot.components.dashboard.GyroDashboardDecorator;
import frc.robot.components.dashboard.SpeedSetterDashboardDecorator;
import frc.robot.components.hardware.AHRSAngleGetterComponent;

/**
 * A factory that creates {@link Swerve} drives with default configurations. "Default" refers to the
 * configuration used at competition.
 */
public class DefaultSwerveFactory implements ISwerveFactory {
    private static final int TIMEOUT = 0;

    /**
     * Creates a {@link Swerve} drive with default configurations.
     * 
     * @return a configured swerve drive
     */
    public Swerve makeSwerve() {
        var bl = MakeWheelModule(SwerveMap.BL_ANGLE_PORT, SwerveMap.BL_SPEED_PORT, false, false,
                "BL Wheel Module");
        var br = MakeWheelModule(SwerveMap.BR_ANGLE_PORT, SwerveMap.BR_SPEED_PORT, false, true,
                "BR Wheel Module");
        var fl = MakeWheelModule(SwerveMap.FL_ANGLE_PORT, SwerveMap.FL_SPEED_PORT, false, false,
                "FL Wheel Module");
        var fr = MakeWheelModule(SwerveMap.FR_ANGLE_PORT, SwerveMap.FR_SPEED_PORT, false, true,
                "FR Wheel Module");

        return new Swerve(1, 1, fl, fr, bl, br, new GyroDashboardDecorator("Gyro", "Swerve",
                new AHRSAngleGetterComponent(Port.kMXP)));
    }

    /**
     * A helper method for creating {@link WheelModule}s with default configurations.
     * 
     * @param anglePort     the port number of the motor controlling the angle of the wheel module
     * @param speedPort     the port number of the motor controlling the speed of the wheel module
     * @param angleInverted whether to invert the direction of the angle motor or not
     * @param speedInverted whether to invert the direction of the speed motor or not
     * @param subsystem     the name of the wheel module as it shows up in the dashboard
     * @return a properly configured wheel module
     */
    private static WheelModule MakeWheelModule(int anglePort, int speedPort, boolean angleInverted,
            boolean speedInverted, String subsystem) {

        var angleMotor = new TalonSRXComponent(anglePort);
        var speedMotor = new TalonSRXComponent(speedPort);

        speedMotor.configPeakOutputForward(1);
        speedMotor.configPeakOutputReverse(-1);
        speedMotor.setInverted(speedInverted);

        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
                TIMEOUT);

        angleMotor.setSensorPhase(false);
        angleMotor.configAllowableClosedloopError(0, 0, TIMEOUT);
        angleMotor.config_kP(0, SwerveMap.ANGLE_P, TIMEOUT); // 0.8
        angleMotor.config_kI(0, SwerveMap.ANGLE_I, TIMEOUT);
        angleMotor.config_kD(0, SwerveMap.ANGLE_D, TIMEOUT); // 80
        angleMotor.config_kF(0, SwerveMap.ANGLE_F, TIMEOUT);
        angleMotor.config_IntegralZone(0, 50, TIMEOUT);
        angleMotor.configMotionCruiseVelocity(SwerveMap.ANGLE_V, TIMEOUT);
        angleMotor.configMotionAcceleration(SwerveMap.ANGLE_A, TIMEOUT); // 1800
        angleMotor.setInverted(angleInverted);

        var sentAngle = new AngleSetterDashboardDecorator("Sent Angle", subsystem, angleMotor);
        var sentSpeed = new SpeedSetterDashboardDecorator("Sent Speed", subsystem, speedMotor);
        var actualAngle = new AngleGetterDashboardDecorator("Actual Angle", subsystem, angleMotor);

        return new WheelModule(sentAngle, sentSpeed, actualAngle);
    }
}
