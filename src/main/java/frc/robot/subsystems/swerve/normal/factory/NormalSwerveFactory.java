package frc.robot.subsystems.swerve.normal.factory;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.components.dashboard.GyroDashboardDecorator;
import frc.robot.components.hardware.AHRSAngleGetterComponent;
import frc.robot.components.hardware.TalonFXComponent;
import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.swerve.ISwerveFactory;
import frc.robot.subsystems.swerve.normal.NormalSwerve;
import frc.robot.subsystems.swerve.normal.NormalSwerveMap;
import frc.robot.subsystems.swerve.normal.NormalWheelModule;

public class NormalSwerveFactory implements ISwerveFactory {
    public NormalSwerve makeSwerve(){
        NormalWheelModule fl = new NormalWheelModule(new TalonSRXComponent(NormalSwerveMap.FL_ANGLE_PORT), new TalonFXComponent(NormalSwerveMap.FL_SPEED_PORT));
        NormalWheelModule fr = new NormalWheelModule(new TalonSRXComponent(NormalSwerveMap.FR_ANGLE_PORT), new TalonFXComponent(NormalSwerveMap.FR_SPEED_PORT));
        NormalWheelModule bl = new NormalWheelModule(new TalonSRXComponent(NormalSwerveMap.BL_ANGLE_PORT), new TalonFXComponent(NormalSwerveMap.BL_SPEED_PORT));;
        NormalWheelModule br = new NormalWheelModule(new TalonSRXComponent(NormalSwerveMap.BR_ANGLE_PORT), new TalonFXComponent(NormalSwerveMap.BR_SPEED_PORT));;
        GyroDashboardDecorator gyro = new GyroDashboardDecorator(
                "Gyro",
                "Swerve",
                new AHRSAngleGetterComponent(SPI.Port.kMXP)
        );

        NormalSwerve swerve = new NormalSwerve(NormalSwerveMap.SWERVE_LENGTH,
                NormalSwerveMap.SWERVE_WIDTH, fl, fr, bl, br, gyro);

        return swerve;

    }
}
