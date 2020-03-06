package frc.robot.subsystems.swerve.kinematic.factory;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.components.dashboard.GyroDashboardDecorator;
import frc.robot.components.hardware.AHRSAngleGetterComponent;
import frc.robot.components.hardware.TalonFXComponent;
import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.ISwerveFactory;
import frc.robot.subsystems.swerve.kinematic.KinematicSwerve;
import frc.robot.subsystems.swerve.kinematic.KinematicSwerveMap;
import frc.robot.subsystems.swerve.kinematic.KinematicWheelModule;
import frc.robot.subsystems.swerve.normal.KinematicSwerve;
import frc.robot.subsystems.swerve.normal.KinematicSwerveMap;
import frc.robot.subsystems.swerve.normal.KinematicWheelModule;

public class KinematicSwerveFactory implements ISwerveFactory {
    @Override
    public ISwerve makeSwerve() {
        
            KinematicWheelModule fl = new KinematicWheelModule(new TalonSRXComponent(KinematicSwerveMap.FL_ANGLE_PORT), new TalonFXComponent(KinematicSwerveMap.FL_SPEED_PORT));
            KinematicWheelModule fr = new KinematicWheelModule(new TalonSRXComponent(KinematicSwerveMap.FR_ANGLE_PORT), new TalonFXComponent(KinematicSwerveMap.FR_SPEED_PORT));
            KinematicWheelModule bl = new KinematicWheelModule(new TalonSRXComponent(KinematicSwerveMap.BL_ANGLE_PORT), new TalonFXComponent(KinematicSwerveMap.BL_SPEED_PORT));;
            KinematicWheelModule br = new KinematicWheelModule(new TalonSRXComponent(KinematicSwerveMap.BR_ANGLE_PORT), new TalonFXComponent(KinematicSwerveMap.BR_SPEED_PORT));;
            GyroDashboardDecorator gyro = new GyroDashboardDecorator(
                    "Gyro",
                    "Swerve",
                    new AHRSAngleGetterComponent(SPI.Port.kMXP)
            );

            KinematicSwerve swerve = new KinematicSwerve(gyro, fl, fr, bl, br,);

            return swerve;
    }
}
