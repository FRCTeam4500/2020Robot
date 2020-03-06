package frc.robot.subsystems.swerve.kinematic.factory;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.components.dashboard.GyroDashboardDecorator;
import frc.robot.components.hardware.AHRSAngleGetterComponent;
import frc.robot.components.hardware.TalonFXComponent;
import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.ISwerveFactory;
import frc.robot.subsystems.swerve.kinematic.KinematicSwerve;
import frc.robot.subsystems.swerve.kinematic.KinematicSwerveMap;
import frc.robot.subsystems.swerve.kinematic.KinematicWheelModule;

public class KinematicSwerveFactory implements ISwerveFactory {
    @Override
    public ISwerve makeSwerve() {






        KinematicWheelModule fl = new KinematicWheelModule(new TalonSRXComponent(KinematicSwerveMap.FL_ANGLE_PORT), new TalonFXComponent(KinematicSwerveMap.FL_SPEED_PORT), new Translation2d(KinematicSwerveMap.DRIVE_FORWARD/2,KinematicSwerveMap.DRIVE_LEFTWARD/2), KinematicSwerveMap.MAX_SURFACE_SPEED, KinematicSwerveMap.WHEEL_DIAMETER_METERS);
        KinematicWheelModule fr = new KinematicWheelModule(new TalonSRXComponent(KinematicSwerveMap.FR_ANGLE_PORT), new TalonFXComponent(KinematicSwerveMap.FR_SPEED_PORT), new Translation2d(KinematicSwerveMap.DRIVE_FORWARD/2,-KinematicSwerveMap.DRIVE_LEFTWARD/2), KinematicSwerveMap.MAX_SURFACE_SPEED, KinematicSwerveMap.WHEEL_DIAMETER_METERS);
        KinematicWheelModule bl = new KinematicWheelModule(new TalonSRXComponent(KinematicSwerveMap.BL_ANGLE_PORT), new TalonFXComponent(KinematicSwerveMap.BL_SPEED_PORT), new Translation2d(-KinematicSwerveMap.DRIVE_FORWARD/2,KinematicSwerveMap.DRIVE_LEFTWARD/2), KinematicSwerveMap.MAX_SURFACE_SPEED, KinematicSwerveMap.WHEEL_DIAMETER_METERS);
        KinematicWheelModule br = new KinematicWheelModule(new TalonSRXComponent(KinematicSwerveMap.BR_ANGLE_PORT), new TalonFXComponent(KinematicSwerveMap.BR_SPEED_PORT), new Translation2d(-KinematicSwerveMap.DRIVE_FORWARD/2,-KinematicSwerveMap.DRIVE_LEFTWARD/2), KinematicSwerveMap.MAX_SURFACE_SPEED, KinematicSwerveMap.WHEEL_DIAMETER_METERS);
        GyroDashboardDecorator gyro = new GyroDashboardDecorator(
                "Gyro",
                "Swerve",
                new AHRSAngleGetterComponent(SPI.Port.kMXP)
            );

        return new KinematicSwerve(gyro, fl, fr, bl, br);
    }
}
