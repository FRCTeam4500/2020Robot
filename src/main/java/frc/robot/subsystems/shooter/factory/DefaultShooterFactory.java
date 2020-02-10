package frc.robot.subsystems.shooter.factory;

import frc.robot.components.hardware.TalonFXComponent;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterMap;

public class DefaultShooterFactory implements IShooterFactory {
    public Shooter makeShooter() {
        return new Shooter(new TalonFXComponent(ShooterMap.TOP_MOTOR_PORT),
                new TalonFXComponent(ShooterMap.BOTTOM_MOTOR_PORT));
    }
}
