package frc.robot.subsystems.shooter.factory;

import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.components.hardware.SparkMaxComponent;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterMap;

public class DefaultShooterFactory implements IShooterFactory {
    public Shooter makeShooter() {
        return new Shooter(new SparkMaxComponent(ShooterMap.TOP_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless),
                new SparkMaxComponent(ShooterMap.BOTTOM_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless));
    }

}
