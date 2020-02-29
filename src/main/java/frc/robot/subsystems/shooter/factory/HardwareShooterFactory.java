package frc.robot.subsystems.shooter.factory;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.components.hardware.SparkMaxComponent;
import frc.robot.subsystems.shooter.Shooter;

public class HardwareShooterFactory implements IShooterFactory {
    /**
     *
     */
    private static final int BOTTOM_MOTOR_PORT = 13;
    /**
     *
     */
    private static final int TOP_MOTOR_PORT = 14;

    public Shooter makeShooter() {
        return new Shooter(
            new SparkMaxComponent(TOP_MOTOR_PORT, MotorType.kBrushless), 
        new SparkMaxComponent(BOTTOM_MOTOR_PORT, MotorType.kBrushless));
    }
}
