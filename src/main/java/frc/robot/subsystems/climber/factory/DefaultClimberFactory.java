package frc.robot.subsystems.climber.factory;
import frc.robot.subsystems.climber.factory.IClimberFactory;
import frc.robot.subsystems.climber.Climber;
import frc.robot.components.hardware.SparkMaxComponent;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.subsystems.climber.ClimberMap;
public class DefaultClimberFactory implements IClimberFactory {
    public Climber makeClimber() {
        SparkMaxComponent motor = new SparkMaxComponent(ClimberMap.CLIMBER_MOTOR_PORT, MotorType.kBrushless);
        Climber climber = new Climber(motor);
        return climber;
    }
}