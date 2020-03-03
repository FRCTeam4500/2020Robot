package frc.robot.subsystems.climber.factory;
import com.revrobotics.CANSparkMax;
import frc.robot.subsystems.climber.factory.IClimberFactory;
import frc.robot.subsystems.climber.Climber;
import frc.robot.components.hardware.SparkMaxComponent;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.subsystems.climber.ClimberMap;
public class HardwareClimberFactory implements IClimberFactory {
    public Climber makeClimber() {
        SparkMaxComponent motor = new SparkMaxComponent(ClimberMap.CLIMBER_MOTOR_PORT, MotorType.kBrushless);
        motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ClimberMap.MAX_MOTOR_VALUE);
        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ClimberMap.MIN_MOTOR_VALUE);
        Climber climber = new Climber(motor);
        return climber;
    }
}