package frc.robot.subsystems.climber.factory;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.climber.factory.IClimberFactory;
import frc.robot.subsystems.climber.Climber;
import frc.robot.components.hardware.SparkMaxComponent;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.subsystems.climber.ClimberMap;
public class HardwareClimberFactory implements IClimberFactory {
    public Climber makeClimber() {
        SparkMaxComponent motor = new SparkMaxComponent(ClimberMap.CLIMBER_MOTOR_PORT, MotorType.kBrushless);
        motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ClimberMap.MAX_MOTOR_VALUE);
        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ClimberMap.MIN_MOTOR_VALUE);
        SmartDashboard.putData("CLIMBER MOTOR",new Sendable(){
        
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addBooleanProperty("forwardLimitEnabled", () -> motor.isSoftLimitEnabled(SoftLimitDirection.kForward), null);
                builder.addBooleanProperty("reverseLimitEnabled", () -> motor.isSoftLimitEnabled(SoftLimitDirection.kReverse), null);
                builder.addDoubleProperty("forwardLimit", (() -> motor.getSoftLimit(SoftLimitDirection.kForward)), null);
                builder.addDoubleProperty("reverseLimit", () -> motor.getSoftLimit(SoftLimitDirection.kReverse), null);
                builder.addDoubleProperty("measurement", () -> motor.getEncoder().getPosition(), null);
            }
        });
        Climber climber = new Climber(motor);
        return climber;
    }
}