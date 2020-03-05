package frc.robot.subsystems.intake.factory;

import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.components.hardware.SparkMaxComponent;
import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeMap;

public class DefaultIntakeFactory implements IIntakeFactory {
    public Intake makeIntake(){
        return new Intake(new SparkMaxComponent(IntakeMap.INTAKE_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless));
    }
}
