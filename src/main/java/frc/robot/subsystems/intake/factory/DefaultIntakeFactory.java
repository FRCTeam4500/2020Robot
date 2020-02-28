package frc.robot.subsystems.intake.factory;

import frc.robot.components.hardware.TalonFXComponent;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeMap;

public class DefaultIntakeFactory implements IIntakeFactory {
    public Intake makeIntake(){
        return new Intake(new TalonFXComponent(IntakeMap.INTAKE_MOTOR_PORT));
    }
}
