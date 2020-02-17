package frc.robot.subsystems.Intake.factory;

import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeMap;

public class DefaultIntakeFactory implements IIntakeFactory {
    public Intake makeIntake(){
        return new Intake(new TalonSRXComponent(IntakeMap.INTAKE_MOTOR_PORT));
    }
}
