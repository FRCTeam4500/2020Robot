package frc.robot.subsystems.Intake.factory;

import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.Intake.Intake;

public class HardwareIntakeFactory implements IIntakeFactory {
    /**
     *
     */
    private static final int INTAKE_MOTOR_PORT = 5;

    public Intake makeIntake(){
        return  new Intake(new TalonSRXComponent(INTAKE_MOTOR_PORT));
    }
}
