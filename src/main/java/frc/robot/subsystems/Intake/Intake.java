package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.IOutputSetterComponent;

public class Intake extends SubsystemBase {
    IOutputSetterComponent motor;
    public Intake(IOutputSetterComponent motor){
        this.motor = motor;

    }

    public void setSpeed(double speed){
        motor.setOutput(speed);
    }
}
