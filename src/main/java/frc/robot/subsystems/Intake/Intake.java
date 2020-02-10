package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.ISpeedSetterComponent;

public class Intake extends SubsystemBase {
    ISpeedSetterComponent motor;
    public Intake(ISpeedSetterComponent motor){
        this.motor = motor;

    }

    public void setSpeed(double speed){
        motor.setSpeed(speed);
    }
}
