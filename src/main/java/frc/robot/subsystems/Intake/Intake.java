package frc.robot.subsystems.Intake;

import frc.robot.components.ISpeedSetterComponent;

public class Intake {
    ISpeedSetterComponent motor;
    public Intake(ISpeedSetterComponent motor){
        this.motor = motor;

    }

    public void setSpeed(double speed){
        motor.setSpeed(speed);
    }
}
