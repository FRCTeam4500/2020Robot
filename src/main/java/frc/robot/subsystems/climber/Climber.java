package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.ISmartMotorComponent;
public class Climber extends SubsystemBase{
    private ISmartMotorComponent motor;

    public Climber(ISmartMotorComponent motor) {
        this.motor = motor;
    }

    public void setSpeed(double speed) {
        motor.setOutput(speed);
    }

}