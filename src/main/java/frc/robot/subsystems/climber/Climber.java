package frc.robot.subsystems.climber;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.IAngleSetterComponent;
import frc.robot.components.IOutputSetterComponent;

public class Climber extends SubsystemBase {
    IOutputSetterComponent motor;
    public Climber(IOutputSetterComponent motor){
        this.motor = motor;
    }

    public void setSpeed(double angle){
        motor.setOutput(angle);
    }

}

