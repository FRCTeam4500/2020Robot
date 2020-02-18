package frc.robot.subsystems.climber;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.IAngleSetterComponent;

public class Climber extends SubsystemBase {
    IAngleSetterComponent motor1;
    public Climber(IAngleSetterComponent motor1) {
        this.motor1 = motor1;
    }

    public void setSpeed(double angle){
        motor1.setAngle(angle);
    }

}

