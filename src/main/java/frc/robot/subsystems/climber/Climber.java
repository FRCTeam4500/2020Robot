package frc.robot.subsystems.climber;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.IAngleSetterComponent;

public class Climber extends SubsystemBase {
    IAngleSetterComponent motor1;
    IAngleSetterComponent motor2;
    public Climber(IAngleSetterComponent motor1, IAngleSetterComponent motor2) {
        this.motor1 = motor1;
        this.motor2 = motor2;
    }

    public void setClimberAngle(double angle){
        motor1.setAngle(angle);
        motor2.setAngle(angle);
    }

}

