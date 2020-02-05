package frc.robot.subsystems.climber;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.ISpeedSetterComponent;

public class Climber extends SubsystemBase {
    ISpeedSetterComponent motor1;
    ISpeedSetterComponent motor2;
    public Climber(ISpeedSetterComponent motor1, ISpeedSetterComponent motor2) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    public void setSpeed(double speed){
        motor1.setSpeed(speed);
        motor2.setSpeed(speed);
    }

}

