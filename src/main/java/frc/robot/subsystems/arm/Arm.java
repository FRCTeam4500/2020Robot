package frc.robot.subsystems.arm;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.IAngleSetterComponent;

public class Arm extends SubsystemBase {
    IAngleSetterComponent motor;
    public Arm(IAngleSetterComponent motor) {
        this.motor = motor;

        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    public void setAngle(double angle){
        motor.setAngle(angle);
    }

}

