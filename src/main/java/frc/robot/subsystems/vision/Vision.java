package frc.robot.subsystems.vision;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.hardware.LimelightVisionComponent;

public class Vision extends SubsystemBase {
    private LimelightVisionComponent limelight;
    public Vision(LimelightVisionComponent limelight) {
        this.limelight = limelight;

        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

}

