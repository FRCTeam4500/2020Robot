package frc.robot.subsystems.vision.factory;

import frc.robot.components.hardware.LimelightVisionComponent;
import frc.robot.subsystems.vision.Vision;

public class DefaultVisionFactory implements IVisionFactory {

    public Vision makeVision(){
        LimelightVisionComponent limelight = new LimelightVisionComponent();
        limelight.setPipeline(4);
        return new Vision(limelight);
    }
}
