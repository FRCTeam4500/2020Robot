package frc.robot.subsystems.vision.factory;

import frc.robot.components.hardware.LimelightVisionComponent;
import frc.robot.subsystems.vision.Vision;

public class DefaultVisionFactory implements IVisionFactory {
    public Vision makeVision(){
        return new Vision(new LimelightVisionComponent());
    }
}
