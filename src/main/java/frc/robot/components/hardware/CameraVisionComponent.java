package frc.robot.components.hardware;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.components.IVisionComponent;

public class CameraVisionComponent {
    /**
     * table values
     * hasValidTargets
     * diffX
     * diffY
     * aX
     * aY
     */


    public CameraVisionComponent(){
    }

    public double getHorizontalOffsetFromCrosshair(){
       return Math.toRadians(getEntry("diffX"));
    }

    public double getVerticalOffsetFromCrosshair(){
        return Math.toRadians(getEntry("diffY"));
    }

    public double getAngleX(){
        return Math.toRadians(getEntry("aX"));
    }

    public double getTargetArea(){
        return 0;
    }

    private double getEntry(String key) {
        return SmartDashboard.getNumber(key, 0);
    }
}
