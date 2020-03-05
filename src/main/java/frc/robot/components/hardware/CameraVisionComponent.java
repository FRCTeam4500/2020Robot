package frc.robot.components.hardware;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.components.IVisionComponent;

public class CameraVisionComponent {
    /**
     * table values
     * hasValidTargets
     * diffX
     * diffY
     * loadX
     * loadY
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
        return getEntry("ax");
    }

    private double getEntry(String key) {
        //return table.getEntry(key).getDouble(0);
        return SmartDashboard.getNumber(key, 0);
    }
}
