package frc.robot.components.hardware;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.components.IVisionComponent;

public class CameraVisionComponent implements IVisionComponent {
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

    public double getTargetArea(){
        return 0;
    }

    public double getSkew(){
        return 0;
    }

    public void setCameraMode(CameraMode mode){

    }

    public void setPipeline(int pipeline){
        
    }

    public boolean hasValidTargets(){
        var value = getEntry("hasValidTargets");
        if (value == 1){
            return true;
        }
        else {
            return false;
        }
    }
    private double getEntry(String key) {
        //return table.getEntry(key).getDouble(0);
        return SmartDashboard.getNumber(key, 0);
    }
}
