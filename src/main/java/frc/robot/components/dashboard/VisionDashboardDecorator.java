/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.components.IVisionComponent;

/**
 * A {@link DashboardDecorator} for any {@link IVisionComponent} component.
 */
public class VisionDashboardDecorator extends DashboardDecorator implements IVisionComponent {

    private CameraMode lastSetCameraMode;
    private int lastSetPipeline;

    /**
     * See {@link DashboardDecorator#DashboardDecorator(String, String)} for more
     * details.
     * 
     * @param vision the {@link IVisionComponent} component to decorate
     */
    public VisionDashboardDecorator(String name, String subsystem, IVisionComponent vision) {
        super(name, subsystem);
        this.vision = vision;
        send();
    }

    private IVisionComponent vision;

    @Override
    public boolean hasValidTargets() {
        return vision.hasValidTargets();
    }

    @Override
    public double getHorizontalOffsetFromCrosshair() {
        return vision.getHorizontalOffsetFromCrosshair();
    }

    @Override
    public double getVerticalOffsetFromCrosshair() {
        return vision.getVerticalOffsetFromCrosshair();
    }

    @Override
    public double getTargetArea() {
        return vision.getTargetArea();
    }

    @Override
    public double getSkew() {
        return vision.getSkew();
    }

    @Override
    public void setCameraMode(CameraMode mode) {
        lastSetCameraMode = mode;
        vision.setCameraMode(mode);
    }

    @Override
    public void setPipeline(int index) {
        lastSetPipeline = index;
        vision.setPipeline(index);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Has Valid Targets", this::hasValidTargets, null);
        builder.addDoubleProperty("Horizontal Offset From Crosshair", this::getHorizontalOffsetFromCrosshair, null);
        builder.addDoubleProperty("Vertical Offset From Crosshair", this::getVerticalOffsetFromCrosshair, null);
        builder.addDoubleProperty("Target Area", this::getTargetArea, null);
        builder.addDoubleProperty("Skew", this::getSkew, null);
        builder.addStringProperty("Last Set Camera Mode",
                () -> (lastSetCameraMode != null) ? lastSetCameraMode.toString() : "None", null);
        builder.addDoubleProperty("Last Set Pipeline", () -> lastSetPipeline, null);
    }
}
