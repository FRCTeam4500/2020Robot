/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.virtual;

import frc.robot.components.IVisionComponent;

/**
 * A virtual {@link IVision} component.
 */
public class VirtualVisionComponent implements IVisionComponent {

    private boolean _hasValidTargets = false;

    @Override
    public boolean hasValidTargets() {
        return _hasValidTargets;
    }

    /**
     * Sets the return value of {@link #hasValidTargets()}.
     * 
     * @param hasValidTargets the new return value of hasValidTargets()
     */
    public void setHasValidTargets(boolean hasValidTargets) {
        _hasValidTargets = hasValidTargets;
    }

    private double horizontalOffsetFromCrosshair = 0;

    @Override
    public double getHorizontalOffsetFromCrosshair() {
        return horizontalOffsetFromCrosshair;
    }

    /**
     * Sets the return value of {@link #getHorizontalOffsetFromCrosshair()}
     * 
     * @param horizontalOffsetFromCrosshair the new returned value for
     *                                      getHorizontalOffsetFromCrosshair().
     */
    public void setHorizontalOffsetFromCrosshair(double horizontalOffsetFromCrosshair) {
        this.horizontalOffsetFromCrosshair = horizontalOffsetFromCrosshair;
    }

    private double verticalOffsetFromCrosshar = 0;

    @Override
    public double getVerticalOffsetFromCrosshair() {
        return verticalOffsetFromCrosshar;
    }

    /**
     * Sets the return value of {@link #getVerticalOffsetFromCrosshar()}.
     * 
     * @param verticalOffsetFromCrosshar the return value for getVerticalOffsetFromCrosshar()
     */
    public void setVerticalOffsetFromCrosshar(double verticalOffsetFromCrosshar) {
        this.verticalOffsetFromCrosshar = verticalOffsetFromCrosshar;
    }

    private double targetArea = 0.0;

    @Override
    public double getTargetArea() {
        return targetArea;
    }

    /**
     * Sets the return value for {@link #getTargetArea()}.
     * 
     * @param targetArea the new return value
     */
    public void setTargetArea(double targetArea) {
        this.targetArea = targetArea;
    }

    private double skew = 0.0;

    @Override
    public double getSkew() {
        return skew;
    }

    /**
     * Sets the return value for {@link #getSkew()}.
     * 
     * @param skew the new return value
     */
    public void setSkew(double skew) {
        this.skew = skew;
    }

    private CameraMode mode = CameraMode.DriverCamera;

    @Override
    public void setCameraMode(CameraMode mode) {
        this.mode = mode;
    }

    /**
     * Get the last set value dispatched by {@link #setCameraMode(CameraMode)}.
     * 
     * @return the last set value
     */
    public CameraMode getCameraMode() {
        return mode;
    }

    private int pipelineIndex = 0;

    @Override
    public void setPipeline(int index) {
        pipelineIndex = 0;
    }

    /**
     * Gets the last set value dispatched by {@link #setPipeline(int).
     * 
     * @return the last set value
     */
    public int getPipelineIndex() {
        return pipelineIndex;
    }
}
