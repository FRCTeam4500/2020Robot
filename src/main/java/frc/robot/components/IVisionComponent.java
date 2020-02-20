/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components;

/**
 * Add your docs here.
 */
public interface IVisionComponent {
    enum CameraMode {
        VisionProcessor, DriverCamera
    }

    /**
     * Determines whether any valid targets are detected by the IVisionComponent
     * component. Targets are the final output of a vision component, after
     * thresholding and filtering contours. Usually, the vision component is
     * configurable, and the thresholds and contours can be changed.
     * 
     * @return whether any valid targets are detected
     */
    boolean hasValidTargets();

    /**
     * Gets the horizontal offset (in radians) from the crosshair to the target. The
     * crosshair is essentially the origin from which offset to targets are
     * calculated. With a Limelight, this crosshair can be calibrated so that
     * different positions may be considered "on target" relative to the camera.
     * <br/>
     * <br/>
     * A positive angle means that the target to the right of the crosshair, and a
     * negative angle means that the target is to the left. Zero means that the
     * target is perfectly lined up horizontally with the crosshair.
     * 
     * @return the horizontal offset
     */
    double getHorizontalOffsetFromCrosshair();

    /**
     * Gets the vertial offset (in radians) from the crosshair to the target. See
     * {@link #getHorizontalOffsetFromCrosshair()} for more details on crosshairs.
     * <br/>
     * <br/>
     * A positive angle means that the target is above the crosshair, and a negative
     * angle means that the target is below the crosshair. Zero means that the
     * target is perfectly level with the crosshair.
     * 
     * @return the vertical offset
     */
    double getVerticalOffsetFromCrosshair();

    /**
     * Gets the percentage of the camera's FOV that is taken up by the target. 0%
     * means that the target does not exist, and 100% means that the camera is
     * completely covered by the target. This can be used as a rough measurement of
     * distance from the target.
     * 
     * @return the percentage of area of camera view covered by the target
     */
    double getTargetArea();

    /**
     * Gets the rotation (in radians) of the target relative to the vision
     * processor. A positive value means the target is rotated counter-clockwise,
     * and a negative value means the target is rotated clockwise. The rotation is
     * within a range of 0 to pi/2 radians.
     * 
     * @return the rotation of the target
     */
    double getSkew();

    /**
     * Sets the camera mode of the component. See {@link CameraMode} for more
     * details.
     * 
     * @param mode the desired camera mode
     */
    void setCameraMode(CameraMode mode);

    /**
     * Sets the vision processing pipeline used by the component. A "pipeline"
     * refers to the process the vision component takes to calculate the location,
     * rotation, and size of the target. A vision processor can have multiple
     * pipelines so that each configuration can be switched out as needed.
     * 
     * @param index - the index of the desired pipeline
     */
    void setPipeline(int index);
}
