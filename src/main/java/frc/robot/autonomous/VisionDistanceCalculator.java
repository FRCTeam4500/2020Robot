/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Add your docs here.
 */
public class VisionDistanceCalculator implements Sendable {
    private double cameraPitchRadians;
    private double cameraHeightMeters;
    private double targetHeightMeters;
    private VisionSubsystem vision;

    

    public double getDistanceFromTargetMeters(){
        return (targetHeightMeters - cameraHeightMeters)/Math.tan(cameraPitchRadians + vision.getVerticalOffset());
    }
    public double getDesiredTurretOffset(double turretAngle, double swerveAngle){
        //TODO: The Do
        return 0.0;
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Distance", () -> Units.metersToFeet(getDistanceFromTargetMeters()), null);
    }

    public VisionDistanceCalculator(double cameraPitchRadians, double cameraHeightMeters, double targetHeightMeters,
            VisionSubsystem vision) {
        this.cameraPitchRadians = cameraPitchRadians;
        this.cameraHeightMeters = cameraHeightMeters;
        this.targetHeightMeters = targetHeightMeters;
        this.vision = vision;
    }
     
}
