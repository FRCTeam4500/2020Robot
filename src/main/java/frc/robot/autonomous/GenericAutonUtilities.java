/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Add your docs here.
 */
public class GenericAutonUtilities {
    public static VisionDistanceCalculator makeEntropyVisionDistanceCalculator(VisionSubsystem vision){
        var cameraHeight = Units.inchesToMeters(34.5);
        var targetHeight = Units.feetToMeters(7.2);
        var horizontalDistanceToTarget = Units.feetToMeters(16.25);
        var targetAngleReading = Units.degreesToRadians(-11.6);
        var cameraAngle = Math.atan((targetHeight - cameraHeight)/horizontalDistanceToTarget)-targetAngleReading;

        return new VisionDistanceCalculator(cameraAngle, cameraHeight, targetHeight, vision);
    }
}
