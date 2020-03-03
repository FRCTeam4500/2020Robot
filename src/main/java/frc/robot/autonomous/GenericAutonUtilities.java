/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.swerve.odometric.command.AdvancedSwerveControllerBuilder;
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
    public static AdvancedSwerveControllerBuilder createDefaultControllerBuilder(){
        return new AdvancedSwerveControllerBuilder()
        .withInitialAllowableTranslationError(0.1)
        .withFinalAllowableTranslationError(0.1)
        .withAllowableRotationError(0.1)
        .withTranslationsEnabled(true)
        .with_kP(3)
        .with_kW(3)
        .withRotationsEnabled(true)
        .withEndRotation(new Rotation2d())
        .withMaxVelocity(2.4);
    }
}
