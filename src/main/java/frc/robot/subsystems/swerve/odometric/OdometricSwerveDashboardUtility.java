/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.odometric;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Add your docs here.
 */
public class OdometricSwerveDashboardUtility implements Sendable {

    private OdometricSwerve swerve;
    public OdometricSwerveDashboardUtility(OdometricSwerve swerve){
        this.swerve = swerve;
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("PoseX", () -> swerve.getCurrentPose().getTranslation().getX(), null);
        builder.addDoubleProperty("PoseY", () -> swerve.getCurrentPose().getTranslation().getY(), null);
        builder.addDoubleProperty("PoseR", () -> swerve.getCurrentPose().getRotation().getRadians(), null);
        builder.addDoubleProperty("Robot Angle", () -> swerve.getGyroAngle(), null);
    }
}
