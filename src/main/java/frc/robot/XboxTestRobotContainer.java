/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.OdometricSwerveDashboardUtility;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_FollowTrajecoryCommand;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_ResetPoseCommand;
import frc.robot.subsystems.swerve.odometric.factory.OdometricSimulatedSwerveFactory;

/**
 * Add your docs here.
 */
public class XboxTestRobotContainer implements IRobotContainer {
    private XboxController controller = new XboxController(0);
    private OdometricSimulatedSwerveFactory factory = new OdometricSimulatedSwerveFactory();
    private OdometricSwerve swerve = factory.makeSwerve();
    private OdometricSwerveDashboardUtility utility = new OdometricSwerveDashboardUtility(swerve);

    public XboxTestRobotContainer() {
        swerve.setDefaultCommand(
                new RunCommand(() -> swerve.moveRobotCentric(withDeadzone(controller.getX(Hand.kLeft), 0.2) * 10,
                        -withDeadzone(controller.getY(Hand.kLeft), 0.2) * 10,
                        withDeadzone(controller.getX(Hand.kRight), 0.2) * 3), swerve));
        SendableRegistry.addLW(utility, "Swerve", "Utility");
        SmartDashboard.putData("Reset Pose", new OdometricSwerve_ResetPoseCommand(new Pose2d(), swerve));
        SmartDashboard.putData("Run Auto", makeMoveToTranslationCommand());

    }

    private double withDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0;
        } else {
            return value;
        }
    }

    private CommandBase makeMoveToTranslationCommand() {
        var pid = new PIDController(0.5, 0, 0);
        pid.setTolerance(0.1);
        return new OdometricSwerve_FollowTrajecoryCommand(swerve, pid,
                getTrajectory().getStates().toArray(Trajectory.State[]::new));
    }

    private Trajectory getTrajectory() {
        String trajectoryJSON = "paths/output/Crazy.wpilib.json";
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            return trajectory;
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            return new Trajectory(null);
        }
    }
}
