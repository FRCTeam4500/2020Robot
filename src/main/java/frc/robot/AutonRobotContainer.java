/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.ExtendedTrajectoryUtilities;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.OdometricSwerveDashboardUtility;
import frc.robot.subsystems.swerve.odometric.factory.EntropySwerveFactory;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_FollowTrajecoryCommand;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_ResetPoseCommand;

import static frc.robot.utility.ExtendedMath.withDeadzone;
/**
 * Add your docs here.
 */
public class AutonRobotContainer implements IRobotContainer{
    private OdometricSwerve swerve;
    private Joystick joystick;
    private JoystickButton resetGyro;
    public AutonRobotContainer(){
        swerve = new EntropySwerveFactory().makeSwerve();
        joystick = new Joystick(0);
        
        resetGyro = new JoystickButton(joystick, 1);
        resetGyro.whenPressed(() -> swerve.resetGyro(),swerve);

        swerve.setDefaultCommand(new RunCommand(() -> {
            swerve.moveFieldCentric(
                withDeadzone(-joystick.getY(), 0.3),
                withDeadzone(-joystick.getX(), 0.3),
                withDeadzone(joystick.getZ(), 0.3)
            );
        }
        , swerve));

        SmartDashboard.putData("Swerve Positions",new OdometricSwerveDashboardUtility(swerve));

        SmartDashboard.putData("Reset Pose",new OdometricSwerve_ResetPoseCommand(new Pose2d(13, -5.75, new Rotation2d()), swerve));
        addAutonCommand("CrossTheLine");
        addAutonCommand("CitrusCompatabile");
        addAutonCommand("AwayFromCenterForward");

    }
    private CommandBase makeMoveToTranslationCommand(String trajectoryName) {
        var pid = new PIDController(10, 0, 0);
        pid.setTolerance(0.1);
        return new OdometricSwerve_FollowTrajecoryCommand(swerve, pid, ExtendedTrajectoryUtilities.tryGetDeployedTrajectory(trajectoryName));
    }
    private void addAutonCommand(String trajectoryName){
        SmartDashboard.putData("Run "+trajectoryName, makeMoveToTranslationCommand(trajectoryName));
    }
}
