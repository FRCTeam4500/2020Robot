/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.containers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * Add your docs here.
 */
public class ScratchpadRobotContainer implements IRobotContainer{
    int count = 0;
    public ScratchpadRobotContainer(){
        var timeoutTest = 
            new InstantCommand(() -> SmartDashboard.putBoolean("Timeout Test Finished", false))
            .andThen(() -> count = 0)
            .andThen(new RunCommand(() -> {
                count++;
                SmartDashboard.putNumber("Count", count);
            }
                ))
            .withTimeout(5)
            .andThen(() -> SmartDashboard.putBoolean("Timeout Test Finished", true));
        SmartDashboard.putData("Run Timeout Test", timeoutTest);
    }
}
