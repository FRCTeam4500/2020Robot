/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Autonomous_StopShootingCommand extends InstantCommand {
  Indexer indexer;
  Shooter shooter;
  public Autonomous_StopShootingCommand(Indexer indexer, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexer =  indexer;
    this.shooter = shooter;
    addRequirements(indexer, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.setSpeed(0);
    shooter.run(0, 0);
  }
}
