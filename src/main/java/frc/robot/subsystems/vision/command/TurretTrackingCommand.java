package frc.robot.subsystems.vision.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.ITurretOI;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;


public class TurretTrackingCommand extends CommandBase {
    private Vision vision;
    private ITurretOI oi;

    public TurretTrackingCommand(Vision vision, ITurretOI oi) {
        this.vision = vision;
        addRequirements(vision);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        oi.setTurretDesiredAngle(vision.);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
