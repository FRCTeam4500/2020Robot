package frc.robot.subsystems.vision.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.ITurretOI;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretMap;
import frc.robot.subsystems.vision.Vision;


public class TurretTrackingCommand extends CommandBase {
    private Vision vision;
    private ITurretOI oi;
    private Turret turret;
    private double turretChangeAngle;

    public TurretTrackingCommand(Vision vision, Turret turret) {
        this.vision = vision;
        this.turret = turret;
        addRequirements(vision);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double turretAngle = turret.getTurretAngle();
        if (vision.hasValidTargets()) {
            turretChangeAngle = vision.getHorizontalOffset();
        }
        else{
            turretChangeAngle = 0;
        }


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
