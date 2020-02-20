package frc.robot.subsystems.turret.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.ITurretOI;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.factory.DefaultTurretFactory;


public class SetTurretAngleCommand extends CommandBase {
    private Turret turret;
    private ITurretOI oi;
    private double desiredAngle;
    private double maxAngle;
    private double minAngle;
    public SetTurretAngleCommand(Turret turret, ITurretOI oi) {
        this.turret = turret;
        this.oi = oi;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double desiredAngle = this.oi.getTurretAngle();
        if(desiredAngle > maxAngle) {
            turret.setTurretAngle(maxAngle);
        } else if(desiredAngle < minAngle) {
            turret.setTurretAngle(minAngle);
        } else {
            turret.setTurretAngle(desiredAngle);
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
