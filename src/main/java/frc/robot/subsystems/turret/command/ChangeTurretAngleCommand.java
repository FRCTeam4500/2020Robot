package frc.robot.subsystems.turret.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.ITurretOI;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretMap;


public class ChangeTurretAngleCommand extends CommandBase {
    private Turret turret;
    private ITurretOI oi;
    public ChangeTurretAngleCommand(Turret turret, ITurretOI oi) {
        this.turret = turret;
        this.oi = oi;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double turretAngle = turret.getTurretAngle();
        if ((turretAngle + oi.getTurretDesiredAngle()) > TurretMap.TURRET_MAXIMUM_VALUE){
            turret.setTurretAngle(TurretMap.TURRET_MAXIMUM_VALUE);
        }
        else if((turretAngle + oi.getTurretDesiredAngle()) < TurretMap.TURRET_MINIMUM_VALUE){
            turret.setTurretAngle(TurretMap.TURRET_MINIMUM_VALUE);
        }
        else{
            turretAngle = turretAngle + oi.getTurretDesiredAngle();
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
