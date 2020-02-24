package frc.robot.subsystems.shooter.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.IShooterOI;
import frc.robot.subsystems.shooter.Shooter;


public class ShootStraightCommand extends CommandBase {
    private Shooter shooter;
    private IShooterOI oi;


    

    public ShootStraightCommand(Shooter shooter, IShooterOI oi) {
        this.shooter = shooter;
        this.oi = oi;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("topSpeed", 0.0);
        SmartDashboard.putNumber("bottomSpeed", 0.0);
        SmartDashboard.putNumber("coefficient", 1.0);
    }

    @Override
    public void execute() {
        var topSpeed = SmartDashboard.getNumber("topSpeed", 0.0);
        var bottomSpeed = SmartDashboard.getNumber("bottomSpeed", 0.0);
        var k = SmartDashboard.getNumber("coefficient", 1.0);
        if (oi.getShooterActive()) {
            shooter.run(topSpeed*k, bottomSpeed*k);
        }
        else{
            shooter.run(0,0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
