package frc.robot.subsystems.shooter.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.IShooterOI;
import frc.robot.subsystems.shooter.Shooter;


public class ShootStraightCommand extends CommandBase {
    private Shooter shooter;
    private IShooterOI oi;

    private double topFlywheelSpeed;
    private double bottomFlywheelSpeed;

    public ShootStraightCommand(Shooter shooter, IShooterOI oi) {
        this.shooter = shooter;
        this.oi = oi;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (oi.getShooterActive()) {
            shooter.run(0.8, 0.8);
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
