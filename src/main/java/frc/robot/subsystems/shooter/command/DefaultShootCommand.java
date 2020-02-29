package frc.robot.subsystems.shooter.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.IShooterOI;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterMap;


public class DefaultShootCommand extends CommandBase {
    private Shooter shooter;
    private IShooterOI oi;

    private double topFlywheelSpeed;
    private double bottomFlywheelSpeed;

    public DefaultShootCommand(Shooter shooter, IShooterOI oi) {
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
            shooter.run(ShooterMap.TOP_SPEED * ShooterMap.COEFFICIENT, ShooterMap.BOTTOM_SPEED * ShooterMap.COEFFICIENT);
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
