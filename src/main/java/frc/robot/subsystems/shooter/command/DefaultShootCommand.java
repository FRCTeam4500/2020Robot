package frc.robot.subsystems.shooter.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.IShooterOI;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterMap;
import frc.robot.subsystems.indexer.Indexer;


public class DefaultShootCommand extends CommandBase {
    private Shooter shooter;
    private IShooterOI oi;
    private Indexer indexer;

    private double topFlywheelSpeed;
    private double bottomFlywheelSpeed;

    public DefaultShootCommand(Shooter shooter, IShooterOI oi, Indexer indexer) {
        this.shooter = shooter;
        this.oi = oi;
        this.indexer = indexer;
        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (oi.getShooterActive()) {
            if (shooter.atSpeeds(ShooterMap.THRESHOLD)){
                shooter.run(ShooterMap.TOP_SPEED * ShooterMap.COEFFICIENT, ShooterMap.BOTTOM_SPEED * ShooterMap.COEFFICIENT);
                indexer.setSpeed(1);
            }
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
