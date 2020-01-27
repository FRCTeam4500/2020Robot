package frc.robot.subsystems.shooter.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.IShooterOI;
import frc.robot.subsystems.shooter.Shooter;


public class ShootAtAngleCommand extends CommandBase {
    private Shooter shooter;
    private IShooterOI oi;

    private double topFlywheelSpeed;
    private double bottomFlywheelSpeed;

    public ShootAtAngleCommand(Shooter shooter, IShooterOI oi) {
        this.shooter = shooter;
        this.oi = oi;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //TODO: add logic to determine flywheel speed from angle
        shooter.run(0,0);
    }

    @Override
    public boolean isFinished() {
        return !oi.getTurretActive();
    }

    @Override
    public void end(boolean interrupted) {

    }
}
