package frc.robot.subsystems.swerve.normal.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.ISwerveOI;
import frc.robot.subsystems.swerve.normal.NormalSwerve;


public class MoveFieldCentricCommand extends CommandBase {
    private final ISwerve swerve;
    private ISwerveOI oi;

    public MoveFieldCentricCommand(ISwerve swerve, ISwerveOI oi) {
        this.swerve = swerve;
        this.oi = oi;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        swerve.moveFieldCentric(oi.getX(), oi.getY(), oi.getZ());
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
