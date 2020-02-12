package frc.robot.subsystems.climber.commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberMap;
import frc.robot.subsystems.climber.IClimberOI;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class ClimberMoveVerticalCommand extends CommandBase{
    private Climber climber;
    private IClimberOI oi;
    private double ArmSpeed;
    public ClimberMoveVerticalCommand(Climber climber, IClimberOI oi) {
        addRequirements(climber);
        this.climber = climber;
        this.ArmSpeed = ArmSpeed;
        this.oi = oi;

    }
    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        climber.setSpeed(this.oi.getClimberHeight());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

}