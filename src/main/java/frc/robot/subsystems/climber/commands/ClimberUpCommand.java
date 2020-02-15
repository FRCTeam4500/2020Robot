package frc.robot.subsystems.climber.commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberMap;
import frc.robot.subsystems.climber.IClimberOI;
import frc.robot.RobotContainer;
import frc.robot.components.IAngleSetterComponent;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class ClimberUpCommand extends CommandBase{
    private Climber climber;
    IAngleSetterComponent motor1;
    private IClimberOI oi;
    private double ArmSpeed;
    public ClimberUpCommand(Climber climber, IClimberOI oi) {
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
        motor1.setAngle(this.oi.getClimberHeight());
       
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

}