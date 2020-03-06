package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.components.hardware.CameraVisionComponent;
import frc.robot.subsystems.swerve.kinematic.KinematicSwerve;
import frc.robot.subsystems.vision.CameraVisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;


public class TrackLoadingCommand extends CommandBase {
    private final KinematicSwerve kinematicSwerve;
    private final CameraVisionSubsystem vision;
    private PIDController pid, pid2;
    private double offset;
    private double angleOffset;

    public TrackLoadingCommand(KinematicSwerve swerve, CameraVisionSubsystem vision) {

        SmartDashboard.putNumber("KpOffset",1);
        SmartDashboard.putNumber("KiOffset",0);
        SmartDashboard.putNumber("KdOffset",0);
        SmartDashboard.putNumber("KpAngle",1);
        SmartDashboard.putNumber("KiAngle",1);
        SmartDashboard.putNumber("KdAngle",1);

        this.kinematicSwerve = swerve;
        this.vision = vision;
        this.pid = new PIDController(SmartDashboard.getNumber("KpOffset",0),SmartDashboard.getNumber("KiOffset",0),SmartDashboard.getNumber("KdOffset",0));
        this.pid2 = new PIDController(SmartDashboard.getNumber("KpAngle",0),SmartDashboard.getNumber("KiAngle",0),SmartDashboard.getNumber("KdAngle",0));

        addRequirements(kinematicSwerve, vision);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        offset = vision.getHorizontalOffset();
        offset = pid.calculate(offset);
        offset = setBounds(offset);
        pid.setSetpoint(offset);
        angleOffset = vision.getAngleX();
        angleOffset = pid2.calculate(angleOffset);
        angleOffset = setBounds(angleOffset);
        kinematicSwerve.moveRobotCentric(offset,0,-angleOffset);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

    private double setBounds(double offset){
        if (offset > 1){
            offset = 1;
        }
        else if (offset < -1){
            offset = -1;
        }
        return offset;
    }
}
