package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.arm.ArmTwo;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.NetworkTableBallSensor;
import frc.robot.subsystems.indexer.command.IndexBallsCommand;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.factory.EntropySwerveFactory;
import static frc.robot.utility.ExtendedMath.withDeadzone;

public class RobotContainerTwo implements IRobotContainer {
    private Joystick joystick = new Joystick(0);
    private JoystickButton button1 = new JoystickButton(joystick, 1);
    private JoystickButton button10 = new JoystickButton(joystick , 10);
    private ArmTwo arm;
    private Intake intake;
    private Indexer indexer;
    private OdometricSwerve swerve = new EntropySwerveFactory().makeSwerve();
    public RobotContainerTwo(){
        arm = new ArmTwo();
        intake = new Intake(new TalonSRXComponent(5));
        
        indexer = new Indexer(
            new TalonSRXComponent(12), 
            new NetworkTableBallSensor("Sensor0", 40), 
            new NetworkTableBallSensor("Sensor1", 40), 
            new NetworkTableBallSensor("Sensor2", 40), 
            new NetworkTableBallSensor("Sensor3", 40), 
            new NetworkTableBallSensor("Sensor4", 40), 
            new NetworkTableBallSensor("Sensor5", 40)
        );

        // indexer.setDefaultCommand(new IndexBallsCommand(indexer, intake, 1));

        // button10.whenHeld(new RunCommand(() -> indexer.setSpeed(-1),indexer));
        // button1.whenPressed(new InstantCommand(() -> {
        //     arm.setAngle(-3600);
        //     intake.setSpeed(-1);
        // }, arm, intake));
        // button1.whenReleased(new InstantCommand(() -> {
        //     arm.setAngle(0);
        //     intake.setSpeed(0);
        // }, arm, intake));

        swerve.setDefaultCommand(new RunCommand(() -> {
            swerve.moveFieldCentric(
                withDeadzone(joystick.getY(),0.2), 
                withDeadzone(-joystick.getX(),0.2), 
                withDeadzone(joystick.getZ(), 0.2));
        }, swerve));
    }
}