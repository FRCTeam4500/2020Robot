package frc.robot.containers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.Autonomous_IndexBallsCommand;
import frc.robot.components.hardware.LimelightVisionComponent;
import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.NetworkTableBallSensor;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.factory.EntropySwerveFactory;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.factory.HardwareTurretFactory;
import frc.robot.subsystems.turret.factory.ITurretFactory;
import frc.robot.subsystems.vision.VisionSubsystem;

import static frc.robot.utility.ExtendedMath.withHardDeadzone;

public class RobotContainerThree implements IRobotContainer {
    private Turret turret;
    private ITurretFactory factory;
    private VisionSubsystem vision;
    private Joystick joystick = new Joystick(0);
    




    private JoystickButton button1 = new JoystickButton(joystick, 1);
    private JoystickButton button10 = new JoystickButton(joystick , 10);
    private JoystickButton button2 = getButton(2), button3 = getButton(3), button4 = getButton(4), button5 = getButton(5), button6 = getButton (6), button7 = getButton (7), button8 = getButton(8), button9 = getButton(9), button11 = getButton(11), button12 = getButton(12);
    private Arm arm;
    private Intake intake;
    private Indexer indexer;
    private OdometricSwerve swerve = new EntropySwerveFactory().makeSwerve();

    private boolean armDown = false;

    public RobotContainerThree(){
        arm = new Arm(new TalonSRXComponent(8));
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

        button5.whenPressed(() -> indexer.setSpeed(1), indexer);
        button5.whenReleased(() -> indexer.setSpeed(0), indexer);
        button3.whenPressed(() -> indexer.setSpeed(-1),indexer);
        button3.whenReleased(() -> indexer.setSpeed(0),indexer);
        button6.whenPressed(() -> intake.setSpeed(1), intake);
        button6.whenReleased(() -> intake.setSpeed(0), intake);
        button4.whenPressed(() -> intake.setSpeed(-1), intake);
        button4.whenReleased(() -> intake.setSpeed(0),intake);



        button9.whenPressed(() -> swerve.resetPose(new Translation2d()),swerve);
        
        button7.whenPressed(() -> {
            if(armDown){
                armDown = false;
                arm.setAngle(0.0);
            }else{
                armDown = true;
                arm.setAngle(Math.PI/2);
            }
        },arm);

        button2.whileHeld(new Autonomous_IndexBallsCommand(indexer, intake, 1.0,0.9));
  
        swerve.setDefaultCommand(new FunctionalCommand(() -> swerve.enableWheelInversion(true),() -> {
            swerve.moveFieldCentric(
                withHardDeadzone(-joystick.getY(),0.3), 
                withHardDeadzone(-joystick.getX(),0.3), 
                withHardDeadzone(joystick.getZ(), 0.3));
        }, (interrupted) -> {
            swerve.enableWheelInversion(false);
            swerve.moveFieldCentric(0, 0, 0);
        }, () -> false,  swerve));


    }
    private JoystickButton getButton(int id){
        return new JoystickButton(joystick, id);
    }
    private void configureTurretAuton(){
        factory = new HardwareTurretFactory();
        turret = factory.makeTurret();
        vision = new VisionSubsystem(new LimelightVisionComponent());
        turret.setDefaultCommand(
            new PIDCommand(
                new PIDController(-2, 0, 0), 
                vision::getHorizontalOffset, 
                0, 
                turret::setTurretOutput, 
                turret, vision));
    }
}