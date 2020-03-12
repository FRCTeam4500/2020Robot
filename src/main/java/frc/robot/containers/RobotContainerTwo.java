package frc.robot.containers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.GenericAutonUtilities;
import frc.robot.autonomous.Autonomous_IndexBallsCommand;
import frc.robot.components.hardware.LimelightVisionComponent;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.factory.HardwareIntakeFactory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.factory.HardwareArmFactory;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.factory.HardwareIndexerFactory;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.factory.HardwareShooterFactory;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.factory.EntropySwerveFactory;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.factory.HardwareTurretFactory;
import frc.robot.subsystems.turret.factory.ITurretFactory;
import frc.robot.subsystems.vision.VisionSubsystem;

import static frc.robot.utility.ExtendedMath.withHardDeadzone;

public class RobotContainerTwo implements IRobotContainer {
    private Turret turret;
    private ITurretFactory factory;
    private VisionSubsystem vision;
    private Joystick joystick = new Joystick(0);
    private JoystickButton button1 = new JoystickButton(joystick, 1);
    private JoystickButton button10 = new JoystickButton(joystick , 10);
    private Joystick joystick2 = new Joystick(1);
    private JoystickButton j2button1 = new JoystickButton(joystick2, 1);
    private JoystickButton j2button2 = new JoystickButton(joystick2, 2);
    private JoystickButton j2button3 = new JoystickButton(joystick2, 3);
    private JoystickButton j2button4 = new JoystickButton(joystick2, 4);
    private JoystickButton j2button5 = new JoystickButton(joystick2, 5);
    private JoystickButton j2button6 = new JoystickButton(joystick2, 6);
    private JoystickButton j2button7 = new JoystickButton(joystick2, 7);
    private JoystickButton j2button8 = new JoystickButton(joystick2, 8);
    private JoystickButton j2button9 = new JoystickButton(joystick2, 9);
    private JoystickButton j2button10 = new JoystickButton(joystick2, 10);
    private JoystickButton j2button11 = new JoystickButton(joystick2, 11);
    private JoystickButton j2button12 = new JoystickButton(joystick2, 12);
    private JoystickButton button2 = getButton(2), button3 = getButton(3), button4 = getButton(4), button5 = getButton(5), button6 = getButton (6), button7 = getButton (7), button8 = getButton(8), button9 = getButton(9), button11 = getButton(11), button12 = getButton(12);
    private Arm arm;
    private Intake intake;
    private Indexer indexer;
    private OdometricSwerve swerve = new EntropySwerveFactory().makeSwerve();
    private Shooter shooter;
    private Climber climber;

    private boolean armDown = false;

    public RobotContainerTwo(){
        arm = new HardwareArmFactory().makeArm();
        intake = new HardwareIntakeFactory().makeIntake();
        shooter = new HardwareShooterFactory().makeShooter();
        // climber = new Climber(new );

        // j2button11.whenPressed(() -> climber.setSpeed(1.0),climber);
        // j2button11.whenReleased(() -> climber.setSpeed(0.0),climber);

        // j2button9.whenPressed(() -> climber.setSpeed(-1.0),climber);
        // j2button9.whenReleased(() -> climber.setSpeed(0.0,climber));


        indexer = new HardwareIndexerFactory().makeIndexer();

        button6.whenPressed(() -> indexer.setSpeed(1), indexer);
        button6.whenReleased(() -> indexer.setSpeed(0), indexer);

        button4.whenPressed(() -> indexer.setSpeed(-1),indexer);
        button4.whenReleased(() -> indexer.setSpeed(0),indexer);

        button5.whenPressed(() -> intake.setSpeed(1), intake);
        button5.whenReleased(() -> intake.setSpeed(0), intake);
        button3.whenPressed(() -> intake.setSpeed(-1), intake);
        button3.whenReleased(() -> intake.setSpeed(0),intake);
        
        j2button1.whenHeld(new Autonomous_IndexBallsCommand(indexer, intake, 1, 0.9));
        j2button1.whenPressed(() -> arm.setAngle(Math.PI/2.5), arm);
        j2button1.whenReleased(() -> arm.setAngle(0.0),arm);

        j2button6.whenPressed(() -> indexer.setSpeed(1.0),indexer);
        j2button6.whenReleased(() -> indexer.setSpeed(0.0),indexer);
        j2button4.whenPressed(() -> indexer.setSpeed(-1.0), indexer);
        j2button4.whenPressed(() -> indexer.setSpeed(0.0),indexer);
        
        j2button2.whenPressed(() -> {
            indexer.setSpeed(1.0);
            shooter.run(-3300 * 0.8, -2800 * 0.8);
        }, indexer,shooter);

        j2button5.whenPressed(() -> intake.setSpeed(1), intake);//take ball in
        j2button5.whenReleased(() -> intake.setSpeed(0), intake);

        j2button3.whenPressed(() -> intake.setSpeed(-1), intake);
        j2button3.whenPressed(() -> intake.setSpeed(0),intake);


        button9.whenPressed(() -> swerve.resetPose(new Translation2d()),swerve);
        
        /*button7.whenPressed(() -> {
            if(armDown){
                armDown = false;
                arm.setAngle(0.0);
            }else{

                armDown = true;
                arm.setAngle(Math.PI/3);
            }sdxza
        },arm);*/
        button7.whenPressed(() -> arm.setAngle(Math.PI/2.5), arm);
        button7.whenReleased(() -> arm.setAngle(0.0),arm);

        //button2.whileHeld(new IndexBallsCommand(indexer, intake, 1));
        button2.whenPressed(() -> {
            intake.setSpeed(-1);
            indexer.setSpeed(1);
        }, intake, indexer);
  
        swerve.setDefaultCommand(new FunctionalCommand(() -> swerve.enableWheelInversion(true),() -> {
            swerve.moveFieldCentric(
                withHardDeadzone(-joystick.getY()*1.5,0.3), 
                withHardDeadzone(-joystick.getX()*1.5,0.3), 
                withHardDeadzone(-joystick.getZ()*2.5, 0.3*2));
        }, (interrupted) -> {

            swerve.enableWheelInversion(false);
            swerve.moveFieldCentric(0, 0, 0);
        }, () -> false,  swerve));

        configureTurretAuton();


        SmartDashboard.putData("Swerve Distance Calculator", GenericAutonUtilities.makeEntropyVisionDistanceCalculator(vision));


    }
    private JoystickButton getButton(int id){
        return new JoystickButton(joystick, id);
    }
    private JoystickButton getButton2(int id) {
        return new JoystickButton(joystick2, id);
    }
    private void configureTurretAuton(){
        factory = new HardwareTurretFactory();
        turret = factory.makeTurret();
        vision = new VisionSubsystem(new LimelightVisionComponent());
        SmartDashboard.putNumber("vision offset", 0.0);
        turret.setDefaultCommand(
            new PIDCommand(
                new PIDController(-6, 0, 0), 
                vision::getHorizontalOffset, 
                () -> Units.degreesToRadians(SmartDashboard.getNumber("vision offset", 0.0)), 
                turret::setTurretOutput, 
                turret, vision));
    }
}