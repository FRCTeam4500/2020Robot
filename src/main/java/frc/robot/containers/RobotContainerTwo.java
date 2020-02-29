package frc.robot.containers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.Autonomous_PreciseShootingCommand;
import frc.robot.autonomous.VisionDistanceCalculator;
import frc.robot.components.hardware.LimelightVisionComponent;
import frc.robot.components.hardware.SparkMaxComponent;
import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.NetworkTableBallSensor;
import frc.robot.subsystems.indexer.command.IndexBallsCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.command.ShootStraightCommand;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.factory.EntropySwerveFactory;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.factory.DefaultTurretFactory;
import frc.robot.subsystems.turret.factory.ITurretFactory;
import frc.robot.subsystems.vision.VisionSubsystem;

import static frc.robot.utility.ExtendedMath.withDeadzone;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
        arm = new Arm(new TalonSRXComponent(8));
        intake = new Intake(new TalonSRXComponent(5));
        shooter = new Shooter(
            new SparkMaxComponent(14, MotorType.kBrushless), 
        new SparkMaxComponent(13, MotorType.kBrushless));
        // climber = new Climber(new );

        // j2button11.whenPressed(() -> climber.setSpeed(1.0),climber);
        // j2button11.whenReleased(() -> climber.setSpeed(0.0),climber);

        // j2button9.whenPressed(() -> climber.setSpeed(-1.0),climber);
        // j2button9.whenReleased(() -> climber.setSpeed(0.0,climber));


        indexer = new Indexer(
            new TalonSRXComponent(12), 
            new NetworkTableBallSensor("Sensor0", 40), 
            new NetworkTableBallSensor("Sensor1", 40), 
            new NetworkTableBallSensor("Sensor2", 40), 
            new NetworkTableBallSensor("Sensor3", 40), 
            new NetworkTableBallSensor("Sensor4", 40), 
            new NetworkTableBallSensor("Sensor5", 40)
        );

        button6.whenPressed(() -> indexer.setSpeed(1), indexer);
        button6.whenReleased(() -> indexer.setSpeed(0), indexer);

        button4.whenPressed(() -> indexer.setSpeed(-1),indexer);
        button4.whenReleased(() -> indexer.setSpeed(0),indexer);

        button5.whenPressed(() -> intake.setSpeed(1), intake);
        button5.whenReleased(() -> intake.setSpeed(0), intake);
        button3.whenPressed(() -> intake.setSpeed(-1), intake);
        button3.whenReleased(() -> intake.setSpeed(0),intake);
        
        j2button1.whenHeld(new IndexBallsCommand(indexer, intake, 1));
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
                withDeadzone(-joystick.getY()*1.5,0.3), 
                withDeadzone(-joystick.getX()*1.5,0.3), 
                withDeadzone(-joystick.getZ()*2.5, 0.3*2));
        }, (interrupted) -> {

            swerve.enableWheelInversion(false);
            swerve.moveFieldCentric(0, 0, 0);
        }, () -> false,  swerve));

        configureTurretAuton();

        var preciseShooting = new Autonomous_PreciseShootingCommand(shooter, indexer);
        preciseShooting.createSmartDashboardEntries();
        button1.whenHeld(preciseShooting);

        var calculator = new VisionDistanceCalculator(
            Math.atan(Units.feetToMeters(7.5)-Units.inchesToMeters(34.5)/Units.feetToMeters(11.66))+Units.degreesToRadians(3.28), 
            Units.inchesToMeters(34.5), 
            Units.feetToMeters(7.5), 
            new LimelightVisionComponent());
        SmartDashboard.putData("Swerve Distance Calculator", calculator);


    }
    private JoystickButton getButton(int id){
        return new JoystickButton(joystick, id);
    }
    private JoystickButton getButton2(int id) {
        return new JoystickButton(joystick2, id);
    }
    private void configureTurretAuton(){
        factory = new DefaultTurretFactory();
        turret = factory.makeTurret();
        vision = new VisionSubsystem(new LimelightVisionComponent());
        SmartDashboard.putNumber("vision offset", 3.4);
        turret.setDefaultCommand(
            new PIDCommand(
                new PIDController(-6, 0.15, 0), 
                vision::getHorizontalOffset, 
                () -> Units.degreesToRadians(SmartDashboard.getNumber("vision offset", 0.0)), 
                turret::setTurretOutput, 
                turret, vision));
    }
    public void onDisabled(){
        swerve.enableWheelInversion(false);
        swerve.moveFieldCentric(0, 0, 0);
    }
}