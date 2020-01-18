package frc.robot.autonomous.command;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.command.AutoTankDriveCommand;
import frc.robot.subsystems.tank.Tank;

public class AutoCommandGroup extends SequentialCommandGroup {
    public AutoCommandGroup(Tank tank){
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new FooCommand(), new BarCommand());
        super(new AutoTankDriveCommand(tank,-1,1),
                new WaitCommand(3),
                new AutoTankDriveCommand(tank,0,0),
                new WaitCommand(5),
                new AutoTankDriveCommand(tank,1,-1),
                new WaitCommand(3),
                new AutoTankDriveCommand(tank,0,0));
    }
}