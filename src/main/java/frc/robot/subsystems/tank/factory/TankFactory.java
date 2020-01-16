
package frc.robot.subsystems.tank.factory;

import frc.robot.components.hardware.VictorSPComponent;
import frc.robot.subsystems.tank.Tank;

public class TankFactory{
    public Tank makeTank(){
        var l1 = new VictorSPComponent(2);
        var l2 = new VictorSPComponent(3);
        var r1 = new VictorSPComponent(0);
        var r2 = new VictorSPComponent(1);

        return new Tank(l1, l2, r1, r2);
    }
}