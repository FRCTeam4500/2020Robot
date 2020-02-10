/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.indexer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Add your docs here.
 */
public class NetworkTableBallSensor implements IBallSensor{
    private double threshold;
    private String name;
    public NetworkTableBallSensor(String name, double threshold){
        this.name = name;
        this.threshold = threshold;
    }
    public boolean registersBall(){
        return SmartDashboard.getNumber(name, Double.POSITIVE_INFINITY) < threshold;
    }
}
