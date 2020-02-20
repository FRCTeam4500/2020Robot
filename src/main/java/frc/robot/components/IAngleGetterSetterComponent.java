package frc.robot.components;

public interface IAngleGetterSetterComponent extends IAngleGetterComponent, IAngleSetterComponent {
    double getAngle();
    void setAngle(double angle);
}
