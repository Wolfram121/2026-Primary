package frc.robot.utils;

public class StepMotor {
    private WolfSparkMax motor;
    private double target;

    public StepMotor(WolfSparkMax motor, double target) {
        this.motor = motor;
        this.Target(target);
    }

    public void Target (double target) {
        this.target = target;
        if (motor.getEncoder().getPosition() != this.target * motor.GearRatio()) {
            motor.set(1/(this.target * motor.GearRatio() - motor.getEncoder().getPosition()));
        } else {
            motor.set(0);
        }
    }
}