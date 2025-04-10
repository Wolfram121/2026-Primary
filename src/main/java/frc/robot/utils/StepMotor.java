package frc.robot.utils;

public class StepMotor {
    private final VikingSparkMax_TBD motor;
    private double targetPosition;
    private final double kP = 0.1; // Tune this to your robot

    public StepMotor(VikingSparkMax_TBD motor, double target) {
        this.motor = motor;
        setTarget(target);
    }

    public void setTarget(double target) {
        // Convert real-world target to encoder units
        this.targetPosition = target * motor.getGearRatio();
    }

    public void update() {
        double currentPosition = motor.getEncoder().getPosition();
        double error = targetPosition - currentPosition;

        // Stop the motor if it's close enough
        if (Math.abs(error) < 0.01) {
            motor.set(0);
        } else {
            double output = kP * error;

            // Clamp the output to between -1 and 1
            output = Math.max(-1.0, Math.min(1.0, output));

            motor.set(output);
        }
    }

    // Optional: check if the motor is at the target
    public boolean atTarget() {
        return Math.abs(targetPosition - motor.getEncoder().getPosition()) < 0.01;
    }
}