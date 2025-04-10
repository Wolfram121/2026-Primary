package frc.robot.utils;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Preferences;

public class WolfSparkMax extends SparkMax {
    protected double gearRatio = 1.0; // Default gear ratio
    protected double target;

    /**
     * @param deviceId  The device ID.
     * @param m         The this type (Brushed/Brushless).
     * @param mode      The idle mode (kBrake/kCoast).
     * @param limit     The current limit.
     * @param inverted  The invert type of the this.
     * @param kP        The proportional gain value.
     * @param kI        The integral gain value.
     * @param kD        The derivative gain value.
     * @param minOutput Reverse power minimum to allow the controller to output
     * @param maxOutput Reverse power maximum to allow the controller to output
     */
    public WolfSparkMax(int deviceId, MotorType m, IdleMode mode, int limit, boolean inverted) {
        super(deviceId, m);

        SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(inverted)
                .idleMode(mode)
                .smartCurrentLimit(limit);

        super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        String key = "Spark " + this.getDeviceId() + " Flashes";
        Preferences.setDouble(key, Preferences.getDouble(key, 0) + 1);
    }

    public WolfSparkMax(int deviceId, MotorType m, IdleMode mode, int limit, boolean inverted,
            double kP, double kI, double kD, double minOutput, double maxOutput) {
        super(deviceId, m);

        SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(inverted)
                .idleMode(mode)
                .smartCurrentLimit(limit);
        config.closedLoop.pid(kP, kI, kD);

        super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        String key = "Spark " + this.getDeviceId() + " Flashes";
        Preferences.setDouble(key, Preferences.getDouble(key, 0) + 1);
    }
    /**
     * Moves the motor to a target position using simple manual control.
     * This method is blocking and should NOT be used in competition code.
     *
     * @param target The desired position in mechanism units (pre-gear ratio).
     */
    public void Target(double target) {
        // Convert target from mechanism units to motor encoder units
        this.target = target * this.getGearRatio();

        // Run motor until it's within tolerance of the target
        double distanceRequired = Math.abs(this.target - this.getEncoder().getPosition());
        while (distanceRequired > 0.001) {
            double currentPosition = this.getEncoder().getPosition();
            double error = this.target - currentPosition;

            // Proportional speed, capped at 1
            double speed = Math.copySign(
                Math.min(1.0, Math.abs(error)),
                error
            );

            this.set(speed);
        }

        // Stop the motor once target is reached
        this.set(0);
    }

    public void setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
    }

    public double getGearRatio() {
        return this.gearRatio;
    }
}