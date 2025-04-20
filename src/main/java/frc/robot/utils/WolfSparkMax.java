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
        System.out.println("" + deviceId);

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
                .idleMode(mode);
        config.closedLoop.pid(kP, kI, kD);

        super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        String key = "Spark " + this.getDeviceId() + " Flashes";
        Preferences.setDouble(key, Preferences.getDouble(key, 0) + 1);
    }

    public void Target(double target) {

        while (Math.abs(target * this.GearRatio() - this.getEncoder().getPosition()) > 0.001) {
            this.set(Math.copySign(Math.min(1, Math.abs(target * this.GearRatio() - this.getEncoder().getPosition())), target * this.GearRatio() - this.getEncoder().getPosition()));
        }
        this.set(0);
    }

    public void GearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
    }

    public double GearRatio() {
        return this.gearRatio;
    }
}