package frc.robot.utils;

// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
// import HolonomicPathFollowerConfig;

public final class Constants {
  public static class ModuleIDs {
  }

  public static class SwerveIDs {
    public static final int LFD = 0;
    public static final int LFT = 1;
    public static final int LBD = 2;
    public static final int LBT = 3;
    public static final int RBD = 4;
    public static final int RBT = 5;
    public static final int RFD = 6;
    public static final int RFT = 7;
    public static final int PIGEON = 8;

    public static final int LF_CAN = 0;
    public static final int LB_CAN = 1;
    public static final int RB_CAN = 2;
    public static final int RF_CAN = 3;
  }

  public static class IOConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OP_CONTROLLER_PORT = 1;
  }

  public static final class SwerveConstants {
    public static final double LF_OFFSET = 0; // change
    public static final double RF_OFFSET = 0; // change
    public static final double LB_OFFSET = 0; // change
    public static final double RB_OFFSET = 0; // change

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double TURN_MOTOR_GEAR_RATIO = 150.0 / 7;
    public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
    public static final double TURN_MOTOR_PCONVERSION = 2 * Math.PI / TURN_MOTOR_GEAR_RATIO;
    public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;
    public static final double TURN_MOTOR_VCONVERSION = TURN_MOTOR_PCONVERSION / 60.0;
    public static final double KP_TURNING = 0.5;

    public static final double DRIVETRAIN_MAX_SPEED = 1;
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 0.75 * Math.PI;

    // Swerve Kinematics
    public static final double TRACK_WIDTH = Units.inchesToMeters(20.75);
    public static final double WHEEL_BASE = Units.inchesToMeters(20.75);
    public static final double DRIVE_BASE_RADIUS = Math.sqrt(Math.pow(TRACK_WIDTH, 2) + Math.pow(WHEEL_BASE, 2)) / 2.0;

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    // Teleop constraints
    public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 1;
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 1.75;
    public static final double TELE_DRIVE_MAX_ACCELERATION = 3;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 3;

    // Auton constraints
    public static final double AUTO_kP_TRANSLATION = 0.4;
    public static final double AUTO_kP_ROTATION = 2.4;
  }

  public final class ControllerConstants {
    public static final int AxisLeftStickX = 0;
    public static final int AxisLeftStickY = 1;
    public static final int AxisLeftTrigger = 2;
    public static final int AxisRightTrigger = 3;
    public static final int AxisRightStickX = 4;
    public static final int AxisRightStickY = 5;

    // Gamepad Trigger Threshold
    public static final double TriggerThreshold = 0.1;

    // Gamepad POVs
    public static final int PovUp = 0;
    public static final int PovRight = 90;
    public static final int PovDown = 180;
    public static final int PovLeft = 270;

    // Gamepad buttons
    public static final int ButtonA = 1; // Bottom Button
    public static final int ButtonB = 2; // Right Button
    public static final int ButtonX = 3; // Left Button
    public static final int ButtonY = 4; // Top Button
    public static final int ButtonShoulderL = 5;
    public static final int ButtonShoulderR = 6;
    public static final int ButtonBack = 7;
    public static final int ButtonStart = 8;
    public static final int ButtonLeftStick = 9;
    public static final int ButtonRightStick = 10;
  }

  public static final class JoystickConstants {
    public static final int Y = 0;
    public static final int X = 1;
    public static final int Rot = 2;
    public static final int Slider = 3;

    public static final int PovUp = 0;
    public static final int PovRight = 90;
    public static final int PovDown = 180;
    public static final int PovLeft = 270;

    public static final int Trigger = 1;
    public static final int Side = 2;
    public static final int LB = 3;
    public static final int RB = 4;
    public static final int LF = 5;
    public static final int RF = 6;
    public static final int BaseLF = 7;
    public static final int BaseRF = 8;
    public static final int BaseLM = 9;
    public static final int BaseRM = 10;
    public static final int BaseLB = 11;
    public static final int BaseRB = 12;
  }
}