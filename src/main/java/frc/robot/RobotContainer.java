package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.subsystems.*;
import frc.robot.utils.Constants;
import frc.robot.commands.*;
import frc.robot.controls.SwerveDriveControls;

public class RobotContainer {
  public static final Joystick controller3D = new Joystick(0);
  public static final JoystickButton resetHeading_Start = new JoystickButton(controller3D, Constants.JoystickConstants.BaseLF);
  // public static ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private SwerveDriveCommands swerveStopCmd;

  SendableChooser<Command> auton_chooser;

  //private final Command middleCommand = new MiddleStartAuto();

  public RobotContainer() {
    CameraServer.startAutomaticCapture();
    drivetrain.setDefaultCommand(new SwerveDriveControls());
    configureBindings();
    auton_chooser = new SendableChooser<>();
    swerveStopCmd = new SwerveDriveCommands(0, 0, 0, true);
    NamedCommands.registerCommand("Swerve Stop", swerveStopCmd);

    /** auton_chooser.setDefaultOption("Leave Start Position", nonSpeakerCommand);
    auton_chooser.addOption("Left Start", leftCommand);
    auton_chooser.addOption("Middle Start", middleCommand);
    auton_chooser.addOption("Right Auto", rightCommand); **/

    SmartDashboard.putData("Auton Chooser", auton_chooser);
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));
    drivetrain.setDefaultCommand(new SwerveDriveControls());
    resetHeading_Start.onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));
    // exampleSubsystem (raw subsys btw).setDefaultCommand(new exSubsysCommands(arm, controller3D)); 
  }

  public Command getAutonomousCommand() {
    return auton_chooser.getSelected();
  }
}
