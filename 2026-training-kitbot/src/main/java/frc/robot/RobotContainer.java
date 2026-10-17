package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  private static final double DEADBAND = 0.08;

  private final Drivetrain drivetrain = new Drivetrain();
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    configureAutos();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        Commands.run(
            () ->
                drivetrain.arcadeDrive(
                    MathUtil.applyDeadband(-driverController.getLeftY(), DEADBAND),
                    MathUtil.applyDeadband(-driverController.getRightX(), DEADBAND),
                    driverController.getHID().getRightBumperButton()
                        ? 1.0
                        : DriveConstants.TELEOP_MAX_OUTPUT),
            drivetrain));
  }

  private void configureAutos() {
    autoChooser.setDefaultOption(
        "Drive 10 ft",
        new DriveDistance(drivetrain, 120.0).withTimeout(DriveConstants.AUTO_TIMEOUT_SECONDS));
    autoChooser.addOption("Do nothing", Commands.none());
    SmartDashboard.putData("Auto", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
