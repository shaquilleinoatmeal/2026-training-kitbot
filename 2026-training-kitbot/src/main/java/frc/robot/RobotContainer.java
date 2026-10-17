package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  private static final double DEADBAND = 0.08;

  private final Drivetrain drivetrain = new Drivetrain();
  private final CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        Commands.run(
            () ->
                drivetrain.arcadeDrive(
                    MathUtil.applyDeadband(-driverController.getLeftY(), DEADBAND),
                    MathUtil.applyDeadband(-driverController.getRightX(), DEADBAND)),
            drivetrain));
  }

  public Command getAutonomousCommand() {
    return new DriveDistance(drivetrain, 120.0).withTimeout(DriveConstants.AUTO_TIMEOUT_SECONDS);
  }
}