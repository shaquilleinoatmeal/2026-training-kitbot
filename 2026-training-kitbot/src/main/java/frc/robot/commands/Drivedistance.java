package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Drives straight a set distance using encoder feedback.
 * Distance PID on the average of both encoders, plus a Pigeon 2 correction that holds the
 * heading the robot started with.
 */
public class DriveDistance extends Command {
  private final Drivetrain drivetrain;
  private final double targetInches;

  private final PIDController distancePid =
      new PIDController(DriveConstants.DISTANCE_KP, 0.0, DriveConstants.DISTANCE_KD);

  public DriveDistance(Drivetrain drivetrain, double targetInches) {
    this.drivetrain = drivetrain;
    this.targetInches = targetInches;
    distancePid.setTolerance(
        DriveConstants.DISTANCE_TOLERANCE_INCHES, DriveConstants.VELOCITY_TOLERANCE_IN_PER_SEC);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.resetEncoders();
    drivetrain.resetHeading();
    distancePid.reset();
  }

  @Override
  public void execute() {
    double forward =
        MathUtil.clamp(
            distancePid.calculate(drivetrain.getAverageInches(), targetInches),
            -DriveConstants.AUTO_MAX_SPEED,
            DriveConstants.AUTO_MAX_SPEED);

    // Yaw is counter-clockwise positive. If the robot has drifted left (yaw > 0),
    // speed up the left side and slow the right to steer back toward straight.
    double correction = DriveConstants.HEADING_KP * drivetrain.getHeadingDegrees();

    drivetrain.tankDrive(forward + correction, forward - correction);
  }

  @Override
  public boolean isFinished() {
    return distancePid.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}