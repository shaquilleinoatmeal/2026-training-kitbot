package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends Command {
  private final Drivetrain drivetrain;
  private final double targetInches;

  private final ProfiledPIDController distancePid =
      new ProfiledPIDController(
          DriveConstants.DISTANCE_KP,
          0.0,
          DriveConstants.DISTANCE_KD,
          new TrapezoidProfile.Constraints(
              DriveConstants.MAX_VELOCITY_IN_PER_SEC, DriveConstants.MAX_ACCEL_IN_PER_SEC2));

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
    distancePid.reset(drivetrain.getAverageInches());
  }

  @Override
  public void execute() {
    double pidOutput = distancePid.calculate(drivetrain.getAverageInches(), targetInches);
    double velocitySetpoint = distancePid.getSetpoint().velocity;
    double feedforward =
        DriveConstants.KS * Math.signum(velocitySetpoint) + DriveConstants.KV * velocitySetpoint;
    double forward =
        MathUtil.clamp(
            pidOutput + feedforward,
            -DriveConstants.AUTO_MAX_SPEED,
            DriveConstants.AUTO_MAX_SPEED);

    double correction = DriveConstants.HEADING_KP * drivetrain.getHeadingDegrees();

    drivetrain.tankDrive(forward + correction, forward - correction);
  }

  @Override
  public boolean isFinished() {
    return distancePid.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}