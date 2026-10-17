package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends Command {
  private final Drivetrain drivetrain;
  private final double targetInches;
  private double startInches;
  private double startHeadingDegrees;
  private double lastSetpointVelocity;

  private final ProfiledPIDController distancePid =
      new ProfiledPIDController(
          DriveConstants.DISTANCE_KP,
          0.0,
          DriveConstants.DISTANCE_KD,
          new TrapezoidProfile.Constraints(
              DriveConstants.MAX_VELOCITY_IN_PER_SEC, DriveConstants.MAX_ACCEL_IN_PER_SEC2));
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          DriveConstants.KS_VOLTS / DriveConstants.VOLTAGE_COMPENSATION_VOLTS,
          DriveConstants.KV_VOLTS_PER_IN_PER_SEC / DriveConstants.VOLTAGE_COMPENSATION_VOLTS,
          DriveConstants.KA_VOLTS_PER_IN_PER_SEC2 / DriveConstants.VOLTAGE_COMPENSATION_VOLTS);

  public DriveDistance(Drivetrain drivetrain, double targetInches) {
    this.drivetrain = drivetrain;
    this.targetInches = targetInches;
    distancePid.setTolerance(
        DriveConstants.DISTANCE_TOLERANCE_INCHES, DriveConstants.VELOCITY_TOLERANCE_IN_PER_SEC);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    startInches = drivetrain.getAverageInches();
    startHeadingDegrees = drivetrain.getHeadingDegrees();
    distancePid.reset(0.0);
    lastSetpointVelocity = 0.0;
  }

  @Override
  public void execute() {
    double traveledInches = drivetrain.getAverageInches() - startInches;
    double pidOutput = distancePid.calculate(traveledInches, targetInches);
    TrapezoidProfile.State setpoint = distancePid.getSetpoint();

    double feedforwardOutput;
    if (setpoint.velocity != 0.0) {
      feedforwardOutput =
          feedforward.calculateWithVelocities(lastSetpointVelocity, setpoint.velocity);
    } else {
      double errorInches = targetInches - traveledInches;
      feedforwardOutput =
          Math.abs(errorInches) > DriveConstants.DISTANCE_TOLERANCE_INCHES
              ? feedforward.getKs() * Math.signum(errorInches)
              : 0.0;
    }
    lastSetpointVelocity = setpoint.velocity;

    double forward = pidOutput + feedforwardOutput;
    double correction =
        DriveConstants.HEADING_KP * (drivetrain.getHeadingDegrees() - startHeadingDegrees);
    drivetrain.tankDrive(forward + correction, forward - correction);

    SmartDashboard.putNumber("Auto/Setpoint inches", setpoint.position);
    SmartDashboard.putNumber("Auto/Setpoint in per sec", setpoint.velocity);
    SmartDashboard.putNumber("Auto/Traveled inches", traveledInches);
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