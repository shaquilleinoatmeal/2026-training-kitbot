package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;

public class Drivetrain extends SubsystemBase {
  private final SparkMax leftLeader =
      new SparkMax(DriveConstants.LEFT_LEADER_ID, MotorType.kBrushed);
  private final SparkMax leftFollower =
      new SparkMax(DriveConstants.LEFT_FOLLOWER_ID, MotorType.kBrushed);
  private final SparkMax rightLeader =
      new SparkMax(DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushed);
  private final SparkMax rightFollower =
      new SparkMax(DriveConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushed);

  private final DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);

  private final Pigeon2 pigeon = new Pigeon2(DriveConstants.PIGEON_ID);

  private final Encoder leftEncoder =
      new Encoder(
          DriveConstants.LEFT_ENCODER_PORTS[0],
          DriveConstants.LEFT_ENCODER_PORTS[1],
          DriveConstants.LEFT_ENCODER_REVERSED);
  private final Encoder rightEncoder =
      new Encoder(
          DriveConstants.RIGHT_ENCODER_PORTS[0],
          DriveConstants.RIGHT_ENCODER_PORTS[1],
          DriveConstants.RIGHT_ENCODER_REVERSED);

  public Drivetrain() {
    configureLeader(leftLeader, false);
    configureLeader(rightLeader, DriveConstants.RIGHT_SIDE_INVERTED);
    configureFollower(leftFollower, leftLeader);
    configureFollower(rightFollower, rightLeader);

    leftEncoder.setDistancePerPulse(DriveConstants.INCHES_PER_CYCLE);
    rightEncoder.setDistancePerPulse(DriveConstants.INCHES_PER_CYCLE);
    resetEncoders();
  }

  private static void configureLeader(SparkMax motor, boolean inverted) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(inverted);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(DriveConstants.CURRENT_LIMIT_AMPS);
    config.voltageCompensation(DriveConstants.VOLTAGE_COMPENSATION_VOLTS);
    config.openLoopRampRate(DriveConstants.OPEN_LOOP_RAMP_SECONDS);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private static void configureFollower(SparkMax motor, SparkMax leader) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(DriveConstants.CURRENT_LIMIT_AMPS);
    config.voltageCompensation(DriveConstants.VOLTAGE_COMPENSATION_VOLTS);
    config.openLoopRampRate(DriveConstants.OPEN_LOOP_RAMP_SECONDS);
    config.follow(leader, DriveConstants.FOLLOWER_OPPOSES_LEADER);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void arcadeDrive(double forward, double rotation) {
    drive.arcadeDrive(forward, rotation);
  }

  public void tankDrive(double left, double right) {
    drive.tankDrive(left, right, false);
  }

  public void stop() {
    drive.stopMotor();
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public double getLeftInches() {
    return leftEncoder.getDistance();
  }

  public double getRightInches() {
    return rightEncoder.getDistance();
  }

  public double getAverageInches() {
    return (getLeftInches() + getRightInches()) / 2.0;
  }

  public double getHeadingDegrees() {
    return pigeon.getYaw().getValueAsDouble();
  }

  public void resetHeading() {
    pigeon.setYaw(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drive/Left inches", getLeftInches());
    SmartDashboard.putNumber("Drive/Right inches", getRightInches());
    SmartDashboard.putNumber("Drive/Average inches", getAverageInches());
    SmartDashboard.putNumber("Drive/Heading deg", getHeadingDegrees());
  }
}