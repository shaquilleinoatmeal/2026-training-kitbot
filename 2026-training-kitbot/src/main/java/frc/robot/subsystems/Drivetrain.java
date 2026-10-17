package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

  private double leftOutput;
  private double rightOutput;
  private final DifferentialDrive drive =
      new DifferentialDrive(
          output -> {
            leftOutput = output;
            leftLeader.set(output);
          },
          output -> {
            rightOutput = output;
            rightLeader.set(output);
          });

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

  private final DifferentialDriveOdometry odometry;
  private final Field2d field = new Field2d();

  private DifferentialDrivetrainSim driveSim;
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;

  public Drivetrain() {
    configureLeader(leftLeader, false);
    configureLeader(rightLeader, DriveConstants.RIGHT_SIDE_INVERTED);
    configureFollower(leftFollower, leftLeader);
    configureFollower(rightFollower, rightLeader);
    drive.setDeadband(0.0);

    leftEncoder.setDistancePerPulse(DriveConstants.INCHES_PER_CYCLE);
    rightEncoder.setDistancePerPulse(DriveConstants.INCHES_PER_CYCLE);
    resetEncoders();

    odometry =
        new DifferentialDriveOdometry(
            pigeon.getRotation2d(),
            Units.inchesToMeters(getLeftInches()),
            Units.inchesToMeters(getRightInches()));
    SmartDashboard.putData("Field", field);

    if (RobotBase.isSimulation()) {
      driveSim =
          DifferentialDrivetrainSim.createKitbotSim(
              KitbotMotor.kDualCIMPerSide, KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);
      leftEncoderSim = new EncoderSim(leftEncoder);
      rightEncoderSim = new EncoderSim(rightEncoder);
    }
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

  public void arcadeDrive(double forward, double rotation, double maxOutput) {
    drive.setMaxOutput(maxOutput);
    drive.arcadeDrive(forward, rotation);
  }

  public void tankDrive(double left, double right) {
    drive.setMaxOutput(1.0);
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
    odometry.update(
        pigeon.getRotation2d(),
        Units.inchesToMeters(getLeftInches()),
        Units.inchesToMeters(getRightInches()));
    field.setRobotPose(odometry.getPoseMeters());

    SmartDashboard.putNumber("Drive/Left inches", getLeftInches());
    SmartDashboard.putNumber("Drive/Right inches", getRightInches());
    SmartDashboard.putNumber("Drive/Average inches", getAverageInches());
    SmartDashboard.putNumber("Drive/Heading deg", getHeadingDegrees());
  }

  @Override
  public void simulationPeriodic() {
    double volts = DriveConstants.VOLTAGE_COMPENSATION_VOLTS;
    driveSim.setInputs(leftOutput * volts, rightOutput * volts);
    driveSim.update(0.02);

    leftEncoderSim.setDistance(Units.metersToInches(driveSim.getLeftPositionMeters()));
    rightEncoderSim.setDistance(Units.metersToInches(driveSim.getRightPositionMeters()));
    pigeon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    pigeon.getSimState().setRawYaw(driveSim.getHeading().getDegrees());
  }
}
