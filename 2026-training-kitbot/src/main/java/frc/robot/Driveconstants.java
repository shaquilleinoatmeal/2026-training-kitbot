package frc.robot;

/** Drivetrain constants. Everything marked CHECK needs to match your actual hardware. */
public final class DriveConstants {
  private DriveConstants() {}

  // CAN IDs, set in the REV Hardware Client (must be unique)
  public static final int LEFT_LEADER_ID = 1;
  public static final int LEFT_FOLLOWER_ID = 2;
  public static final int RIGHT_LEADER_ID = 3;
  public static final int RIGHT_FOLLOWER_ID = 4;
  // Pigeon 2.0 CAN ID, set in Phoenix Tuner X
  public static final int PIGEON_ID = 5;

  // Motor settings
  public static final int CURRENT_LIMIT_AMPS = 40;
  public static final boolean RIGHT_SIDE_INVERTED = true;
  // CHECK: if the two CIMs on one gearbox fight each other, flip this.
  public static final boolean FOLLOWER_OPPOSES_LEADER = false;

  // Encoders on roboRIO DIO ports (A channel, B channel)
  public static final int[] LEFT_ENCODER_PORTS = {0, 1};
  public static final int[] RIGHT_ENCODER_PORTS = {2, 3};
  // CHECK: flip so both sides count positive when driving forward.
  public static final boolean LEFT_ENCODER_REVERSED = false;
  public static final boolean RIGHT_ENCODER_REVERSED = true;

  // CHECK: counts per revolution of your REV Through Bore Encoder (see REV's docs).
  // WPILib's Encoder counts all edges by default, so use the counts-per-rev figure, not pulses.
  // Verify it: turn the wheel exactly one rotation by hand and read the raw count.
  public static final double ENCODER_COUNTS_PER_REV = 2048;
  // CHECK: encoder shaft turns per wheel turn (1.0 if mounted on the wheel axle/output shaft).
  public static final double ENCODER_TURNS_PER_WHEEL_TURN = 1.0;
  public static final double WHEEL_DIAMETER_INCHES = 6.0;
  // Tune this by driving a measured distance, then: new = old * (actual / reported).
  public static final double CALIBRATION_FACTOR = 1.0;

  public static final double INCHES_PER_COUNT =
      (Math.PI * WHEEL_DIAMETER_INCHES)
          / (ENCODER_COUNTS_PER_REV * ENCODER_TURNS_PER_WHEEL_TURN)
          * CALIBRATION_FACTOR;

  // Auton tuning (starting values, adjust on the real robot)
  public static final double DISTANCE_KP = 0.02;
  public static final double DISTANCE_KD = 0.002;

  // Motion profile limits (inches/sec and inches/sec^2). Start low, raise once it's reliable.
  public static final double MAX_VELOCITY_IN_PER_SEC = 50.0;
  public static final double MAX_ACCEL_IN_PER_SEC2 = 50.0;
  // Feedforward in motor-output units (-1..1). Measure these, see the notes in the chat.
  public static final double KS = 0.06; // output needed just to start moving
  public static final double KV = 0.007; // output per inch/sec of speed (about 1 / free speed)
  // Motor output per degree of Pigeon yaw error
  public static final double HEADING_KP = 0.02;
  public static final double AUTO_MAX_SPEED = 0.5;
  public static final double DISTANCE_TOLERANCE_INCHES = 1.0;
  public static final double VELOCITY_TOLERANCE_IN_PER_SEC = 2.0;
}