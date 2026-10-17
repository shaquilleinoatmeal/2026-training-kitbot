package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.Drivetrain;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveDistanceSimTest {
  private static final double DT = 0.02;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    SimHooks.resumeTiming();
  }

  @Test
  void drivesTenFeetStraightBeforeTimeout() {
    Drivetrain drivetrain = new Drivetrain();
    Command auto = new DriveDistance(drivetrain, 120.0);
    CommandScheduler.getInstance().schedule(auto);

    double elapsed = 0.0;
    while (auto.isScheduled() && elapsed < DriveConstants.AUTO_TIMEOUT_SECONDS) {
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(DT);
      elapsed += DT;
    }

    System.out.printf(
        "Auton finished in %.2f s at %.2f in (L %.2f, R %.2f), heading %.2f deg%n",
        elapsed,
        drivetrain.getAverageInches(),
        drivetrain.getLeftInches(),
        drivetrain.getRightInches(),
        drivetrain.getHeadingDegrees());
    assertFalse(auto.isScheduled(), "auton never reached its goal before the timeout");
    assertEquals(120.0, drivetrain.getAverageInches(), 2.0);
    assertEquals(0.0, drivetrain.getHeadingDegrees(), 2.0);
  }
}
