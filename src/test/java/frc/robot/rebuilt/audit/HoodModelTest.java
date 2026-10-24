package frc.robot.rebuilt.audit;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;

public class HoodModelTest {
  private static HoodCommand up(double t) {
    return new HoodCommand(t, true, "launcherPrep", "test");
  }

  private static HoodCommand down(double t) {
    return new HoodCommand(t, false, "launcherLow", "test");
  }

  @Test
  public void startsLowWithNoCommands() {
    HoodModel.Trace h = HoodModel.trace(List.of());
    assertEquals(HoodModel.LOW_DEG, h.angleAt(0), 1e-9);
    assertEquals(HoodModel.LOW_DEG, h.angleAt(100), 1e-9);
  }

  @Test
  public void raisesAndSettlesHigh() {
    HoodModel.Trace h = HoodModel.trace(List.of(up(1.0)));
    assertEquals(HoodModel.LOW_DEG, h.angleAt(0.9), 1e-9);
    assertTrue(h.angleAt(1.3) > 20 && h.angleAt(1.3) < 30, "mid-travel at +0.3s");
    assertTrue(h.angleAt(3.0) > 33, "settled high");
  }

  @Test
  public void lowerReachesTheDownThresholdAboutSixTenthsOfASecondAfterTheCommand() {
    HoodModel.Trace h = HoodModel.trace(List.of(up(0.0), down(3.0)));
    assertTrue(h.angleAt(3.0 + 0.3) > HoodModel.DOWN_THRESHOLD_DEG, "still high 0.3s after low");
    assertTrue(h.angleAt(3.0 + 0.4) > HoodModel.DOWN_THRESHOLD_DEG, "still high 0.4s after low");
    assertTrue(h.angleAt(3.0 + 0.7) <= HoodModel.DOWN_THRESHOLD_DEG, "below threshold by 0.7s");
    assertEquals(HoodModel.LOW_DEG, h.angleAt(6.0), 0.05);
  }

  /** Regression: a launcherLow while the hood is already down must not read as "travelling". */
  @Test
  public void lowWhileAlreadyDownStaysDown() {
    HoodModel.Trace h = HoodModel.trace(List.of(down(1.0), down(2.0)));
    for (double t = 0; t < 4; t += 0.05) {
      assertEquals(HoodModel.LOW_DEG, h.angleAt(t), 1e-9, "t=" + t);
    }
  }

  @Test
  public void interruptedMoveContinuesFromCurrentAngle() {
    HoodModel.Trace h = HoodModel.trace(List.of(up(0.0), down(0.3), up(0.6)));
    double before = h.angleAt(0.3 - 1e-6);
    double after = h.angleAt(0.3 + 1e-6);
    assertEquals(before, after, 0.05, "continuous at the down command");
    double before2 = h.angleAt(0.6 - 1e-6);
    double after2 = h.angleAt(0.6 + 1e-6);
    assertEquals(before2, after2, 0.05, "continuous at the up command");
  }

  @Test
  public void commandOrderDoesNotMatterForInput() {
    HoodModel.Trace a = HoodModel.trace(List.of(up(0.0), down(3.0)));
    HoodModel.Trace b = HoodModel.trace(List.of(down(3.0), up(0.0)));
    for (double t = 0; t < 6; t += 0.1) assertEquals(a.angleAt(t), b.angleAt(t), 1e-9);
  }

  @Test
  public void severityBands() {
    assertEquals(Severity.OK, Severity.of(17.2));
    assertEquals(Severity.MARGINAL, Severity.of(17.3));
    assertEquals(Severity.MARGINAL, Severity.of(22.0));
    assertEquals(Severity.POOR, Severity.of(22.1));
    assertEquals(Severity.POOR, Severity.of(28.0));
    assertEquals(Severity.FAIL, Severity.of(28.1));
  }
}
