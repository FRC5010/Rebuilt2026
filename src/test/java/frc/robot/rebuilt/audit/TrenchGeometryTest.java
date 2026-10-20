package frc.robot.rebuilt.audit;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

public class TrenchGeometryTest {
  /**
   * Pins the field numbers the audit (and the baseline) depend on. Update the baseline if these
   * change.
   */
  @Test
  public void pinnedFieldNumbers() {
    assertEquals(4.62, TrenchGeometry.HUB_X, 0.02);
    assertEquals(11.91, TrenchGeometry.OPP_HUB_X, 0.02);
    assertEquals(1.194, TrenchGeometry.DEPTH, 0.005);
    assertEquals(1.279, TrenchGeometry.OPENING_WIDTH, 0.005);
    assertEquals(0.419, TrenchGeometry.BUMPER_HALF, 0.005);
  }

  @Test
  public void zoneClassification() {
    double hub = TrenchGeometry.HUB_X;
    assertEquals("bottom-near", TrenchGeometry.zone(hub, 0.6, TrenchGeometry.CENTER));
    assertEquals(
        "top-near",
        TrenchGeometry.zone(hub, TrenchGeometry.FIELD_WIDTH - 0.6, TrenchGeometry.CENTER));
    assertEquals(
        "bottom-far", TrenchGeometry.zone(TrenchGeometry.OPP_HUB_X, 0.6, TrenchGeometry.CENTER));
    assertNull(TrenchGeometry.zone(hub, 3.0, TrenchGeometry.CENTER), "mid-field is not a trench");
    assertNull(
        TrenchGeometry.zone(hub - 1.5, 0.6, TrenchGeometry.BODY), "well outside the structure");
  }

  @Test
  public void bodyCriterionIsWiderThanCentre() {
    double x = TrenchGeometry.HUB_X - TrenchGeometry.DEPTH / 2 - 0.2; // just outside the beam edge
    assertNull(TrenchGeometry.zone(x, 0.6, TrenchGeometry.CENTER));
    assertEquals("bottom-near", TrenchGeometry.zone(x, 0.6, TrenchGeometry.BODY));
  }
}
