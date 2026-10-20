package frc.robot.rebuilt.audit;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;

/** Behaviour of the PathPlanner analyzer on synthetic autos with known answers. */
public class PathPlannerAuditTest {
  private static final Path FIXTURES =
      Path.of("src/test/resources/auto-audit/fixtures/pathplanner");
  private final PathPlannerAudit audit = new PathPlannerAudit(FIXTURES);

  private List<Stay> stays(String auto) throws IOException {
    return TrenchAudit.stays(audit.timeline(auto), TrenchGeometry.CENTER);
  }

  @Test
  public void cleanAutoPassesTheTrenchWithHoodDown() throws IOException {
    List<Stay> s = stays("Clean");
    assertEquals(1, s.size());
    assertEquals("bottom-near", s.get(0).zone);
    assertEquals(Severity.OK, s.get(0).severity());
    assertEquals(HoodModel.LOW_DEG, s.get(0).maxAngle, 1e-6);
    // ideal rest-to-rest 4 m/s, 9 m/s^2 trapezoid reaches the beam edge (x=4.02, 2.02 m in) at
    // ~0.73 s
    assertEquals(0.73, s.get(0).t0, 0.03);
  }

  @Test
  public void prepWithoutLowIsAFailure() throws IOException {
    List<Stay> s = stays("PrepThenCross");
    assertEquals(Severity.FAIL, s.get(0).severity());
    assertTrue(s.get(0).angleAtEntry > 33);
    assertEquals(
        AutoTimeline.LowerAfterRaise.NEVER_LOWERS,
        audit.timeline("PrepThenCross").lowerAfterRaise());
  }

  @Test
  public void lowWithPlentyOfLeadIsOk() throws IOException {
    Stay s = stays("LowWellBefore").get(0);
    assertEquals(Severity.OK, s.severity());
    assertEquals(
        AutoTimeline.LowerAfterRaise.YES, audit.timeline("LowWellBefore").lowerAfterRaise());
    assertEquals("launcherLow", s.lastCommandBeforeEntry.name());
  }

  /**
   * The recurring real-world pattern: launcherLow, then a path that starts ~0.5 m before the beam.
   */
  @Test
  public void lowImmediatelyBeforeAPathStartingAtTheEdgeIsMarginal() throws IOException {
    Stay s = stays("LowTooLate").get(0);
    assertEquals(0.34, s.t0 - s.lastCommandBeforeEntry.t(), 0.03, "lead time");
    assertEquals(Severity.MARGINAL, s.severity());
    assertTrue(s.angleAtEntry > 18 && s.angleAtEntry < 22, "angle at beam " + s.angleAtEntry);
  }

  @Test
  public void markerFiringInsideTheTrenchIsFlagged() throws IOException {
    Stay s = stays("MarkerInsideTrench").get(0);
    assertTrue(s.violation(), s.toString());
    assertEquals(1, s.commandsInside.size());
    assertEquals("launcherPrep", s.commandsInside.get(0).name());
  }

  @Test
  public void markerAfterTheExitIsOkButReportsItsMargin() throws IOException {
    Stay s = stays("MarkerAfterExit").get(0);
    assertEquals(Severity.OK, s.severity());
    assertFalse(Double.isNaN(s.exitMarginSeconds));
    assertTrue(s.exitMarginSeconds > 0.0, "hood rises only after leaving: " + s.exitMarginSeconds);
  }

  @Test
  public void parallelGroupStartsBothChildrenAtTheSameTime() throws IOException {
    AutoTimeline tl = audit.timeline("ParallelPrep");
    assertEquals(0.0, tl.hoodCommands.get(0).t(), 1e-9, "prep fires at group start");
    assertEquals(Severity.FAIL, stays("ParallelPrep").get(0).severity());
  }

  @Test
  public void secondPassIsRatedIndependently() throws IOException {
    List<Stay> lowBetween = stays("TwoPassesLowBetween");
    assertEquals(2, lowBetween.size());
    assertEquals(Severity.OK, lowBetween.get(0).severity());
    assertEquals(Severity.OK, lowBetween.get(1).severity());

    List<Stay> noLow = stays("TwoPassesNoLow");
    assertEquals(2, noLow.size());
    assertEquals(Severity.OK, noLow.get(0).severity(), "first pass is before any prep");
    assertEquals(Severity.FAIL, noLow.get(1).severity(), "second pass has the hood up");
    assertEquals(List.of(2), TrenchAudit.violationPasses(noLow));
  }

  @Test
  public void autoThatNeverEntersATrenchHasNoStays() throws IOException {
    assertTrue(stays("NoTrench").isEmpty());
  }

  @Test
  public void bodyCriterionStartsEarlierThanCentre() throws IOException {
    AutoTimeline tl = audit.timeline("Clean");
    double centre = TrenchAudit.stays(tl, TrenchGeometry.CENTER).get(0).t0;
    double body = TrenchAudit.stays(tl, TrenchGeometry.BODY).get(0).t0;
    assertTrue(body < centre);
  }

  @Test
  public void unknownCommandTypesFailLoudly() {
    IllegalArgumentException e =
        assertThrows(IllegalArgumentException.class, () -> audit.timeline("UnsupportedNode"));
    assertTrue(e.getMessage().contains("conditional"));
  }

  @Test
  public void analysisIsDeterministic() throws IOException {
    for (String name : audit.autoNames()) {
      if (name.equals("UnsupportedNode")) continue;
      List<Stay> a = stays(name);
      List<Stay> b =
          new PathPlannerAudit(FIXTURES).timeline(name) == null ? List.of() : stays(name);
      assertEquals(a.toString(), b.toString(), name);
    }
  }
}
