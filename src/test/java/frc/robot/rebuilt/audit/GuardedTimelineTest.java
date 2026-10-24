package frc.robot.rebuilt.audit;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;

/** The trench guard applied to the synthetic fixture autos (ideal path following). */
public class GuardedTimelineTest {
  private final PathPlannerAudit pp =
      new PathPlannerAudit(Path.of("src/test/resources/auto-audit/fixtures/pathplanner"));

  private List<Stay> guarded(String auto) throws IOException {
    return TrenchAudit.stays(GuardedTimeline.apply(pp.timeline(auto)), TrenchGeometry.CENTER);
  }

  @Test
  public void prepThenCrossWithNoLowIsProtectedByTheGuard() throws IOException {
    assertTrue(
        TrenchAudit.stays(pp.timeline("PrepThenCross"), TrenchGeometry.CENTER).get(0).violation(),
        "precondition: unguarded this auto fails");
    assertEquals(Severity.OK, guarded("PrepThenCross").get(0).severity());
  }

  /**
   * Known limit: the path starts from rest 0.5 m before the beam and accelerates at 9 m/s^2, so the
   * robot is under the beam 0.34 s after it first moves. The auto's own launcherLow already fires
   * at that moment and the guard only sees velocity once it exists, so it adds nothing. Such autos
   * need an earlier launcherLow.
   */
  @Test
  public void lowFromRestRightBesideTheBeamIsNotCuredByTheGuard() throws IOException {
    Stay plain = TrenchAudit.stays(pp.timeline("LowTooLate"), TrenchGeometry.CENTER).get(0);
    Stay guarded = guarded("LowTooLate").get(0);
    assertEquals(plain.angleAtEntry, guarded.angleAtEntry, 0.5);
    assertEquals(Severity.MARGINAL, guarded.severity());
  }

  @Test
  public void markerThatFiresInsideTheTrenchIsDeferredUntilClear() throws IOException {
    Stay s = guarded("MarkerInsideTrench").get(0);
    assertEquals(Severity.OK, s.severity(), s.toString());
  }

  @Test
  public void autoThatNeverRaisesTheHoodIsUnchanged() throws IOException {
    AutoTimeline tl = pp.timeline("Clean");
    assertEquals(tl.hoodCommands, GuardedTimeline.apply(tl).hoodCommands);
  }

  @Test
  public void autoWithoutAPathIsReturnedAsIs() throws IOException {
    AutoTimeline tl = new AutoTimeline("x", 1.0, List.of(), null);
    assertSame(tl, GuardedTimeline.apply(tl));
  }

  @Test
  public void guardDoesNotLowerTheHoodAwayFromTrenches() throws IOException {
    AutoTimeline g = GuardedTimeline.apply(pp.timeline("NoTrench"));
    assertEquals(1, g.hoodCommands.size());
    assertTrue(g.hoodCommands.get(0).up());
  }
}
