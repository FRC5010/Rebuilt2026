package frc.robot.rebuilt.audit;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;

/** Behaviour of the B-Line analyzer on synthetic paths with known answers. */
public class BLineAuditTest {
  private final BLineAudit audit;

  public BLineAuditTest() throws IOException {
    audit = new BLineAudit(Path.of("src/test/resources/auto-audit/fixtures/bline"));
  }

  private List<Stay> stays(String path) throws IOException {
    return TrenchAudit.stays(audit.timeline(path), TrenchGeometry.CENTER);
  }

  @Test
  public void pathWithoutHoodEventsIsClean() throws IOException {
    List<Stay> s = stays("NoHood");
    assertEquals(1, s.size());
    assertEquals(Severity.OK, s.get(0).severity());
    assertEquals(AutoTimeline.LowerAfterRaise.NO_RAISE, audit.timeline("NoHood").lowerAfterRaise());
  }

  @Test
  public void prepAtStartWithNoLowFails() throws IOException {
    assertEquals(Severity.FAIL, stays("PrepAtStart").get(0).severity());
    assertEquals(
        AutoTimeline.LowerAfterRaise.NEVER_LOWERS, audit.timeline("PrepAtStart").lowerAfterRaise());
  }

  @Test
  public void eventTimeIsFractionOfItsSegment() throws IOException {
    // 5 m segment at 4 m/s = 1.25 s; low at t_ratio 0.3 -> 0.375 s; prep at 0.0
    List<HoodCommand> c = audit.timeline("LowTooLate").hoodCommands;
    assertEquals(2, c.size());
    assertEquals(0.0, c.get(0).t(), 1e-9);
    assertEquals(0.375, c.get(1).t(), 1e-9);
  }

  @Test
  public void lowTooCloseToTheTrenchIsFlagged() throws IOException {
    Stay s = stays("LowTooLate").get(0);
    assertTrue(s.violation(), s.toString());
    assertEquals("launcherLow", s.lastCommandBeforeEntry.name());
  }

  @Test
  public void lowOnAnEarlierSegmentGivesEnoughLead() throws IOException {
    Stay s = stays("LowEarly").get(0);
    assertEquals(Severity.OK, s.severity(), s.toString());
  }

  @Test
  public void analysisIsDeterministic() throws IOException {
    for (String name : audit.pathNames()) {
      assertEquals(stays(name).toString(), stays(name).toString(), name);
    }
  }
}
