package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Degrees;
import static org.sciborgs1155.lib.Test.runUnitTest;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.sciborgs1155.robot.arm.Arm;

public class ArmTest {
  private Arm arm;

  final double DELTA = 1e-1;
  final double ANGLE_DELTA = 3.0 * Math.PI / 180;

  @BeforeEach
  public void setup() {
    setupTests();
    arm = Arm.create();
  }

  @AfterEach
  public void destroy() throws Exception {
    reset(arm);
  }

  @ParameterizedTest
  @ValueSource(doubles = {-40, -30, 0, 30, 40, 60})
  public void armSystemCheck(double theta) {
    runUnitTest(arm.moveToTest(Degrees.of(theta)));
  }

  // @Test
  // public void endConditions() {
  //   Consumer<Command> testEndCondition =
  //       c_ -> {
  //         var c = c_.ignoringDisable(true);
  //         c.schedule();
  //         fastForward(10);
  //         assert !c.isFinished();
  //       };
  //   testEndCondition.accept(shooting.shootWithPivot(() -> 4, () -> 100));
  //   testEndCondition.accept(shooting.shoot(RadiansPerSecond.of(150)));
  // }
}
