package org.sciborgs1155.robot.claw;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Claw extends SubsystemBase implements Logged {
  private final ClawIO hardware;

  public static Claw create() {
    return Robot.isReal() ? new Claw(new RealClaw()) : new Claw(new NoClaw());
  }

  public Claw(ClawIO hardware) {
    this.hardware = hardware;
  }

  public Command runRollers(double speed) {
    return run(() -> hardware.set(0.5)).finallyDo(() -> hardware.set(0));
  }

  public Command intake() {
    return runRollers(0.5);
  }

  public Command spit() {
    return runRollers(-0.5);
  }
}
