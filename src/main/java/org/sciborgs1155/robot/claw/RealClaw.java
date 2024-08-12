package org.sciborgs1155.robot.claw;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;

public class RealClaw implements ClawIO {
  private final PWMSparkFlex motor = new PWMSparkFlex(0);

  @Override
  public void set(double speed) {
    motor.set(speed);
  }
}
