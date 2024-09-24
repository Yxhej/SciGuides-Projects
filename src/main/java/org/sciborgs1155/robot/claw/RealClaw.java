package org.sciborgs1155.robot.claw;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class RealClaw implements ClawIO {
  private final CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless);

  @Override
  public void set(double speed) {
    motor.set(speed);
  }
}
