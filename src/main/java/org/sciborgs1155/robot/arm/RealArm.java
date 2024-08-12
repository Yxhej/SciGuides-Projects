package org.sciborgs1155.robot.arm;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;

public class RealArm implements ArmIO {
  private final PWMSparkFlex motor = new PWMSparkFlex(0);
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(1);

  public RealArm() {
    // configurations go here...
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public double position() {
    return encoder.get();
  }
}
