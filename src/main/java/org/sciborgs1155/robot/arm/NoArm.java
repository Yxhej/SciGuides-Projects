package org.sciborgs1155.robot.arm;

public class NoArm implements ArmIO {
  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double position() {
    return 0;
  }
}
