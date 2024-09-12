package org.sciborgs1155.robot.arm;

public interface ArmIO extends AutoCloseable {
  void setVoltage(double voltage);

  double position();

  @Override
  void close();
}
