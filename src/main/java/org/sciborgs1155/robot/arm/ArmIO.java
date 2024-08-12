package org.sciborgs1155.robot.arm;

import monologue.Logged;

public interface ArmIO extends Logged {
  void setVoltage(double voltage);

  double position();
}
