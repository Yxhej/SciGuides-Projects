package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.arm.ArmConstants.POSITION_FACTOR;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import monologue.Annotations.Log;

public class RealArm implements ArmIO {
  private final PWMSparkFlex motor = new PWMSparkFlex(0);
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(1);

  public RealArm() {
    encoder.setDistancePerRotation(POSITION_FACTOR.in(Radians));
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  @Log.NT
  public double position() {
    return encoder.get();
  }
}
