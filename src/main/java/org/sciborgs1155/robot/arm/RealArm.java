package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.arm.ArmConstants.POSITION_FACTOR;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import monologue.Annotations.Log;

public class RealArm implements ArmIO {
  private final CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless);
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

  @Override
  public void close() {
    motor.close();
    encoder.close();
  }

  @Override
  public double velocity() {
    return 0;
  }
}
