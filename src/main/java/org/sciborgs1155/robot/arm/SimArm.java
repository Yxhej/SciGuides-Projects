package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.arm.ArmConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimArm implements ArmIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          LinearSystemId.createSingleJointedArmSystem(
              DCMotor.getNEO(4),
              MOI.in((Meters).mult(Meters).mult(Kilograms)),
              1.0 / MOTOR_GEARING),
          DCMotor.getNEO(4),
          1.0 / MOTOR_GEARING,
          -LENGTH.in(Meters),
          MIN_ANGLE.in(Radians),
          MAX_ANGLE.in(Radians),
          true,
          STARTING_ANGLE.in(Radians));

  @Override
  public void setVoltage(double voltage) {
    sim.setInputVoltage(voltage);
    sim.update(PERIOD.in(Seconds));
  }

  @Override
  public double position() {
    return sim.getAngleRads();
  }

  @Override
  public void close() {}

  @Override
  public double velocity() {
    return sim.getVelocityRadPerSec();
  }
}
