package org.sciborgs1155.robot.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Arm extends SubsystemBase implements Logged {
  private final ArmIO hardware;
  private final ProfiledPIDController pid;
  private final ArmFeedforward ff;

  public static Arm create() {
    return Robot.isReal() ? new Arm(new RealArm()) : new Arm(new SimArm());
  }

  public Arm(ArmIO hardware) {
    this.hardware = hardware;
    pid = new ProfiledPIDController(0, 0, 0, null);
    ff = new ArmFeedforward(0, 0, 0);
  }

  public Command moveTo(double goal) {
    return run(() -> updateGoal(goal)).until(pid::atGoal).finallyDo(() -> hardware.setVoltage(0));
  }

  public void updateGoal(double goal) {
    double pidOutput = pid.calculate(hardware.position(), goal);
    double ffOutput = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);
    hardware.setVoltage(pidOutput + ffOutput);
  }
}
