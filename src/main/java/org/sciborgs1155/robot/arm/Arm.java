package org.sciborgs1155.robot.arm;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.arm.ArmConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;

public class Arm extends SubsystemBase implements Logged {
  private final ArmIO hardware;

  @Log.NT private final ProfiledPIDController pid;
  private final ArmFeedforward ff;

  @Log.NT private final Mechanism2d mech = new Mechanism2d(2, 2);

  private final MechanismRoot2d chassis =
      mech.getRoot("Chassis", 1 + AXLE_FROM_CHASSIS.getX(), AXLE_FROM_CHASSIS.getZ());
  private final MechanismLigament2d arm =
      chassis.append(
          new MechanismLigament2d("Arm", LENGTH.in(Meters), 0, 4, new Color8Bit(Color.kAliceBlue)));

  public static Arm create() {
    return Robot.isReal() ? new Arm(new RealArm()) : new Arm(new SimArm());
  }

  public Arm(ArmIO hardware) {
    this.hardware = hardware;
    pid =
        new ProfiledPIDController(
            kP, kI, kD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL));
    ff = new ArmFeedforward(kS, kV, kA);

    pid.setTolerance(POSITION_TOLERANCE.in(Radians));

    setDefaultCommand(moveTo(0).withName("default"));
  }

  public Command moveTo(Measure<Angle> goal) {
    return moveTo(goal.in(Radians));
  }

  public Command moveTo(double goal) {
    return run(() -> updateGoal(goal)).until(pid::atGoal).finallyDo(() -> hardware.setVoltage(0));
  }

  public void updateGoal(double goal) {
    pid.setGoal(goal);
    double pidOutput = pid.calculate(hardware.position(), goal);
    double ffOutput = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);
    hardware.setVoltage(pidOutput + ffOutput);
  }

  public void setState(double angle) {
    arm.setAngle(Units.radiansToDegrees(angle + Math.PI));
  }

  @Override
  public void periodic() {
    setState(hardware.position());
    log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
    log("at goal", pid.atGoal());
    log("position", hardware.position());
  }
}
