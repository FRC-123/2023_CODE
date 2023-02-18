package frc.robot.subsystems;

import static com.revrobotics.CANSparkMax.SoftLimitDirection.kForward;
import static com.revrobotics.CANSparkMax.SoftLimitDirection.kReverse;
import static com.revrobotics.SparkMaxLimitSwitch.Type.kNormallyOpen;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HiArmConstants;

/**
 * Subsystem for wrist mechanism
 */
public class HiArmSubsystem extends SubsystemBase {

  private static final int SMART_MOTION_SLOT = 0;
  private static final double GRAVITY_FF = 0.01;
  private static final double GEARBOX_RATIO = 75.0;
  private static final double CHAIN_RAIO = 1.0;

  // limits in degrees rotation
  private static final float LIMIT_BOTTOM = -5.0f;
  private static final float LIMIT_TOP = 185.0f;

  private final CANSparkMax HiarmMotor;
  private final SparkMaxPIDController HipidController;
  private final RelativeEncoder HiarmEncoder;

  private final CANSparkMax HiRollerMotor;

  private final static double CUBE_INTAKE_SPEED = 0.3;
  private final static double CUBE_EXPELL_SPEED = -0.3;
  private final static double CONE_INTAKE_SPEED = -0.3;
  private final static double CONE_EXPELL_SPEED = 0.3;

  private Double HitargetPosition = null;

  public HiArmSubsystem() {

    HiRollerMotor = new CANSparkMax(HiArmConstants.HIROLLER_MOTOR_CANID, MotorType.kBrushless);
    HiRollerMotor.restoreFactoryDefaults();
    // Voltage compensation and current limits
    HiRollerMotor.enableVoltageCompensation(12);
    HiRollerMotor.setSmartCurrentLimit(20);
    HiRollerMotor.setIdleMode(IdleMode.kBrake);
    HiRollerMotor.burnFlash();


    HiarmMotor = new CANSparkMax(HiArmConstants.HIARM_MOTOR_CANID, MotorType.kBrushless);
    HiarmMotor.restoreFactoryDefaults();
    
    // Get the motor relative encoder
    HiarmEncoder = HiarmMotor.getEncoder();
    HiarmEncoder.setPositionConversionFactor(360/CHAIN_RAIO/GEARBOX_RATIO);
    HiarmEncoder.setVelocityConversionFactor(1);
    HiarmEncoder.setPosition(0);
    HipidController = HiarmMotor.getPIDController();
    HipidController.setFeedbackDevice(HiarmEncoder);

    // Configure closed-loop control - FIX ME!!!
    double kP = 0.0032075*60;  // .0025; 
    double kI = 0.0;
    double kD = 0.00028705*60; 
    double kIz = 0.1; 
    double kFF = 0;
    double kMaxOutput = .6;
    double kMinOutput = -.6;
    double allowedErr = 0.125; // Error in rotations, not radians

    // Smart Motion Coefficients
    double maxVel = 1500; // rpm
    double maxAcc = 1000;
    double minVel = 0;

    HipidController.setP(kP);
    HipidController.setI(kI);
    HipidController.setD(kD);
    HipidController.setIZone(kIz);
    HipidController.setFF(kFF);
    HipidController.setOutputRange(kMinOutput, kMaxOutput); 

    HipidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_SLOT);
    HipidController.setSmartMotionMinOutputVelocity(minVel, SMART_MOTION_SLOT);
    HipidController.setSmartMotionMaxAccel(maxAcc, SMART_MOTION_SLOT);
    HipidController.setSmartMotionAllowedClosedLoopError(allowedErr, SMART_MOTION_SLOT);

    // Voltage compensation and current limits
    HiarmMotor.enableVoltageCompensation(12);
    HiarmMotor.setSmartCurrentLimit(20);
 
    // Configure soft limits
    HiarmMotor.setSoftLimit(kForward, LIMIT_TOP);
    HiarmMotor.setSoftLimit(kReverse, LIMIT_BOTTOM);
    HiarmMotor.enableSoftLimit(kForward, true);
    HiarmMotor.enableSoftLimit(kReverse, true);

    // Disable limit switches, we don't have any
    HiarmMotor.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false);
    HiarmMotor.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false);
    
    // Brake mode helps hold arm in place - but not for closed loop
    HiarmMotor.setIdleMode(IdleMode.kCoast);
 
    // Save settings to motor flash, so they persist between power cycles
    HiarmMotor.burnFlash();
    
  }

  @Override
  public void periodic() {
    if (HitargetPosition != null) {
      // Calculate feed forward based on angle to counteract gravity
      //   double cosineScalar = Math.cos(getArmPosition());
      //   double feedForward = GRAVITY_FF * cosineScalar;
        SmartDashboard.putNumber("HiArm Setpoint",HitargetPosition);
      HipidController.setReference(HitargetPosition, 
          ControlType.kPosition ); // feedForward, ArbFFUnits.kPercentOut);
    }

    // SmartDashboard.putNumber("Arm Setpoint",targetPosition);
    SmartDashboard.putNumber("HiArm Position Raw", HiarmEncoder.getPosition());
    SmartDashboard.putNumber("HiArm Output", HiarmMotor.getAppliedOutput());
  }

  /**
   * Moves the wrist using duty cycle. There is no motor safety, so calling this will continue to move until another
   * method is called.
   * @param speed duty cycle [-1,1]
   */
  public void moveArm(double speed){
    HitargetPosition =  null;
    HiarmMotor.set(speed);
  }

  /**
   * Moves the rollers using duty cycle. There is no motor safety, so calling this will continue to move until another
   * method is called.
   * @param speed duty cycle [-1,1]
   */
  public void moveRollers(double speed){
    HiRollerMotor.set(speed);
  }

  /**
   * Stop the rollers
   */
  public void stopRollers() {
    HiRollerMotor.stopMotor();
  }

  /**
   * intake cube
   * 
   */
  public void intakeCube() {
    HiRollerMotor.set(CUBE_INTAKE_SPEED);
  }

    /**
   * expell cube
   * 
   */
  public void expellCube() {
    HiRollerMotor.set(CUBE_EXPELL_SPEED);
  }

    /**
   * intake cube
   * 
   */
  public void intakeCone() {
    HiRollerMotor.set(CONE_INTAKE_SPEED);
  }

    /**
   * intake cube
   * 
   */
  public void expellCone() {
    HiRollerMotor.set(CONE_EXPELL_SPEED);
  }

  /**
   * Moves the elevator to a position. Zero is horizontal, up is positive. There is no motor safety, so calling this
   * will continue to move to this position, and hold it until another method is called.
   * @param radians position in radians
   */
  public void moveToPosition(double degrees) {
    // Set the target position, but move in execute() so feed forward keeps updating
    HitargetPosition = degrees;
  }

  /**
   * Gets the arm position
   * @return position in degrees
   */
  public double getArmPosition() {
    return (HiarmEncoder.getPosition());
  }

  /**
   * Stop the arm
   */
  public void stop() {
    // targetPosition = null;
    HiarmMotor.stopMotor();
    HitargetPosition = HiarmEncoder.getPosition();
  }

}