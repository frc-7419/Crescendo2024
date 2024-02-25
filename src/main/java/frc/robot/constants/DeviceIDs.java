package frc.robot.constants;

public final class DeviceIDs {
  public static enum CanIds {
    // Arm CAN IDs
    shooterWrist(13),

    // Gyro CAN IDss
    pigeon(16),

    wrist(18),
    intakeMotor(7),

    topShooter(19),
    bottomShooter(20),
    serializerBack(21), 
    ;

    public final int id;

    private CanIds(int id) {
      this.id = id;
    }
  }

  public static enum SensorIds {

    // Beambreak DIO ID
    beamBreak(2),

    // Limit switch DIO ID
    limitSwitch(0),

    // Arm Through Bore Encoder DIO ID
    armAbsoluteEncoder(2),
    armRelativeEncoder1(5),
    armRelativeEncoder2(4),
    elevatorAbsoluteEncoder(3);

    // Wrist Through Bore Encoder DIO ID
    // wristAbsoluteEncoder(1),
    // elevatorAbsoluteEncoder(1);

    public final int id;

    private SensorIds(int id) {
      this.id = id;
    }
  }
}