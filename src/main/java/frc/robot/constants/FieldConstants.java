package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;

public class FieldConstants {
    //All measurmeents in inches
    public static final double speakerBottomHeight = 78;
    public static final double speakerTopHeight = 82.875;
    public static final double speakerMiddleHeight = (speakerTopHeight-speakerBottomHeight)/2;

    public static final double subwooferHeight = 37;
    public static final double subwooferHorizontal = 36.125;

    public static final double noteInsideDiameter = 10;
    public static final double noteOutsideDiameter = 14;
    public static final double noteWidth = 2;

    public static final double ampBottomHeight = 26;
    public static final double ampTopHeight = 44;
    public static final double ampDepth = 3.875;   
    public static final double ampMiddleHeight = (ampTopHeight-ampBottomHeight)/2;
    public static final double ampZoneWidth = 17.75;
    
    public static final double allianceAreaWidth = 323.125;
    public static final double allianceAreaDepth = 118.125;

    public static final double stageHeight = 28.25;

    public static final double lowestChainPoint = 28.25;

    public static final double trapOpeningBottomHeight = 56.5;

    public static final double microphoneHeight = 12;

    public static final double podiumHeight = 17.75;
    public static final double podiumWidth = 10;
    
    public static final double sourceZonedepth = 18.75;

    //someone double check this - also its in meters
    public static final Pose3d speakerPose = new Pose3d(3.85, 5.56, speakerMiddleHeight * 0.0254, null);

    
}
