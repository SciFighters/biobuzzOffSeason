package org.firstinspires.ftc.teamcode.Utilities;

public enum GobildaPlanetery {
    RPM6000(1),
    RPM1620(3.7),
    RPM1150(5.2),
    RPM425(13.7),
    RPM312(19.2),
    RPM223(26.9),
    RPM117(50.9),
    RPM84(71.2),
    RPM60(99.5),
    RPM43(139),
    RPM30(188),
    ;
    public final double ratio;

    GobildaPlanetery(double ratio) {
        this.ratio = ratio;
    }

    public double getValue() {
        return ratio;
    }

    public static double toRatio(String rpmName) {
        return GobildaPlanetery.valueOf(rpmName).getValue();
    }
}
