package frc.lib.configs;

public class offsetConfig {
    public final double xoffset;
    public final double yoffset;
    public final double zoffset;
    public final double yawoffset;
    public final double pitchoffset;
    public final double rolloffset;


    /**
     * To be used when creating an offset ex: camera offset
     * @param x
     * @param y
     * @param z
     * @param yaw
     * @param pitch
     * @param roll
     */

    public offsetConfig(double x, double y, double z, double yaw, double pitch, double roll) {
        this.xoffset = x;
        this.yoffset = y;
        this.zoffset = z;
        this.yawoffset = yaw;
        this.pitchoffset = pitch;
        this.rolloffset = roll;
    }


}
