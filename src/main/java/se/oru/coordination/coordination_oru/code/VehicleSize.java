package se.oru.coordination.coordination_oru.code;

public class VehicleSize {
    public double length;
    public double width;
    public double frontSafeDistance;
    public double backSafeDistance;
    public double leftSafeDistance;
    public double rightSafeDistance;

    public VehicleSize(double length, double width,
                       double frontSafeDistance, double backSafeDistance,
                       double leftSafeDistance, double rightSafeDistance) {
        this.length = length;
        this.width = width;
        this.frontSafeDistance = frontSafeDistance;
        this.backSafeDistance = backSafeDistance;
        this.leftSafeDistance = leftSafeDistance;
        this.rightSafeDistance = rightSafeDistance;
    }

    public String toString() {
        return "[length=" + length + ", width=" + width + ", safe distance: " +
                "front=" + frontSafeDistance + ", back=" + backSafeDistance +
                ", left=" + leftSafeDistance + ", right=" + rightSafeDistance + "]";
    }

    public double calcCircumradius() {
        // Calculate the radius of the circumscribed circle.
        // In other words, the radius of the minimal circle big enough to turn around in any direction.
        // E.g., for a clock hand, this is the clock itself.

        double front = length / 2 + frontSafeDistance;
        double back = length / 2 + backSafeDistance;
        double left = width / 2 + leftSafeDistance;
        double right = width / 2 + rightSafeDistance;
        return Math.max(
                Math.max(front, back),
                Math.max(left, right)
        );
    }


}
