package frc.robot.config;

public class ArmConfiguration {
    public double elevatorkP;
    public double elevatorkI;
    public double elevatorkD;
    public double elevatorkS;

    public double shoulderkP;
    public double shoulderkI;
    public double shoulderkD;
    public double shoulderkS;

    public double wristkP;
    public double wristkI;
    public double wristkD;
    public double wristkS;

    public ArmConfiguration withElevatorkP(double elevatorkP) {
        this.elevatorkP = elevatorkP;
        return this;
    }

    public ArmConfiguration withElevatorkI(double elevatorkI) {
        this.elevatorkI = elevatorkI;
        return this;
    }

    public ArmConfiguration withElevatorkD(double elevatorkD) {
        this.elevatorkD = elevatorkD;
        return this;
    }

    public ArmConfiguration withElevatorkS(double elevatorkS) {
        this.elevatorkS = elevatorkS;
        return this;
    }

    public ArmConfiguration withShoulderkP(double shoulderkP) {
        this.shoulderkP = shoulderkP;
        return this;
    }

    public ArmConfiguration withShoulderkI(double shoulderkI) {
        this.shoulderkI = shoulderkI;
        return this;
    }

    public ArmConfiguration withShoulderkD(double shoulderkD) {
        this.shoulderkD = shoulderkD;
        return this;
    }

    public ArmConfiguration withShoulderkS(double shoulderkS) {
        this.shoulderkS = shoulderkS;
        return this;
    }

    public ArmConfiguration withWristkP(double wristkP) {
        this.wristkP = wristkP;
        return this;
    }

    public ArmConfiguration withWristkI(double wristkI) {
        this.wristkI = wristkI;
        return this;
    }

    public ArmConfiguration withWristkD(double wristkD) {
        this.wristkD = wristkD;
        return this;
    }

    public ArmConfiguration withWristkS(double wristkS) {
        this.wristkS = wristkS;
        return this;
    }
}
