package org.tahomarobotics.robot.motion;

public class CalcSwerveLocations {

	private final double WHEELBASE = 20;
	private final double TRACKWIDTH = 20;

	private double forward, strafe, rotation, heading;
	private boolean fieldCentric;

	public CalcSwerveLocations(){
		this(0.0,0.0,0.0,false,0.0);
	}
	public CalcSwerveLocations(double forward, double strafe, double rotation){
		this(forward, strafe, rotation, false, 0.0);
	}
	public CalcSwerveLocations(double forward, double strafe, boolean fieldCentric, double heading){
		this(forward, strafe, 0.0, fieldCentric, heading);
	}
	public CalcSwerveLocations(double forward, double strafe, double rotation, boolean fieldCentric){
		this(forward, strafe, rotation, fieldCentric, 0.0);
	}
	public CalcSwerveLocations(double forward, double strafe, double rotation, boolean fieldCentric, double heading){
		this.forward = forward;
		this.strafe = strafe;
		this.rotation = rotation;
		this.fieldCentric = fieldCentric;
		this.heading = heading;
	}

	public void setR(double r){
		this.rotation = r;
	}

	public void update(double forward, double strafe, double rotation, boolean fieldCentric, double heading){
		this.forward = forward;
		this.strafe = strafe;
		this.rotation = rotation;
		this.fieldCentric = fieldCentric;
		this.heading = heading;
	}
	public double[][] calculateSwerveModuleAnglesSpeeds() {

		if(fieldCentric) {
			double angleRad = Math.toRadians(heading);
			double temp = forward * Math.cos(angleRad) + strafe * Math.sin(angleRad);
			strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad);
			forward = temp;
		}

		double a = strafe - rotation * (WHEELBASE / TRACKWIDTH);
		double b = strafe + rotation * (WHEELBASE / TRACKWIDTH);
		double c = forward - rotation * (TRACKWIDTH / WHEELBASE);
		double d = forward + rotation * (TRACKWIDTH / WHEELBASE);

		return new double[][]{{
				Math.atan2(b, c) * 180 / Math.PI,
				Math.atan2(b, d) * 180 / Math.PI,
				Math.atan2(a, d) * 180 / Math.PI,
				Math.atan2(a, c) * 180 / Math.PI}
				, {
				Math.sqrt(b * b + c * c),
				Math.sqrt(b * b + d * d),
				Math.sqrt(a * a + d * d),
				Math.sqrt(a * a + c * c)}};
	}

}
