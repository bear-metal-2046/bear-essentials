package org.tahomarobotics.robot.motion;

public class SwerveMaxRotationalAlgorithm {

	private double forward, strafe, p, wanted, actual;

	public SwerveMaxRotationalAlgorithm(double forward, double strafe, double wantedRotationAngle, double actualRotationAngle){
		this.forward = forward;
		this.strafe = strafe;
		this.wanted = wantedRotationAngle;
		this.actual = actualRotationAngle;
		p = Math.sqrt(this.strafe * this.strafe + this.forward * this.forward);// i think p is supposed to be the power vector
	}

	public double getOutput(){
		if(p < .8 && Math.abs(wanted - actual) > 3.0) {

			p = .8;
			boolean isRotationNeg = wanted < actual;
			boolean isStrafeNeg = strafe < 0;
			boolean isForwardNEg = forward < 0;
			double inside;
			double rotationalOutput;//goes to the swerve math with the forwards and strafe that we started with
			double rMinus;// has inside subtracted instead of added
			if((!isStrafeNeg && isForwardNEg && !isRotationNeg) || (isStrafeNeg && !isForwardNEg && isRotationNeg)){
				inside = (2* strafe - 2* forward) * (2* strafe - 2* forward) - 8 * (strafe * strafe + forward * forward - p*p);
				if(inside > 0){
					rotationalOutput = ((-2* strafe + 2* forward) + Math.sqrt(inside)) / 4.0;
					rMinus = ((-2* strafe + 2* forward) - Math.sqrt(inside)) / 4.0;
					rotationalOutput = isRotationNeg ? Math.min(rotationalOutput, rMinus) : Math.max(rotationalOutput, rMinus);
					if(isRotationNeg ? (rotationalOutput < 0) : (rotationalOutput > 0)){
						return rotationalOutput;
					}
				}
			}else if((isStrafeNeg && isForwardNEg && isRotationNeg) || (!isStrafeNeg && !isForwardNEg && !isRotationNeg)){
				inside = (2* strafe + 2* forward) * (2* strafe + 2* forward) - 8 * (strafe * strafe + forward * forward - p*p);
				if(inside > 0){
					rotationalOutput = (-(2* strafe + 2* forward) + Math.sqrt(inside)) / 4.0;
					rMinus = (-(2* strafe + 2* forward) - Math.sqrt(inside)) / 4.0;
					rotationalOutput = isRotationNeg ? Math.min(rotationalOutput, rMinus) : Math.max(rotationalOutput, rMinus);
					if(isRotationNeg ? (rotationalOutput < 0) : (rotationalOutput > 0)){
						return rotationalOutput;
					}
				}
			}else if((isStrafeNeg && !isForwardNEg) || (!isStrafeNeg && isForwardNEg)){
				inside = (-2* strafe + 2* forward) * (-2* strafe + 2* forward) - 8 * (strafe * strafe + forward * forward - p*p);
				if(inside > 0){
					rotationalOutput = ((2* strafe - 2* forward) + Math.sqrt(inside)) / 4.0;
					rMinus = ((2* strafe - 2* forward) - Math.sqrt(inside)) / 4.0;
					rotationalOutput = isRotationNeg ? Math.min(rotationalOutput, rMinus) : Math.max(rotationalOutput, rMinus);
					if(isRotationNeg ? (rotationalOutput < 0) : (rotationalOutput > 0)){
						return rotationalOutput;
					}
				}
			}else {
				inside = (-2* strafe - 2* forward) * (-2* strafe - 2* forward) - 8 * (strafe * strafe + forward * forward - p*p);
				if(inside > 0){
					rotationalOutput = ((2* strafe + 2* forward) + Math.sqrt(inside)) / 4.0;
					rMinus = ((2* strafe + 2* forward) - Math.sqrt(inside)) / 4.0;
					rotationalOutput = isRotationNeg ? Math.min(rotationalOutput, rMinus) : Math.max(rotationalOutput, rMinus);
					if(isRotationNeg ? (rotationalOutput < 0) : (rotationalOutput > 0)){
						return rotationalOutput;
					}
				}
			}
		}
		return 0.0;
	}

}
