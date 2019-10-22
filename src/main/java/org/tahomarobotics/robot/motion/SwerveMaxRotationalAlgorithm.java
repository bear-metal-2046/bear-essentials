package org.tahomarobotics.robot.motion;

public class SwerveMaxRotationalAlgorithm {

	private double f, s, p, wanted, actual;

	public SwerveMaxRotationalAlgorithm(double f, double s, double wanted, double actual){
		this.f = f;
		this.s = s;
		this.wanted = wanted;
		this.actual = actual;
		p = Math.sqrt(this.s * this.s + this.f * this.f);
	}

	public double getOutput(){
		if(p < .8 && Math.abs(wanted - actual) > 3.0) {

			p = .8;
			boolean rNeg = wanted < actual;
			boolean sNeg = s < 0;
			boolean fNeg = f < 0;
			double inside;
			double r;
			double rMinus;
			if((!sNeg && fNeg && !rNeg) || (sNeg && !fNeg && rNeg)){
				inside = (2*s - 2*f) * (2*s - 2*f) - 8 * (s*s + f*f - p*p);
				if(inside > 0){
					r = ((-2*s + 2*f) + Math.sqrt(inside)) / 4.0;
					rMinus = ((-2*s + 2*f) - Math.sqrt(inside)) / 4.0;
					r = rNeg ? Math.min(r, rMinus) : Math.max(r, rMinus);
					if(rNeg ? (r < 0) : (r > 0)){
						return r;
					}
				}
			}else if((sNeg && fNeg && rNeg) || (!sNeg && !fNeg && !rNeg)){
				inside = (2*s + 2*f) * (2*s + 2*f) - 8 * (s*s + f*f - p*p);
				if(inside > 0){
					r = (-(2*s + 2*f) + Math.sqrt(inside)) / 4.0;
					rMinus = (-(2*s + 2*f) - Math.sqrt(inside)) / 4.0;
					r = rNeg ? Math.min(r, rMinus) : Math.max(r, rMinus);
					if(rNeg ? (r < 0) : (r > 0)){
						return r;
					}
				}
			}else if((sNeg && !fNeg) || (!sNeg && fNeg)){
				inside = (-2*s + 2*f) * (-2*s + 2*f) - 8 * (s*s + f*f - p*p);
				if(inside > 0){
					r = ((2*s - 2*f) + Math.sqrt(inside)) / 4.0;
					rMinus = ((2*s - 2*f) - Math.sqrt(inside)) / 4.0;
					r = rNeg ? Math.min(r, rMinus) : Math.max(r, rMinus);
					if(rNeg ? (r < 0) : (r > 0)){
						return r;
					}
				}
			}else {
				inside = (-2*s - 2*f) * (-2*s - 2*f) - 8 * (s*s + f*f - p*p);
				if(inside > 0){
					r = ((2*s + 2*f) + Math.sqrt(inside)) / 4.0;
					rMinus = ((2*s + 2*f) - Math.sqrt(inside)) / 4.0;
					r = rNeg ? Math.min(r, rMinus) : Math.max(r, rMinus);
					if(rNeg ? (r < 0) : (r > 0)){
						return r;
					}
				}
			}
		}
		return 0.0;
	}

}
