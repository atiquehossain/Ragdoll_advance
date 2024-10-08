package pbgLecture4lab;


import static pbgLecture4lab.BasicPhysicsEngine.DELTA_T;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;

public class ControllableSpaceShip extends BasicParticle {
	/* Author: Michael Fairbank
	 * Creation Date: 2016-01-28
	 * Significant changes applied:
	 */
	private double angle=0; // direction ship is facing.
	public static final double STEER_RATE = 2 * Math.PI;
	public static final double MAGNITUDE_OF_ENGINE_THRUST_FORCE = 500;

	public ControllableSpaceShip(double sx, double sy, double vx, double vy, double radius, boolean improvedEuler,
			double mass) {
		super(sx, sy, vx, vy, radius, improvedEuler, Color.CYAN, mass, 0, BasicParticle.ShapeType.CIRCLE);
	}
	
	@Override
	public void draw(Graphics2D g) {
		int x = BasicPhysicsEngine.convertWorldXtoScreenX(getPos().x);
		int y = BasicPhysicsEngine.convertWorldYtoScreenY(getPos().y);
		g.setColor(col);
		final int[] XP = { -2, 0, 2, 0 };
		final int[] YP = { 2, -2, 2, 0 };
		final int[] XPTHRUST = { -2, 0, 2, 0 };
		final int[] YPTHRUST = { 2, 3, 2, 0 };
		final double SCALE = SCREEN_RADIUS;

		AffineTransform at = g.getTransform();
		g.translate(x,y);
		double rot = -angle;
		g.rotate(rot);
		g.scale(SCALE, SCALE);
		g.setColor(col);
		g.fillPolygon(XP, YP, XP.length);
		if (BasicKeyListener.isThrustKeyPressed()) {
			g.setColor(Color.red);
			g.fillPolygon(XPTHRUST, YPTHRUST, XPTHRUST.length);
		}
		g.setTransform(at);
	}
	
	@Override
	public void update(double gravity, double deltaT) {
		if (BasicKeyListener.isRotateLeftKeyPressed()) 
			angle+=STEER_RATE * DELTA_T;
		if (BasicKeyListener.isRotateRightKeyPressed()) 
			angle-=STEER_RATE * DELTA_T;
		if (BasicKeyListener.isThrustKeyPressed()) {
			Vect2D force = new Vect2D(0,MAGNITUDE_OF_ENGINE_THRUST_FORCE);
			force=force.rotate(angle);
			applyForceToParticle(force);
		}
		super.update(gravity, deltaT); // do usual move due to gravity.
	}
}
