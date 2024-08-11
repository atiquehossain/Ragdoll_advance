package pbgLecture4lab;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.geom.Path2D;
import java.awt.geom.Path2D.Float;
import java.awt.geom.PathIterator;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;


public class BasicPolygon  {
	/* Author: Michael Fairbank
	 * Creation Date: 2016-02-05 (JBox2d version)
	 * Significant changes applied:
	 */
	public final float ratioOfScreenScaleToWorldScale;

	private final float rollingFriction,mass;
	public final Color col;
	protected final Body body;
	private final Path2D.Float polygonPath;

	public BasicPolygon(String name,float sx, float sy, float vx, float vy,float radius, float width, float height, Color col, float mass, float rollingFriction, int numSides) {
		this(name,sx, sy, vx, vy,radius, width, height, col, mass, rollingFriction,mkRectangle(name, radius,width, height,numSides),numSides);
	}
	
	public BasicPolygon(String name,float sx, float sy, float vx, float vy,float radius, float width, float height, Color col, float mass, float rollingFriction, Path2D.Float polygonPath, int numSides) {
		World w=BasicPhysicsEngine.world; // a Box2D object
		BodyDef bodyDef = new BodyDef();  // a Box2D object
		bodyDef.type = BodyType.DYNAMIC; // this says the physics engine is to move it automatically
		bodyDef.position.set(sx, sy);
		bodyDef.linearVelocity.set(vx, vy);
		bodyDef.angularDamping = 0.1f;
		this.body = w.createBody(bodyDef);
		PolygonShape shape = new PolygonShape();
		Vec2[] vertices = verticesOfPath2D(polygonPath, numSides);
		shape.set(vertices, numSides);
		FixtureDef fixtureDef = new FixtureDef();// This class is from Box2D
		fixtureDef.shape = shape;
		if(name=="Rectangle")
		    fixtureDef.density = (float) (mass/((float) numSides)/2f*(width*height)*Math.sin(2*Math.PI/numSides));
		else
			fixtureDef.density = (float) (mass/((float) numSides)/2f*(radius*radius)*Math.sin(2*Math.PI/numSides));
			
		fixtureDef.friction = 0.1f;// this is surface friction;
		fixtureDef.restitution = 0.5f;
		body.createFixture(fixtureDef);

//		// code to test adding a second fixture:
//		PolygonShape shape2 = new PolygonShape();
//		Vec2[] vertices2 = verticesOfPath2D(polygonPath, numSides);
//		for (int i=0;i<vertices2.length;i++) {
//			vertices2[i]=new Vec2(vertices2[i].x+0.7f,vertices2[i].y+0.7f);
//		}
//		shape2.set(vertices2, numSides);
//		FixtureDef fixtureDef2 = new FixtureDef();// This class is from Box2D
//		fixtureDef2.shape = shape2;
//		fixtureDef2.density = 1;//(float) (mass/(Math.PI*radius*radius));
//		fixtureDef2.friction = 0.1f;
//		fixtureDef2.restitution = 0.5f;
//		body.createFixture(fixtureDef2);
		this.rollingFriction=rollingFriction;
		this.mass=mass;
		this.ratioOfScreenScaleToWorldScale=BasicPhysicsEngine.convertWorldLengthToScreenLength(1);
		System.out.println("Screenradius="+ratioOfScreenScaleToWorldScale);
		this.col=col;
		this.polygonPath=polygonPath;
	}
	
	public void draw(Graphics2D g) {
		g.setColor(col);
		Vec2 position = body.getPosition();
		float angle = body.getAngle(); 
		AffineTransform af = new AffineTransform();
		af.translate(BasicPhysicsEngine.convertWorldXtoScreenX(position.x), BasicPhysicsEngine.convertWorldYtoScreenY(position.y));
		af.scale(ratioOfScreenScaleToWorldScale, -ratioOfScreenScaleToWorldScale);// there is a minus in here because screenworld is flipped upsidedown compared to physics world
		af.rotate(angle); 
		Path2D.Float p = new Path2D.Float (polygonPath,af);
		g.fill(p);
	}



	public void notificationOfNewTimestep() {
		if (rollingFriction>0) {
			Vec2 rollingFrictionForce=new Vec2(body.getLinearVelocity());
			rollingFrictionForce=rollingFrictionForce.mul(-rollingFriction*mass);
			body.applyForceToCenter(rollingFrictionForce);
		}
	}
	
	// Vec2 vertices of Path2D
	public static Vec2[] verticesOfPath2D(Path2D.Float p, int n) {
		Vec2[] result = new Vec2[n];
		float[] values = new float[6];
		PathIterator pi = p.getPathIterator(null);
		int i = 0;
		while (!pi.isDone() && i < n) {
			int type = pi.currentSegment(values);
			if (type == PathIterator.SEG_LINETO) {
				result[i++] = new Vec2(values[0], values[1]);
			}
			pi.next();
		}
		return result;
	}
	public static Path2D.Float mkRectangle(String name,float radius,float width, float height,int n) 
	{
		Path2D.Float p = new Path2D.Float();
		
		if(name=="Rectangle")
		{
			  
		        p.moveTo(-width / 2, -height / 2);
		        p.lineTo(width / 2, -height / 2);
		        p.lineTo(width / 2, height / 2);
		        p.lineTo(-width / 2, height / 2);
		        p.lineTo(-width / 2, -height / 2);
		}
     
        else if(name=="Other")
        {
        	p.moveTo(radius, 0);
    		for (int i = 0; i < n; i++) {
    			float x = (float) (Math.cos((Math.PI * 2 * i) / n) * radius);
    			float y = (float) (Math.sin((Math.PI * 2 * i) / n) * radius);
    			p.lineTo(x, y);
    		}
        }
        
        p.closePath();
        return p;
        
    }
	
}
