package pbgLecture4lab;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;

public class Segway {
    private World world;
    private Body frame;
    private Body frontWheel;
    private Body rearWheel;
    private RevoluteJoint frontJoint;
    private RevoluteJoint rearJoint;

    private static final float MOTOR_SPEED = 0.0f;
    private static final float JUMP_FORCE = 20.0f;

    public Segway(World world, float posX, float posY) {
        this.world = world;
        initializeFrame(posX, posY);
        initializeWheels(posX, posY);
        initializeJoints();
    }

    private void initializeFrame(float posX, float posY) {
        BodyDef frameDef = new BodyDef();
        frameDef.type = BodyType.DYNAMIC;
        frameDef.position.set(posX, posY);
        frame = world.createBody(frameDef);

        PolygonShape frameShape = new PolygonShape();
        frameShape.setAsBox(1.0f, 0.1f);
        FixtureDef frameFixture = new FixtureDef();
        frameFixture.shape = frameShape;
        frameFixture.density = 1.0f;
        frameFixture.friction = 0.3f;
        frame.createFixture(frameFixture);
    }

    private void initializeWheels(float posX, float posY) {
        CircleShape wheelShape = new CircleShape();
        wheelShape.m_radius = 0.5f;
        FixtureDef wheelFixture = new FixtureDef();
        wheelFixture.shape = wheelShape;
        wheelFixture.density = 1.0f;
        wheelFixture.friction = 0.9f;

        BodyDef wheelDef = new BodyDef();
        wheelDef.type = BodyType.DYNAMIC;
        wheelDef.position.set(posX + 1.0f, posY - 1.0f);
        frontWheel = world.createBody(wheelDef);
        frontWheel.createFixture(wheelFixture);

        wheelDef.position.set(posX - 1.0f, posY - 1.0f);
        rearWheel = world.createBody(wheelDef);
        rearWheel.createFixture(wheelFixture);
    }

    private void initializeJoints() {
        RevoluteJointDef jointDef = new RevoluteJointDef();
        jointDef.bodyA = frame;
        jointDef.bodyB = frontWheel;
        jointDef.localAnchorA.set(1.0f, -1.0f);
        jointDef.localAnchorB.set(0.0f, 0.0f);
        jointDef.enableMotor = true;
        jointDef.motorSpeed = MOTOR_SPEED;
        jointDef.maxMotorTorque = 10.0f;
        frontJoint = (RevoluteJoint) world.createJoint(jointDef);

        jointDef.bodyB = rearWheel;
        jointDef.localAnchorA.set(-1.0f, -1.0f);
        rearJoint = (RevoluteJoint) world.createJoint(jointDef);
    }

    public void moveRight() {
        frontJoint.setMotorSpeed(-30.0f);
    }

    public void moveLeft() {
        frontJoint.setMotorSpeed(30.0f);
    }

    public void stop() {
        frontJoint.setMotorSpeed(0);
    }

    public void jump() {
        Vec2 force = new Vec2(0, JUMP_FORCE);  // Apply vertical force only
        frame.applyLinearImpulse(force, frame.getWorldCenter());  // Apply the impulse
    }

    public Body getFrame() {
        return frame;
    }

    public Body getFrontWheel() {
        return frontWheel;
    }

    public Body getRearWheel() {
        return rearWheel;
    }
}