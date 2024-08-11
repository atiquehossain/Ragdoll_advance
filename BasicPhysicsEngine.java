package pbgLecture4lab;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics2D;
import java.awt.Toolkit;
import java.awt.geom.AffineTransform;
import java.util.ArrayList;
import java.util.List;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.dynamics.joints.WeldJointDef;


public class BasicPhysicsEngine {
    /* Author: Michael Fairbank
     * Creation Date: 2016-01-28
     * Significant changes applied:
     */

    // frame dimensions
    public static final int SCREEN_HEIGHT = 680;
    public static final int SCREEN_WIDTH = 740;
    public static final Dimension FRAME_SIZE = new Dimension(
            SCREEN_WIDTH, SCREEN_HEIGHT);
    public static final float WORLD_WIDTH = 10;//metres
    public static final float WORLD_HEIGHT = SCREEN_HEIGHT * (WORLD_WIDTH / SCREEN_WIDTH);// meters - keeps world dimensions in same aspect ratio as screen dimensions, so that circles get transformed into circles as opposed to ovals


    public static World world;
    public List<Body> bodies;
    private final float SCALE = 30.0f;
    private final float GRAVITY = -0.1f;
    private Body segwayFrame;
    private Body frontWheel;
    private Body rearWheel;

    // sleep time between two drawn frames in milliseconds
    public static final int DELAY = 20;
    public static final int NUM_EULER_UPDATES_PER_SCREEN_REFRESH = 10;
    // estimate for time between two frames in seconds
    public static final double DELTA_T = DELAY / 1000.0 / NUM_EULER_UPDATES_PER_SCREEN_REFRESH;


    public static int convertWorldXtoScreenX(double worldX) {
        return (int) (worldX / WORLD_WIDTH * SCREEN_WIDTH);
    }

    public static int convertWorldYtoScreenY(double worldY) {
        // minus sign in here is because screen coordinates are upside down.
        return (int) (SCREEN_HEIGHT - (worldY / WORLD_HEIGHT * SCREEN_HEIGHT));
    }

    public static int convertWorldLengthToScreenLength(double worldLength) {
        return (int) (worldLength / WORLD_WIDTH * SCREEN_WIDTH);
    }

    public static double convertScreenXtoWorldX(int screenX) {
        return screenX * WORLD_WIDTH / SCREEN_WIDTH;
        //throw new RuntimeException("Not implemented");
        // TODO: For speed, just copy your solution from lab 3 into here
        // to get this to work you need to program the inverse function to convertWorldXtoScreenX
        // this means rearranging the equation z=(worldX/WORLD_WIDTH*SCREEN_WIDTH) to make worldX the subject,
        // and then returning worldX
        // Use the UnitTest TestScripts_lab4.java to check your solution
        // Ask for help if you need it!
        // Note that a common problem students have found here is that integer truncation happens if you divide two ints in java, e.g. 2/3 is 0!!
        // To work around this problem, instead of x/y, do "((double)x)/((double)y)" or similar.  (Actually, you only need to cast either one of them to a double, not both).
    }

    public static double convertScreenYtoWorldY(int screenY) {
        return (SCREEN_HEIGHT - screenY) * WORLD_HEIGHT / SCREEN_HEIGHT;
        //throw new RuntimeException("Not implemented");
        // to get this to work you need to program the inverse function to convertWorldYtoScreenY
        // this means rearranging the equation z= (SCREEN_HEIGHT-(worldY/WORLD_HEIGHT*SCREEN_HEIGHT)) to make
        // worldY the subject, and then returning worldY
        // Use the UnitTest TestScripts_lab4.java to check your solution
        // Ask for help if you need it!
        // Note that a common problem students have found here is that integer truncation happens if you divide two ints in java, e.g. 2/3 is 0!!
        // To work around this problem, instead of x/y, do "((double)x)/((double)y)" or similar.  (Actually, you only need to cast either one of them to a double, not both).
    }


    public List<BasicParticle> particles;
    public List<AnchoredBarrier> barriers;
    public List<ElasticConnector> connectors;
    public List<BasicPolygon> polygons;

    private Segway segway;
    private Body person;
    private List<Body> obstacles;


    public static enum LayoutMode {CONVEX_ARENA, CONCAVE_ARENA, CONVEX_ARENA_WITH_CURVE, PINBALL_ARENA, RECTANGLE, SNOOKER_TABLE}

    ;

    public BasicPhysicsEngine() {
        barriers = new ArrayList<AnchoredBarrier>();
        // empty particles array, so that when a new thread starts it clears current particle state:
        particles = new ArrayList<BasicParticle>();
        connectors = new ArrayList<ElasticConnector>();
        LayoutMode layout = LayoutMode.RECTANGLE;
        // pinball:
        double r = .2;

        // Simple pendulum attached under mouse pointer
        double rollingFriction = .5;
        double springConstant = 10000, springDampingConstant = 10;
        Double hookesLawTruncation = null;
        boolean canGoSlack = false;
        //particles.add(new ParticleAttachedToMousePointer(WORLD_WIDTH/2,WORLD_HEIGHT/2,0,0, r, true, 10000));
        //particles.add(new BasicParticle(WORLD_WIDTH/2,WORLD_HEIGHT/2-2,0,0, r,true, Color.BLUE, 2*4, rollingFriction));

        world = new World(new Vec2(0, GRAVITY)); // Initialize the world with gravity
        bodies = new ArrayList<>();
        particles = new ArrayList<>();
        connectors = new ArrayList<>();
        polygons = new ArrayList<BasicPolygon>();
        barriers = new ArrayList<>();

        particles.add(new BasicParticle(WORLD_WIDTH / 2, WORLD_HEIGHT / 2 - 2, 0, 0, r, true, Color.BLUE, 2 * 4, rollingFriction, BasicParticle.ShapeType.HEAD));
//		particles.add(new BasicParticle(WORLD_WIDTH/2, WORLD_HEIGHT/2, 0, 0, r, true, Color.RED, 2*4, rollingFriction, BasicParticle.ShapeType.TORSO));
//		particles.add(new BasicParticle(WORLD_WIDTH/2, WORLD_HEIGHT/2+2, 0, 0, r, true, Color.GREEN, 2*4, rollingFriction, BasicParticle.ShapeType.ARM));
//		particles.add(new BasicParticle(WORLD_WIDTH/2, WORLD_HEIGHT/2+4, 0, 0, r, true, Color.YELLOW, 2*4, rollingFriction, BasicParticle.ShapeType.LEG));


        // Create the person body
        createPerson();
        //  addMotorcycle();
        createSegway();

        barriers = new ArrayList<AnchoredBarrier>();

        switch (layout) {
            case RECTANGLE: {
                // rectangle walls:
                // anticlockwise listing
                barriers.add(new AnchoredBarrier_StraightLine(0, 0, WORLD_WIDTH, 0, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH, 0, WORLD_WIDTH, WORLD_HEIGHT, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH, WORLD_HEIGHT, 0, WORLD_HEIGHT, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(0, WORLD_HEIGHT, 0, 0, Color.WHITE));
                break;
            }
            case CONVEX_ARENA: {
                barriers.add(new AnchoredBarrier_StraightLine(0, WORLD_HEIGHT / 3, WORLD_WIDTH / 2, 0, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH / 2, 0, WORLD_WIDTH, WORLD_HEIGHT / 3, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH, WORLD_HEIGHT / 3, WORLD_WIDTH, WORLD_HEIGHT, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH, WORLD_HEIGHT, 0, WORLD_HEIGHT, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(0, WORLD_HEIGHT, 0, WORLD_HEIGHT / 3, Color.WHITE));
                break;
            }
            case CONCAVE_ARENA: {
                barriers.add(new AnchoredBarrier_StraightLine(0, WORLD_HEIGHT / 3, WORLD_WIDTH / 2, 0, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH / 2, 0, WORLD_WIDTH, WORLD_HEIGHT / 3, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH, WORLD_HEIGHT / 3, WORLD_WIDTH, WORLD_HEIGHT, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH, WORLD_HEIGHT, 0, WORLD_HEIGHT, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(0, WORLD_HEIGHT, 0, WORLD_HEIGHT / 3, Color.WHITE));
                double width = WORLD_HEIGHT / 20;
                barriers.add(new AnchoredBarrier_StraightLine(0, WORLD_HEIGHT * 2 / 3, WORLD_WIDTH / 2, WORLD_HEIGHT * 1 / 2, Color.WHITE, width / 10));
                barriers.add(new AnchoredBarrier_Point(WORLD_WIDTH / 2, WORLD_HEIGHT * 1 / 2));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH / 2, WORLD_HEIGHT * 1 / 2, WORLD_WIDTH / 2, WORLD_HEIGHT * 1 / 2 - width, Color.WHITE, width / 10));
                barriers.add(new AnchoredBarrier_Point(WORLD_WIDTH / 2, WORLD_HEIGHT * 1 / 2 - width));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH / 2, WORLD_HEIGHT * 1 / 2 - width, 0, WORLD_HEIGHT * 2 / 3 - width, Color.WHITE, width / 10));
                break;
            }
            case CONVEX_ARENA_WITH_CURVE: {
                barriers.add(new AnchoredBarrier_StraightLine(0, WORLD_HEIGHT / 3, WORLD_WIDTH / 2, 0, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH / 2, 0, WORLD_WIDTH, WORLD_HEIGHT / 3, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH, WORLD_HEIGHT / 3, WORLD_WIDTH, WORLD_HEIGHT, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH, WORLD_HEIGHT, 0, WORLD_HEIGHT, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(0, WORLD_HEIGHT, 0, 0, Color.WHITE));
                barriers.add(new AnchoredBarrier_Curve(WORLD_WIDTH / 2, WORLD_HEIGHT - WORLD_WIDTH / 2, WORLD_WIDTH / 2, 0.0, 180.0, true, Color.WHITE));
                break;
            }
            case PINBALL_ARENA: {
                // simple pinball board
                barriers.add(new AnchoredBarrier_StraightLine(0, 0, WORLD_WIDTH, 0, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH, 0, WORLD_WIDTH, WORLD_HEIGHT, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH, WORLD_HEIGHT, 0, WORLD_HEIGHT, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(0, WORLD_HEIGHT, 0, 0, Color.WHITE));
                barriers.add(new AnchoredBarrier_Curve(WORLD_WIDTH / 2, WORLD_HEIGHT - WORLD_WIDTH / 2, WORLD_WIDTH / 2, 0.0, 200.0, true, Color.WHITE));
                barriers.add(new AnchoredBarrier_Curve(WORLD_WIDTH / 2, WORLD_HEIGHT * 3 / 4, WORLD_WIDTH / 15, -0.0, 360.0, false, Color.WHITE));
                barriers.add(new AnchoredBarrier_Curve(WORLD_WIDTH * 1 / 3, WORLD_HEIGHT * 1 / 2, WORLD_WIDTH / 15, -0.0, 360.0, false, Color.WHITE));
                barriers.add(new AnchoredBarrier_Curve(WORLD_WIDTH * 2 / 3, WORLD_HEIGHT * 1 / 2, WORLD_WIDTH / 15, -0.0, 360.0, false, Color.WHITE));
                break;
            }
            case SNOOKER_TABLE: {
                double snookerTableHeight = WORLD_HEIGHT;
                double pocketSize = 0.4;
                double cushionDepth = 0.3;
                double cushionLength = snookerTableHeight / 2 - pocketSize - cushionDepth;
                double snookerTableWidth = cushionLength + cushionDepth * 2 + pocketSize * 2;

                createCushion(barriers, snookerTableWidth - cushionDepth / 2, snookerTableHeight * 0.25, 0, cushionLength, cushionDepth);
                createCushion(barriers, snookerTableWidth - cushionDepth / 2, snookerTableHeight * 0.75, 0, cushionLength, cushionDepth);
                createCushion(barriers, snookerTableWidth / 2, snookerTableHeight - cushionDepth / 2, Math.PI / 2, cushionLength, cushionDepth);
                createCushion(barriers, cushionDepth / 2, snookerTableHeight * 0.25, Math.PI, cushionLength, cushionDepth);
                createCushion(barriers, cushionDepth / 2, snookerTableHeight * 0.75, Math.PI, cushionLength, cushionDepth);
                createCushion(barriers, snookerTableWidth / 2, cushionDepth / 2, Math.PI * 3 / 2, cushionLength, cushionDepth);


                break;
            }
        }


        //
        //	particles.add(new BasicParticle(r,r,-3,12, r));
//		particles.add(new BasicParticle(0,0,4,10, r,true, Color.BLUE, includeInbuiltCollisionDetection, 2));
//		particles.add(new BasicParticle(0,0,4,10, r,false, Color.RED, includeInbuiltCollisionDetection, 2));
    }

    private void createCushion(List<AnchoredBarrier> barriers, double centrex, double centrey, double orientation, double cushionLength, double cushionDepth) {
        // on entry, we require centrex,centrey to be the centre of the rectangle that contains the cushion.
        Color col = Color.WHITE;
        Vect2D p1 = new Vect2D(cushionDepth / 2, -cushionLength / 2 - cushionDepth / 2);
        Vect2D p2 = new Vect2D(-cushionDepth / 2, -cushionLength / 2);
        Vect2D p3 = new Vect2D(-cushionDepth / 2, +cushionLength / 2);
        Vect2D p4 = new Vect2D(cushionDepth / 2, cushionLength / 2 + cushionDepth / 2);
        p1 = p1.rotate(orientation);
        p2 = p2.rotate(orientation);
        p3 = p3.rotate(orientation);
        p4 = p4.rotate(orientation);
        // we are being careful here to list edges in an anticlockwise manner, so that normals point inwards!
        barriers.add(new AnchoredBarrier_StraightLine(centrex + p1.x, centrey + p1.y, centrex + p2.x, centrey + p2.y, col));
        barriers.add(new AnchoredBarrier_StraightLine(centrex + p2.x, centrey + p2.y, centrex + p3.x, centrey + p3.y, col));
        barriers.add(new AnchoredBarrier_StraightLine(centrex + p3.x, centrey + p3.y, centrex + p4.x, centrey + p4.y, col));
        // oops this will have concave corners so will need to fix that some time!
    }

    public static void main(String[] args) throws Exception {
        final BasicPhysicsEngine game = new BasicPhysicsEngine();
        final BasicView view = new BasicView(game);
        JEasyFrame frame = new JEasyFrame(view, "Basic Physics Engine");
        frame.addKeyListener(new BasicKeyListener());
        view.addMouseMotionListener(new BasicMouseListener());
        game.startThread(view);
    }

    private void startThread(final BasicView view) throws InterruptedException {
        final BasicPhysicsEngine game = this;
        while (true) {
            for (int i = 0; i < NUM_EULER_UPDATES_PER_SCREEN_REFRESH; i++) {
                game.update();
            }
            view.repaint();
            Toolkit.getDefaultToolkit().sync();

            try {
                Thread.sleep(DELAY);
            } catch (InterruptedException e) {
            }
        }
    }


    public void update() {
        world.step(1.0f / 60.0f, 6, 2);
        for (BasicParticle p : particles) {
            p.resetTotalForce();// reset to zero at start of time step, so accumulation of forces can begin.
        }
        for (ElasticConnector ec : connectors) {
            ec.applyTensionForceToBothParticles();
        }
        for (BasicParticle p : particles) {
            p.update(9.8, DELTA_T); // tell each particle to move
        }
        for (BasicParticle particle : particles) {
            for (AnchoredBarrier b : barriers) {
                if (b.isCircleCollidingBarrier(particle.getPos(), particle.getRadius())) {
                    Vect2D bouncedVel = b.calculateVelocityAfterACollision(particle.getPos(), particle.getVel(), 1);
                    particle.setVel(bouncedVel);
                }
            }
        }


        // Check collisions between bodies and barriers
        for (Body body : bodies) {
            Vec2 bodyPos = body.getPosition();
            float radius = 0.3f; // Assuming a standard radius for body (or customize as needed)

            for (AnchoredBarrier b : barriers) {
                if (b.isCircleCollidingBarrier(new Vect2D(bodyPos.x, bodyPos.y), radius)) {
                    Vec2 vel = body.getLinearVelocity();
                    Vect2D bouncedVel = b.calculateVelocityAfterACollision(new Vect2D(bodyPos.x, bodyPos.y), new Vect2D(vel.x, vel.y), 1);
                    body.setLinearVelocity(new Vec2(0, 0));
                }
            }
        }


        double e = 0.9; // coefficient of restitution for all particle pairs
        for (int n = 0; n < particles.size(); n++) {
            for (int m = 0; m < n; m++) {// avoids double check by requiring m<n
                BasicParticle p1 = particles.get(n);
                BasicParticle p2 = particles.get(m);
                if (p1.collidesWith(p2)) {
                    BasicParticle.implementElasticCollision(p1, p2, e);
                }
            }
        }


    }

    private void createSegway() {
        // Define the body definition for the Segway frame
        BodyDef frameDef = new BodyDef();
        frameDef.type = BodyType.DYNAMIC;
        frameDef.position.set(WORLD_WIDTH / 2, WORLD_HEIGHT / 2);

        // Create the frame in the world
        segwayFrame = world.createBody(frameDef);

        // Define and create the frame shape (rectangle)
        PolygonShape frameShape = new PolygonShape();
        frameShape.setAsBox(1.0f, 0.1f); // Frame dimensions (half-width and half-height)

        FixtureDef frameFixture = new FixtureDef();
        frameFixture.shape = frameShape;
        frameFixture.density = 1.0f;
        frameFixture.friction = 0.3f;

        segwayFrame.createFixture(frameFixture);

        // Create the wheels
        CircleShape wheelShape = new CircleShape();
        wheelShape.m_radius = 0.5f; // Wheel radius

        FixtureDef wheelFixture = new FixtureDef();
        wheelFixture.shape = wheelShape;
        wheelFixture.density = 1.0f;
        wheelFixture.friction = 0.9f;

        BodyDef wheelDef = new BodyDef();
        wheelDef.type = BodyType.DYNAMIC;

        // Front wheel
        wheelDef.position.set(WORLD_WIDTH / 2 + 1.0f, WORLD_HEIGHT / 2 - 1.0f);
        frontWheel = world.createBody(wheelDef);
        frontWheel.createFixture(wheelFixture);

        // Rear wheel
        wheelDef.position.set(WORLD_WIDTH / 2 - 1.0f, WORLD_HEIGHT / 2 - 1.0f);
        rearWheel = world.createBody(wheelDef);
        rearWheel.createFixture(wheelFixture);

        // Add bodies to the list for update purposes
        bodies.add(segwayFrame);
        bodies.add(frontWheel);
        bodies.add(rearWheel);

        // Attach wheels to the frame using revolute joints
        RevoluteJointDef jointDef = new RevoluteJointDef();
        jointDef.bodyA = segwayFrame;
        jointDef.localAnchorA.set(1.0f, -1.0f);
        jointDef.bodyB = frontWheel;
        jointDef.localAnchorB.set(0.0f, 0.0f);
        jointDef.enableMotor = false; // Disable motor for simplicity
        world.createJoint(jointDef);

        jointDef.bodyB = rearWheel;
        jointDef.localAnchorA.set(-1.0f, -1.0f);
        world.createJoint(jointDef);
    }


    private void createPerson() {
        // Define the body definition for the person
        BodyDef personBodyDef = new BodyDef();
        personBodyDef.type = BodyType.DYNAMIC;

        // Position the person above the Segway frame
        float segwayHeight = 0.2f; // Frame height from createSegway()
        float wheelRadius = 0.5f; // Wheel radius from createSegway()
        float personHeight = 1.5f; // Approximate height of the person
        personBodyDef.position.set(WORLD_WIDTH / 2, WORLD_HEIGHT / 2 + segwayHeight / 2 + wheelRadius + personHeight / 2);

        // Create the body in the world
        person = world.createBody(personBodyDef);

        // Create the head (circle shape)
        CircleShape headShape = new CircleShape();
        headShape.m_radius = 0.3f;

        FixtureDef headFixture = new FixtureDef();
        headFixture.shape = headShape;
        headFixture.density = 1.0f;
        headFixture.friction = 0.3f;

        person.createFixture(headFixture);

        // Add the person to the bodies list
        bodies.add(person);
    }


    public void draw(Graphics2D g) {
		drawGround(g);
		drawObstacles(g, 5);
        drawSegway(g);
          drawPerson(g);


    }

    private void drawPerson(Graphics2D g) {
        Vec2 personPos = person.getPosition();
        double x = personPos.x * SCALE;
        double y = SCREEN_HEIGHT - (personPos.y * SCALE); // Invert Y for screen coordinates

        int personHeight = (int) (SCALE * 4); // Height of the person in pixels
        int personWidth = (int) (SCALE * 1.5); // Width of the person in pixels
        int bodyHeight = personHeight - personWidth;
        int legWidth = personWidth / 2;
        int legHeight = bodyHeight / 2;
        int shoeWidth = legWidth + 5;
        int shoeHeight = 10;
        int armLength = personWidth;
        int armY = (int) (y - personHeight + personWidth + bodyHeight / 4);

        int headX = (int) (x - personWidth / 2);
        int headY = (int) (y - personHeight);
        int bodyX = headX;
        int bodyY = headY + personWidth;
        int legX = (int) (x - legWidth / 2);
        int legY = bodyY + bodyHeight;
        int shoeX = legX - 2;
        int shoeY = legY + legHeight - shoeHeight;

        g.setStroke(new BasicStroke(5));

        // Draw the head
        g.setColor(new Color(255, 224, 189)); // Skin color
        g.fillOval(headX, headY, personWidth, personWidth);

        // Draw the helmet
        g.setColor(Color.ORANGE);
        g.fillOval(headX, headY, personWidth, personWidth / 2);

        // Draw the body
        g.setColor(new Color(255, 165, 0)); // Uniform color
        g.fillRect(bodyX, bodyY, personWidth, bodyHeight);

        // Draw the right arm
        g.setColor(new Color(128, 128, 128)); // Arm color
        int rightArmStartX = (int) (x + armLength / 2);
        int rightArmStartY = armY;
        int rightArmEndX = rightArmStartX + 10;
        int rightArmEndY = rightArmStartY + 20;
        int shortHorizontalEndX = rightArmEndX + 15;

        g.drawLine(rightArmStartX, rightArmStartY, rightArmEndX, rightArmEndY);
        g.drawLine(rightArmEndX, rightArmEndY, shortHorizontalEndX, rightArmEndY);
        g.setColor(Color.BLACK);
        g.fillOval(shortHorizontalEndX, rightArmEndY - 5, 20, 15);

        // Draw the legs
        g.setColor(new Color(255, 165, 0)); // Pant color
        g.fillRect(legX, legY, legWidth, legHeight);

        // Draw the shoes
        g.setColor(Color.GREEN);
        g.fillRect(shoeX, shoeY, shoeWidth, shoeHeight);
    }


	private void drawSegway(Graphics2D g) {
		final float SCALE = 30.0f;

		// Ground position
		double groundHeight = 10.0;
		double groundY = SCREEN_HEIGHT - groundHeight;

		// Draw the frame
		Vec2 framePos = segwayFrame.getPosition();
		double frameX = framePos.x * SCALE;
		double frameY = groundY - (0.5 * SCALE); // Position the frame just above the ground

		double frameWidth = 2.0 * SCALE;
		double frameHeight = 0.2 * SCALE;
		g.setColor(Color.GRAY); // Frame color

		// Draw the frame (person stands here)
		g.fillRect((int) (frameX - frameWidth / 2), (int) (frameY - frameHeight / 2), (int) frameWidth, (int) frameHeight);

		// Save the original transformation
		AffineTransform originalTransform = g.getTransform();

		// Define the handle dimensions
		double handleWidth = 0.1 * SCALE;
		double handleHeight = 1.5 * SCALE;

		// Calculate the base point of the handle on the frame
		double handleBaseX = frameX + frameWidth / 2; // Right end of the frame
		double handleBaseY = frameY - frameHeight / 2; // Align with the top of the frame

		// Translate to the base of the handle (this is the pivot point for rotation)
		g.translate(handleBaseX, handleBaseY);

		// Rotate by 30 degrees (in radians: Math.toRadians(30))
		g.rotate(Math.toRadians(30));

		// Draw the rotated handle
		g.setColor(Color.DARK_GRAY); // Handle color
		g.fillRect(0, (int) (-handleHeight), (int) handleWidth, (int) handleHeight);

		// Restore the original transformation
		g.setTransform(originalTransform);

		// Draw the front wheel
		Vec2 frontWheelPos = frontWheel.getPosition();
		double frontWheelX = frontWheelPos.x * SCALE;
		double frontWheelY = groundY - (0.5 * SCALE); // Position the wheel just above the ground
		double wheelRadius = 0.5 * SCALE;
		g.setColor(Color.BLACK); // Wheel color
		g.fillOval((int) (frontWheelX - wheelRadius), (int) (frontWheelY - wheelRadius), (int) (2 * wheelRadius), (int) (2 * wheelRadius));

		// Draw the rear wheel
		Vec2 rearWheelPos = rearWheel.getPosition();
		double rearWheelX = rearWheelPos.x * SCALE;
		double rearWheelY = groundY - (0.5 * SCALE); // Position the wheel just above the ground
		g.fillOval((int) (rearWheelX - wheelRadius), (int) (rearWheelY - wheelRadius), (int) (2 * wheelRadius), (int) (2 * wheelRadius));
	}


	private void drawGround(Graphics2D g) {
		// Define the ground position and dimensions
		double groundHeight = 10.0; // Height of the ground in pixels
		double groundY = SCREEN_HEIGHT - groundHeight; // Y position of the ground (bottom of the screen)

		// Draw the ground as a filled rectangle
		g.setColor(Color.DARK_GRAY); // Ground color
		g.fillRect(0, (int) groundY, SCREEN_WIDTH, (int) groundHeight);
	}

	private void drawObstacles(Graphics2D g, int numberOfObstacles) {
		// Define the ground position
		double groundHeight = 10.0;
		double groundY = SCREEN_HEIGHT - groundHeight;

		// Set obstacle properties
		g.setColor(Color.RED); // Obstacle color

		for (int i = 0; i < numberOfObstacles; i++) {
			// Random width and height for the obstacle
			double obstacleWidth = 20 + Math.random() * 40;  // Random width between 20 and 60 pixels
			double obstacleHeight = 20 + Math.random() * 40; // Random height between 20 and 60 pixels

			// Random X position for the obstacle
			double obstacleX = Math.random() * (SCREEN_WIDTH - obstacleWidth);

			// Y position should place the obstacle on the ground
			double obstacleY = groundY - obstacleHeight;

			// Draw the obstacle as a filled rectangle
			g.fillRect((int) obstacleX, (int) obstacleY, (int) obstacleWidth, (int) obstacleHeight);
		}
	}




}


