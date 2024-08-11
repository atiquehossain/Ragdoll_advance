package pbgLecture4lab;

import java.awt.*;
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
import org.jbox2d.dynamics.joints.RevoluteJointDef;

import javax.swing.*;

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
    public static final float WORLD_WIDTH = 10; // meters
    public static final float WORLD_HEIGHT = SCREEN_HEIGHT * (WORLD_WIDTH / SCREEN_WIDTH); // meters

    public static World world;
    public List<Body> bodies;
    private final float SCALE = 30.0f;
    private final float GRAVITY = -0.1f;
    private Body segwayFrame;
    private Body frontWheel;
    private Body rearWheel;
    private double wheelRotation = 0.0;
    private int score = 0;  // To keep track of the score


    // sleep time between two drawn frames in milliseconds
    public static final int DELAY = 20;
    public static final int NUM_EULER_UPDATES_PER_SCREEN_REFRESH = 30;
    // estimate for time between two frames in seconds
    public static final double DELTA_T = DELAY / 1000.0 / NUM_EULER_UPDATES_PER_SCREEN_REFRESH;

    private boolean gameOver = false;



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

    // Obstacle properties
    private double[] obstacleX = new double[2];
    private double[] obstacleY = new double[2];
    private double[] obstacleWidth = new double[2];
    private double[] obstacleHeight = new double[2];


    public List<BasicParticle> particles;
    public List<AnchoredBarrier> barriers;
    public List<ElasticConnector> connectors;
    public List<BasicPolygon> polygons;

    private Body person;

    public static enum LayoutMode {
        CONVEX_ARENA, CONCAVE_ARENA, CONVEX_ARENA_WITH_CURVE, PINBALL_ARENA, RECTANGLE, SNOOKER_TABLE
    }

    public BasicPhysicsEngine() {
        barriers = new ArrayList<>();
        particles = new ArrayList<>();
        connectors = new ArrayList<>();
        polygons = new ArrayList<>();
        bodies = new ArrayList<>();
        LayoutMode layout = LayoutMode.RECTANGLE;

        world = new World(new Vec2(0, GRAVITY));

        initializeObstacles();

        // Create the person body
        createPerson();
        createSegway();
        attachPersonToSegway();

        // Define barriers based on layout
        defineBarriers(layout);
    }

    private void initializeObstacles() {
        double groundHeight = 10.0;
        double groundY = SCREEN_HEIGHT - groundHeight;

        // Initialize the first obstacle
        obstacleWidth[0] = 20 + Math.random() * 40;
        obstacleHeight[0] = 20 + Math.random() * 40;
        obstacleX[0] = Math.random() * (5 - obstacleWidth[0]);
        obstacleY[0] = groundY - obstacleHeight[0];


        do {
            obstacleWidth[1] = 20 + Math.random() * 40;
            obstacleHeight[1] = 20 + Math.random() * 40;
            obstacleX[1] = Math.random() * (900 - obstacleWidth[1]);
            obstacleY[1] = groundY - obstacleHeight[1];
        } while (Math.abs(obstacleX[1] - obstacleX[0]) < obstacleWidth[0] + obstacleWidth[1]);

        // The while loop ensures that the obstacles have a sufficient gap between them
    }

    public void moveObstacleCloser(double speed) {
        for (int i = 0; i < 2; i++) {
            obstacleX[i] -= speed;
            if (obstacleX[i] < 0) {
                obstacleX[i] = SCREEN_WIDTH;
                score++;
            }
        }
    }


// jump functiion
    public void jumpSegwayPerson() {
        Vec2 jumpForce = new Vec2(0, 5);  // The impulse vector (upward force)

        // Apply the impulse to the Segway frame at its center of mass
        segwayFrame.applyLinearImpulse(jumpForce, segwayFrame.getWorldCenter());
        person.applyLinearImpulse(jumpForce, person.getWorldCenter());


    }


    //// attaching person and Segway to jump
    /// trying   to jump
    private void attachPersonToSegway() {
        RevoluteJointDef jointDef = new RevoluteJointDef();
        jointDef.bodyA = segwayFrame;
        jointDef.bodyB = person;
        jointDef.localAnchorA.set(0, 1.0f); // Set the anchor relative to the Segway frame
        jointDef.localAnchorB.set(0, 0);    // Set the anchor relative to the person
        jointDef.collideConnected = false;
        world.createJoint(jointDef);
    }





    private void defineBarriers(LayoutMode layout) {
        switch (layout) {
            case RECTANGLE:
                barriers.add(new AnchoredBarrier_StraightLine(0, 0, WORLD_WIDTH, 0, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH, 0, WORLD_WIDTH, WORLD_HEIGHT, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH, WORLD_HEIGHT, 0, WORLD_HEIGHT, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(0, WORLD_HEIGHT, 0, 0, Color.WHITE));
                break;
            case CONVEX_ARENA:
                barriers.add(new AnchoredBarrier_StraightLine(0, WORLD_HEIGHT / 3, WORLD_WIDTH / 2, 0, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH / 2, 0, WORLD_WIDTH, WORLD_HEIGHT / 3, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH, WORLD_HEIGHT / 3, WORLD_WIDTH, WORLD_HEIGHT, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(WORLD_WIDTH, WORLD_HEIGHT, 0, WORLD_HEIGHT, Color.WHITE));
                barriers.add(new AnchoredBarrier_StraightLine(0, WORLD_HEIGHT, 0, WORLD_HEIGHT / 3, Color.WHITE));
                break;
            case CONCAVE_ARENA:
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
            // Other cases omitted for brevity
        }
    }

    public static void main(String[] args) throws Exception {
        final BasicPhysicsEngine game = new BasicPhysicsEngine();
        final BasicView view = new BasicView(game);
        JEasyFrame frame = new JEasyFrame(view, "Basic Physics Engine");
        frame.addKeyListener(new BasicKeyListener(game)); // Pass the game instance here
        view.addMouseMotionListener(new BasicMouseListener());
        game.startThread(view);
    }

    private void startThread(final BasicView view) throws InterruptedException {
        final BasicPhysicsEngine game = this;
       /* while (true) {
            for (int i = 0; i < NUM_EULER_UPDATES_PER_SCREEN_REFRESH; i++) {
                game.update();
            }*/
        while (!gameOver) {
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
        if (gameOver) return;
        moveObstacleCloser(0.05);
        world.step(1.0f / 60.0f, 6, 2);
        checkCollisionWithObstacles();
        for (BasicParticle p : particles) {
            p.resetTotalForce();
        }
        for (ElasticConnector ec : connectors) {
            ec.applyTensionForceToBothParticles();
        }
        for (BasicParticle p : particles) {
            p.update(9.8, DELTA_T);
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
            for (int m = 0; m < n; m++) {
                BasicParticle p1 = particles.get(n);
                BasicParticle p2 = particles.get(m);
                if (p1.collidesWith(p2)) {
                    BasicParticle.implementElasticCollision(p1, p2, e);
                }
            }
        }
    }

    private void checkCollisionWithObstacles() {
        for (int i = 0; i < 2; i++) {
            if (isColliding(segwayFrame.getPosition(), obstacleX[i], obstacleY[i], obstacleWidth[i], obstacleHeight[i])) {
                gameOver = true;
                showGameOverPopup();
                return;
            }
            if (isColliding(person.getPosition(), obstacleX[i], obstacleY[i], obstacleWidth[i], obstacleHeight[i])) {
                gameOver = true;
                showGameOverPopup();
                return;
            }
        }
    }

    private boolean isColliding(Vec2 bodyPos, double obsX, double obsY, double obsWidth, double obsHeight) {
        float bodyX = bodyPos.x * SCALE;
        float bodyY = SCREEN_HEIGHT - (bodyPos.y * SCALE);

        return bodyX > obsX && bodyX < obsX + obsWidth && bodyY > obsY && bodyY < obsY + obsHeight;
    }


    private void showGameOverPopup() {
        JOptionPane.showMessageDialog(null, "Game Over! Your score is "+score);
        System.exit(0);  // Exits the application after the game over popup
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
        BodyDef personBodyDef = new BodyDef();
        personBodyDef.type = BodyType.DYNAMIC;

        float segwayHeight = 0.2f;
        float wheelRadius = 0.5f;
        float personHeight = 1.5f;
        personBodyDef.position.set(WORLD_WIDTH / 2, WORLD_HEIGHT / 2 + segwayHeight / 2 + wheelRadius + personHeight / 2);

        person = world.createBody(personBodyDef);

        CircleShape headShape = new CircleShape();
        headShape.m_radius = 0.3f;

        FixtureDef headFixture = new FixtureDef();
        headFixture.shape = headShape;
        headFixture.density = 1.0f;
        headFixture.friction = 0.3f;

        person.createFixture(headFixture);
        bodies.add(person);

    }

    public void draw(Graphics2D g) {
        drawGround(g);
        drawObstacles(g);
        drawSegway(g);
        drawPerson(g);
        drawScore(g);
    }

    private void drawPerson(Graphics2D g) {
        Vec2 personPos = person.getPosition();
        double x = personPos.x * SCALE;
        double y = SCREEN_HEIGHT - (personPos.y * SCALE);

        int personHeight = (int) (SCALE * 4);
        int personWidth = (int) (SCALE * 1.5);
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

        g.setColor(new Color(255, 224, 189));
        g.fillOval(headX, headY, personWidth, personWidth);

        g.setColor(Color.ORANGE);
        g.fillOval(headX, headY, personWidth, personWidth / 2);

        g.setColor(new Color(255, 165, 0));
        g.fillRect(bodyX, bodyY, personWidth, bodyHeight);

        g.setColor(new Color(128, 128, 128));
        int rightArmStartX = (int) (x + armLength / 2);
        int rightArmStartY = armY;
        int rightArmEndX = rightArmStartX + 10;
        int rightArmEndY = rightArmStartY + 20;
        int shortHorizontalEndX = rightArmEndX + 15;

        g.drawLine(rightArmStartX, rightArmStartY, rightArmEndX, rightArmEndY);
        g.drawLine(rightArmEndX, rightArmEndY, shortHorizontalEndX, rightArmEndY);
        g.setColor(Color.BLACK);
        g.fillOval(shortHorizontalEndX, rightArmEndY - 5, 20, 15);

        g.setColor(new Color(255, 165, 0));
        g.fillRect(legX, legY, legWidth, legHeight);

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
        double handleHeight = 2.6 * SCALE;

        // Calculate the base point of the handle on the frame
        double handleBaseX = frameX + frameWidth / 2; // Right end of the frame
        double handleBaseY = frameY - frameHeight / 2; // Align with the top of the frame

        // Translate to the base of the handle (this is the pivot point for rotation)
        g.translate(handleBaseX, handleBaseY);

        // Rotate by 30 degrees (in radians: Math.toRadians(30))
        g.rotate(Math.toRadians(20));

        // Draw the rotated handle
        g.setColor(Color.DARK_GRAY); // Handle color
        g.fillRect(0, (int) (-handleHeight), (int) handleWidth, (int) handleHeight);

        // Restore the original transformation
        g.setTransform(originalTransform);

        // Draw the front wheel with spokes
        drawWheel(g, frontWheel, SCALE, groundY);

        // Draw the rear wheel with spokes
        drawWheel(g, rearWheel, SCALE, groundY);
    }

    private void drawWheel(Graphics2D g, Body wheel, float scale, double groundY) {
        Vec2 wheelPos = wheel.getPosition();
        double wheelX = wheelPos.x * scale;
        double wheelRadius = 0.5 * scale;
        double wheelY = groundY - wheelRadius; // Adjust Y to align the wheel bottom with the ground

        // Use the drawSpinningWheel method to draw the spinning wheel
        drawSpinningWheel(g, wheelX, wheelY, wheelRadius);
    }

    private void drawSpinningWheel(Graphics g, double x, double y, double radius) {
        Graphics2D g2d = (Graphics2D) g;
        AffineTransform old = g2d.getTransform();

        // Translate to the center of the wheel, ensuring that the wheel bottom aligns with the ground
        g2d.translate(x, y);

        // Rotate the wheel
        wheelRotation += 0.1; // Increment rotation for animation
        g2d.rotate(wheelRotation);

        // Draw the wheel (centered at the origin after translation)
        g2d.setColor(Color.BLACK);
        g2d.fillOval((int) -radius, (int) -radius, (int) (2 * radius), (int) (2 * radius));

        // Draw the spokes
        g2d.setColor(Color.GRAY);
        for (int i = 0; i < 6; i++) {
            g2d.drawLine(0, 0, (int) radius, 0);
            g2d.rotate(Math.PI / 3); // Rotate by 60 degrees for the next spoke
        }

        g2d.setTransform(old); // Restore the original transform
    }

    private void drawScore(Graphics2D g) {
        g.setColor(Color.WHITE);  // Set the text color
        g.setFont(new Font("Arial", Font.BOLD, 20));  // Set the font
        g.drawString("Score: " + score, 10, 30);  // Draw the score at the top left
    }






    private void drawGround(Graphics2D g) {
        // Define the ground position and dimensions
        double groundHeight = 10.0; // Height of the ground in pixels
        double groundY = SCREEN_HEIGHT - groundHeight; // Y position of the ground (bottom of the screen)

        // Draw the ground as a filled rectangle
        Color bottomColor = new Color(101, 67, 33);
        g.setColor(bottomColor); // Ground color
        g.fillRect(0, (int) groundY, SCREEN_WIDTH, (int) groundHeight);
    }

    private void drawObstacles(Graphics2D g) {
        // Set obstacle properties
        g.setColor(Color.RED); // Obstacle color

        for (int i = 0; i < 2; i++) {
            // Draw each obstacle using the stored properties
            g.fillRect((int) obstacleX[i], (int) obstacleY[i], (int) obstacleWidth[i], (int) obstacleHeight[i]);
        }
    }




}
