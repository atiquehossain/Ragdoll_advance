package pbgLecture4lab;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;

import javax.swing.JComponent;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.World;


public class BasicView extends JComponent {
	/* Author: Michael Fairbank
	 * Creation Date: 2016-01-28
	 * Significant changes applied:
	 */
	// background colour
	public static final Color BG_COLOR = Color.white;

	private BasicPhysicsEngine game;

	public BasicView(BasicPhysicsEngine game) {
		this.game = game;
	}
	
	@Override
	public void paintComponent(Graphics g0) {
		BasicPhysicsEngine game;
		synchronized(this) {
			game=this.game;
		}
		
		
		Graphics2D g = (Graphics2D) g0;
		// paint the background
		g.setColor(BG_COLOR);
		g.fillRect(0, 0, getWidth(), getHeight());
		game.draw(g);
		
		for (BasicParticle p : game.particles)
			p.draw(g);
		for (ElasticConnector c : game.connectors)
			c.draw(g);
		for (AnchoredBarrier b : game.barriers)
			b.draw(g);
		for (BasicPolygon p : game.polygons)
			p.draw(g);
		
		
	}

	
	
	


	@Override
	public Dimension getPreferredSize() {
		return BasicPhysicsEngine.FRAME_SIZE;
	}
	
	public synchronized void updateGame(BasicPhysicsEngine game) {
		this.game=game;
	}
}