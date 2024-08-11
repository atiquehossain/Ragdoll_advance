package pbgLecture4lab;

import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;

public class BasicKeyListener extends KeyAdapter {
	private BasicPhysicsEngine engine;

	public BasicKeyListener(BasicPhysicsEngine engine) {
		this.engine = engine;
	}

	private static boolean rotateRightKeyPressed, rotateLeftKeyPressed, thrustKeyPressed;

	public static boolean isRotateRightKeyPressed() {
		return rotateRightKeyPressed;
	}

	public static boolean isRotateLeftKeyPressed() {
		return rotateLeftKeyPressed;
	}

	public static boolean isThrustKeyPressed() {
		return thrustKeyPressed;
	}

	@Override
	public void keyPressed(KeyEvent e) {
		int keyCode = e.getKeyCode();
		switch (keyCode) {
			case KeyEvent.VK_LEFT:
				rotateLeftKeyPressed = true;
				break;
			case KeyEvent.VK_RIGHT:
				rotateRightKeyPressed = true;
				break;
			case KeyEvent.VK_SPACE:
				thrustKeyPressed = true;
				engine.jumpSegwayPerson();
				break;
		}
	}

	@Override
	public void keyReleased(KeyEvent e) {
		int key = e.getKeyCode();
		switch (key) {
			case KeyEvent.VK_LEFT:
				rotateLeftKeyPressed = false;
				break;
			case KeyEvent.VK_RIGHT:
				rotateRightKeyPressed = false;
				break;
			case KeyEvent.VK_SPACE:
				thrustKeyPressed = false;
				break;
		}
	}
}
