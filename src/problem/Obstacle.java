package problem;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;

/**
 * This class represents one of the rectangular obstacles in Assignment 1.
 * 
 * @author lackofcheese
 */
public class Obstacle {
	/** Stores the obstacle as a Rectangle2D */
	private Rectangle2D rect;
	private Map<String, String> orientation = new HashMap<String, String>();
	private static final double boundError = 0.001;

	/**
	 * Constructs an obstacle with the given (x,y) coordinates of the
	 * bottom-left corner, as well as the width and height.
	 * 
	 * @param x
	 *            the minimum x-value.
	 * @param y
	 *            the minimum y-value.
	 * @param w
	 *            the width of the obstacle.
	 * @param h
	 *            the height of the obstacle.
	 */
	public Obstacle(double x, double y, double w, double h) {
		this.rect = new Rectangle2D.Double(x, y, w, h);
		initOrientation(this.rect, boundError);
	}

	/**
	 * Constructs an obstacle from the representation used in the input file:
	 * that is, the x- and y- coordinates of all of the corners of the
	 * rectangle.
	 * 
	 * @param str
	 */
	public Obstacle(String str) {
		Scanner s = new Scanner(str);
		List<Double> xs = new ArrayList<Double>();
		List<Double> ys = new ArrayList<Double>();
		for (int i = 0; i < 4; i++) {
			xs.add(s.nextDouble());
			ys.add(s.nextDouble());
		}
		double xMin = Collections.min(xs);
		double xMax = Collections.max(xs);
		double yMin = Collections.min(ys);
		double yMax = Collections.max(ys);
		this.rect = new Rectangle2D.Double(xMin, yMin, xMax - xMin, yMax - yMin);
		s.close();
		initOrientation(this.rect, boundError);
	}

	public void initOrientation(Rectangle2D rect, double boundError){
		if (rect.getMinX() <= boundError){
			orientation.put("left", "bounded");
		}else {
			orientation.put("left", "notBounded");
		}

		if (1.0 - rect.getMaxX() <= boundError){
			orientation.put("right", "bounded");
		}else {
			orientation.put("right", "notBounded");
		}

		if (1.0 - rect.getMaxY() <= boundError){
			orientation.put("up", "bounded");
		}else {
			orientation.put("up", "notBounded");
		}

		if (rect.getMinY() <= boundError){
			orientation.put("down", "bounded");
		}else {
			orientation.put("down", "notBounded");
		}
	}
	/**
	 * Returns a copy of the Rectangle2D representing this obstacle.
	 * 
	 * @return a copy of the Rectangle2D representing this obstacle.
	 */
	public Rectangle2D getRect() {
		return (Rectangle2D) rect.clone();
	}

	public Map<String, String> getOrientation(){
		return orientation;
	}
	/**
	 * Returns a String representation of this obstacle.
	 * 
	 * @return a String representation of this obstacle.
	 */
	public String toString() {
		return rect.toString();
	}
}
