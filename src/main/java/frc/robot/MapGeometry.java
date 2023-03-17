package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;


// NOTE: ALL UNITS ARE METRIC
public class MapGeometry {
	public static List<Translation2d> verts = new ArrayList<>(
		// Java is super ugly sometimes
		// this is necessary to build an arraylist "naturally"
		// note that this goes in a counter clockwise order
		// and duplicates the first point, which should make things easier
		Arrays.asList(
			new Translation2d( 1.33, 0.00), // Lower Left
			new Translation2d(15.00, 0.00), // Lower Right
			new Translation2d(15.00, 5.30),
			new Translation2d(13.10, 5.30),
			new Translation2d(13.10, 5.60),
			new Translation2d(15.80, 5.60),
			new Translation2d(15.80, 7.85), // Upper Right
			new Translation2d( 0.25, 7.85), // Upper Left
			new Translation2d( 0.25, 5.60),
			new Translation2d( 3.30, 5.60),
			new Translation2d( 3.30, 5.30),
			new Translation2d( 1.33, 5.30),
			new Translation2d( 1.33, 0.00)
		)
	);

	public static List<Translation2d> blueBalanceBeam = new ArrayList<>(
		Arrays.asList(
			new Translation2d(2.78, 1.40), // Lower Left
			new Translation2d(4.78, 1.40), // Lower Right
			new Translation2d(4.78, 3.90),
			new Translation2d(2.78, 3.90)
		)
	);

	public static List<Translation2d> redBalanceBeam = new ArrayList<>(
		Arrays.asList(
			new Translation2d(11.60, 1.40), // Lower Left
			new Translation2d(13.55, 1.40), // Lower Right
			new Translation2d(13.55, 3.90),
			new Translation2d(11.60, 3.90)
		)
	);

	private static double BLUE_SLOW_ZONE =  2.85; // less than 2.85 is slow
	private static double RED_SLOW_ZONE  = 13.50; // more than 13.5 is slow
	// anywhere in between in ZOOM ZOOM
	
	/**
	 * Runs a check if a point is inside of the geometry listed in the verts arraylist
	 * 
	 * @param coord the starting coordinate to use
	 * @return boolean True if inside the geometry
	 */
	public static boolean isInMap(Translation2d coord) {
		int count = 0;
		// our geometry is simple enough, we can just do a vertical line (X component)
		double val = coord.getX();
		for(int i = 0; i < verts.size(); i++) {
			// no modulo since we include the starting point a second time at the end
			double min = Math.min(verts.get(i).getX(), verts.get(i+1).getX());
			double max = Math.max(verts.get(i).getX(), verts.get(i+1).getX());
			if (min < val && val < max) {
				count++;
			}
		}
		// if we have an odd number of intersections, we're inside the map!
		return count % 2 == 1;
	}

	/**
	 * We assume the camera is axis aligned forwards for this, so that we can take a lower left
	 * and an upper right coordinate as parameters to define the bounds of the robots
	 * Remember the bumpers, and keep units in metric!
	 * 
	 * @param coord 
	 * @return 
	 */
	public static boolean robotHasGoodClearance(Translation2d coord, Translation2d LL, Translation2d UR) {
		// first, just look if we're inside the field
		// then we can look for whichever edge we're closest to
		if (!isInMap(coord)) {
			return false;
		}

		var center = new Translation2d();
		// only doing two, assuming the camera is in the dead center
		double ll_dist = LL.getDistance(center);
		double ur_dist = UR.getDistance(center);
		double min_dist = Math.max(ll_dist, ur_dist);

		for(int i = 0; i < verts.size(); i++) {
			var a = verts.get(i);
			var ax = a.getX();
			var ay = a.getY();
			var b = verts.get(i+1);
			var bx = b.getX();
			var by = b.getY();

			var dist = Math.abs( (bx-ax)*(ay-coord.getY()) - (ax-coord.getX())*(by-ay) ) / Math.sqrt( (bx-ax)*(bx-ax) + (by-ay)*(by-ay) );

			if( dist < min_dist) {
				return false;
			}
		}

		return true;
	}

	/**
	 * A simple calculation to detect if the robot is near the scoring, or piece pickup zones.
	 * This is useful for 
	 * 
	 * @param coord 
	 * @return 
	 */
	public static boolean isSlowZone(Translation2d coord) {
		if(coord.getX() < BLUE_SLOW_ZONE) {
			return true;
		}
		if(RED_SLOW_ZONE < coord.getX()) {
			return true;
		}
		return false;
	}
}
