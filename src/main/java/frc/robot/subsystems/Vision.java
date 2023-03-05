package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
	Thread m_visionThread;

	public Vision() {
		m_visionThread = new Thread(
			() -> {
				// Get the UsbCamera from CameraServer
				UsbCamera camera = CameraServer.startAutomaticCapture();
				// Set the resolution
				camera.setResolution(640, 480);

				// Get a CvSink. This will capture Mats from the camera
				CvSink cvSink = CameraServer.getVideo();
				// Setup a CvSource. This will send images back to the Dashboard
				CvSource outputStream = CameraServer.putVideo("Regular", 640, 480);
				CvSource purpleStream = CameraServer.putVideo("Purple", 640, 480);

				// Mats are very memory expensive. Lets reuse this Mat.
				Mat mat = new Mat();
				Mat purp_image = new Mat();

				// This cannot be 'true'. The program will never exit if it is. This
				// lets the robot stop this thread when restarting robot code or
				// deploying.
				while (!Thread.interrupted()) {
					// Tell the CvSink to grab a frame from the camera and put it
					// in the source mat.  If there is an error notify the output.
					if (cvSink.grabFrame(mat) == 0) {
						// Send the output the error.
						outputStream.notifyError(cvSink.getError());
						// skip the rest of the current iteration
						continue;
					}
					for(int r = 0; r < mat.rows(); r++) {
						for(int c = 0; c < mat.rows(); c += 1) {
							var res = transformImage(mat.get(r, c));
							purp_image.put(r, c, res);
						}
					}

					// Give the output stream a new image to display
					outputStream.putFrame(mat);
					purpleStream.putFrame(purp_image);
				}
			}
		);
		m_visionThread.setDaemon(true);
		m_visionThread.start();
	}

	// Normalizes the array to a magnitude of 1
	private static void normalize(double[] array) {
		double mag = magnitude(array);
		for(int i = 0; i < array.length; i++) {
			array[i] = array[i] / mag;
		}
	}

	private static double magnitude(double[] array) {
		double mag = 0;
		for (int i = 0; i < array.length; i++) {
			mag += array[i] * array[i];
		}
		return Math.sqrt(mag);
	}

	private static double sum(double[] items) {
		double total = 0;
		for (double item : items) {
			total += item;
		}
		return total;
	}

	private static long sum(int[] items) {
		long total = 0;
		for (int item : items) {
			total += item;
		}
		return total;
	}

	private static double dot(double a, double b, double c, double d, double e, double f) {
		return dot(new double[] {a, b, c}, new double[] {d, e, f});
	}

	private static double dot(double[] a, double[] b) {
		double sum = 0;
		for (int i = 0; i < a.length; i++) {
			sum += a[i] * b[i];	
		}
		return sum;
	}

	private static double[] transformImage(double[] inp) {
		var sim_vector = Constants.VisionConstants.purpleVector;
		// calculate our new color here
		normalize(inp);
		normalize(sim_vector);
		double sim = dot(inp, sim_vector);

		double clr = sim * sim; // optional, just exaggerates the midband
		if(Constants.VisionConstants.colorSimilarityThreshold < clr) {
			double[] final_color = {sim*256, 0, 0};
			return final_color;
		}

		double[] zeros = {0, 0, 0};
		return zeros;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}
}
