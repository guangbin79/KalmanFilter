import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.PrintWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.Reader;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.Arrays;

public class KalmanFilter {

    private static float MIN_ACCURACY;
    private long timestamp; // millis
    private double latitude; // degree
    private double longitude; // degree
    private float variance; // P matrix. Initial estimate of error

    public KalmanFilter(float accuracy) {
        MIN_ACCURACY = accuracy;
        variance = -1;
    }

    public void update_gps(float newSpeed, double newLatitude, double newLongitude, long newTimeStamp) {
        // if gps receiver is able to return 'accuracy' of position, change last variable
        process(newSpeed, newLatitude, newLongitude, newTimeStamp, MIN_ACCURACY);
    }

    public double get_latitude() {
        return latitude;
    }

    public double get_longitude() {
        return longitude;
    }

    private void setState(double latitude, double longitude, long timestamp, float accuracy) {
        this.latitude = latitude;
        this.longitude = longitude;
        this.timestamp = timestamp;
        this.variance = accuracy * accuracy;
    }

    /**
     * Kalman filter processing for latitude and longitude
     *
     * newLatitude - new measurement of latitude
     * newLongitude - new measurement of longitude
     * accuracy - measurement of 1 standard deviation error in meters
     * newTimeStamp - time of measurement in millis
     */
    private void process(float newSpeed, double newLatitude, double newLongitude, long newTimeStamp, float newAccuracy) {
        if (newAccuracy < MIN_ACCURACY) {
            newAccuracy = MIN_ACCURACY;
        }

        if (variance < 0) {
            // if variance < 0, object is unitialised, so initialise with current values
            setState(newLatitude, newLongitude, newTimeStamp, newAccuracy);
        } else {
            // else apply Kalman filter
            long duration = newTimeStamp - timestamp;
            if (duration > 0) {
                // time has moved on, so the uncertainty in the current position increases
                variance += duration * newSpeed * newSpeed / 1000;
                timestamp = newTimeStamp;
            }

            // Kalman gain matrix 'k' = Covariance * Inverse(Covariance + MeasurementVariance)
            // because 'k' is dimensionless,
            // it doesn't matter that variance has different units to latitude and longitude
            float k = variance / (variance + newAccuracy * newAccuracy);
            // apply 'k'
            latitude += k * (newLatitude - latitude);
            longitude += k * (newLongitude - longitude);
            // new Covariance matrix is (IdentityMatrix - k) * Covariance
            variance = (1 - k) * variance;
        }
    }

    public static void main(String[] args) {
        String in = args[0];
        String out = args[1];
        float accuracy = Float.parseFloat(args[2]);

        System.out.print(in + " --> " + out + " : " + accuracy + "\r\n");

        ArrayList line_list = readFile(in);
        line_list = smooth(line_list, accuracy);
        writeFile(out, line_list);
    }

    private static ArrayList readFile(String filePath){
        try {
            String encoding = "UTF-8";
            File file = new File(filePath);
            InputStreamReader read = new InputStreamReader(new FileInputStream(file), encoding);
            BufferedReader bufferedReader = new BufferedReader(read);
            String line = null;
            ArrayList<String> line_list = new ArrayList<String>();

            while ((line = bufferedReader.readLine()) != null) {
                line_list.add(line);
            }

            read.close();
            return line_list;
        } catch (Exception e) {
            e.printStackTrace();
        }

        return null;
    }

    private static ArrayList smooth(ArrayList al, float accuracy) {
        // *****************************
        KalmanFilter filter = new KalmanFilter(accuracy);
        // *****************************

        ArrayList<String> line_list = new ArrayList<String>();
        String line = null;
        String[] info = null;
        Iterator iter = al.iterator();

        while (iter.hasNext()) {
            line = (String)iter.next();
            info = line.split(" ");

            // *****************************
            filter.update_gps(Float.parseFloat(info[1]), Double.parseDouble(info[2]), Double.parseDouble(info[3]),  Long.parseLong(info[4]));
            info[2] = Double.toString(filter.get_latitude());
            info[3] = Double.toString(filter.get_longitude());
            // *****************************

            line = String.join(" ", info);
            line_list.add(line);
        }

        return line_list;
    }

    private static void writeFile(String filePath, ArrayList al) {
        try {
            String encoding = "UTF-8";
            File file = new File(filePath);
            OutputStreamWriter write = new OutputStreamWriter(new FileOutputStream(file), encoding);
            BufferedWriter bufferedWriter = new BufferedWriter(write);
            PrintWriter out = new PrintWriter(bufferedWriter);
            String line = null;
            Iterator iter = al.iterator();

            while (iter.hasNext()) {
                line = (String)iter.next();
                out.println(line);
            }

            out.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}

