import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;
public class ToggleServo{
    private Servo servo;
    private ArrayList<Integer> angles = new ArrayList<>();
    private int anglesLength;
    public int pos;
    public ToggleServo(Servo s, int[] states, Servo.Direction direction){
        this.servo = s;
        this.anglesLength = states.length;
        for(int i = 0; i < this.anglesLength; i++){
            this.angles.add(states[i]);
        }
        this.pos = 0;
        this.servo.setDirection(direction);
        this.servo.setPosition(angles.get(0) / 355.0);
    }
    public void toggle(){
        this.pos++;
        if(this.pos == this.anglesLength) this.pos = 0;
        this.servo.setPosition(angles.get(this.pos) / 355.0);
    }
    public Servo getServo(){return this.servo;}
}