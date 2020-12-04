package application;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/**
 * Moves the LBR in a start position, creates an FRI-Session and executes a
 * PositionHold motion with FRI overlay. During this motion joint angles and
 * joint torques can be additionally commanded via FRI.
 */
public class FRIOverlayGripper extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;
    private Tool _toolAttached;

    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _clientName = "192.170.10.1";

        // attach a gripper
        _toolAttached = getApplicationData().createFromTemplate("peg");
        _toolAttached.attachTo(_lbr.getFlange());
    }

    @Override
    public void run()
    {
        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
        // for torque mode, there has to be a command value at least all 5ms
        friConfiguration.setSendPeriodMilliSec(5);
        friConfiguration.setReceiveMultiplier(1);

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        FRISession friSession = new FRISession(friConfiguration);

        // wait until FRI session is ready to switch to command mode
        try
        {
            friSession.await(10, TimeUnit.SECONDS);
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");

        int dialog_reply = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose control mode", "Choose freely", "Position Control", "Preset Joint Imp.", "Preset Cart. Imp.");
        boolean dialog = false;
        if (dialog_reply == 0) {
        	dialog = true;
        }
        int modeChoice;
        if (!dialog) {
        	modeChoice = 1;
        } else {
        	modeChoice = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose control mode", "Torque", "Position", "Wrench");
        }
        
        ClientCommandMode mode = ClientCommandMode.TORQUE;
        if (modeChoice == 0) {
            getLogger().info("Torque control mode chosen");
            mode = ClientCommandMode.TORQUE;
        }
        else if (modeChoice == 1) {
            getLogger().info("Position control mode chosen");
            mode = ClientCommandMode.POSITION;
        }
        else if (modeChoice == 2) {
            getLogger().warn("Wrench control mode not supported yet. Using position control mode instead");
            mode = ClientCommandMode.POSITION;
        }
        else {
            getLogger().warn("Invalid choice: using position control mode");
            mode = ClientCommandMode.POSITION;
        }
        FRIJointOverlay jointOverlay = new FRIJointOverlay(friSession, mode);

        double stiffness = 400.;
        if (dialog) {
        	int choice = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose stiffness for actuators", "0", "20", "50", "150", "300", "400", "500", "600", "800", "1000","1500","2000");

	        if (choice == 0) {
	            getLogger().info("Stiffness of '0' chosen");
	            stiffness = 0.;
	        }
	        else if (choice == 1) {
	            getLogger().info("Stiffness of '20' chosen");
	            stiffness = 20.;
	        }
	        else if (choice == 2) {
	            getLogger().info("Stiffness of '50' chosen");
	            stiffness = 50.;
	        }
	        else if (choice == 3) {
	            getLogger().info("Stiffness of '150' chosen");
	            stiffness = 150.;
	        }
	        else if (choice == 4) {
	            getLogger().info("Stiffness of '300' chosen");
	            stiffness = 300.;
	        }
	        else if (choice == 5) {
	            getLogger().info("Stiffness of '400' chosen");
	            stiffness = 400.;
	        }
	        else if (choice == 6) {
	            getLogger().info("Stiffness of '500' chosen");
	            stiffness = 500.;
	        }
	        else if (choice == 7) {
	            getLogger().info("Stiffness of '600' chosen");
	            stiffness = 600.;
	        }
	        else if (choice == 8) {
	            getLogger().info("Stiffness of '800' chosen");
	            stiffness = 800.;
	        }
	        else if (choice == 9) {
	            getLogger().info("Stiffness of '1000' chosen");
	            stiffness = 1000.;
	        }
	        else if (choice == 10) {
	            getLogger().info("Stiffness of '1500' chosen");
	            stiffness = 1500.;
	        }
	        else if (choice == 11) {
	            getLogger().info("Stiffness of '2000' chosen");
	            stiffness = 2000.;
	        }
	        else {
	            getLogger().warn("Invalid choice: setting stiffness to '20'");
	            stiffness = 20.;
	        }
        }
        // start PositionHold with overlay
        PositionHold posHold;
        if (dialog_reply == 0 || dialog_reply == 2) {
        	getLogger().info("Stiffness set to " + stiffness);
        	JointImpedanceControlMode ctrMode = new JointImpedanceControlMode(stiffness, stiffness, stiffness, stiffness, stiffness, stiffness, stiffness);
        	if (mode == ClientCommandMode.TORQUE)
        		ctrMode.setDampingForAllJoints(0.);
        	posHold = new PositionHold(ctrMode, -1, TimeUnit.SECONDS);
        } else if (dialog_reply == 1) {
	        getLogger().info("PositionControl selected");
	        PositionControlMode pos_controller = new PositionControlMode();
        	posHold = new PositionHold(pos_controller, -1, TimeUnit.SECONDS);
        } else if (dialog_reply == 3) {
        	double stiffness_trans = 1000;
        	double stiffness_rot = 100;
        	double damping_trans = 0.7;
        	double damping_rot = 0.7;
        	double nullspace_damping = 0.7;
        	getLogger().info("Stiffness Translation: " + stiffness_trans + "\nStiffness rotation: " + stiffness_rot + 
        					 "\nDamping Translation: " + damping_trans + "\nDamping Rotation: " + damping_rot + "\nNullspace Damping: " + nullspace_damping);
        			
	        CartesianImpedanceControlMode cart_imp_controller = new CartesianImpedanceControlMode();
	        cart_imp_controller.parametrize(CartDOF.X).setStiffness(stiffness_trans);
	        cart_imp_controller.parametrize(CartDOF.Y).setStiffness(stiffness_trans);
	        cart_imp_controller.parametrize(CartDOF.Z).setStiffness(stiffness_trans);
	        cart_imp_controller.parametrize(CartDOF.X).setDamping(damping_trans);
	        cart_imp_controller.parametrize(CartDOF.Y).setDamping(damping_trans);
	        cart_imp_controller.parametrize(CartDOF.Z).setDamping(damping_trans);
	        cart_imp_controller.parametrize(CartDOF.A).setStiffness(stiffness_rot);
	        cart_imp_controller.parametrize(CartDOF.B).setStiffness(stiffness_rot);
	        cart_imp_controller.parametrize(CartDOF.C).setStiffness(stiffness_rot);
	        cart_imp_controller.parametrize(CartDOF.A).setDamping(damping_rot);
	        cart_imp_controller.parametrize(CartDOF.B).setDamping(damping_rot);
	        cart_imp_controller.parametrize(CartDOF.C).setDamping(damping_rot);
	        cart_imp_controller.setNullSpaceStiffness(0);
	        cart_imp_controller.setNullSpaceDamping(nullspace_damping);
        	posHold = new PositionHold(cart_imp_controller, -1, TimeUnit.SECONDS);

        } else {
        	getLogger().info("Invalid choice. Leaving");
        	return;
        }

        _toolAttached.move(posHold.addMotionOverlay(jointOverlay));

        // done
        friSession.close();
    }

    /**
     * main.
     *
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final FRIOverlayGripper app = new FRIOverlayGripper();
        app.runApplication();
    }

}
