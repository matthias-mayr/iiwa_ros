package fri;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.geometricModel.CartDOF;


/**
 * Moves the LBR in a start position, creates an FRI-Session and executes a
 * PositionHold motion with FRI overlay. During this motion joint angles and
 * joint torques can be additionally commanded via FRI.
 */
public class FRI_PC extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;

    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _clientName = "192.170.10.1";
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
            friSession.await(30, TimeUnit.SECONDS);
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }
        getLogger().info("FRI connection established.");

        // Choose control mode
        int modeChoice = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose control mode", "Torque", "Position");
        ClientCommandMode mode = ClientCommandMode.TORQUE;
        if (modeChoice == 0) {
            getLogger().info("Torque control mode chosen");
            mode = ClientCommandMode.TORQUE;
        }
        else if (modeChoice == 1) {
            getLogger().info("Position control mode chosen");
            mode = ClientCommandMode.POSITION;
        }
        else {
            getLogger().warn("Invalid choice: using position control mode");
            mode = ClientCommandMode.POSITION;
        }
        FRIJointOverlay jointOverlay = new FRIJointOverlay(friSession, mode);

        // Define controller 
        PositionHold posHold;  
        if (mode == ClientCommandMode.TORQUE){
        	JointImpedanceControlMode joint_imp_controller = new JointImpedanceControlMode(_lbr.getJointCount());
        	joint_imp_controller.setDampingForAllJoints(0.);
        	
            int choiceStiff = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose stiffness for actuators", "0", "500", "1000", "1500", "2000", "2500");
            double stiffness = 500;
            if (choiceStiff == 0) {
                getLogger().info("Stiffness of '0' chosen");
                stiffness = 0.;
            }
            else if (choiceStiff == 1) {
                getLogger().info("Stiffness of '500' chosen");
                stiffness = 500.;
            }
            else if (choiceStiff == 2) {
                getLogger().info("Stiffness of '1000' chosen");
                stiffness = 1000.;
            }
            else if (choiceStiff == 3) {
                getLogger().info("Stiffness of '1500' chosen");
                stiffness = 1500.;
            }
            else if (choiceStiff == 4) {
                getLogger().info("Stiffness of '2000' chosen");
                stiffness = 2000.;
            }
            else if (choiceStiff == 5) {
                getLogger().info("Stiffness of '2500' chosen");
                stiffness = 2500.;
            }
            else {
                getLogger().warn("Invalid choice: setting stiffness to '1000'");
                stiffness = 1000.;
            }
        	joint_imp_controller.setStiffnessForAllJoints(stiffness);
        	posHold = new PositionHold(joint_imp_controller, -1, TimeUnit.SECONDS);
        }
        else{
		    int controllerChoice = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose controller", "PositionControl", "JointImpedance", "CartesianImpedance");
		    if (controllerChoice == 0) {
		        getLogger().info("PositionControl selected");
		        PositionControlMode pos_controller = new PositionControlMode();
	        	posHold = new PositionHold(pos_controller, -1, TimeUnit.SECONDS);
		    }
		    else if (controllerChoice == 1) {
		        getLogger().info("JointImpedance selected");
	            int choiceStiff = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose stiffness", "0", "100", "300", "500", "800", "1000");
	            double stiffness = 500;
	            if (choiceStiff == 0) {
	                getLogger().info("Stiffness of '0' chosen");
	                stiffness = 0.;
	            }
	            else if (choiceStiff == 1) {
	                getLogger().info("Stiffness of '100' chosen");
	                stiffness = 100.;
	            }
	            else if (choiceStiff == 2) {
	                getLogger().info("Stiffness of '300' chosen");
	                stiffness = 300.;
	            }
	            else if (choiceStiff == 3) {
	                getLogger().info("Stiffness of '500' chosen");
	                stiffness = 500.;
	            }
	            else if (choiceStiff == 4) {
	                getLogger().info("Stiffness of '800' chosen");
	                stiffness = 800.;
	            }
	            else if (choiceStiff == 5) {
	                getLogger().info("Stiffness of '1000' chosen");
	                stiffness = 1000.;
	            }
	            else {
	                getLogger().warn("Invalid choice: setting stiffness to '500'");
	                stiffness = 500.;
	            }
	            
	            double damping = 0.7;
	            int choiceDamp = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose damping", "0", "0.3", "0.5", "0.7", "1");
	            if (choiceDamp == 0) {
	                getLogger().info("Damping of '0' chosen");
	                damping = 0.;
	            }
	            else if (choiceDamp == 1) {
	                getLogger().info("Damping of '0.3' chosen");
	                damping = 0.3;
	            }
	            else if (choiceDamp == 2) {
	                getLogger().info("Damping of '0.5' chosen");
	                damping = 0.5;
	            }
	            else if (choiceDamp == 3) {
	                getLogger().info("Damping of '0.7' chosen");
	                damping = 0.7;
	            }
	            else if (choiceDamp == 4) {
	                getLogger().info("Damping of '1' chosen");
	                damping = 1.;
	            }
	            else {
	                getLogger().warn("Invalid choice: setting damping to '0.7'");
	                damping = 0.7;
	            }
	            
	        	JointImpedanceControlMode joint_imp_controller = new JointImpedanceControlMode(_lbr.getJointCount());
	        	joint_imp_controller.setStiffnessForAllJoints(stiffness);
	        	joint_imp_controller.setDampingForAllJoints(damping);
	        	posHold = new PositionHold(joint_imp_controller, -1, TimeUnit.SECONDS);
		    }
		    else if (controllerChoice == 2) {
		    	getLogger().info("CartesianImpedance selected");
		    	
	            int choiceStiffTrans = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose Stiffness TRANS", "0", "500", "1000", "2000", "3000", "5000");
	            double stiffness_trans = 2000;
	            if (choiceStiffTrans == 0) {
	                getLogger().info("Stiffness TRANS of '0' chosen");
	                stiffness_trans = 0.;
	            }
	            else if (choiceStiffTrans == 1) {
	                getLogger().info("Stiffness TRANS of '500' chosen");
	                stiffness_trans = 500.;
	            }
	            else if (choiceStiffTrans == 2) {
	                getLogger().info("Stiffness TRANS of '1000' chosen");
	                stiffness_trans = 1000.;
	            }
	            else if (choiceStiffTrans == 3) {
	                getLogger().info("Stiffness TRANS of '2000' chosen");
	                stiffness_trans = 2000.;
	            }
	            else if (choiceStiffTrans == 4) {
	                getLogger().info("Stiffness TRANS of '3000' chosen");
	                stiffness_trans = 3000.;
	            }
	            else if (choiceStiffTrans == 5) {
	                getLogger().info("Stiffness TRANS of '5000' chosen");
	                stiffness_trans = 5000.;
	            }
	            else {
	                getLogger().warn("Invalid choice: setting Stiffness TRANS to '2000'");
	                stiffness_trans = 2000.;
	            }
		        
	            int choiceStiffRot = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose Stiffness ROT", "0", "50", "100", "200", "300");
	            double stiffness_rot = 200;
	            if (choiceStiffRot == 0) {
	                getLogger().info("Stiffness TRANS of '0' chosen");
	                stiffness_rot = 0.;
	            }
	            else if (choiceStiffRot == 1) {
	                getLogger().info("Stiffness TRANS of '50' chosen");
	                stiffness_rot = 50.;
	            }
	            else if (choiceStiffRot == 2) {
	                getLogger().info("Stiffness TRANS of '100' chosen");
	                stiffness_rot = 100.;
	            }
	            else if (choiceStiffRot == 3) {
	                getLogger().info("Stiffness TRANS of '200' chosen");
	                stiffness_rot = 200.;
	            }
	            else if (choiceStiffRot == 4) {
	                getLogger().info("Stiffness TRANS of '300' chosen");
	                stiffness_rot = 300.;
	            }
	            else {
	                getLogger().warn("Invalid choice: setting Stiffness TRANS to '200'");
	                stiffness_rot = 200.;
	            }
	            
		        
	            double damping_trans = 0.7;
	            int choiceTransDamp = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose damping TRANS", "0.1", "0.3", "0.5", "0.7", "1");
	            if (choiceTransDamp == 0) {
	                getLogger().info("TRANS Damping of '0.1' chosen");
	                damping_trans = 0.1;
	            }
	            else if (choiceTransDamp == 1) {
	                getLogger().info("TRANS Damping of '0.3' chosen");
	                damping_trans = 0.3;
	            }
	            else if (choiceTransDamp == 2) {
	                getLogger().info("TRANS Damping of '0.5' chosen");
	                damping_trans = 0.5;
	            }
	            else if (choiceTransDamp == 3) {
	                getLogger().info("TRANS Damping of '0.7' chosen");
	                damping_trans = 0.7;
	            }
	            else if (choiceTransDamp == 4) {
	                getLogger().info("TRANS Damping of '1' chosen");
	                damping_trans = 1.;
	            }
	            else {
	                getLogger().warn("Invalid choice: setting TRANS damping to '0.7'");
	                damping_trans = 0.7;
	            }
		        
	            double damping_rot = 0.7;
	            int choiceRotDamp = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose damping ROT", "0.1", "0.3", "0.5", "0.7", "1");
	            if (choiceRotDamp == 0) {
	                getLogger().info("Rot Damping of '0.1' chosen");
	                damping_rot = 0.1;
	            }
	            else if (choiceRotDamp == 1) {
	                getLogger().info("Rot Damping of '0.3' chosen");
	                damping_rot = 0.3;
	            }
	            else if (choiceRotDamp == 2) {
	                getLogger().info("Rot Damping of '0.5' chosen");
	                damping_rot = 0.5;
	            }
	            else if (choiceRotDamp == 3) {
	                getLogger().info("Rot Damping of '0.7' chosen");
	                damping_rot = 0.7;
	            }
	            else if (choiceRotDamp == 4) {
	                getLogger().info("Rot Damping of '1' chosen");
	                damping_rot = 1.;
	            }
	            else {
	                getLogger().warn("Invalid choice: setting ROT damping to '0.7'");
	                damping_rot = 0.7;
	            }

		        double nullSpaceDamping = 0.7;
	            int choiceDampNull = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose NullSpace damping", "0.3", "0.5", "0.7", "0.9", "1");
	            if (choiceDampNull == 0) {
	                getLogger().info("NullSpace damping of '0.3' chosen");
	                nullSpaceDamping = 0.3;
	            }
	            else if (choiceDampNull == 1) {
	                getLogger().info("NullSpace damping of '0.5' chosen");
	                nullSpaceDamping = 0.5;
	            }
	            else if (choiceDampNull == 2) {
	                getLogger().info("NullSpace damping of '0.7' chosen");
	                nullSpaceDamping = 0.7;
	            }
	            else if (choiceDampNull == 3) {
	                getLogger().info("NullSpace damping of '0.9' chosen");
	                nullSpaceDamping = 0.9;
	            }
	            else if (choiceDampNull == 4) {
	                getLogger().info("NullSpace damping of '1' chosen");
	                nullSpaceDamping = 1.;
	            }
	            else {
	                getLogger().warn("Invalid choice: setting NullSpace damping to '0.7'");
	                nullSpaceDamping = 0.7;
	            }
		        
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
		        cart_imp_controller.setNullSpaceDamping(nullSpaceDamping);
	        	posHold = new PositionHold(cart_imp_controller, -1, TimeUnit.SECONDS);
		    }
		    else {
		        getLogger().warn("Invalid choice: using PositionControl");
		        PositionControlMode pos_controller = new PositionControlMode();
	        	posHold = new PositionHold(pos_controller, -1, TimeUnit.SECONDS);
		    }
        }
        getLogger().info("Robot is ready for ROS control.");

        // Control loop
        try {
            _lbr.move(posHold.addMotionOverlay(jointOverlay));
        }
        catch(final CommandInvalidException e) {
            getLogger().error("ROS has been disconnected.");
        }

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
        final FRI_PC app = new FRI_PC();
        app.runApplication();
    }

}
