/*
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "CDSPlotRF.h"

double module_dt = 0.001; //rate at which to querry the learned models

CDSPlotRF::CDSPlotRF():RobotInterface(){\
        mState = MODE_NONE;
}

CDSPlotRF::~CDSPlotRF(){
}

RobotInterface::Status CDSPlotRF::RobotInit(){

	AddConsoleCommand("home");
	AddConsoleCommand("reach");
	AddConsoleCommand("swipe");
	AddConsoleCommand("flip");
	AddConsoleCommand("back");
	AddConsoleCommand("draw_reach");
	AddConsoleCommand("draw_swipe");
	AddConsoleCommand("draw_flip");
	AddConsoleCommand("draw_back");

 	mCurrentRobotEEPose.Identity();
	mNextRobotEEPose.Identity();

 	mRobotInitialPose.Identity();
	mRobotInitialPose = GetStartingPose();

	mTargetPose.Identity();
	GetObjectPose().Mult(GetAttractorPose_Reach(), mTargetPose);

	//cout<< "TargetPose "; mTargetPose.Print();

	bAttSet      = false;
	bNextSegment = false;
	mCurrentSegment = 1;
        reachingThreshold = 0.002;

    return STATUS_OK;
}
RobotInterface::Status CDSPlotRF::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status CDSPlotRF::RobotStart(){
    return STATUS_OK;
}    
RobotInterface::Status CDSPlotRF::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status CDSPlotRF::RobotUpdate(){
    return STATUS_OK;
}
RobotInterface::Status CDSPlotRF::RobotUpdateCore(){

/*	if (bSync == true){
		mRobTraj.Resize(12000, 3);
		timeIDX = 0;
		bSync = false;
	}
*/	
	switch (mState) {
	case MODE_NONE:
	{
		break;
	}
	case MODE_HOME:
	{
		break;
	}
	case MODE_REACH_CDS:
	{

		mNextRobotEEPose = cdsRun->getNextEEPose();
		mCurrentRobotEEPose = mNextRobotEEPose; // We assume we moved there already
		cdsRun->setCurrentEEPose(mCurrentRobotEEPose);
		
/*		Vector3 eepos; eepos.Zero();
		eepos = mCurrentRobotEEPose.GetTranslation();
		mRobTraj(timeIDX, 0) = eepos(0);	
		mRobTraj(timeIDX, 1) = eepos(1);	
		mRobTraj(timeIDX, 2) = eepos(2);	
		timeIDX++;
*/
		break;
	}
	default:
		break;
	}


    return STATUS_OK;
}
int CDSPlotRF::RespondToConsoleCommand(const string cmd, const vector<string> &args){
	if (cmd == "home") {
		mState = MODE_HOME;
		bSync = true;
	} else if (cmd == "reach"){
		mState = MODE_REACH_CDS;
		cdsRun = new CDSExecution();
		cdsRun->initSimple(REACH); 
		cdsRun->setObjectFrame(GetObjectPose());
		cdsRun->setAttractorFrame(GetAttractorPose_Reach());
		cdsRun->setCurrentEEPose(GetStartingPose());
		cdsRun->setDT(module_dt);
		cdsRun->setMotionParameters(1,1,1,reachingThreshold, MODEL_DYNAMICS);
		cdsRun->postInit(); 
		bAttSet = true;
		cout << " >> Reaching with CDS" << endl;
		bSync = true; 
	} else if (cmd == "swipe"){
		mState = MODE_REACH_CDS;
		// Reinitialize CDS and Read new attractor
		delete cdsRun; 
		cdsRun = new CDSExecution();
		cdsRun->initSimple(SWIPE);
		cdsRun->setObjectFrame(GetObjectPose());
		cdsRun->setAttractorFrame(GetAttractorPose_Swipe());
		// Set robot starting pose as the current pose (segments should be continuous)
		cdsRun->setCurrentEEPose(mCurrentRobotEEPose);
		cdsRun->setDT(module_dt);
		cdsRun->setMotionParameters(1,1,1,reachingThreshold, MODEL_DYNAMICS);
		cdsRun->postInit(); 
		bAttSet = true;
		// Re-compute target frame
		bSync = true; 
		GetObjectPose().Mult(GetAttractorPose_Swipe(), mTargetPose);
		cout << " >> Swiping with CDS" << endl;
	} else if (cmd == "flip"){
		mState = MODE_REACH_CDS;
		// Reinitialize CDS and Read new attractor
		delete cdsRun; 
		cdsRun = new CDSExecution();
		cdsRun->initSimple(FLIP);
		cdsRun->setObjectFrame(GetObjectPose());
		cdsRun->setAttractorFrame(GetAttractorPose_Flip());
		// Set robot starting pose as the current pose (segments should be continuous)
		cdsRun->setCurrentEEPose(mCurrentRobotEEPose);
		cdsRun->setDT(0.0001);
		cdsRun->setMotionParameters(1,1,1,reachingThreshold, LINEAR_DYNAMICS);
		cdsRun->postInit(); 
		bAttSet = true;
		bSync = true; 
		// Re-compute target frame
		GetObjectPose().Mult(GetAttractorPose_Flip(), mTargetPose);
		cout << " >> Flipping with CDS" << endl;
	} else if (cmd == "back"){
		mState = MODE_REACH_CDS;
		// Reinitialize CDS and Read new attractor
		delete cdsRun; 
		cdsRun = new CDSExecution();
		cdsRun->initSimple(BACK);
		cdsRun->setObjectFrame(GetObjectPose());
		cdsRun->setAttractorFrame(GetAttractorPose_Back());
		// Set robot starting pose as the current pose (segments should be continuous)
		cdsRun->setCurrentEEPose(mCurrentRobotEEPose);
		cdsRun->setDT(0.0005);
		cdsRun->setMotionParameters(2,1,1,reachingThreshold, LINEAR_DYNAMICS);
		cdsRun->postInit(); 
		bAttSet = true;
		bSync = true; 
		// Re-compute target frame
		GetObjectPose().Mult(GetAttractorPose_Back(), mTargetPose);
		cout << " >> Back with CDS" << endl;
	} else if (cmd == "draw_back"){
		delete cdsRun; 
		cdsRun = new CDSExecution();
		cdsRun->initSimple(BACK);
		cdsRun->setObjectFrame(GetObjectPose());
		cdsRun->setAttractorFrame(GetAttractorPose_Back());
		// Set robot starting pose as the current pose (segments should be continuous)
		cdsRun->setCurrentEEPose(mCurrentRobotEEPose);
		cdsRun->setDT(0.0005);
		cdsRun->setMotionParameters(2,1,1,reachingThreshold, LINEAR_DYNAMICS);
		cdsRun->postInit(); 
		bAttSet = true;
		// Re-compute target frame
		GetObjectPose().Mult(GetAttractorPose_Back(), mTargetPose);

		mRobTraj.Resize(9000,3);
		Vector3 eepos; eepos.Zero();
		for (int i = 0; i<9000; i++){
			mNextRobotEEPose = cdsRun->getNextEEPose();
			mCurrentRobotEEPose = mNextRobotEEPose; // We assume we moved there already
			cdsRun->setCurrentEEPose(mCurrentRobotEEPose);
			eepos = mCurrentRobotEEPose.GetTranslation();
			mRobTraj(i, 0) = eepos(0);	
			mRobTraj(i, 1) = eepos(1);	
			mRobTraj(i, 2) = eepos(2);
		}	
	} else if (cmd == "draw_reach"){
		cdsRun = new CDSExecution();
		cdsRun->initSimple(REACH); 
		cdsRun->setObjectFrame(GetObjectPose());
		cdsRun->setAttractorFrame(GetAttractorPose_Reach());
		cdsRun->setCurrentEEPose(GetStartingPose());
		cdsRun->setDT(module_dt);
		cdsRun->setMotionParameters(1,1,1,reachingThreshold, MODEL_DYNAMICS);
		cdsRun->postInit(); 
		bAttSet = true;
		// Re-compute target frame
		GetObjectPose().Mult(GetAttractorPose_Back(), mTargetPose);

		mRobTraj.Resize(9000,3);
		Vector3 eepos; eepos.Zero();
		for (int i = 0; i<9000; i++){
			mNextRobotEEPose = cdsRun->getNextEEPose();
			mCurrentRobotEEPose = mNextRobotEEPose; // We assume we moved there already
			cdsRun->setCurrentEEPose(mCurrentRobotEEPose);
			eepos = mCurrentRobotEEPose.GetTranslation();
			mRobTraj(i, 0) = eepos(0);	
			mRobTraj(i, 1) = eepos(1);	
			mRobTraj(i, 2) = eepos(2);	
		}
	} else if (cmd == "draw_swipe"){
		delete cdsRun; 
		cdsRun = new CDSExecution();
		cdsRun->initSimple(SWIPE);
		cdsRun->setObjectFrame(GetObjectPose());
		cdsRun->setAttractorFrame(GetAttractorPose_Swipe());
		// Set robot starting pose as the current pose (segments should be continuous)
		cdsRun->setCurrentEEPose(mCurrentRobotEEPose);
		cdsRun->setDT(module_dt);
		cdsRun->setMotionParameters(1,1,1,reachingThreshold, MODEL_DYNAMICS);
		cdsRun->postInit(); 
		bAttSet = true;
		// Re-compute target frame
		bSync = true; 
		GetObjectPose().Mult(GetAttractorPose_Swipe(), mTargetPose);

		mRobTraj.Resize(9000,3);
		Vector3 eepos; eepos.Zero();
		for (int i = 0; i<9000; i++){
			mNextRobotEEPose = cdsRun->getNextEEPose();
			mCurrentRobotEEPose = mNextRobotEEPose; // We assume we moved there already
			cdsRun->setCurrentEEPose(mCurrentRobotEEPose);
			eepos = mCurrentRobotEEPose.GetTranslation();
			mRobTraj(i, 0) = eepos(0);	
			mRobTraj(i, 1) = eepos(1);	
			mRobTraj(i, 2) = eepos(2);	
		}
	} else if (cmd == "draw_flip"){
		delete cdsRun; 
		cdsRun = new CDSExecution();
		cdsRun->initSimple(FLIP);
		cdsRun->setObjectFrame(GetObjectPose());
		cdsRun->setAttractorFrame(GetAttractorPose_Flip());
		// Set robot starting pose as the current pose (segments should be continuous)
		cdsRun->setCurrentEEPose(mCurrentRobotEEPose);
		cdsRun->setDT(0.0001);
		cdsRun->setMotionParameters(5,5,0.5,reachingThreshold, NO_DYNAMICS);
		cdsRun->postInit(); 
		bAttSet = true;
		bSync = true; 
		// Re-compute target frame
		GetObjectPose().Mult(GetAttractorPose_Flip(), mTargetPose);

		mRobTraj.Resize(12000,3);
		Vector3 eepos; eepos.Zero();
		for (int i = 0; i<12000; i++){
			mNextRobotEEPose = cdsRun->getNextEEPose();
			mCurrentRobotEEPose = mNextRobotEEPose; // We assume we moved there already
			cdsRun->setCurrentEEPose(mCurrentRobotEEPose);
			eepos = mCurrentRobotEEPose.GetTranslation();
			mRobTraj(i, 0) = eepos(0);	
			mRobTraj(i, 1) = eepos(1);	
			mRobTraj(i, 2) = eepos(2);	
		}

		bSync = true; 
	}
    return 0;
}

void CDSPlotRF::RobotDraw() {

	Matrix4 mSpatulaTip; mSpatulaTip.Identity();
	mSpatulaTip = mNextRobotEEPose;
	Vector3 vSpatulaTranslation; vSpatulaTranslation.Zero();
	vSpatulaTranslation(0) = 0; vSpatulaTranslation(1) = 0; vSpatulaTranslation(2) = 0.38;
	mSpatulaTip.SetTranslation(mNextRobotEEPose.Transform(vSpatulaTranslation));


	Matrix4 mSpatulaTarget; mSpatulaTarget.Identity();
	mSpatulaTarget = mTargetPose;
	Vector3 vTargetTranslation; vTargetTranslation.Zero();
	vTargetTranslation(0) = 0; vTargetTranslation(1) = 0; vTargetTranslation(2) = 0.38;
	mSpatulaTarget.SetTranslation(mTargetPose.Transform(vTargetTranslation));

	Matrix tmpTraj;
	tmpTraj.Resize(mRobTraj.RowSize(), mRobTraj.ColumnSize());
	tmpTraj = mRobTraj;
	tmpTraj.RemoveZeroRows();
	if (bAttSet == true){
		//GLTools::DrawRef(0.1, &mNextRobotEEPose);
//		GLTools::DrawRef(0.1, &mTargetPose);
		//GLTools::DrawRef(0.1, &mRobotInitialPose);
		GLTools::DrawRef(0.1, &mSpatulaTarget);
		GLTools::DrawRef(0.1, &mSpatulaTip);
		//GLTools::DrawLines(&mNextRobotEEPose.GetTranslation(),0);
		//GLTools::DrawLines(tmpTraj, 1000);
		GLTools::DrawLines(mRobTraj);
		GLTools::DrawCylinder(0.1, 0.3, 10, 1);
	}
}

double CDSPlotRF::matNorm(MathLib::Matrix4& mat)
{
 
  int i,j;
  double sum,nrm;
  for(i=0;i<4;i++)
  {
    for(j=0;j<4;j++)
    {
      sum=sum+pow(mat(i,j),2);
    }
  }
  nrm=sqrt(sum);
  return nrm;
}

Matrix4 CDSPlotRF::GetAttractorPose_Reach(){
	Matrix4 mAttractorPose;
	mAttractorPose.Identity();

	mAttractorPose.SetTranslation(Vector3(-0.3995, -0.2047, 0.3195));

	Matrix3 mAttractorOrient; mAttractorOrient.Identity();
	mAttractorOrient.SetRow(Vector3(-0.6855,   -0.4471,    0.5747),0);
	mAttractorOrient.SetRow(Vector3(-0.3645,    0.8940,    0.2606),1);
	mAttractorOrient.SetRow(Vector3(-0.6303,   -0.0309,   -0.7758),2);	
	mAttractorPose.SetOrientation(mAttractorOrient);

	return mAttractorPose;
}

Matrix4 CDSPlotRF::GetAttractorPose_Swipe(){
	Matrix4 mAttractorPose;
	mAttractorPose.Identity();

	mAttractorPose.SetTranslation(Vector3(-0.2345, -0.1227, 0.1945));
       
	Matrix3 mAttractorOrient; mAttractorOrient.Identity();
	mAttractorOrient.SetRow(Vector3(-0.4255,   -0.4508,    0.7847),0);
	mAttractorOrient.SetRow(Vector3(-0.2966,    0.8887,    0.3497),1);
	mAttractorOrient.SetRow(Vector3(-0.8550,   -0.0840,   -0.5118),2);	
	mAttractorPose.SetOrientation(mAttractorOrient);

 	return mAttractorPose;
}

Matrix4 CDSPlotRF::GetAttractorPose_Flip(){
	Matrix4 mAttractorPose;
	mAttractorPose.Identity();

	mAttractorPose.SetTranslation(Vector3(-0.3474, -0.2123, 0.5265));
	Matrix3 mAttractorOrient; mAttractorOrient.Identity();
    
   	mAttractorOrient.SetRow(Vector3( 0.4269,    0.3564,    0.8311),0);
	mAttractorOrient.SetRow(Vector3(-0.8770,   -0.0609,    0.4766),1);
	mAttractorOrient.SetRow(Vector3( 0.2205,   -0.9324,    0.2866),2);	      
	mAttractorPose.SetOrientation(mAttractorOrient);   

	return mAttractorPose;
}

Matrix4 CDSPlotRF::GetAttractorPose_Back(){
	Matrix4 mAttractorPose;
	mAttractorPose.Identity();            

	mAttractorPose.SetTranslation(Vector3(-0.6486, -0.4265, 0.5713));
       
	Matrix3 mAttractorOrient; mAttractorOrient.Identity();
	mAttractorOrient.SetRow(Vector3(-0.3206,    0.3559,    0.8778),0);
	mAttractorOrient.SetRow(Vector3(-0.1912,    0.8834,   -0.4280),1);
	mAttractorOrient.SetRow(Vector3(-0.9277,   -0.3051,   -0.2152),2);	
	mAttractorPose.SetOrientation(mAttractorOrient);

	return mAttractorPose;
}


Matrix4 CDSPlotRF::GetObjectPose(){

	WorldObject*    oTargetObj;             // the object we're trying to reach
	Matrix4		mObjectPose;
	mObjectPose.Identity();

	// Setting Object
	oTargetObj = GetWorld()->Find("oven");
	if (!oTargetObj) GetConsole()->Print("ERROR: Target object not found");
	mObjectPose = oTargetObj->GetReferenceFrame().GetHMatrix();		

	return mObjectPose;

}

Matrix4 CDSPlotRF::GetStartingPose(){

	Matrix4 mStartingPose, tmpTargetPose;
	mStartingPose.Identity(); tmpTargetPose.Identity();

	// Setting robot starting position as relative to the target (attractor n object)
	Vector3 vStartingPos; vStartingPos.Zero();
	vStartingPos.Set(0.0819, -0.1264, -0.2484);

	Matrix3 mStartingOrient; mStartingOrient.Identity(); 
	mStartingOrient.SetRow(Vector3( 0.9340,    0.3046,   -0.1864),0);
	mStartingOrient.SetRow(Vector3(-0.3180,    0.9470,   -0.0451),1);
	mStartingOrient.SetRow(Vector3( 0.1628,    0.1014,    0.9814),2);

	// Computing starting pose in world            	
	GetObjectPose().Mult(GetAttractorPose_Reach(), tmpTargetPose);
	vStartingPos = tmpTargetPose.Transform(vStartingPos);
	//tmpTargetPose.Print();
	Matrix3 mTmpM3; mTmpM3.Identity();
	tmpTargetPose.GetOrientation().Mult(mStartingOrient, mTmpM3);

	mStartingPose.SetOrientation(mTmpM3);
	mStartingPose.SetTranslation(vStartingPos);

	cout << "Robot Starting Pose in World" << endl; 
	mStartingPose.Print();
	return mStartingPose;

}

extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    CDSPlotRF* create(){return new CDSPlotRF();}
    void destroy(CDSPlotRF* module){delete module;}
}



