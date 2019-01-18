/*
  * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef CDSPlotRF_H_
#define CDSPlotRF_H_

#include "RobotLib/RobotInterface.h"
#include "GLTools/GLTools.h"
#include "GMR.h"
#include "CDSExecution.h"
#include "RobotLib/WorldObject.h"
/*
#include "RobotLib/KinematicChain.h"
#include "MathLib/IKGroupSolver.h"
#include "KUKARobotModel/LWRRobot.h"
*/

#define MODE_NONE  0
#define MODE_HOME  1
#define MODE_REACH_CDS 2

#define REACH 1
#define SWIPE 2
#define FLIP  3
#define BACK  4

#define MODEL_DYNAMICS    1
#define LINEAR_DYNAMICS   2
#define NO_DYNAMICS	  3


class CDSPlotRF : public RobotInterface
{
protected:

	Matrix4 	mCurrentRobotEEPose;
	Matrix4		mNextRobotEEPose;
	Matrix4		mRobotAbsolutePose;
	Matrix4		mNextDesiredPose;
	Matrix4 	mRobotInitialPose;
	Matrix4		mTargetPose;

	Matrix4		GetObjectPose();
	Matrix4 	GetStartingPose();

	Matrix4		GetAttractorPose_Reach();
	Matrix4		GetAttractorPose_Swipe();
	Matrix4		GetAttractorPose_Flip();
	Matrix4		GetAttractorPose_Back();

	Matrix mRobTraj;

	CDSExecution* cdsRun;
	
	int mState;
	int reachingThreshold;
	int mCurrentSegment;
	int timeIDX;
	double matNorm(MathLib::Matrix4& mat);

	bool bAttSet;
	bool bSync;
	bool bNextSegment;

public:
             CDSPlotRF();
    virtual ~CDSPlotRF();
  
    virtual Status              RobotInit();
    virtual Status              RobotFree();
  
    virtual Status              RobotStart();    
    virtual Status              RobotStop();
  
    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);
    virtual void              	RobotDraw();
};



#endif 
