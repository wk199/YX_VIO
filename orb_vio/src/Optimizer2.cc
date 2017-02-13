
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"

#include <Eigen/StdVector>

#include "Converter.h"
#include "Optimizer.h"

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>
#include <gtsam/slam/BetweenFactor.h>
// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the structure-from-motion problem incrementally, so
// include iSAM2 here
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Projection factors to model the camera's landmark observations.
// Also, we will initialize the robot at some location using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
/***************************************/
//#include <Imu/include/usb2io.h>
//#include <Imu/include/MPU9250.h>
#include "Imu/include/MyImu.h"
/***************************************/
#include <gtsam/navigation/ImuFactor.h>
//#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <CppUnitLite/TestHarness.h>

#include <boost/bind.hpp>
#include <list>

/***************************************/
#include <vector>
using namespace std;
using namespace gtsam;
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;
using symbol_shorthand::L;
namespace ORB_SLAM
{

	void Optimizer::LocalBundleAdjustmentUseIMU(KeyFrame* pKF, IMUMeasurementList* imu, bool *pbStopFlag )
	{

		// Local KeyFrames: First Breath Search from Current Keyframe
	   // cout << "In localmapping with Frame " << pKF->mnFrameId << ", pbStopFlag = " << *pbStopFlag << std::endl;
		int frameUseForBA = 10;

		if(*pbStopFlag){
			
			cout << "Abort LocalBA in KeyFrame "<<pKF->mnFrameId << std::endl;
			return;
		}
		
		cout<<"Begin LocalBundleAdjustmentUseISAM with KeyFrame Id "<<pKF->mnFrameId<<endl;
		vector<KeyFrame*> lLocalKeyFrames;
		vector<KeyFrame*> lLocalKeyFrames1;

		lLocalKeyFrames1.push_back(pKF);
		pKF->mnBALocalForKF = pKF->mnId;
        long unsigned int curKFId = pKF->mnId;


		vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
		for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
		{
			KeyFrame* pKFi = vNeighKFs[i];
			pKFi->mnBALocalForKF = pKF->mnId;
			if(!pKFi->isBad())
				lLocalKeyFrames1.push_back(pKFi);
		}
		sort(lLocalKeyFrames1.begin(),lLocalKeyFrames1.end(),ORB_SLAM::KeyFrame::lId2);
		bool isEnd = true;
		bool isGood = true;
		for(vector<KeyFrame*>::iterator lit = lLocalKeyFrames1.begin() , lend = lLocalKeyFrames1.end()-1; lit != lend ; lit ++){
			KeyFrame* pKFi = *lit;
			KeyFrame* pKFj = *(lit+1);
			if(isGood){
				if(lLocalKeyFrames.size() < frameUseForBA){
					lLocalKeyFrames.push_back(pKFi);
				}
				else{
					pKFi->mnBALocalForKF = -1;
					pKFi->mnBAFixedForKF = -1;
				}
					
			}
			else{
				pKFi->mnBALocalForKF = -1;
				pKFi->mnBAFixedForKF = -1;
			}
			
			if(pKFi->mnId != pKFj->mnId + 1){
				isEnd = false;
				isGood = false;
				//break;
			}
		}
		//cout << "here"<<std::endl;
		if(isEnd){
			vector<KeyFrame*>::iterator lit = lLocalKeyFrames1.end()-1;
			if(lLocalKeyFrames.size() < frameUseForBA){
				lLocalKeyFrames.push_back(*lit);
			}
			else{
				(*lit)->mnBAFixedForKF = -1;
				(*lit)->mnBALocalForKF = -1;				
			}
				
		}
		else{
			vector<KeyFrame*>::iterator lit = lLocalKeyFrames1.end()-1;
			(*lit)->mnBAFixedForKF = -1;
			(*lit)->mnBALocalForKF = -1;
		}
		if(lLocalKeyFrames.size() < 2){
			return;
		}
		//cout << "here 2" << std::endl;
		sort(lLocalKeyFrames.begin() , lLocalKeyFrames.end(), ORB_SLAM::KeyFrame::lId);
		//cout << "lLocalKeyFrames.size() = " << lLocalKeyFrames.size() << endl;
		 /***************************************************************************/
		LieVector currentvelglobal(Vector3(0.0, 0.0, 0.0));
		Vector3 g(0,9.8015,0);
		Vector3 w_coriolis(0,0,0);
		//imuBias::ConstantBias currentBias = imuBias::ConstantBias(Vector3(-0.466400100200400,0.144949062124249,-0.15159671), Vector3(-0.021269993987976,0.001001244488978,-0.005077731462926));
		Cal3_S2::shared_ptr K(new Cal3_S2(458.654, 457.296, 0.0, 367.215, 248.375));
		noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
		noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 0.1); // one pixel in u and v
		imuBias::ConstantBias currentBias = imuBias::ConstantBias(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0));
		noiseModel::Diagonal::shared_ptr  sigma_init_x = noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<<0.03,0.03,0.03,0.001,0.001,0.001));
		noiseModel::Isotropic::shared_ptr sigma_init_v = noiseModel::Isotropic::Sigma(3, 1000);
		noiseModel::Diagonal::shared_ptr sigma_init_b = noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<<0.0100, 0.0100, 0.0100, 5.00e-05, 5.00e-05, 5.00e-05));
		noiseModel::Diagonal::shared_ptr  sigma_between_b  = noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<<0.00010, 0.00010, 0.00010, 5.00e-06, 5.00e-06, 5.00e-06));
		// Create an iSAM2 object. Unlike iSAM1, which performs periodic batch steps to maintain proper linearization
		// and efficient variable ordering, iSAM2 performs partial relinearization/reordering at each step. A parameter
		// structure is available that allows the user to set various properties, such as the relinearization threshold
		// and type of linear solver. For this example, we we set the relinearization threshold small so the iSAM2 result
		// will approach the batch result.
		ISAM2Params parameters;
		const std::string cc="CHOLESKY";
		parameters.setFactorization(cc);
		//parameters.relinearizeThreshold = 0.0000001;
		parameters.relinearizeSkip = 10;
		ISAM2 isam(parameters);
		// Create a Factor Graph and Values to hold the new data
		NonlinearFactorGraph graph;
		Values initialEstimate;
		unsigned long maxKFid = 0;
		// Loop over the different poses, adding the observations to iSAM incrementally
		vector<int>imu_num;
		//gtsam::Pose3 currentposglobal(pose[0].rotation(),pose[0].translation());
		gtsam::Values currentEstimate;
		////////////////////////////////////////////////////////////////////////////
		vector<MapPoint*> lLocalMapPoints;
		//cout << "lLocalKeyFrames = " << std::endl;
        for(vector<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
        {
			//cout << "lLocalKeyFrames  " << (*lit)->mnId << " has vpMPs "<< endl; 
			//cout << (*lit)->mnId << " ";
            vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
			//cout << vpMPs.size() <<endl;
            for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
            {
                 MapPoint* pMP = *vit;
                 if(pMP){
					 //cout << "pMP->mnBALocalForKF = " << pMP->mnBALocalForKF << ", curKFId = " << curKFId << endl; 
                    if(!pMP->isBad())
                        if(pMP->mnBALocalForKF!= curKFId)
                        {
                            lLocalMapPoints.push_back(pMP);
                            pMP->mnBALocalForKF= curKFId;
                         }					 
				 }
            }
        }
		//cout << " lLocalMapPoints.size() = " << lLocalMapPoints.size() << endl;
		// Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
/*        vector<KeyFrame*> lFixedCameras;
        for(vector<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
        {
             map<KeyFrame*,size_t> observations = (*lit)->GetObservations();
             for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
             {
                 KeyFrame* pKFi = mit->first;

                 if(pKFi->mnBALocalForKF!= curKFId && pKFi->mnBAFixedForKF!= curKFId)
                 {
                     pKFi->mnBAFixedForKF= curKFId;
                     if(!pKFi->isBad())
                         lFixedCameras.push_back(pKFi);
                 }
             }
         }
		 sort(lFixedCameras.begin(),lFixedCameras.end(),ORB_SLAM::KeyFrame::lId2);
*/
		 //cout << "lFixedCameras.size() = " << lFixedCameras.size() << endl;
		 //cout << "lLocalKeyFrames.size() = " << lLocalKeyFrames.size() << endl;
		 // Set Fixed KeyFrame vertices
         //std::ofstream fout;
		 //fout.open("graph.txt");
		 map<int,int> factorindexmap;
		 int factorIndex = 0;
		 int lFixedCamerasIndex = 0;
		 int FixedCamerasNumber = 3;
/*		 for(vector<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++ , lFixedCamerasIndex ++)
         {   
			 if(lFixedCamerasIndex > FixedCamerasNumber)
		        break;    
			 KeyFrame* pKFi = *lit;
             gtsam::Pose3 Tcw = toGTPose3(pKFi->GetPose());
 			 Vector3 currentvelglobal(Vector3(pKFi->mSpeedAndBias.vel[0], pKFi->mSpeedAndBias.vel[1], pKFi->mSpeedAndBias.vel[2]));
			 imuBias::ConstantBias currentBias = imuBias::ConstantBias(Vector3(pKFi->mSpeedAndBias.accbias[0],pKFi->mSpeedAndBias.accbias[1],
			 pKFi->mSpeedAndBias.accbias[2]), Vector3(pKFi->mSpeedAndBias.gyrobias[0],pKFi->mSpeedAndBias.gyrobias[1],pKFi->mSpeedAndBias.gyrobias[2]));
                 
			 initialEstimate.insert(X(pKFi->mnId), Tcw.inverse());
             initialEstimate.insert(V(pKFi->mnId), currentvelglobal);
             initialEstimate.insert(B(pKFi->mnId), currentBias);	

 
             graph.push_back(PriorFactor<Pose3>(X(pKFi->mnId), Tcw.inverse() , sigma_init_x));
			 factorIndex ++;
			 graph.push_back(PriorFactor<imuBias::ConstantBias>(B(pKFi->mnId) , currentBias, sigma_init_b));
			 factorIndex ++;
			 graph.push_back(PriorFactor<Vector3>(V(pKFi->mnId) , currentvelglobal, sigma_init_v));
			 factorIndex ++;
             
			 if(pKFi->mnId>maxKFid)
                 maxKFid=pKFi->mnId;
         }
*/         //std::cerr <<"fixed keyframes "<< lFixedCameras.size()<< std::endl;

         // Set Local KeyFrame vertices
		 int fId = 0;
         for(vector<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
         {
             KeyFrame* pKFi = *lit;
			 fId ++;
             gtsam::Pose3 Tcw = toGTPose3(pKFi->GetPose());
			 if(lit==lend-1)
			 {
				 KeyFrame* Pre_pKFi = *(lit-1);
				 Vector3 currentvelglobal(Vector3(Pre_pKFi->mSpeedAndBias.vel[0], Pre_pKFi->mSpeedAndBias.vel[1], Pre_pKFi->mSpeedAndBias.vel[2]));
			     imuBias::ConstantBias currentBias = imuBias::ConstantBias(Vector3(Pre_pKFi->mSpeedAndBias.accbias[0],Pre_pKFi->mSpeedAndBias.accbias[1],
			     Pre_pKFi->mSpeedAndBias.accbias[2]), Vector3(Pre_pKFi->mSpeedAndBias.gyrobias[0],Pre_pKFi->mSpeedAndBias.gyrobias[1],Pre_pKFi->mSpeedAndBias.gyrobias[2]));
                 
			     initialEstimate.insert(X(pKFi->mnId), Tcw.inverse());
                 initialEstimate.insert(V(pKFi->mnId), currentvelglobal);
                 initialEstimate.insert(B(pKFi->mnId), currentBias);	
			 }
			 else
			 {
				 Vector3 currentvelglobal(Vector3(pKFi->mSpeedAndBias.vel[0], pKFi->mSpeedAndBias.vel[1], pKFi->mSpeedAndBias.vel[2]));
			     imuBias::ConstantBias currentBias = imuBias::ConstantBias(Vector3(pKFi->mSpeedAndBias.accbias[0],pKFi->mSpeedAndBias.accbias[1],
			     pKFi->mSpeedAndBias.accbias[2]), Vector3(pKFi->mSpeedAndBias.gyrobias[0],pKFi->mSpeedAndBias.gyrobias[1],pKFi->mSpeedAndBias.gyrobias[2]));
                 
			     initialEstimate.insert(X(pKFi->mnId), Tcw.inverse());
                 initialEstimate.insert(V(pKFi->mnId), currentvelglobal);
                 initialEstimate.insert(B(pKFi->mnId), currentBias);	
			 }             
             if(/*pKFi->mnId ==0*/fId < FixedCamerasNumber)
		     {
                  //graph.push_back(NonlinearEquality<Pose3>(X(0), Tcw.inverse()));
                  //graph.push_back(PriorFactor<Pose3>( 0, Tcw.inverse(), priorNoisePose));
				  		// the IMU factor and the noise model
				 PreintegratedCombinedMeasurements* imu_measurement_i=imu->Getimufactor(pKFi->mnId);
				 Matrix3 accelerometerCovariance = imu_measurement_i->p().accelerometerCovariance;
				 Matrix3 gyroscopeCovariance = imu_measurement_i->p().gyroscopeCovariance;
				 sigma_init_b = noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<<
						accelerometerCovariance(0,0), gyroscopeCovariance(1,1), gyroscopeCovariance(2,2), 
						gyroscopeCovariance(0,0), gyroscopeCovariance(1,1), gyroscopeCovariance(2,2)));
				  Vector3 currentvelglobal(Vector3(pKFi->mSpeedAndBias.vel[0], pKFi->mSpeedAndBias.vel[1], pKFi->mSpeedAndBias.vel[2]));
			      imuBias::ConstantBias currentBias = imuBias::ConstantBias(Vector3(pKFi->mSpeedAndBias.accbias[0],pKFi->mSpeedAndBias.accbias[1],
			      pKFi->mSpeedAndBias.accbias[2]), Vector3(pKFi->mSpeedAndBias.gyrobias[0],pKFi->mSpeedAndBias.gyrobias[1],pKFi->mSpeedAndBias.gyrobias[2]));
				  graph.push_back(PriorFactor<Pose3>(X(pKFi->mnId), Tcw.inverse() , sigma_init_x));
				  factorIndex ++;
				  //graph.push_back(PriorFactor<Pose3>(X(pKFi->mnId), Tcw.inverse() , sigma_init_x));
				  //factorIndex ++;
				  graph.push_back(PriorFactor<imuBias::ConstantBias>(B(pKFi->mnId) , currentBias, sigma_init_b));
				  factorIndex ++;
				  graph.push_back(PriorFactor<Vector3>(V(pKFi->mnId) , currentvelglobal , sigma_init_v));
				  factorIndex ++;

				  //graph.push_back(NonlinearEquality<imuBias::ConstantBias>(B(pKFi->mnId) , currentBias));
				  //factorIndex ++;
				  //graph.push_back(NonlinearEquality<Vector3>(V(pKFi->mnId) , currentvelglobal));
				  //factorIndex ++;
             }
			 
			 if(lit!=lLocalKeyFrames.begin())
			 {
				 PreintegratedCombinedMeasurements* imu_measurement_i=imu->Getimufactor(pKFi->mnId);
			     int pre_mnId;
			     for(vector<int>::iterator imu_lit=imu->imu_frame_mnId.begin(), imu_lend=imu->imu_frame_mnId.end(); imu_lit!=imu_lend; imu_lit++)
			     {
			    	 if(*imu_lit==pKFi->mnId)
			    	 {
					      imu_lit--;
					      pre_mnId=*imu_lit;
					      break;
				     }
			     }
				 //cout << "Add a IMUFactor between X" << pre_mnId << " and X" <<pKFi->mnId<<endl;  
			     graph.add(CombinedImuFactor(X(pre_mnId),V(pre_mnId),X(pKFi->mnId),V(pKFi->mnId),B(pre_mnId) , B(pKFi->mnId),*imu_measurement_i));
				 factorIndex ++;
			 }
             if(pKFi->mnId>maxKFid)
                 maxKFid=pKFi->mnId;
         }  
		 //build graph
		     // SET MAP POINT VERTICES
		 typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;
		 int mpNum = 0;
		 for(vector<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
         {

            MapPoint* pMP = *lit;
			//vector<KeyFrame*> tEdgeKF;

			//World_Rotation_CtoI
			Cal3_S2::shared_ptr K(new Cal3_S2(pKF->fx , pKF->fy , 0 , pKF->cx , pKF->cy));
			const map<KeyFrame*,size_t> observations = pMP->GetObservations();
			float invSigma2 = pKF->mvInvLevelSigma2[1];	
			for(map<KeyFrame*,size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend ; mit ++){
				KeyFrame* pKFi = mit->first;
				if(pKFi->mnId == pKF->mnId){
					//Point2 obs = curKF->mvKeysUn[(int)(mit->second)];
					const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];
                    // Monocular observation
                    invSigma2 = pKFi->GetInvSigma2(kpUn.octave);
				}
			}
			noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(2,invSigma2);		
			//SmartFactor::shared_ptr smt(new SmartFactor(model , K));
		    SmartProjectionParams sparas;
			sparas.setLinearizationMode(LinearizationMode::JACOBIAN_Q);
			sparas.setDynamicOutlierRejectionThreshold(5);
			SmartFactor::shared_ptr smt(new SmartFactor(model , K , boost::none , sparas ));
            //Set edges
            for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
            {
                KeyFrame* pKFi = mit->first;

                if(pKFi->mnBALocalForKF == curKFId || pKFi->mnBAFixedForKF == curKFId)
                {
                   const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                    // Monocular observation
                    Point2 obs(kpUn.pt.x, kpUn.pt.y);
					smt->add(obs , X(pKFi->mnId));
					//tEdgeKF.push_back(pKFi);
                }
			}
			graph.push_back(smt);
			//vpEdgeKF.push_back(tEdgeKF);
			//factorIndex ++;
			factorindexmap[pMP->mnId] = factorIndex;
			factorIndex ++;
		 }
		 //fout.close();
		 //cout<<"22222This way!!!!!"<<endl;
		 //graph.print();		
		LevenbergMarquardtParams lmParams;  
    	//    lmParams.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
    	lmParams.verbosity = NonlinearOptimizerParams::ERROR;
    	lmParams.maxIterations = 6;
		lmParams.setErrorTol(40.0);
		lmParams.setlambdaFactor(5);
		lmParams.setlambdaInitial(100.0);
		//lmParams.setVerbosityLM(string("TRYDELTA"));
		lmParams.setVerbosityLM(string("SILENT"));
		//lmParams.setVerbosity(string("ERROR"));
		//lmParams.setLogFile(string("log.csv"));
		lmParams.setDiagonalDamping(true);
		//lmParams.setlambdaUpperBound(0.1);
		//lmParams.setVerbosityLM("LevenbergMarquardtParams::TRYLAMBDA");
		//lmParams.print();

    	LevenbergMarquardtOptimizer lmOptimizer(graph, initialEstimate, lmParams);

		//cout << "begin to optimize the IMU" << endl;
    	currentEstimate = lmOptimizer.optimize();
		//cout << "Finish optimize the IMU"<<endl;
		 
         // Recover optimized data
         //Keyframes
		char buf[255];
		sprintf(buf, "result_%d.txt" , pKF->mnId);

		std::ofstream fout;
		//fout.open(buf);
         for(vector<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
         {

            KeyFrame* pKFi = *lit;
            Pose3 Twc =currentEstimate.at<Pose3>(X(pKFi->mnId));
			gtsam::Vector val=Twc.inverse().rotation().quaternion();
			double pX = Twc.inverse().x();
		    double pY = Twc.inverse().y();
			double pZ = Twc.inverse().z();
			//cout<<"XYZ::  "<<pKFi->mTimeStamp<<"  "<<pX<<","<<pY<<","<<pZ<<endl;
			Eigen::Quaterniond _q(val(0), val(1) , val(2) , val(3));
			Eigen::Vector3d _t(pX,pY,pZ);
			g2o::Sim3 sim3_vec(_q , _t , 1);
			Mat Tcw = Converter::toCvMat(sim3_vec);//Rot3(val).toMat();
            pKFi->SetPose(Tcw);
			Vector3 vel=currentEstimate.at<Vector3>(V(pKFi->mnId));
            imuBias::ConstantBias bias = currentEstimate.at<imuBias::ConstantBias>(B(pKFi->mnId));
			Vector3 accbias=bias.accelerometer();
			Vector3 gyrobias=bias.gyroscope();
			for(int i=0;i<3;i++)
			{
			    pKFi->mSpeedAndBias.vel[i]= vel[i];
				pKFi->mSpeedAndBias.gyrobias[i]=gyrobias[i];
				pKFi->mSpeedAndBias.accbias[i]=accbias[i];
			}
			//fout << "Camera " << pKFi->mnId << " = " << Tcw << std::endl;
			//fout << "Velocity = " << vel << std::endl;
			//fout << "Bais = " << accbias << ", " << gyrobias << std::endl;
         }

        //Points
		int pIndex = 0;
		int cpIndex = 0;
        for(vector<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
        {
             MapPoint* pMP = *lit;
			 if(!pMP){
				 //pIndex ++;
				 continue;
			 }
			 
			 int cfactorIndex = factorindexmap[pMP->mnId];
			 SmartFactor::shared_ptr smt = boost::dynamic_pointer_cast<SmartFactor>(graph[cfactorIndex]);
			 if(smt){
				 
				 boost::optional<Point3> pinw = smt->point();
				 if(!pinw){
					const map<KeyFrame*,size_t> observations = pMP->GetObservations();
					for(map<KeyFrame*,size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend ; mit ++){
						KeyFrame* pKFi = mit->first;
						for(vector<KeyFrame*>::iterator kit = lLocalKeyFrames.begin(), kend = lLocalKeyFrames.end(); kit != kend ; kit ++){
							KeyFrame* pKFt = *kit;
							if(pKFi->mnId == pKFt->mnId){
								pKFi->EraseMapPointMatch(pMP);
								pMP->EraseObservation(pKFi);
								break;									
							}	
						}
					}						 
				 }
				 else{
					cv::Mat pinw_pos(3,1,CV_32F);
					pinw_pos.at<float>(0) = pinw->x(); pinw_pos.at<float>(1) = pinw->y() ; pinw_pos.at<float>(2) = pinw->z();
					pMP->SetWorldPos(pinw_pos);
					pMP->UpdateNormalAndDepth();	
					/*const map<KeyFrame*,size_t> observations = pMP->GetObservations();
					//fout << "Point "<< pMP->mnId << " = " << pMP->GetWorldPos().t() << std::endl;
					for(map<KeyFrame*,size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend ; mit ++){
						KeyFrame* pKFi = mit->first;
						if(pKFi->mnBALocalForKF == curKFId || pKFi->mnBAFixedForKF == curKFId){
							//Point2 obs = curKF->mvKeysUn[(int)(mit->second)];
							const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];
							cv::Point2f obs1 = pKFi->projectMapPoint(pMP);
							//fout << "pMP = " << pMP->GetWorldPos() << endl;;
							//fout << "Reproject in Camera " << pKFi->mnId << std::endl;
                            //fout << "obs = (" << obs1.x << ", " << obs1.y << "), KeyPoint = (" << kpUn.pt.x << ", " << kpUn.pt.y << ")" << endl;
							if(pKFi->mnId == pKF->mnId){
								cpIndex ++;
							}		
						}
					}
					*/pIndex ++;
				 
				 }
			 }
			 else{
					const map<KeyFrame*,size_t> observations = pMP->GetObservations();
					for(map<KeyFrame*,size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend ; mit ++){
						KeyFrame* pKFi = mit->first;
						for(vector<KeyFrame*>::iterator kit = lLocalKeyFrames.begin(), kend = lLocalKeyFrames.end(); kit != kend ; kit ++){
							KeyFrame* pKFt = *kit;
							if(pKFi->mnId == pKFt->mnId){
								pKFi->EraseMapPointMatch(pMP);
								pMP->EraseObservation(pKFi);
								break;									
							}	
						}
					}
			 }
			 //pIndex ++;
        }
		//fout << "Total Valid Point Number is " << pIndex << endl;
		//fout << "Camera " << pKF->mnId << " Valid Point Number is " << cpIndex << std::endl;
		//fout << "Total error = " << graph.error(currentEstimate) << std::endl;
		//fout.close();
		cout<<"Finish LocalBundleAdjustmentUseISAM"<<endl;
		
	}

	void Optimizer::LocalBundleAdjustmentUseIMUTemporal(KeyFrame* pKF, IMUMeasurementList* imu, Map* pMap, bool *OptStopFlag, bool *pbStopFlag )
	{

		int frameUseForBA = 10;

		if(*pbStopFlag){
			
			cout << "Abort LocalBA in KeyFrame "<<pKF->mnFrameId << std::endl;
			return;
		}
		
		cout<<"BA with KeyFrame Id "<<pKF->mnId << " FrameId" << pKF->mnFrameId << "......................" << endl;
		vector<KeyFrame*> lLocalKeyFrames;
		//KeyFrame* Nplus1KF ;
		
        long unsigned int curKFId = pKF->mnId;

		vector<KeyFrame*> allKeyFrames = pMap->GetAllKeyFrames();
		sort(allKeyFrames.begin() , allKeyFrames.end(), ORB_SLAM::KeyFrame::lId2);
		for(vector<KeyFrame*>::iterator lit=allKeyFrames.begin(), lend=allKeyFrames.end(); lit!=lend; lit++)
		{
			KeyFrame* pKFi = *lit;
			//cout << pKFi->mnId << " ";
		}
			
		bool iffull = false;
		KeyFrame* Nplus1KF;
		/////////last N keyframes/////////
		for(vector<KeyFrame*>::iterator lit=allKeyFrames.begin() , lend=allKeyFrames.end(); lit!=lend; lit++)
		{
			KeyFrame* pKFi = *lit;
			//cout << pKFi->mnId << " ";
			pKFi->mnBALocalForKF = curKFId;
			lLocalKeyFrames.push_back(pKFi);
			if(lLocalKeyFrames.size() == frameUseForBA && pKFi->mnId > 0)
			{
				lit++;
				Nplus1KF = *lit;
				//cout << Nplus1KF->mnId << std::endl;
				iffull = true;
				break;
			}
				
		}


		sort(lLocalKeyFrames.begin() , lLocalKeyFrames.end(), ORB_SLAM::KeyFrame::lId);
		cout << "Local Keyframes: ";
		for(vector<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
		{
			KeyFrame* pKFi = *lit;
			cout << pKFi->mnId << " ";
		}
		cout << endl;
		//cout << "lLocalKeyFrames.size() = " << lLocalKeyFrames.size() << endl;
		 /***************************************************************************/
		LieVector currentvelglobal(Vector3(0.0, 0.0, 0.0));
		Vector3 g(0,9.8015,0);
		Vector3 w_coriolis(0,0,0);
		//imuBias::ConstantBias currentBias = imuBias::ConstantBias(Vector3(-0.466400100200400,0.144949062124249,-0.15159671), Vector3(-0.021269993987976,0.001001244488978,-0.005077731462926));
		Cal3_S2::shared_ptr K(new Cal3_S2(458.654, 457.296, 0.0, 367.215, 248.375));
		noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
		noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 0.1); // one pixel in u and v
		imuBias::ConstantBias currentBias = imuBias::ConstantBias(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0));
		noiseModel::Diagonal::shared_ptr  sigma_init_x = noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<<0.03,0.03,0.03,0.001,0.001,0.001));
		noiseModel::Isotropic::shared_ptr sigma_init_v = noiseModel::Isotropic::Sigma(3, 1000);
		noiseModel::Diagonal::shared_ptr sigma_init_b = noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<<0.0100, 0.0100, 0.0100, 5.00e-05, 5.00e-05, 5.00e-05));
		noiseModel::Diagonal::shared_ptr  sigma_between_b  = noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<<0.00010, 0.00010, 0.00010, 5.00e-06, 5.00e-06, 5.00e-06));

		// Create a Factor Graph and Values to hold the new data
		NonlinearFactorGraph graph;
		Values initialEstimate;
		unsigned long maxKFid = 0;
		// Loop over the different poses, adding the observations to iSAM incrementally
		vector<int>imu_num;
		//gtsam::Pose3 currentposglobal(pose[0].rotation(),pose[0].translation());
		gtsam::Values currentEstimate;
		////////////////////////////////////////////////////////////////////////////
		vector<MapPoint*> lLocalMapPoints;
        for(vector<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
        {
			//cout << "lLocalKeyFrames  " << (*lit)->mnId << " has vpMPs "<< endl; 
			//cout << (*lit)->mnId << " ";
            vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
			//cout << vpMPs.size() <<endl;
            for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
            {
                 MapPoint* pMP = *vit;
                 if(pMP){
					 //cout << "pMP->mnBALocalForKF = " << pMP->mnBALocalForKF << ", curKFId = " << curKFId << endl; 
                    if(!pMP->isBad())
                        if(pMP->mnBALocalForKF!= curKFId)
                        {
                            lLocalMapPoints.push_back(pMP);
                            pMP->mnBALocalForKF= curKFId;
                         }					 
				 }
            }
        }
		cout << " lLocalMapPoints.size() = " << lLocalMapPoints.size() << endl;
		// Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
 

		vector<KeyFrame*> lFixedCameras;

		 map<int,int> factorindexmap;
		 int factorIndex = 0;
		 int lFixedCamerasIndex = 0;
		 int FixedCamerasNumber = 3;

		 //////// fixed cameras only include the N+1 keyframe
	   	
	
		if(iffull)
		{
			lFixedCameras.push_back(Nplus1KF);
			Nplus1KF->mnBAFixedForKF= curKFId;

			gtsam::Pose3 Tcw = toGTPose3(Nplus1KF->GetPose());
			initialEstimate.insert(X(Nplus1KF->mnId), Tcw.inverse());
			graph.push_back(NonlinearEquality<Pose3>(X(Nplus1KF->mnId), Tcw.inverse()));
			factorIndex ++;
			Vector3 currentvelglobal(Vector3(Nplus1KF->mSpeedAndBias.vel[0], Nplus1KF->mSpeedAndBias.vel[1], Nplus1KF->mSpeedAndBias.vel[2]));
			imuBias::ConstantBias currentBias = imuBias::ConstantBias(Vector3(Nplus1KF->mSpeedAndBias.accbias[0],Nplus1KF->mSpeedAndBias.accbias[1],
			 	Nplus1KF->mSpeedAndBias.accbias[2]), Vector3(Nplus1KF->mSpeedAndBias.gyrobias[0],Nplus1KF->mSpeedAndBias.gyrobias[1],Nplus1KF->mSpeedAndBias.gyrobias[2]));
                 
			 
            initialEstimate.insert(V(Nplus1KF->mnId), currentvelglobal);
            initialEstimate.insert(B(Nplus1KF->mnId), currentBias);	

 			graph.push_back(NonlinearEquality<imuBias::ConstantBias>(B(Nplus1KF->mnId) , currentBias));
			factorIndex ++;
			graph.push_back(NonlinearEquality<Vector3>(V(Nplus1KF->mnId) , currentvelglobal));
			factorIndex ++;
		}
		
		int conum = 3*frameUseForBA;
		vector<KeyFrame*> NeighberKeyFrames = pKF->GetBestCovisibilityKeyFrames(2*frameUseForBA);
		for(vector<KeyFrame*>::iterator lit=NeighberKeyFrames.begin(), lend=NeighberKeyFrames.end(); lit!=lend; lit++)
        {
			if(lFixedCameras.size() >= 10)
				break;
            KeyFrame* pKFi = *lit;
            if(pKFi->mnBALocalForKF!= curKFId && pKFi->mnBAFixedForKF!= curKFId)
            {
                pKFi->mnBAFixedForKF= curKFId;
                if(!pKFi->isBad() && lFixedCameras.size() < frameUseForBA)
                	lFixedCameras.push_back(pKFi);
                 
             }
         }
		sort(lFixedCameras.begin(),lFixedCameras.end(),ORB_SLAM::KeyFrame::lId2);


		for(vector<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
		{
			KeyFrame* pKFi = *lit;
			if(lit != lFixedCameras.begin())
			{
				gtsam::Pose3 Tcw = toGTPose3(pKFi->GetPose());
				initialEstimate.insert(X(pKFi->mnId), Tcw.inverse());
				graph.push_back(NonlinearEquality<Pose3>(X(pKFi->mnId), Tcw.inverse()));
				factorIndex ++;
			}
		}

         // Set Local KeyFrame vertices
		 int fId = 0;
         for(vector<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
         {
             KeyFrame* pKFi = *lit;
			 fId ++;
             gtsam::Pose3 Tcw = toGTPose3(pKFi->GetPose());
			 if(lit==lend-1)
			 {
				 KeyFrame* Pre_pKFi = *(lit-1);
				 Vector3 currentvelglobal(Vector3(Pre_pKFi->mSpeedAndBias.vel[0], Pre_pKFi->mSpeedAndBias.vel[1], Pre_pKFi->mSpeedAndBias.vel[2]));
			     imuBias::ConstantBias currentBias = imuBias::ConstantBias(Vector3(Pre_pKFi->mSpeedAndBias.accbias[0],Pre_pKFi->mSpeedAndBias.accbias[1],
			     Pre_pKFi->mSpeedAndBias.accbias[2]), Vector3(Pre_pKFi->mSpeedAndBias.gyrobias[0],Pre_pKFi->mSpeedAndBias.gyrobias[1],Pre_pKFi->mSpeedAndBias.gyrobias[2]));
                 
			     initialEstimate.insert(X(pKFi->mnId), Tcw.inverse());
                 initialEstimate.insert(V(pKFi->mnId), currentvelglobal);
                 initialEstimate.insert(B(pKFi->mnId), currentBias);	
			 }
			 else
			 {
				 Vector3 currentvelglobal(Vector3(pKFi->mSpeedAndBias.vel[0], pKFi->mSpeedAndBias.vel[1], pKFi->mSpeedAndBias.vel[2]));
			     imuBias::ConstantBias currentBias = imuBias::ConstantBias(Vector3(pKFi->mSpeedAndBias.accbias[0],pKFi->mSpeedAndBias.accbias[1],
			     pKFi->mSpeedAndBias.accbias[2]), Vector3(pKFi->mSpeedAndBias.gyrobias[0],pKFi->mSpeedAndBias.gyrobias[1],pKFi->mSpeedAndBias.gyrobias[2]));
                 
			     initialEstimate.insert(X(pKFi->mnId), Tcw.inverse());
                 initialEstimate.insert(V(pKFi->mnId), currentvelglobal);
                 initialEstimate.insert(B(pKFi->mnId), currentBias);	
			 }             
             if(pKFi->mnId ==0)
		     {
                  
				  Vector3 currentvelglobal(Vector3(pKFi->mSpeedAndBias.vel[0], pKFi->mSpeedAndBias.vel[1], pKFi->mSpeedAndBias.vel[2]));
			      imuBias::ConstantBias currentBias = imuBias::ConstantBias(Vector3(pKFi->mSpeedAndBias.accbias[0],pKFi->mSpeedAndBias.accbias[1],
			      pKFi->mSpeedAndBias.accbias[2]), Vector3(pKFi->mSpeedAndBias.gyrobias[0],pKFi->mSpeedAndBias.gyrobias[1],pKFi->mSpeedAndBias.gyrobias[2]));
				  graph.push_back(PriorFactor<Pose3>(X(pKFi->mnId), Tcw.inverse() , sigma_init_x));
				  factorIndex ++;
				  graph.push_back(PriorFactor<imuBias::ConstantBias>(B(pKFi->mnId) , currentBias, sigma_init_b));
				  factorIndex ++;
				  graph.push_back(PriorFactor<Vector3>(V(pKFi->mnId) , currentvelglobal , sigma_init_v));
				  factorIndex ++;
             }
			 
			 {
				 PreintegratedCombinedMeasurements* imu_measurement_i=imu->Getimufactor(pKFi->mnId);
			     int pre_mnId;
			     for(vector<int>::iterator imu_lit=imu->imu_frame_mnId.begin(), imu_lend=imu->imu_frame_mnId.end(); imu_lit!=imu_lend; imu_lit++)
			     {
			    	 if(*imu_lit==pKFi->mnId)
			    	 {
					      imu_lit--;
					      pre_mnId=*imu_lit;
					      break;
				     }
			     }
				 //cout << "Add a IMUFactor between X" << pre_mnId << " and X" <<pKFi->mnId<<endl;  
			     graph.add(CombinedImuFactor(X(pre_mnId),V(pre_mnId),X(pKFi->mnId),V(pKFi->mnId),B(pre_mnId) , B(pKFi->mnId),*imu_measurement_i));
				 factorIndex ++;
			 }
             if(pKFi->mnId>maxKFid)
                 maxKFid=pKFi->mnId;
         }  

		 //build graph
		     // SET MAP POINT VERTICES
		 typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;
		 int mpNum = 0;
		 for(vector<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
         {

            MapPoint* pMP = *lit;
			//vector<KeyFrame*> tEdgeKF;

			//World_Rotation_CtoI
			Cal3_S2::shared_ptr K(new Cal3_S2(pKF->fx , pKF->fy , 0 , pKF->cx , pKF->cy));
			const map<KeyFrame*,size_t> observations = pMP->GetObservations();
			float Sigma2 = 1;	
			for(map<KeyFrame*,size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend ; mit ++){
				KeyFrame* pKFi = mit->first;
				if(pKFi->mnId == pKF->mnId){
					//Point2 obs = curKF->mvKeysUn[(int)(mit->second)];
					const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];
                    // Monocular observation
                    Sigma2 = pKFi->GetSigma2(kpUn.octave);
				}
			}
			noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(2,sqrt(Sigma2));		
			//SmartFactor::shared_ptr smt(new SmartFactor(model , K));
		    SmartProjectionParams sparas;
			sparas.setLinearizationMode(LinearizationMode::JACOBIAN_Q);
			sparas.setDynamicOutlierRejectionThreshold(5);
			SmartFactor::shared_ptr smt(new SmartFactor(model , K , boost::none , sparas ));
            //Set edges
            for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
            {
                KeyFrame* pKFi = mit->first;

                if(pKFi->mnBALocalForKF == curKFId || pKFi->mnBAFixedForKF == curKFId)
                {
                   const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                    // Monocular observation
                    Point2 obs(kpUn.pt.x, kpUn.pt.y);
					smt->add(obs , X(pKFi->mnId));
					//tEdgeKF.push_back(pKFi);
                }
			}
			graph.push_back(smt);
			//vpEdgeKF.push_back(tEdgeKF);
			//factorIndex ++;
			factorindexmap[pMP->mnId] = factorIndex;
			factorIndex ++;
		 }
		 //fout.close();
		 //cout<<"22222This way!!!!!"<<endl;
		 //graph.print();		
		LevenbergMarquardtParams lmParams;  
    	//    lmParams.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
    	lmParams.verbosity = NonlinearOptimizerParams::SILENT;
    	lmParams.maxIterations = 6;
		lmParams.setErrorTol(40.0);
		lmParams.setlambdaFactor(10);
		lmParams.setlambdaInitial(150);
		//lmParams.setVerbosityLM(string("TRYDELTA"));
		lmParams.setVerbosityLM(string("SILENT"));
		//lmParams.setVerbosity(string("ERROR"));
		//lmParams.setLogFile(string("log.csv"));
		lmParams.setDiagonalDamping(true); 
		//lmParams.setlambdaUpperBound(0.1);
		//lmParams.setVerbosityLM("LevenbergMarquardtParams::TRYLAMBDA");
		//lmParams.print();  

    	LevenbergMarquardtOptimizer lmOptimizer(graph, initialEstimate, lmParams);
		lmOptimizer.setOptStopFlag(OptStopFlag);

		//cout << "begin to optimize the IMU" << endl;
    	currentEstimate = lmOptimizer.optimize();

		*OptStopFlag = false;
		//cout << "Finish optimize the IMU"<<endl;
		 
         // Recover optimized data
         //Keyframes
		char buf[255];
		sprintf(buf, "result_%d.txt" , pKF->mnId);

		std::ofstream fout;
		//fout.open(buf);
         for(vector<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
         {

            KeyFrame* pKFi = *lit;
            Pose3 Twc =currentEstimate.at<Pose3>(X(pKFi->mnId));
			gtsam::Vector val=Twc.inverse().rotation().quaternion();
			double pX = Twc.inverse().x();
		    double pY = Twc.inverse().y();
			double pZ = Twc.inverse().z();
			//cout<<"XYZ::  "<<pKFi->mTimeStamp<<"  "<<pX<<","<<pY<<","<<pZ<<endl;
			Eigen::Quaterniond _q(val(0), val(1) , val(2) , val(3));
			Eigen::Vector3d _t(pX,pY,pZ);
			g2o::Sim3 sim3_vec(_q , _t , 1);
			Mat Tcw = Converter::toCvMat(sim3_vec);//Rot3(val).toMat();
            pKFi->SetPose(Tcw);
			Vector3 vel=currentEstimate.at<Vector3>(V(pKFi->mnId));
            imuBias::ConstantBias bias = currentEstimate.at<imuBias::ConstantBias>(B(pKFi->mnId));
			Vector3 accbias=bias.accelerometer();
			Vector3 gyrobias=bias.gyroscope();
			for(int i=0;i<3;i++)
			{
			    pKFi->mSpeedAndBias.vel[i]= vel[i];
				pKFi->mSpeedAndBias.gyrobias[i]=gyrobias[i];
				pKFi->mSpeedAndBias.accbias[i]=accbias[i];
			}
			//fout << "Camera " << pKFi->mnId << " = " << Tcw << std::endl;
			//fout << "Velocity = " << vel << std::endl;
			//fout << "Bais = " << accbias << ", " << gyrobias << std::endl;
         }

        //Points
		int pIndex = 0;
		int cpIndex = 0;
        for(vector<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
        {
             MapPoint* pMP = *lit;
			 if(!pMP){
				 //pIndex ++;
				 continue;
			 }
			 
			 int cfactorIndex = factorindexmap[pMP->mnId];
			 SmartFactor::shared_ptr smt = boost::dynamic_pointer_cast<SmartFactor>(graph[cfactorIndex]);
			 if(smt){
				 
				 boost::optional<Point3> pinw = smt->point();
				 if(!pinw){
					const map<KeyFrame*,size_t> observations = pMP->GetObservations();
					for(map<KeyFrame*,size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend ; mit ++){
						KeyFrame* pKFi = mit->first;
						for(vector<KeyFrame*>::iterator kit = lLocalKeyFrames.begin(), kend = lLocalKeyFrames.end(); kit != kend ; kit ++){
							KeyFrame* pKFt = *kit;
							if(pKFi->mnId == pKFt->mnId){
								pKFi->EraseMapPointMatch(pMP);
								pMP->EraseObservation(pKFi);
								break;									
							}	
						}
					}						 
				 }
				 else{
					cv::Mat pinw_pos(3,1,CV_32F);
					pinw_pos.at<float>(0) = pinw->x(); pinw_pos.at<float>(1) = pinw->y() ; pinw_pos.at<float>(2) = pinw->z();
					pMP->SetWorldPos(pinw_pos);
					pMP->UpdateNormalAndDepth();	
					/*const map<KeyFrame*,size_t> observations = pMP->GetObservations();
					//fout << "Point "<< pMP->mnId << " = " << pMP->GetWorldPos().t() << std::endl;
					for(map<KeyFrame*,size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend ; mit ++){
						KeyFrame* pKFi = mit->first;
						if(pKFi->mnBALocalForKF == curKFId || pKFi->mnBAFixedForKF == curKFId){
							//Point2 obs = curKF->mvKeysUn[(int)(mit->second)];
							const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];
							cv::Point2f obs1 = pKFi->projectMapPoint(pMP);
							//fout << "pMP = " << pMP->GetWorldPos() << endl;;
							//fout << "Reproject in Camera " << pKFi->mnId << std::endl;
                            //fout << "obs = (" << obs1.x << ", " << obs1.y << "), KeyPoint = (" << kpUn.pt.x << ", " << kpUn.pt.y << ")" << endl;
							if(pKFi->mnId == pKF->mnId){
								cpIndex ++;
							}		
						}
					}
					*/pIndex ++;
				 
				 }
			 }
			 else{
					const map<KeyFrame*,size_t> observations = pMP->GetObservations();
					for(map<KeyFrame*,size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend ; mit ++){
						KeyFrame* pKFi = mit->first;
						for(vector<KeyFrame*>::iterator kit = lLocalKeyFrames.begin(), kend = lLocalKeyFrames.end(); kit != kend ; kit ++){
							KeyFrame* pKFt = *kit;
							if(pKFi->mnId == pKFt->mnId){
								pKFi->EraseMapPointMatch(pMP);
								pMP->EraseObservation(pKFi);
								break;									
							}	
						}
					}
			 }
			 //pIndex ++;
        }
		//fout << "Total Valid Point Number is " << pIndex << endl;
		//fout << "Camera " << pKF->mnId << " Valid Point Number is " << cpIndex << std::endl;
		//fout << "Total error = " << graph.error(currentEstimate) << std::endl;
		//fout.close();pIndex
		cout << "pIndex= " << pIndex << endl;
		cout<<"BA end ............................................................."<<endl;
		
	}

		
	void Optimizer::InitialBAUseIMU(KeyFrame* preKF, KeyFrame* curKF, IMUMeasurementList* imu){

		 float  matTcs_v[9] = {0.0148655429818, -0.999880929698, 0.00414029679422, 
         0.999557249008, 0.0149672133247, 0.025715529948, 
        -0.0257744366974, 0.00375618835797, 0.999660727178 };
		cv::Mat matTcs(3,3,CV_32F , matTcs_v);
	    //cout << "matTcs = " << matTcs << endl;  
		float initMat[9] = {0.90480912, -0.32536578, -0.27469578, 0.14457464, -0.37204891, 0.91688496, -0.40052322, -0.86931986, -0.28959358};
		cv::Mat preKFRot(3,3,CV_32F , initMat);
		//cout << "preKFRot1 = " << preKFRot << endl;
		preKFRot = matTcs.t() * preKFRot.t();
		//cout << "preKFRot = " << preKFRot << endl;
		cv::Mat preKFPose(4,4,CV_32F);
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				preKFPose.at<float>(i,j) = preKFRot.at<float>(i,j);
			}
		}
		preKFPose.at<float>(3,3) = 1.0; 
		//preKF->SetPose(preKFPose);
		cout << "preKFPose = " << preKFPose << std::endl;
		cout << "preKF pose = " << preKF->GetPose() << std::endl;

		// note that after setFirstTimeStampleandmnId(),
		// the camera has pose in the Nav frame    
		
         //0.0, 0.0, 0.0, 1.0;  
		vector<MapPoint*> lLocalMapPoints;
		vector<MapPoint*> curMapPoints = curKF->GetMapPointMatches();
		for(vector<MapPoint*>::iterator lit = curMapPoints.begin(), lend = curMapPoints.end(); lit != lend; lit ++){
			MapPoint* pMP = *lit;
			if(pMP){
				lLocalMapPoints.push_back(pMP);
			}
		}
		sort(lLocalMapPoints.begin() , lLocalMapPoints.end(), ORB_SLAM::MapPoint::plId);
		
		Vector3 currentvelglobal(0.0, 0.0, 0.0);
		Vector3 g(0,9.8015,0);
		Vector3 w_coriolis(0,0,0);
		//imuBias::ConstantBias currentBias = imuBias::ConstantBias(Vector3(-0.466400100200400,0.144949062124249,-0.15159671), Vector3(-0.021269993987976,0.001001244488978,-0.005077731462926));
		//Cal3_S2::shared_ptr K(new Cal3_S2(458.654, 457.296, 0.0, 367.215, 248.375));
		noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
		noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 1); // one pixel in u and v
		imuBias::ConstantBias constBias = imuBias::ConstantBias(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0));
		imuBias::ConstantBias currentBias = imuBias::ConstantBias(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0));
		noiseModel::Diagonal::shared_ptr  sigma_init_x = noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<<0.03,0.03,0.03,0.001,0.001,0.001));
		noiseModel::Isotropic::shared_ptr sigma_init_v = noiseModel::Isotropic::Sigma(3, 2);
		noiseModel::Diagonal::shared_ptr  sigma_between_b  = noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<<0.00010, 0.00010, 0.00010, 5.00e-06, 5.00e-06, 5.00e-06));

		// Create an iSAM2 object. Unlike iSAM1, which performs periodic batch steps to maintain proper linearization
		// and efficient variable ordering, iSAM2 performs partial relinearization/reordering at each step. A parameter
		// structure is available that allows the user to set various properties, such as the relinearization threshold
		// and type of linear solver. For this example, we we set the relinearization threshold small so the iSAM2 result
		// will approach the batch result.
		ISAM2Params parameters;
		const std::string cc="CHOLESKY";
		parameters.setFactorization(cc);
		//parameters.relinearizeThreshold = 0.0000001;
		parameters.relinearizeSkip = 100;
		ISAM2 isam(parameters);
		// Create a Factor Graph and Values to hold the new data
		NonlinearFactorGraph graph;
		Values initialEstimate;
		unsigned long maxKFid = 0;		
		int factorIndex = 0;
		gtsam::Values currentEstimate;

		// the IMU factor and the noise model
		PreintegratedCombinedMeasurements* imu_measurement=imu->Getimufactor(curKF->mnId);
		//imu_measurement->print();
		Matrix3 accelerometerCovariance = imu_measurement->p().accelerometerCovariance;
		Matrix3 gyroscopeCovariance = imu_measurement->p().gyroscopeCovariance;
		noiseModel::Diagonal::shared_ptr sigma_init_b = noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<<
			accelerometerCovariance(0,0), gyroscopeCovariance(1,1), gyroscopeCovariance(2,2), 
			gyroscopeCovariance(0,0), gyroscopeCovariance(1,1), gyroscopeCovariance(2,2)));
		
		cout << "accelerometerCovariance = " << accelerometerCovariance << std::endl;
		cout << "gyroscopeCovariance = " << gyroscopeCovariance << std::endl;
		
		// the pose1, we set the pose1 as zero
		//gtsam::Quaternion _q(1,0,0,0);
		//Point3 _t(0,0,0);
		Pose3 pos1 = toGTPose3(preKF->GetPose());//(Rot3(_q),_t); //= Pose3(gtsam::Vector(6) << 0 , 0 , 0, 0, 0, 0); // note that we set the pose1 as zero
		NavState nav1(pos1.inverse() , currentvelglobal); // navstate for pose 1
		Vector3 v1 = currentvelglobal;
		imuBias::ConstantBias b1 = currentBias;												  // note that in the paper, the Pose is frome the body to the navigation
		
		// add pose1 priorfactor
		initialEstimate.insert(X(preKF->mnId) , pos1.inverse());
		initialEstimate.insert(B(preKF->mnId) , b1);
		initialEstimate.insert(V(preKF->mnId) , v1);		
		
		graph.push_back(PriorFactor<Pose3>(X(preKF->mnId),pos1.inverse() , sigma_init_x));
		factorIndex ++;
		graph.push_back(PriorFactor<Vector3>(V(preKF->mnId),v1 , sigma_init_v));
		factorIndex ++;
		graph.push_back(PriorFactor<imuBias::ConstantBias>(B(preKF->mnId) , b1 , sigma_init_b));
		factorIndex ++;
		//graph.push_back(NonlinearEquality<Pose3>(X(preKF->mnId), pos1.inverse()));
		//factorIndex++;
		///////////////////////////////////////////////////////////////////

		// the pose2, we predict it according to the PIM
		NavState nav2 = imu_measurement->predict(nav1 , currentBias); 
		Pose3 pos2 = nav2.pose();
		Velocity3 v2 = nav2.v();
		imuBias::ConstantBias b2 = currentBias;
		cout << "pose2 = " << endl;
		pos2.print();
		
		// add pose2 priorfactor
		initialEstimate.insert(X(curKF->mnId) , pos2);
		initialEstimate.insert(B(curKF->mnId) , b2);
		initialEstimate.insert(V(curKF->mnId) , v2);		
		
	    //graph.push_back(PriorFactor<Pose3>(X(1),pos2 , sigma_init_x));
		//factorIndex ++;
		//graph.push_back(PriorFactor<Vector3>(V(1),currentvelglobal , sigma_init_v));
		//factorIndex ++;
		//graph.push_back(PriorFactor<imuBias::ConstantBias>(B(1) , currentBias , sigma_init_b));
		//factorIndex ++;
		////////////////////////////////////////////////////////////////////
		
		// add the combimedIMUfactor
		graph.push_back(CombinedImuFactor(X(0) , V(0) , X(1) , V(1) , B(0) , B(1) ,*imu_measurement));
		factorIndex ++;
		int cfactorIndex = factorIndex - 1;
		////////////////////////////////////////////////////////////////////
		
		// add the bias between factor 
		double sqrt_tij = sqrt(imu_measurement->deltaTij());
		Matrix3 biasAccCovariance = imu_measurement->p().biasAccCovariance;
		Matrix3 biasOmegaCovariance = imu_measurement->p().biasOmegaCovariance;
		double sigma_b_1 = biasAccCovariance(0,0);
		double sigma_b_2 = biasOmegaCovariance(1,1);
		noiseModel::Diagonal::shared_ptr bias_between_randwalk = gtsam::noiseModel::Diagonal::Sigmas(
			(gtsam::Vector(6) << sqrt_tij * sigma_b_1 , sqrt_tij * sigma_b_1 , sqrt_tij * sigma_b_1 ,
			sqrt_tij * sigma_b_2 , sqrt_tij * sigma_b_2 , sqrt_tij * sigma_b_2)
        );
		imuBias::ConstantBias deltaB = b2 - b1;
		//graph.push_back(BetweenFactor<imuBias::ConstantBias>(B(0) , B(1) , deltaB ,bias_between_randwalk));
		//factorIndex ++;
		///////////////////////////////////////////////////////////////////
		
		//Vector3 deltaVij = imu_measurement->deltaVij();
		//double delta_tij = imu_measurement->deltaTij();
		//deltaVij(0) += n_g.x() * delta_tij ; deltaVij(1) += n_g.y() * delta_tij; deltaVij(2) += n_g.z() * delta_tij;
		
		// add a betweenfactor for the velocity, 
		// maybe we don't need to add this factor, since velocity is estimated from others
		//Vector3 deltaV = v2 - v1; 
		//graph.push_back(BetweenFactor<Vector3>(V(0) , V(1) , deltaV , sigma_init_v));
		//factorIndex ++;
		///////////////////////////////////////////////////////////////////
		
		// add the smartprojectfactor
				     // SET MAP POINT VERTICES
         //const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();
		 //vector<vector<KeyFrame*>> vpEdgeKF;
		 //vpEdgeKF.reserve(nExpectedSize);
		 
		typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;
		std::ofstream fout1;
		fout1.open("graph1.txt");
		map<int , int> mpIndex_map_factorIndex;
		for(vector<MapPoint*>::iterator lit = lLocalMapPoints.begin(), 
		len = lLocalMapPoints.end(); lit != len ; lit ++){
			MapPoint* pMP = *lit;
			//vector<KeyFrame*> tEdgeKF;
			if(!pMP){
				continue;
			}
			
			Cal3_S2::shared_ptr K(new Cal3_S2(curKF->fx , curKF->fy , 0 , curKF->cx , curKF->cy));
			
            const map<KeyFrame*,size_t> observations = pMP->GetObservations();
			float invSigma2 = curKF->mvInvLevelSigma2[1];	
			for(map<KeyFrame*,size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend ; mit ++){
				KeyFrame* pKFi = mit->first;
				if(pKFi->mnId == curKF->mnId){
					//Point2 obs = curKF->mvKeysUn[(int)(mit->second)];
					const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];
                    // Monocular observation
                    invSigma2 = pKFi->GetInvSigma2(kpUn.octave);
				}
			}

			noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(2,invSigma2);
		    SmartProjectionParams sparas;
			sparas.setLinearizationMode(LinearizationMode::JACOBIAN_Q);
			sparas.setDynamicOutlierRejectionThreshold(5);
			SmartFactor::shared_ptr smt(new SmartFactor(model , K , boost::none , sparas ));
			for(map<KeyFrame*,size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend ; mit ++){
				KeyFrame* pKFi = mit->first;
				if(pKFi->mnId == preKF->mnId || pKFi->mnId == curKF->mnId){
					//Point2 obs = curKF->mvKeysUn[(int)(mit->second)];
					const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];
                    // Monocular observation
                    Point2 obs(kpUn.pt.x, kpUn.pt.y);
					smt->add(obs, X(pKFi->mnId));
					//tEdgeKF.push_back(pKFi);
				}
			}
			graph.push_back(smt);
			//vpEdgeKF.push_back(tEdgeKF);
			fout1 << "add a smartprojectfactor pMP->mnId = " << pMP->mnId << " factorIndex = " << factorIndex << endl;
			mpIndex_map_factorIndex[pMP->mnId] = factorIndex;
			factorIndex ++ ;
		}
		fout1.close();
		// finish
		//////////////////////////////////////////////////////////////////////
		// solve the optimization
		//graph.print();
		//cout << "line 890" << endl;
		//isam.update(graph , initialEstimate);
		//cout << "line 892" << endl;
		//isam.update();
		//cout << "line 894" << endl;
		//currentEstimate = isam.calculateEstimate();
		//cout << "line 896" << endl;
		LevenbergMarquardtParams lmParams;
    	//    lmParams.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
    	lmParams.verbosity = NonlinearOptimizerParams::ERROR;
    	lmParams.maxIterations = 6;
		lmParams.setErrorTol(40.0);
		lmParams.setlambdaFactor(5);
		//lmParams.setVerbosityLM(string("TRYDELTA"));
		//lmParams.setVerbosity(string("ERROR"));
		//lmParams.setLogFile(string("log.csv"));
		lmParams.setDiagonalDamping(true);
		lmParams.setlambdaUpperBound(0.1);
		//lmParams.setVerbosityLM("LevenbergMarquardtParams::TRYLAMBDA");
		lmParams.print();
		
    	LevenbergMarquardtOptimizer lmOptimizer(graph, initialEstimate, lmParams);

		//cout << "begin to optimize the IMU" << endl;
    	currentEstimate = lmOptimizer.optimize();
		//cout << "Finish optimize the IMU"<<endl;

		// recover the optimized data
		typedef boost::shared_ptr<NonlinearFactor> sharedFactor;  ///< Shared pointer to a factor
		std::ofstream fout;
		char buf[255];
		sprintf(buf, "result_init.txt");
		fout.open(buf);
/*		
		fout << "Error list for each factor"<<std::endl;
		fout << "***********************************************************"<<std::endl;
		double total_error = 0.0;
		int ppIndex = 0;
		BOOST_FOREACH(const sharedFactor& factor, graph) {
			if(factor){
				total_error += factor->error(currentEstimate);
				fout << "Factor " << ppIndex ++ << " error = " << factor->error(currentEstimate) << std::endl;      
			}
		}
		fout << "total_error = " << total_error << std::endl;
		fout << "***********************************************************"<<std::endl;
*/		
		Pose3 pos1_opt = currentEstimate.at<Pose3>(X(preKF->mnId));
		gtsam::Vector val = pos1_opt.rotation().quaternion();
		double pX = pos1_opt.x();
		double pY = pos1_opt.y();
		double pZ = pos1_opt.z();
		Eigen::Quaterniond _q1(val(0) , val(1) , val(2) , val(3));
		Eigen::Vector3d _t1(pX,pY,pZ);
		g2o::Sim3 sim3_vec(_q1,_t1,1);
		cv::Mat Tcw1 = Converter::toCvMat(sim3_vec);
		preKF->SetPose(Tcw1.inv());
		Vector3 vel = currentEstimate.at<Vector3>(V(preKF->mnId));
		imuBias::ConstantBias bias = currentEstimate.at<imuBias::ConstantBias>(B(preKF->mnId));
		Vector3 accbias = bias.accelerometer();
		Vector3 gyrobias = bias.gyroscope();
		for(int i=0;i<3;i++){
			preKF->mSpeedAndBias.vel[i] = vel[i];
			preKF->mSpeedAndBias.gyrobias[i] = gyrobias[i];
			preKF->mSpeedAndBias.accbias[i] = accbias[i];
		}
		fout << "Camera " << preKF->mnId << " = " << Tcw1.inv() << std::endl;
		fout << "Velocity = " << vel << std::endl;
		fout << "Bias = " << accbias << ", " << gyrobias << std::endl;
		
		Pose3 pos2_opt = currentEstimate.at<Pose3>(X(curKF->mnId));
		val = pos2_opt.rotation().quaternion();
		pX = pos2_opt.x();
		pY = pos2_opt.y();
		pZ = pos2_opt.z();
		Eigen::Quaterniond _q2(val(0) , val(1) , val(2) , val(3));
		Eigen::Vector3d _t2(pX,pY,pZ);
		g2o::Sim3 sim3_vec1(_q2,_t2,1);
		cv::Mat Tcw2 = Converter::toCvMat(sim3_vec1);
		curKF->SetPose(Tcw2.inv());
		vel = currentEstimate.at<Vector3>(V(curKF->mnId));
		bias = currentEstimate.at<imuBias::ConstantBias>(B(curKF->mnId));
		accbias = bias.accelerometer();
		gyrobias = bias.gyroscope();
		for(int i = 0 ;i < 3;i++){
			curKF->mSpeedAndBias.vel[i] = vel[i];
			curKF->mSpeedAndBias.gyrobias[i] = gyrobias[i];
			curKF->mSpeedAndBias.accbias[i] = accbias[i];
		}
		fout << "Camera " << curKF->mnId << " = " << Tcw2.inv() << std::endl;
		fout << "Velocity = " << vel << std::endl;
		fout << "Bias = " << accbias << ", " << gyrobias << std::endl;		
		int pIndex = 0;
		int cpIndex = 0;
		for(vector<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit ++){
			MapPoint* pMP = *lit;
			if(!pMP){
				continue;
			}
			 fout << "pMP->mnId = " << pMP->mnId << " factor = ";
			 int curfactorIndex = mpIndex_map_factorIndex[pMP->mnId];
			 fout << curfactorIndex << endl;
			 SmartFactor::shared_ptr smt = boost::dynamic_pointer_cast<SmartFactor>(graph[curfactorIndex]);
			 if(smt){
				 boost::optional<Point3> pinw = smt->point();
				 if(!pinw){
					 fout << "Point3D invalid"<<endl;
					 const map<KeyFrame*,size_t> observations = pMP->GetObservations();
					for(map<KeyFrame*,size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend ; mit ++){
						KeyFrame* pKFi = mit->first;
							if(pKFi->mnId == preKF->mnId || pKFi->mnId == curKF->mnId){
								pKFi->EraseMapPointMatch(pMP);
								pMP->EraseObservation(pKFi);
								break;									
							}	
					}
				 }
				 else{
					pIndex ++;
					cv::Mat pinw_pos(3,1,CV_32F);
					pinw_pos.at<float>(0) = pinw->x(); pinw_pos.at<float>(1) = pinw->y() ; pinw_pos.at<float>(2) = pinw->z();
					pMP->SetWorldPos(pinw_pos);
					pMP->UpdateNormalAndDepth();
					const map<KeyFrame*,size_t> observations = pMP->GetObservations();
					//fout << "MapPoint: " << endl;
					for(map<KeyFrame*,size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend ; mit ++){
						KeyFrame* pKFi = mit->first;
						if(pKFi->mnId == preKF->mnId || pKFi->mnId == curKF->mnId){
							const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];
							cv::Point2f obs1 = pKFi->projectMapPoint(pMP);
							fout << "pMP = " << pMP->GetWorldPos().t() << endl;;
							fout << "obs = (" << obs1.x << ", " << obs1.y << "), KeyPoint = (" << kpUn.pt.x << ", " << kpUn.pt.y << ")" << endl;				
						}
					}					 
				 }
			 }
			 else{
				 	const map<KeyFrame*,size_t> observations = pMP->GetObservations();
					for(map<KeyFrame*,size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend ; mit ++){
						KeyFrame* pKFi = mit->first;
							if(pKFi->mnId == preKF->mnId || pKFi->mnId == curKF->mnId){
								pKFi->EraseMapPointMatch(pMP);
								pMP->EraseObservation(pKFi);
								break;									
							}	
					}
			 }
		}
		fout << "Valid Point Number is " << pIndex << endl;
		fout << "Total error = " << graph.error(currentEstimate) << endl;
		fout.close();
		// finish
		cout << "Finish InitialBAISAM2"<<endl;
		preKF->mnBALocalForKF = -1;
		curKF->mnBAFixedForKF = -1;
		preKF->mnBAFixedForKF = -1;
		curKF->mnBALocalForKF = -1;
		for(vector<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit ++){
			MapPoint* pMP = *lit;
			if(pMP){
				pMP->mnBALocalForKF = -1;
			}
		}
		
	}

	void Optimizer::PoseOptimizationUseImu(KeyFrame* LastKF, Frame *pLastFrame, Frame *pCurrentFrame, Map* pMap, IMUMeasurementList* imu, bool ifKF)
	{
		cout << "begin to PoseOptimizationUseImu" << std::endl;
		Values initialEstimate;
		Values currentEstimate;
		NonlinearFactorGraph graph;
		gtsam::Pose3 Tcw = toGTPose3(pCurrentFrame->mTcw.clone());
		Vector3 currentvel(Vector3(pLastFrame->mSpeedAndBias.vel[0], pLastFrame->mSpeedAndBias.vel[1], pLastFrame->mSpeedAndBias.vel[2]));
		imuBias::ConstantBias currentBias = imuBias::ConstantBias(Vector3(pLastFrame->mSpeedAndBias.accbias[0],pLastFrame->mSpeedAndBias.accbias[1],
			pLastFrame->mSpeedAndBias.accbias[2]), Vector3(pLastFrame->mSpeedAndBias.gyrobias[0],pLastFrame->mSpeedAndBias.gyrobias[1],pLastFrame->mSpeedAndBias.gyrobias[2]));
		Cal3_S2::shared_ptr K(new Cal3_S2(pCurrentFrame->fx , pCurrentFrame->fy , 0 , pCurrentFrame->cx , pCurrentFrame->cy));
		cout <<  "IFKf.................." << ifKF << endl;
		if(ifKF)
		{
			/////////////reprojection error////////////////////
			initialEstimate.insert(X(pCurrentFrame->mnId), Tcw.inverse());
			initialEstimate.insert(V(pCurrentFrame->mnId), currentvel);
			initialEstimate.insert(B(pCurrentFrame->mnId), currentBias);
			const int N = pCurrentFrame->mvpMapPoints.size();
			for(int i=0;i<N;i++)
			{
				MapPoint* pMP = pCurrentFrame->mvpMapPoints[i];
				if(pMP)
				{
					cv::Mat pMPCoor = pMP->GetWorldPos();
					gtsam::Point3 LPoint(pMPCoor.at<float>(0),pMPCoor.at<float>(1),pMPCoor.at<float>(2));
					initialEstimate.insert(L(pMP->mnId), LPoint);
					cv::KeyPoint kpUn = pCurrentFrame->mvKeysUn[i];
					gtsam::Point2 measured_(kpUn.pt.x,kpUn.pt.y);
					float Sigma = pCurrentFrame->mvScaleFactors[kpUn.octave];
					noiseModel::Isotropic::shared_ptr measurementNoise= noiseModel::Isotropic::Sigma(2, Sigma);
					SharedNoiseModel rm = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(5.991f),measurementNoise);
					graph.push_back(GenericProjectionFactor<Pose3, Point3, Cal3_S2>(measured_,rm,X(pCurrentFrame->mnId),L(pMP->mnId),K));
					graph.push_back(NonlinearEquality<Point3>(L(pMP->mnId), LPoint));
				}
			}
			//////////////imu error ///////////////
			gtsam::Pose3 TKFcw = toGTPose3(LastKF->GetPose());
			initialEstimate.insert(X(LastKF->mnFrameId), TKFcw.inverse());
			Vector3 KFvel(Vector3(LastKF->mSpeedAndBias.vel[0], LastKF->mSpeedAndBias.vel[1], LastKF->mSpeedAndBias.vel[2]));
			imuBias::ConstantBias KFBias = imuBias::ConstantBias(Vector3(LastKF->mSpeedAndBias.accbias[0],LastKF->mSpeedAndBias.accbias[1],
			      LastKF->mSpeedAndBias.accbias[2]), Vector3(LastKF->mSpeedAndBias.gyrobias[0],LastKF->mSpeedAndBias.gyrobias[1],LastKF->mSpeedAndBias.gyrobias[2]));
			initialEstimate.insert(V(LastKF->mnFrameId), KFvel);
			initialEstimate.insert(B(LastKF->mnFrameId), KFBias);
			graph.push_back(NonlinearEquality<Pose3>(X(LastKF->mnFrameId) , TKFcw.inverse()));
			graph.push_back(NonlinearEquality<imuBias::ConstantBias>(B(LastKF->mnFrameId) , KFBias));
			graph.push_back(NonlinearEquality<Vector3>(V(LastKF->mnFrameId), KFvel));
			PreintegratedCombinedMeasurements* imu_measurement_i = imu->KFframeimupre;
			graph.push_back(CombinedImuFactor(X(LastKF->mnFrameId),V(LastKF->mnFrameId),X(pCurrentFrame->mnId),V(pCurrentFrame->mnId),B(LastKF->mnFrameId),B(pCurrentFrame->mnId),*imu_measurement_i));

			LevenbergMarquardtParams lmParams;
    		lmParams.verbosity = NonlinearOptimizerParams::ERROR;
    		lmParams.maxIterations = 6;
			lmParams.setErrorTol(40.0);
			lmParams.setlambdaFactor(5);
			lmParams.setlambdaInitial(150);
			lmParams.setVerbosityLM(string("SILENT"));
			lmParams.setDiagonalDamping(true);

    		LevenbergMarquardtOptimizer lmOptimizer(graph, initialEstimate, lmParams);

			cout << "begin to optimize use LM1" << endl;
    		currentEstimate = lmOptimizer.optimize();
			cout << "Finish optimize use LM2"<<endl;
		 
         // Recover optimized data
            Pose3 Twc =currentEstimate.at<Pose3>(X(pCurrentFrame->mnId));
			gtsam::Vector val=Twc.inverse().rotation().quaternion();
			double pX = Twc.inverse().x();
		    double pY = Twc.inverse().y();
			double pZ = Twc.inverse().z();
			Eigen::Quaterniond _q(val(0), val(1) , val(2) , val(3));
			Eigen::Vector3d _t(pX,pY,pZ);
			g2o::Sim3 sim3_vec(_q , _t , 1);
			cv::Mat frameTcw = Converter::toCvMat(sim3_vec);//Rot3(val).toMat();
			frameTcw.copyTo(pCurrentFrame->mTcw);
			Vector3 vel=currentEstimate.at<Vector3>(V(pCurrentFrame->mnId));
            imuBias::ConstantBias bias = currentEstimate.at<imuBias::ConstantBias>(B(pCurrentFrame->mnId));
			Vector3 accbias=bias.accelerometer();
			Vector3 gyrobias=bias.gyroscope();
			for(int i=0;i<3;i++)
			{
			    pCurrentFrame->mSpeedAndBias.vel[i]= vel[i];
				pCurrentFrame->mSpeedAndBias.gyrobias[i]=gyrobias[i];
				pCurrentFrame->mSpeedAndBias.accbias[i]=accbias[i];
			}
			LastKF->updateAfterBA = false;
		}
		else
		{
			
			noiseModel::Diagonal::shared_ptr  sigma_init_x = noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<<0.03,0.03,0.03,0.001,0.001,0.001));
			noiseModel::Isotropic::shared_ptr sigma_init_v = noiseModel::Isotropic::Sigma(3, 1000);
			noiseModel::Diagonal::shared_ptr sigma_init_b = noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<<0.0100, 0.0100, 0.0100, 5.00e-05, 5.00e-05, 5.00e-05));
			initialEstimate.insert(X(pCurrentFrame->mnId), Tcw.inverse());
			initialEstimate.insert(V(pCurrentFrame->mnId), currentvel);
			initialEstimate.insert(B(pCurrentFrame->mnId), currentBias);

			gtsam::Pose3 LastTcw = toGTPose3(pLastFrame->mTcw.clone());
			initialEstimate.insert(X(pLastFrame->mnId), LastTcw.inverse());
			initialEstimate.insert(V(pLastFrame->mnId), currentvel);
			initialEstimate.insert(B(pLastFrame->mnId), currentBias);

			///////////////prior factor///////////////
			graph.push_back(PriorFactor<Pose3>(X(pLastFrame->mnId), LastTcw.inverse() , sigma_init_x));
			graph.push_back(PriorFactor<imuBias::ConstantBias>(B(pLastFrame->mnId) , currentBias, sigma_init_b));
			graph.push_back(PriorFactor<Vector3>(V(pLastFrame->mnId) , currentvel , sigma_init_v));
			///////////////reprojection error factor ////////////
			int N = pLastFrame->mvpMapPoints.size();
			for(int i=0;i<N;i++)
			{
				MapPoint* pMP = pLastFrame->mvpMapPoints[i];
				if(pMP)
				{
					cv::Mat pMPCoor = pMP->GetWorldPos();
					gtsam::Point3 LPoint(pMPCoor.at<float>(0),pMPCoor.at<float>(1),pMPCoor.at<float>(2));
					initialEstimate.insert(L(pMP->mnId), LPoint);
					graph.push_back(NonlinearEquality<Point3>(L(pMP->mnId), LPoint));
					pMP->mnBALocalForFrame = pCurrentFrame->mnId;
					cout << pMP->mnId << " " ;
					cv::KeyPoint kpUn = pLastFrame->mvKeysUn[i];
					gtsam::Point2 measured_(kpUn.pt.x,kpUn.pt.y);
					float Sigma = pLastFrame->mvScaleFactors[kpUn.octave];
					noiseModel::Isotropic::shared_ptr measurementNoise= noiseModel::Isotropic::Sigma(2, Sigma);
					SharedNoiseModel rm = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(5.991f),measurementNoise);
					graph.push_back(GenericProjectionFactor<Pose3, Point3, Cal3_S2>(measured_,rm,X(pLastFrame->mnId),L(pMP->mnId),K));
					
				}
			}
			cout << endl ;
			//cout << "..................................2" << std::endl;
			N = pCurrentFrame->mvpMapPoints.size();
			for(int i=0;i<N;i++)
			{
				MapPoint* pMP = pCurrentFrame->mvpMapPoints[i];
				//cout << "..................................3" << std::endl;
				if(pMP)
				{
					if(pMP->mnBALocalForFrame!=pCurrentFrame->mnId)
					{
						cout << pMP->mnId << " " ;
						cv::Mat pMPCoor = pMP->GetWorldPos();
						gtsam::Point3 LPoint(pMPCoor.at<float>(0),pMPCoor.at<float>(1),pMPCoor.at<float>(2));
						initialEstimate.insert(L(pMP->mnId), LPoint);
						graph.push_back(NonlinearEquality<Point3>(L(pMP->mnId), LPoint));
					}
					
					//cout << "..................................4" << std::endl;
					cv::KeyPoint kpUn = pCurrentFrame->mvKeysUn[i];
					gtsam::Point2 measured_(kpUn.pt.x,kpUn.pt.y);
					float Sigma = pCurrentFrame->mvScaleFactors[kpUn.octave];
					noiseModel::Isotropic::shared_ptr measurementNoise= noiseModel::Isotropic::Sigma(2, Sigma);
					SharedNoiseModel rm = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(5.991f),measurementNoise);
					graph.push_back(GenericProjectionFactor<Pose3, Point3, Cal3_S2>(measured_,rm,X(pCurrentFrame->mnId),L(pMP->mnId),K));
					
				}
			}

			//////////////imu error factor///////////////
			PreintegratedCombinedMeasurements* imu_measurement_i = imu->frameimupre;
			graph.push_back(CombinedImuFactor(X(pLastFrame->mnId),V(pLastFrame->mnId),X(pCurrentFrame->mnId),V(pCurrentFrame->mnId),B(pLastFrame->mnId),B(pCurrentFrame->mnId),*imu_measurement_i));

			LevenbergMarquardtParams lmParams;
    		lmParams.verbosity = NonlinearOptimizerParams::ERROR;
    		lmParams.maxIterations = 6;
			lmParams.setErrorTol(40.0);
			lmParams.setlambdaFactor(5);
			lmParams.setlambdaInitial(150);
			lmParams.setVerbosityLM(string("SILENT"));
			lmParams.setDiagonalDamping(true);

    		LevenbergMarquardtOptimizer lmOptimizer(graph, initialEstimate, lmParams);

			cout << "begin to optimize use LM 0" << endl;
    		currentEstimate = lmOptimizer.optimize();
			cout << "Finish optimize use LM 0"<<endl;
		 
         	///////////// Recover optimized data -- lastframe//////////
		 	Pose3 Twc =currentEstimate.at<Pose3>(X(pLastFrame->mnId));
			gtsam::Vector val=Twc.inverse().rotation().quaternion();
			double pX = Twc.inverse().x();
		    double pY = Twc.inverse().y();
			double pZ = Twc.inverse().z();
			Eigen::Quaterniond _q(val(0), val(1) , val(2) , val(3));
			Eigen::Vector3d _t(pX,pY,pZ);
			g2o::Sim3 sim3_vec(_q , _t , 1);
			cv::Mat frameTcw = Converter::toCvMat(sim3_vec);//Rot3(val).toMat();
			frameTcw.copyTo(pLastFrame->mTcw);
			Vector3 vel=currentEstimate.at<Vector3>(V(pLastFrame->mnId));
            imuBias::ConstantBias bias = currentEstimate.at<imuBias::ConstantBias>(B(pLastFrame->mnId));
			Vector3 accbias=bias.accelerometer();
			Vector3 gyrobias=bias.gyroscope();
			for(int i=0;i<3;i++)
			{
			    pLastFrame->mSpeedAndBias.vel[i]= vel[i];
				pLastFrame->mSpeedAndBias.gyrobias[i]=gyrobias[i];
				pLastFrame->mSpeedAndBias.accbias[i]=accbias[i];
			}

		 	///////////// Recover optimized data -- lastframe/////////////
            Twc =currentEstimate.at<Pose3>(X(pCurrentFrame->mnId));
			val=Twc.inverse().rotation().quaternion();
			pX = Twc.inverse().x();
		    pY = Twc.inverse().y();
			pZ = Twc.inverse().z();
			Eigen::Quaterniond q(val(0), val(1) , val(2) , val(3));
			Eigen::Vector3d t(pX,pY,pZ);
			g2o::Sim3 sim3vec(q , t , 1);
			frameTcw = Converter::toCvMat(sim3vec);//Rot3(val).toMat();
            frameTcw.copyTo(pCurrentFrame->mTcw);
			vel=currentEstimate.at<Vector3>(V(pCurrentFrame->mnId));
            bias = currentEstimate.at<imuBias::ConstantBias>(B(pCurrentFrame->mnId));
			accbias=bias.accelerometer();
			gyrobias=bias.gyroscope();
			for(int i=0;i<3;i++)
			{
			    pCurrentFrame->mSpeedAndBias.vel[i]= vel[i];
				pCurrentFrame->mSpeedAndBias.gyrobias[i]=gyrobias[i];
				pCurrentFrame->mSpeedAndBias.accbias[i]=accbias[i];
			}
			
		}
	}
	
	bool Optimizer::InitialScalityAndVelocity(Map* mpMap, IMUMeasurementList* imu, Frame* pLastFrame, cv::Mat &e_gw){
		
	     vector<KeyFrame*> lLocalKeyFrames = mpMap->GetAllKeyFrames();
		 sort(lLocalKeyFrames.begin(),lLocalKeyFrames.end(), ORB_SLAM::KeyFrame::lId);
		 
		 std::ofstream fout;
		 fout.open("InitialScalityAndVelocity.txt");
		 
		 int nSize = lLocalKeyFrames.size();
		 cv::Mat A(3*(nSize-2),4,CV_32F);
		 cv::Mat b(3*(nSize-2),1,CV_32F);
		 
		 PreintegratedCombinedMeasurements::Params params = (imu->Getimufactor(0))->p();
		 boost::optional<gtsam::Pose3> pTcb = params.getBodyPSensor();
		 cv::Mat mTcb = Optimizer::Pose3TocvMat(pTcb);
		 cv::Mat mTbc = mTcb.inv();
		 cv::Mat mRcb = mTcb.rowRange(0,3).colRange(0,3);
		 cv::Mat mRbc = mTbc.rowRange(0,3).colRange(0,3);
		 cv::Mat tbc = mTbc.rowRange(0,3).col(3);
		 cv::Mat cPb = - mRbc * tbc;
		 
		 fout << "mRcb = " << mRcb << std::endl;
		 fout << "cPb = " << cPb << std::endl;
		 fout << "=====================================================" << std::endl;
		 int ii = 0;
		 for(vector<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end()-2; lit != lend; lit++){
			 fout <<"the " << ii << "th equation"<< std::endl;
			 KeyFrame* pKF1 = *(lit);
			 KeyFrame* pKF2 = *(lit+1);
			 KeyFrame* pKF3 = *(lit+2);
			 
			 PreintegratedCombinedMeasurements* pim12 = imu->Getimufactor(pKF2->mnId);
			 PreintegratedCombinedMeasurements* pim23 = imu->Getimufactor(pKF3->mnId);
			 
			 cv::Mat Tcw1 = pKF1->GetPose(); 
			 cv::Mat Tcw2 = pKF2->GetPose();
			 cv::Mat Tcw3 = pKF3->GetPose();
			 fout << "Tcw1 = " << Tcw1 << std::endl;
			 fout << "Tcw2 = " << Tcw2 << std::endl;
			 fout << "Tcw3 = " << Tcw3 << std::endl;
			 
			 cv::Mat wPc1 = - (Tcw1.rowRange(0,3).colRange(0,3)).inv() * Tcw1.rowRange(0,3).col(3);
			 cv::Mat wPc2 = - (Tcw2.rowRange(0,3).colRange(0,3)).inv() * Tcw2.rowRange(0,3).col(3);
			 cv::Mat wPc3 = - (Tcw3.rowRange(0,3).colRange(0,3)).inv() * Tcw3.rowRange(0,3).col(3);
			 
			 cv::Mat Rwc1 = (Tcw1.rowRange(0,3).colRange(0,3)).inv();
			 cv::Mat Rwc2 = (Tcw2.rowRange(0,3).colRange(0,3)).inv();
			 cv::Mat Rwc3 = (Tcw3.rowRange(0,3).colRange(0,3)).inv();
			 
			 fout << "Rwc1 = " << Rwc1 << std::endl;
			 fout << "Rwc2 = " << Rwc2 << std::endl;
			 fout << "Rwc3 = " << Rwc3 << std::endl;
			 
			 cv::Mat Rwb1 = Rwc1 * mRcb; 
			 cv::Mat Rwb2 = Rwc2 * mRcb; 
			 cv::Mat Rwb3 = Rwc3 * mRcb; 
			 
			 fout << "Rwb1 = " << Rwb1 << std::endl;
			 fout << "Rwb2 = " << Rwb2 << std::endl;
			 fout << "Rwb3 = " << Rwb3 << std::endl;			 
			 			 			 
			 double dt12 = pim12->deltaTij();
			 double dt23 = pim23->deltaTij();
			 
			 Vector3 dp12v = pim12->deltaPij();		 
			 cv::Mat dp12(3,1,CV_32F);
			 dp12.at<float>(0) = dp12v(0); dp12.at<float>(1) = dp12v(1); dp12.at<float>(2) = dp12v(2);
			 
			 Vector3 dp23v = pim23->deltaPij();
			 cv::Mat dp23(3,1,CV_32F);
			 dp23.at<float>(0) = dp23v(0); dp23.at<float>(1) = dp23v(1); dp23.at<float>(2) = dp23v(2);
			 
			 Vector3 dv12v = pim12->deltaVij();
			 cv::Mat dv12(3,1,CV_32F);
			 dv12.at<float>(0) = dv12v(0); dv12.at<float>(1) = dv12v(1); dv12.at<float>(2) = dv12v(2);
		     
			 Vector3 dv23v = pim23->deltaVij();
			 cv::Mat dv23(3,1,CV_32F);
			 dv23.at<float>(0) = dv23v(0); dv23.at<float>(1) = dv23v(1); dv23.at<float>(2) = dv23v(2);
			 
			 fout << "dp12 = " << dp12.t() << std::endl;
			 fout << "dp23 = " << dp23.t() << std::endl;
			 fout << "dv12 = " << dv12.t() << std::endl;
			 fout << "dv23 = " << dv23.t() << std::endl;
			 fout << "dt12 = " << dt12 << std::endl;
			 fout << "dt23 = " << dt23 << std::endl;
			 
			 cv::Mat lambdai = (wPc2 - wPc1) * dt23 - (wPc3 - wPc2) * dt12;
			 cv::Mat beltai = 0.5 * cv::Mat::eye(3,3,CV_32F) * (dt12 * dt12 * dt23 + dt23 * dt23 * dt12);
			 //cv::Mat beltai = 0.5 * gw * (dt23 * dt23 * dt12 + dt12 * dt12 * dt23);
			 cv::Mat gammai =  Rwc2*dp23*dt12 + Rwc1*dv12*dt12*dt23 - Rwc1*dp12*dt23;
								 
			 fout << "lambdai = " << lambdai.t() << std::endl;
			 fout << "beltai = " << beltai << std::endl;
			 fout << "gammi = " << gammai.t() << std::endl;
			 
			 for(int j=0;j<3;j++){
				 A.at<float>(3*ii+j,0) = lambdai.at<float>(j);
			 }					 
			 for(int j=0;j<3;j++){
				 for(int k=0;k<3;k++){
					 A.at<float>(3*ii+j,k+1) = beltai.at<float>(j,k);
				 }
			 }
			 for(int j=0;j<3;j++){
				 b.at<float>(3*ii+j,0) = -1.0*(gammai.at<float>(j));
			 }
			 //A.rowRange(3*i,3*i+3).col(0) = lambdai;
			 //A.rowRange(3*i,3*i+3).colRange(1,4) = beltai;
			 //b.rowRange(3*i,3*i+3).col(0) = gammai;	
			 ii ++;		 	
			 fout << "================================================" << std::endl;
			 fout << "================================================" << std::endl;		  
		 }
		 fout << "=====================================" << std::endl;
		 cout << "finish set A and b" << std::endl;
		 // debug
		cv::Mat u,w,vt;
    	cv::SVD::compute(A,w,u,vt);
		
		cout << "finish SVD decompose A" << std::endl;
		cout << "w = " << w << std::endl;
		int nonZeroSize = 0;
		for(int i=0;i<4;i++){
			if(abs(w.at<float>(i)) > 0.0000001){
				nonZeroSize ++;
			}
			else if(abs(w.at<float>(i)) > 9999999){
				return false;
			}
		}
		cv::Mat w_mat_t = cv::Mat::zeros(4,4,CV_32F);
		for(int i=0;i<nonZeroSize;i++){
			w_mat_t.at<float>(i,i) = 1.0 / w.at<float>(i);
		}
		cout << "w = " << w_mat_t << std::endl;
		cout << "vt.t() = " << vt.t() << std::endl;
		cout << "u.t() = " << u.t() << std::endl;
		cv::Mat ans = vt.t() * w_mat_t * u.t() * b;
		cout << "ans = " << ans << std::endl;
 
		fout <<  "A = " << std::endl;
		fout << A << endl;
		fout << " b = " << b.t() << std::endl;
		fout << "w = " <<w.t() << std::endl;
		fout << "ans = " << ans.t() << std::endl;
		fout << "====================================================" << std::endl;
		//fout.close();
		
		// refine the estimate considering the acc bias
		double scale = ans.at<float>(0);
		cv::Mat gw(3,1,CV_32F);
		gw.at<float>(0) = ans.at<float>(1); gw.at<float>(1) = ans.at<float>(2) ; gw.at<float>(2) = ans.at<float>(3);			

        cv::Mat gi(3,1,CV_32F);
		gi.at<float>(0) = -1.0; gi.at<float>(1) = 0.0; gi.at<float>(2) = 0.0;
		
		cv::Mat vv = gw.cross(gi);
		double n_vv = cv::norm(vv);
		vv = vv / n_vv;
		double dot_vv = gw.dot(gi);
		double thelta = atan2(n_vv , dot_vv);
		cv::Mat r_vec = vv * thelta;
		Rot3 rRwi = Rot3::Expmap(Vector3(r_vec.at<float>(0) , r_vec.at<float>(1) , r_vec.at<float>(2)));
		Matrix3 m_rRwi = rRwi.matrix();
		cv::Mat Rwi(3,3,CV_32F);
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				Rwi.at<float>(i,j) = m_rRwi(i,j);
			}
		}
		
		double g_gravity = 9.81;
		cv::Mat skew_gi = Converter::toSkewMatrix(gi);
		
		 cv::Mat AA(3*(nSize-2),6,CV_32F);
		 cv::Mat bb(3*(nSize-2),1,CV_32F);
		 
		 ii = 0;
		 fout << "Begin to refine the scality and gravity "<<std::endl;
		 fout << "====================================================" << std::endl;
		 fout << "====================================================" << std::endl;
		 for(vector<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end()-2; lit != lend; lit++){
			 fout <<"the " << ii << "th equation"<< std::endl;
			 KeyFrame* pKF1 = *(lit);
			 KeyFrame* pKF2 = *(lit+1);
			 KeyFrame* pKF3 = *(lit+2);
			 
			 PreintegratedCombinedMeasurements* pim12 = imu->Getimufactor(pKF2->mnId);
			 PreintegratedCombinedMeasurements* pim23 = imu->Getimufactor(pKF3->mnId);
			 
			 Matrix93 Ja12_9X3 = pim12->preintegrated_H_biasAcc();
			 Matrix93 Ja23_9X3 = pim23->preintegrated_H_biasAcc();
			 
			 cv::Mat Jpa12(3,3,CV_32F);
			 for(int j=0;j<3;j++){
				 for(int k=0;k<3;k++){
					 Jpa12.at<float>(j,k) = Ja12_9X3(3+j,k);
				 }
			 }
			 cv::Mat Jva12(3,3,CV_32F);
			 for(int j=0;j<3;j++){
				 for(int k=0;k<3;k++){
					 Jva12.at<float>(j,k) = Ja12_9X3(6+j,k);
				 }
			 }			 
			 cv::Mat Jpa23(3,3,CV_32F);
			 for(int j=0;j<3;j++){
				 for(int k=0;k<3;k++){
					 Jpa23.at<float>(j,k) = Ja23_9X3(3+j,k);
				 }
			 }
			 
			 cv::Mat Tcw1 = pKF1->GetPose(); 
			 cv::Mat Tcw2 = pKF2->GetPose();
			 cv::Mat Tcw3 = pKF3->GetPose();
			 fout << "Tcw1 = " << Tcw1 << std::endl;
			 fout << "Tcw2 = " << Tcw2 << std::endl;
			 fout << "Tcw3 = " << Tcw3 << std::endl;
			 
			 cv::Mat wPc1 = - (Tcw1.rowRange(0,3).colRange(0,3)).inv() * Tcw1.rowRange(0,3).col(3);
			 cv::Mat wPc2 = - (Tcw2.rowRange(0,3).colRange(0,3)).inv() * Tcw2.rowRange(0,3).col(3);
			 cv::Mat wPc3 = - (Tcw3.rowRange(0,3).colRange(0,3)).inv() * Tcw3.rowRange(0,3).col(3);
			 
			 cv::Mat Rwc1 = (Tcw1.rowRange(0,3).colRange(0,3)).inv();
			 cv::Mat Rwc2 = (Tcw2.rowRange(0,3).colRange(0,3)).inv();
			 cv::Mat Rwc3 = (Tcw3.rowRange(0,3).colRange(0,3)).inv();
			 
			 fout << "Rwc1 = " << Rwc1 << std::endl;
			 fout << "Rwc2 = " << Rwc2 << std::endl;
			 fout << "Rwc3 = " << Rwc3 << std::endl;
			 
			 cv::Mat Rwb1 = Rwc1 * mRcb; 
			 cv::Mat Rwb2 = Rwc2 * mRcb; 
			 cv::Mat Rwb3 = Rwc3 * mRcb; 
			 
			 fout << "Rwb1 = " << Rwb1 << std::endl;
			 fout << "Rwb2 = " << Rwb2 << std::endl;
			 fout << "Rwb3 = " << Rwb3 << std::endl;			 
			 			 			 
			 double dt12 = pim12->deltaTij();
			 double dt23 = pim23->deltaTij();
			 
			 Vector3 dp12v = pim12->deltaPij();		 
			 cv::Mat dp12(3,1,CV_32F);
			 dp12.at<float>(0) = dp12v(0); dp12.at<float>(1) = dp12v(1); dp12.at<float>(2) = dp12v(2);
			 
			 Vector3 dp23v = pim23->deltaPij();
			 cv::Mat dp23(3,1,CV_32F);
			 dp23.at<float>(0) = dp23v(0); dp23.at<float>(1) = dp23v(1); dp23.at<float>(2) = dp23v(2);
			 
			 Vector3 dv12v = pim12->deltaVij();
			 cv::Mat dv12(3,1,CV_32F);
			 dv12.at<float>(0) = dv12v(0); dv12.at<float>(1) = dv12v(1); dv12.at<float>(2) = dv12v(2);
		     
			 Vector3 dv23v = pim23->deltaVij();
			 cv::Mat dv23(3,1,CV_32F);
			 dv23.at<float>(0) = dv23v(0); dv23.at<float>(1) = dv23v(1); dv23.at<float>(2) = dv23v(2);
			 
			 fout << "dp12 = " << dp12.t() << std::endl;
			 fout << "dp23 = " << dp23.t() << std::endl;
			 fout << "dv12 = " << dv12.t() << std::endl;
			 fout << "dv23 = " << dv23.t() << std::endl;
			 fout << "dt12 = " << dt12 << std::endl;
			 fout << "dt23 = " << dt23 << std::endl;
			 
			 cv::Mat lambdai = (wPc3 - wPc2) * dt12 - (wPc2 - wPc1) * dt23;
			 cv::Mat beltai = 0.5 * Rwi * skew_gi * g_gravity * (dt12 * dt12 * dt23 + dt23 * dt23 * dt12);
			 //cv::Mat beltai = 0.5 * gw * (dt23 * dt23 * dt12 + dt12 * dt12 * dt23);
			 cv::Mat ittai = -1.0 * (Rwc2 * Jpa23 * dt12 + Rwc1 * Jva12 * dt12 * dt23 - Rwc1 * Jpa12*dt23);
			 cv::Mat gammai = Rwc2*dp23*dt12 + Rwc1*dv12*dt12*dt23 - 
								 Rwc1*dp12*dt23 + 0.5 * Rwi * gi * g_gravity * (dt23*dt23*dt12+dt12*dt12*dt23);
								 
			 fout << "lambdai = " << lambdai.t() << std::endl;
			 fout << "beltai = " << beltai.t() << std::endl;
			 fout << "ittai = " << ittai << std::endl;
			 fout << "gammi = " << gammai.t() << std::endl;
			 
			 for(int j=0;j<3;j++){
				 AA.at<float>(3*ii+j,0) = lambdai.at<float>(j);
			 }					 
			 for(int j=0;j<3;j++){
				 for(int k=0;k<2;k++){
					 AA.at<float>(3*ii+j,k+1) = beltai.at<float>(j,k+1);
				 }
			 }
			 for(int j=0;j<3;j++){
				 for(int k=0;k<3;k++){
					 AA.at<float>(3*ii+j,k+3) = ittai.at<float>(j,k);
				 }
			 }
			 for(int j=0;j<3;j++){
				 bb.at<float>(3*ii+j,0) = gammai.at<float>(j);
			 }
			 //A.rowRange(3*i,3*i+3).col(0) = lambdai;
			 //A.rowRange(3*i,3*i+3).colRange(1,4) = beltai;
			 //b.rowRange(3*i,3*i+3).col(0) = gammai;	
			 ii ++;		 	
			 fout << "================================================" << std::endl;
			 fout << "================================================" << std::endl;		  
		 }
		 fout << "=====================================" << std::endl;
		 cout << "finish set AA and bb" << std::endl;
        
	    cv::Mat uu,ww,vvt;
    	cv::SVD::compute(AA,ww,uu,vvt);
		
		cout << "finish SVD decompose AA" << std::endl;
		cout << "ww = " << ww << std::endl;
		nonZeroSize = 0;
		for(int i=0;i<6;i++){
			if(abs(ww.at<float>(i)) > 0.0000001 && abs(ww.at<float>(i)) < 999999){
				nonZeroSize ++;
			}
			else if(abs(ww.at<float>(i)) > 9999999){
				return false;
			}
		}
		cv::Mat ww_mat_t = cv::Mat::zeros(6,6,CV_32F);
		for(int i=0;i<nonZeroSize;i++){
			ww_mat_t.at<float>(i,i) = 1.0 / ww.at<float>(i);
		}
		
		ans = vvt.t() * ww_mat_t * uu.t() * bb;
		fout <<  "AA = " << std::endl;
		fout << AA << endl;
		fout << " bb = " << bb.t() <<  std::endl;
		fout << "ww = " << ww.t() << std::endl;
		fout << "ans = " << ans.t() << std::endl;
		
		cv::Mat d_thelta(3,1,CV_32F);
		d_thelta.at<float>(0) = 0.0; d_thelta.at<float>(1) = ans.at<float>(1);
		d_thelta.at<float>(2) = ans.at<float>(2);

		Rot3 rRg = Rot3::Expmap(Vector3(d_thelta.at<float>(0) , d_thelta.at<float>(1) , d_thelta.at<float>(2)));
		Matrix3 m_rRg = rRg.matrix();
		cv::Mat mm_rRg(3,3,CV_32F);
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				mm_rRg.at<float>(i,j) = m_rRg(i,j);
			}
		}
		
		e_gw = Rwi * mm_rRg * gi * g_gravity;
		cout << "e_gw = " << e_gw.t() << std::endl;
		fout << "estimated gw = " << e_gw.t() << std::endl;
		fout.close();
		
		scale = ans.at<float>(0);
		cv::Mat ba(3,1,CV_32F);
		ba.at<float>(0) = ans.at<float>(3);
		ba.at<float>(1) = ans.at<float>(4);
		ba.at<float>(2) = ans.at<float>(5);
		
		// update all the scale of the mappoint
		vector<MapPoint*> points = mpMap->GetAllMapPoints();
		for(vector<MapPoint*>::iterator mit = points.begin(), mend = points.end(); mit != mend; mit ++){
			MapPoint* pMP = (*mit);
			pMP->SetWorldPos(pMP->GetWorldPos()*scale);
		}
		
		// update the translation of all the keyframes
		for(vector<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++){
			KeyFrame* pKF = *lit;
			cv::Mat mTcw = pKF->GetPose();
			mTcw.rowRange(0,3).col(3) = mTcw.rowRange(0,3).col(3) * scale;
			pKF->SetPose(mTcw);
			if(lit == lend-1)
			{
				mTcw.copyTo(pLastFrame->mTcw);
			}

		}
		
		// computer the every keyframe's velocity
		for(vector<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end()-1; lit != lend; lit++){
			KeyFrame* pKF1 = *lit;
			KeyFrame* pKF2 = *(lit+1);
			PreintegratedCombinedMeasurements* pim = imu->Getimufactor(pKF2->mnId);
			 
			cv::Mat Tcw1 = pKF1->GetPose(); 
			cv::Mat Tcw2 = pKF2->GetPose();
			 
			cv::Mat wPc1 = - (Tcw1.rowRange(0,3).colRange(0,3)).inv() * Tcw1.rowRange(0,3).col(3);
			cv::Mat wPc2 = - (Tcw2.rowRange(0,3).colRange(0,3)).inv() * Tcw2.rowRange(0,3).col(3);
			 
			cv::Mat Rwc1 = (Tcw1.rowRange(0,3).colRange(0,3)).inv();
			cv::Mat Rwc2 = (Tcw2.rowRange(0,3).colRange(0,3)).inv();
			 
			cv::Mat Rwb1 = Rwc1 * mRcb; 
			 			 			 
			double dt = pim->deltaTij();
			
			Matrix93 Ja_9X3 = pim->preintegrated_H_biasAcc();
			 
			 
			 cv::Mat Jpa(3,3,CV_32F);
			 for(int j=0;j<3;j++){
				 for(int k=0;k<3;k++){
					 Jpa.at<float>(j,k) = Ja_9X3(3+j,k);
				 }
			 }
			 cv::Mat Jva(3,3,CV_32F);
			 for(int j=0;j<3;j++){
				 for(int k=0;k<3;k++){
					 Jva.at<float>(j,k) = Ja_9X3(6+j,k);
				 }
			 }			 
 
			Vector3 dpv = pim->deltaPij();		 
			cv::Mat dp(3,1,CV_32F);
			dp.at<float>(0) = dpv(0); dp.at<float>(1) = dpv(1); dp.at<float>(2) = dpv(2);
			
			cv::Mat vel = ((wPc2-wPc1) + 
				0.5 * Rwi * skew_gi * g_gravity * dt * dt * d_thelta - 
				Rwc1 * ( dp + Jpa * ba ) - 0.5 * Rwi * gi * g_gravity * dt * dt)/dt;
			//vel = mRcb * vel;
			for(int i=0;i<3;i++){
				pKF1->mSpeedAndBias.vel[i] = vel.at<float>(i);
				
			}					
		}
		
		vector<KeyFrame*>::iterator lit1 = lLocalKeyFrames.end()-2;
		vector<KeyFrame*>::iterator lit2 = lLocalKeyFrames.end()-1;
		KeyFrame* pKF1 = *(lit1);
		KeyFrame* pKF2 = *(lit2);
		
		cv::Mat mTcw1 = pKF1->GetPose();
		cv::Mat mRcw1 = mTcw1.rowRange(0,3).colRange(0,3);
		cv::Mat mRwc1 = mRcw1.inv();
		PreintegratedCombinedMeasurements* pim = imu->Getimufactor(pKF2->mnId);
	    double dt = pim->deltaTij();
			 
		Vector3 dvv = pim->deltaVij();		 
		cv::Mat dv(3,1,CV_32F);
		dv.at<float>(0) = dvv(0); dv.at<float>(1) = dvv(1); dv.at<float>(2) = dvv(2);
		
		Matrix93 Ja_9X3 = pim->preintegrated_H_biasAcc();	 
		cv::Mat Jpa(3,3,CV_32F);
		for(int j=0;j<3;j++){
			for(int k=0;k<3;k++){
				 Jpa.at<float>(j,k) = Ja_9X3(3+j,k);
			 }
		}		
		cv::Mat vel1(3,1,CV_32F);
		cv::Mat vel2(3,1,CV_32F);
		for(int i=0;i<3;i++){
			vel1.at<float>(i) = pKF1->mSpeedAndBias.vel[i];
		}
		
		vel2 = vel1 + e_gw * dt + mRwc1 * ( dv + Jpa * ba);
		//vel2 = mRcb * vel2;
		for(int i=0;i<3;i++){
			pKF2->mSpeedAndBias.vel[i] = vel2.at<float>(i);
			pLastFrame->mSpeedAndBias.vel[i] = vel2.at<float>(i);
		}
		
		// finish
		return true;		
		 
	}
	
	gtsam::Pose3 Optimizer::toGTPose3(const cv::Mat& _m){
		
		gtsam::Rot3 rot (_m.at<float>(0,0) , _m.at<float>(0,1) , _m.at<float>(0,2),
		                _m.at<float>(1,0) , _m.at<float>(1,1) , _m.at<float>(1,2),
						_m.at<float>(2,0) , _m.at<float>(2,1) , _m.at<float>(2,2));
	    gtsam::Point3 _p(_m.at<float>(0,3) , _m.at<float>(1,3) , _m.at<float>(2,3));
		return Pose3(rot , _p);
		
	}
	
	cv::Mat Optimizer::Pose3TocvMat(const gtsam::Pose3 _p){
		
		gtsam::Vector val = _p.rotation().quaternion();
		double pX = _p.x();
		double pY = _p.y();
		double pZ = _p.z();
		Eigen::Quaterniond _q1(val(0) , val(1) , val(2) , val(3));
		Eigen::Vector3d _t1(pX,pY,pZ);
		g2o::Sim3 sim3_vec(_q1,_t1,1);
		cv::Mat Tcw1 = Converter::toCvMat(sim3_vec);
		return Tcw1;
	}
	
	cv::Mat Optimizer::Pose3TocvMat(const boost::optional<gtsam::Pose3> _p){
		
		gtsam::Vector val = _p->rotation().quaternion();
		double pX = _p->x();
		double pY = _p->y();
		double pZ = _p->z();
		Eigen::Quaterniond _q1(val(0) , val(1) , val(2) , val(3));
		Eigen::Vector3d _t1(pX,pY,pZ);
		g2o::Sim3 sim3_vec(_q1,_t1,1);
		cv::Mat Tcw1 = Converter::toCvMat(sim3_vec);
		return Tcw1;
	}	
}
