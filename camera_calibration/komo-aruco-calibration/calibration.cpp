#include "calibration.h"
#include <Geo/geo.h>
#include <Kin/frame.h>
#include <Kin/kin.h>
#include <Core/h5.h>
#include <KOMO/komo.h>
#include <Optim/NLP.h>
#include <Kin/feature.h>

#include <math.h>

using std::cout;
using std::endl;

// void decomposeInvProjectionMatrix(arr& K, arr& R, arr& t, const arr& Pinv){
//   t = Pinv.col(3); t.resizeCopy(3);
//   arr KRt = inverse(Pinv.sub({0,3},{0,3}));
//   lapack_RQ(K, R, KRt);
//   transpose(R);
// }

void decomposeProjectionMatrix(arr& K, arr& R, arr& t, const arr& P){
  arr KRt = P.sub({0,3},{0,3});
  lapack_RQ(K, R, KRt);
  transpose(R);
  // t = - (inverse(KRt) * P.col(3));
  t = - (R * inverse(K) * P.col(3));
  { //flip K signs for positive diagonals
    arr I=eye(3);
    uint num=0;
    for(uint i=0;i<3;i++) if(K(i,i)<0.){ I(i,i) = -1.; num++; }
    CHECK(!(num%2), "can't flip odd number of axes -- is your camera left-handed?");
    R = R*I;
    K = K*I;
  }
}

void computeCalibration(const arr& X3, const arr& p, const arr& d){
  arr x = rai::catCol(d%p, d); //homogeneous pixel coords
  arr X = rai::catCol(X3, ones(X3.d0,1)); //homogeneous point world coordinates

  //-- solve for project matrix
  arr P = ~x * X * inverse_SymPosDef(~X*X);
  double err = ::sqrt(sumOfSqr(x - X*~P)/double(X.d0));
  std::cout <<"RMSE in pixels = " <<err <<std::endl;

  //-- decompose
  arr K, R, t;
  decomposeProjectionMatrix(K, R, t, P);
  arr X_pred = x * ~inverse(K) * ~R + repmat(~t,x.d0,1);  // X = R * Kinv * x + t
  std::cout <<"RMSE in points = " <<::sqrt(sumOfSqr(X3 - X_pred)/double(X3.d0)) <<std::endl;

  //-- output
  rai::Transformation T;
  T.rot.setMatrix(R);
  T.pos=t;
  arr fxycxy = {K(0,0), K(1,1), K(0,2), K(1,2)};
  std::cout <<"*** camera intrinsics K:\n" <<K <<std::endl;
  std::cout <<"*** camera fxycxy :\n" <<fxycxy <<std::endl;
  std::cout <<"*** camera world pose: " <<T <<std::endl;
}

//===========================================================================

CalibFromArucos::CalibFromArucos(bool calibJoints){
  //-- build calibration configuration

  C.addFile("$RAI_PATH/scenarios/pandaSingle.g");
  //C.addFile("config.yml");
  //C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  C.getFrame("table")->setColor({.2,.2});
  C.ensure_indexedJoints();
  qDofs = C.activeDofs;

  //-- camera mount calib
  Fcam = C.getFrame("cameraWrist");
  Fcam->setJoint(rai::JT_free);
  Fcam->joint->isStable=true;
  stableDofs.append(Fcam->joint);

  //-- pixel view frames
  for(uint id:aruco_ids){
    rai::Frame* pix = C.addFrame(STRING("pix_"<<id), "cameraWrist");
    pix->setColor({.2,.2});
    pix->setShape(rai::ST_marker, {.05});
  }

  //-- aruco position calib
  for(uint id:aruco_ids){
    rai::Frame* ar = C.addFrame(STRING("aruco_" <<id), aruco_parent);
    ar->setShape(rai::ST_marker, {.2});
    ar->setJoint(rai::JT_trans3);
    ar->joint->isStable=true;
    Farucos.append(ar);
    stableDofs.append(ar->joint);
  }

  //-- joints calib
  if(calibJoints) for(int i=1;i<=7;i++){
      rai::Frame *joint_calib = C.getFrame(STRING("l_panda_joint"<<i));
      joint_calib = joint_calib->insertPreLink(0, STRING("_calib"));
      joint_calib->setJoint(rai::JT_hingeZ);
      joint_calib->joint->isStable=true;
      Fjoint_calibs.append(joint_calib);
      stableDofs.append(joint_calib->joint);
    }

  C.checkConsistency();
  // C.view(true);
}

void CalibFromArucos::load_data(const char* file){
  auto h5 = rai::H5_Reader(file);
  auto manifest = h5.readDict("manifest");
  uint n =  manifest.get<double>("n_datasets");
  Q.resize(n);
  P.resize(n);
  I.resize(n);
  for(uint i=0;i<n;i++){
    Q(i) = h5.read<double>(STRING("/dataset_" <<i <<"/joint_state"));
    P(i) = h5.read<double>(STRING("/dataset_" <<i <<"/marker_positions"));
    I(i) = h5.read<uint>(STRING("/dataset_" <<i <<"/marker_ids"));
  }
}

void CalibFromArucos::display_data(){
  // rai::Configuration C;
  // C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  // C.getFrame("table")->setColor({.2,.2});

  for(uint i=0;i<Q.N;i++){
    C.setDofState(Q(i), qDofs);
    C.view(true);
  }
}

void CalibFromArucos::calibFullProjectionMatrix(const arr& q_opt){
  rai::Frame *Fcam = C.getFrame("cameraWrist");

  double Err=0., Enorm=0.;
  arr dat_X(0,3), dat_p(0,2), dat_d;
  for(uint t=0;t<Q.d0;t++){
    if(blockList.contains(t)) continue;
    C.setDofState(q_opt, stableDofs);
    C.setDofState(Q(t), qDofs);
    rai::Transformation TwristInv= -(Fcam->parent->get_X());
    rai::Transformation TcamInv= -(Fcam->get_X());

    for(uint j=0;j<I(t).N;j++){
      uint id = I(t)(j);
      arr pos = C.getFrame(STRING("aruco_"<<id))->getPosition();
      arr p = P(t)[j];
      double d = (TcamInv*pos)(2);
      Err += sqrt(rai::sqr(p(2)-d));  Enorm += 1.;

      dat_X.append(TwristInv*pos);
      dat_p.append(p({0,2}));
      dat_d.append(d);
    }
  }
  cout <<"mean error of depth values (RMSE is huge -> outliers!): " <<(Err/Enorm) <<endl;
  computeCalibration(dat_X, dat_p, dat_d);
}

void CalibFromArucos::solve(){
  //cout <<"=== loading data..." <<endl;
  uint n = Q.N;


  //-- create and initialize komo
  KOMO komo;
  komo.setConfig(C, false);
  komo.setTiming(n, 1, 0., 0);
  {
    C.setDofState(Q(0), qDofs); //komo initial pose must be Q(0)
    arr x = C.getJointState();
    for(uint i=1;i<n;i++) x.append(Q(i));
    komo.set_x(x);
  }

  //-- deactivate normal joint dofs
  for(rai::Dof *dof:komo.pathConfig.activeDofs) if(!dof->isStable) dof->active=false;
  komo.pathConfig.reset_q();

  //-- add the pixel view frames with z-axis as pixel ray
  for(uint t=0;t<n;t++){
    if(blockList.contains(t)) continue;
    for(uint j=0;j<I(t).N;j++){
      uint id = I(t)(j);
      arr p = P(t)[j];
      double d = p(2); //.2;
      arr x_cam = arr{(p(0)-px)*d/fx, (p(1)-py)*d/fy, d};
      rai::Frame* C_pix = C.getFrame(STRING("pix_"<<id));
      rai::Frame* pix = komo.timeSlices(t, C_pix->ID);
      rai::Quaternion R;
      R.setDiff(Vector_z, x_cam);
      pix->setShape(rai::ST_marker, {.1});
      pix->setRelativePosition(x_cam);
      pix->setRelativeQuaternion(R.getArr());
    }
  }
  // komo.view(true, "initialized with poses");

  //-- komo objectives
  // komo.addControlObjective({}, 0, 1e-2);
  komo.addQuaternionNorms({}, 1e1, false);

  //-- core objective: arucos on rays
  double scale=1e2;
  arr S = arr({2, 3}, {1., 0., 0., 0., 1., 0.}); //pick the xy- coordinated
  for(uint t=0;t<n;t++){
    if(blockList.contains(t)) continue;
    for(uint id:I(t)){
      komo.addObjective({double(t+1)}, FS_positionRel, {STRING("aruco_" <<id), STRING("pix_"<<id)}, OT_sos, scale*S);
    }
  }
  // for(uint id:aruco_ids){
  //   komo.addObjective({1}, FS_position, {STRING("aruco_" <<id)}, OT_sos, arr{0,0,1e0}, arr{0,0,.65});
  // }
  for(rai::Frame *f:Fjoint_calibs) komo.addObjective({1}, FS_qItself, { f->name }, OT_sos, arr{1e1});

  //-- solve
  komo.opt.verbose = 0;
  // komo.opt.animateOptimization = 2; //interesting!!
  rai::OptOptions opt;
  opt.set_stopTolerance(1e-6);
  opt.set_damping(1e2);
  // opt.set_method(rai::M_rprop);
  komo.solve(1e-3, -1., opt);
  arr q_opt = komo.pathConfig.getJointState();
  C.setDofState(q_opt, stableDofs); //for the readouts below

  //-- output
  /*cout <<"\n=== solution" <<endl;
  cout <<"optimal dofs:" <<q_opt <<endl;
  cout <<"optimal cam rel frame: " <<Fcam->get_Q().getArr7d() <<endl;
  arr q_jointCalib;
  for(rai::Frame *f:Fjoint_calibs) q_jointCalib.append( f->joint->getDofState() );
  cout <<"optimal joint calibs: " <<q_jointCalib <<endl;
  cout <<"\n>>>" <<endl;
  cout <<"Edit cameraWrist: { Q: " <<Fcam->get_Q().getArr7d() <<" }" <<endl;
  for(rai::Frame *f:Fjoint_calibs){
      cout <<"Edit " <<f->parent->name <<": { pose: " <<(f->parent->get_Q() * f->get_Q()).getArr7d() <<" }" <<endl;
    }
  for(rai::Frame *f:Farucos) cout <<f->name <<"(" << aruco_parent << "): { Q: " <<f->getRelativePosition() <<", shape: marker, size: [.05] }" <<endl;
  cout <<"<<<" <<endl;*/
  cout << Fcam->get_Q().getArr7d() <<endl;

  //-- summarize error
  double Err=0., Enorm=0.;
  for(shared_ptr<GroundedObjective>& ob:komo.objs){
    if(ob->name().startsWith("F_PositionRel")){
      arr y = ob->feat->eval(ob->frames);
      y /= scale;
      Err += sumOfSqr(y);  Enorm += 1.;
    }
  }
  cout << ::sqrt(Err/Enorm) <<endl; //RMSE

  /*
  cout <<"\n=== double check intrinsics" <<endl;
  //-- recalibrate intrinsics
  calibFullProjectionMatrix(q_opt);

  komo.view(true, "optimal");*/
}
