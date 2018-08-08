/*
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff
 * email:  vadim.tikhanoff@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "yarpOpenFace.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace cer::kinematics;

/****************************************************************/
void getIntrinsics(ResourceFinder &rf, map<string,Matrix> &intrinsics)
{
    set<string> avFrames=HeadParameters::getTypes();
    for (auto &camera:avFrames)
    {
        yInfo("#### Retrieving intrinsics for \"%s\" camera",camera.c_str());

        string groupName=camera;
        transform(groupName.begin(),groupName.end(),groupName.begin(),::toupper);
        groupName="CAMERA_CALIBRATION_"+groupName;

        bool ok=false;
        Bottle &group=rf.findGroup(groupName);
        if (!group.isNull())
        {
            if (group.check("fx") && group.check("fy") &&
                group.check("cx") && group.check("cy"))
            {
                Matrix K=eye(3,4);
                K(0,0)=group.find("fx").asDouble();
                K(1,1)=group.find("fy").asDouble();
                K(0,2)=group.find("cx").asDouble();
                K(1,2)=group.find("cy").asDouble();

                yInfo("%s",K.toString(3,3).c_str());
                intrinsics[camera]=pinv(K.transposed()).transposed();
                ok=true;
            }
        }

        if (!ok)
        {
            yWarning("Intrinsics for \"%s\" camera not configured!",camera.c_str());
        }
    }
}

/****************************************************************/
void releaseJointsBounds(map<string,HeadSolver> &kin)
{
    for (auto &it:kin)
    {
        HeadSolver &k=it.second;
        HeadParameters p=k.getHeadParameters();

        p.torso.l_min=-numeric_limits<double>::max();
        p.torso.l_max=+numeric_limits<double>::max();
        p.torso.alpha_max=numeric_limits<double>::max();
        p.head.asChain()->setAllConstraints(false);

        k.setHeadParameters(p);
    }
}

/****************************************************************/
void openDrivers(vector<PolyDriver> &drivers)
{
    Property option;
    option.put("device","remote_controlboard");
    option.put("remote","/cer/torso_tripod");
    option.put("local","/local/torso_tripod");
    drivers[0].open(option);

    option.clear();
    option.put("device","remote_controlboard");
    option.put("remote","/cer/torso");
    option.put("local","/local/torso");
    drivers[1].open(option);

    option.clear();
    option.put("device","remote_controlboard");
    option.put("remote","/cer/head");
    option.put("local","/local/head");
    drivers[2].open(option);
}

/****************************************************************/
void closeDrivers(vector<PolyDriver> &drivers)
{
    drivers[0].close();
    drivers[1].close();
    drivers[2].close();
}

/****************************************************************/
Vector getEncoders(vector<PolyDriver> &drivers)
{
    vector<IEncoders*> ienc(3);
    drivers[0].view(ienc[0]);
    drivers[1].view(ienc[1]);
    drivers[2].view(ienc[2]);

    Vector encs(6,0.0);
    Vector encs_=encs;

    ienc[0]->getEncoders(encs_.data());
    encs.setSubvector(0,encs_.subVector(0,2));

    ienc[1]->getEncoders(encs_.data());
    encs[3]=encs_[3];

    ienc[2]->getEncoders(encs_.data());
    encs.setSubvector(4,encs_.subVector(0,1));

    return encs;
}

/****************************************************************/
int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network::init();

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP server not available!";
        return 1;
    }

    /* create the module */
    FACEModule module;

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setVerbose( true );
    rf.setDefaultContext( "yarpOpenFace" );
    rf.setDefaultConfigFile( "yarpOpenFace.ini" );
    rf.setDefault("name","yarpOpenFace");

    rf.configure( argc, argv );

//get drivers and camera intrinsics
    map<string,Matrix> intrinsics;
    getIntrinsics(rf,intrinsics);

    //module.setIntrinsics(rf, intrinsics); //XXX

    HeadParameters params_depth("depth");
    HeadParameters params_left("left");
    HeadParameters params_right("right");

    map<string,HeadSolver> kin;
    kin["depth"].setHeadParameters(params_depth);
    kin["left"].setHeadParameters(params_left);
    kin["right"].setHeadParameters(params_right);
    releaseJointsBounds(kin);

    vector<PolyDriver> drivers(3);
    openDrivers(drivers);

    Vector q=getEncoders(drivers);

    closeDrivers(drivers);


    /* run the module: runModule() calls configure first and, if successful, it then runs */
    module.runModule(rf);
    yarp::os::Network::fini();

    return 0;
}
//empty line to make gcc happy
