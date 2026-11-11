#include "arm_trajectory.hpp"
#include <cassert>
#include <iostream>
#include <fstream>
using json=nlohmann::json;
ArmLimits limits{{{{-90,90},{-6,206},{-6,172},{-74,74},{-85,85},{-100,100}}},{{-3.14,0}},10};
TrajectoryPolicy policy{5.7,12,.5,2,.05,.15,60,.2};
json plan(){return {{"version",1},{"request_id","test-1"},{"interpolation","segment_quintic_scalar"},
    {"time_from_start_s",{0.,2.,4.}},
    {"joint_waypoints_rad",{{0.,0.,0.,0.,0.,0.},{.05,-.02,.03,0.,0.,.04},{.03,0.,.02,0.,0.,0.}}}};}
void bad(const json& p){ArmTrajectory t(limits,policy);bool rejected=false;
    try{t.start(p,std::vector<double>(7,0),std::vector<double>(7,0),0);}catch(const std::exception&){rejected=true;}
    assert(rejected&&!t.active());}
int main(int argc,char** argv){
    if(argc==2){ // Shared Python-generated timing fixture; sample the C++ executor only.
        std::ifstream input(argv[1]);json p;input>>p;auto config=p.at("test_policy");
        TrajectoryPolicy v{config[0],config[1],config[2],config[3],config[4],config[5],config[6],config[7]};
        ArmTrajectory t(limits,v);std::vector<double> actual=p["joint_waypoints_rad"][0].get<std::vector<double>>();actual.push_back(0);
        t.start(p,actual,std::vector<double>(7,0),0);
        json out=json::array();double end=p["time_from_start_s"].back();
        for(int i=0;i<=1000;++i){double now=end*i/1000;t.keepalive(p["request_id"],now);
            auto q=t.tick(now,actual,true);if(!q.empty()){out.push_back(q);std::copy(q.begin(),q.end(),actual.begin());}}
        assert(t.status()=="reference_complete");std::cout<<out.dump()<<"\n";return 0;
    }
    auto p=plan();
    for(auto kind:{0,1,2,3,4,5}){auto x=p;
        if(kind==0)x["time_from_start_s"]={0.,0.,4.};
        if(kind==1)x["time_from_start_s"]={0.,.01,.02};
        if(kind==2)x["joint_waypoints_rad"][1][0]=5.;
        if(kind==3)x["interpolation"]="linear";
        if(kind==4)x["joint_waypoints_rad"][1]=json::array({0.,0.});
        if(kind==5)x["time_from_start_s"]={1.,2.,4.};
        bad(x);
    }
    {ArmTrajectory t(limits,policy);auto actual=std::vector<double>(7,0);actual[0]=.1;
     bool rejected=false;try{t.start(p,actual,std::vector<double>(7,0),0);}catch(...){rejected=true;}assert(rejected);}
    {ArmTrajectory t(limits,policy);std::vector<double> actual(7,0);t.start(p,actual,std::vector<double>(7,0),0);
     for(int i=0;i<=400;++i){double now=i*.01;t.keepalive("test-1",now);auto q=t.tick(now,actual,true);
       assert(q.size()==6);std::copy(q.begin(),q.end(),actual.begin());}
     assert(t.status()=="reference_complete"&&!t.active());
     bool duplicate=false;try{t.start(p,actual,std::vector<double>(7,0),5);}catch(...){duplicate=true;}assert(duplicate);}
    {ArmTrajectory t(limits,policy);t.start(p,std::vector<double>(7,0),std::vector<double>(7,0),0);
     t.tick(.1,std::vector<double>(7,0),true);assert(t.status()=="failed"&&t.reason()=="timer_gap");
     assert(t.tick(.11,std::vector<double>(7,0),true).empty());}
    {ArmTrajectory t(limits,policy);t.start(p,std::vector<double>(7,0),std::vector<double>(7,0),0);
     for(int i=1;i<=16;++i)t.tick(i*.01,std::vector<double>(7,0),true);
     assert(t.status()=="failed"&&t.reason()=="lease_expired");assert(!t.keepalive("test-1",.17));}
    {ArmTrajectory t(limits,policy);t.start(p,std::vector<double>(7,0),std::vector<double>(7,0),0);std::vector<double> drift(7,0);drift[0]=.1;
     assert(t.tick(.01,drift,true).empty());assert(t.reason()=="tracking_error");}
    {ArmTrajectory t(limits,policy);t.start(p,std::vector<double>(7,0),std::vector<double>(7,0),0);
     assert(t.tick(.01,std::vector<double>(7,0),false).empty());assert(t.reason()=="feedback_invalid");}
    {ArmTrajectory t(limits,policy);t.start(p,std::vector<double>(7,0),std::vector<double>(7,0),0);t.cancel("manual_stop");
     assert(t.tick(.01,std::vector<double>(7,0),true).empty());assert(t.reason()=="manual_stop");}
    {ArmTrajectory t(limits,policy);t.start(p,std::vector<double>(7,0),std::vector<double>(7,0),1);
     assert(!t.keepalive("other",1.01));t.tick(.99,std::vector<double>(7,0),true);assert(t.reason()=="invalid_clock");}
    {ArmTrajectory t(limits,policy);auto velocity=std::vector<double>(7,0);velocity[2]=.02;
     bool rejected=false;try{t.start(p,std::vector<double>(7,0),velocity,0);}catch(...){rejected=true;}assert(rejected);}
    {ArmTrajectory t(limits,policy);t.start(p,std::vector<double>(7,0),std::vector<double>(7,0),0);
     bool rejected=false;auto other=p;other["request_id"]="other";
     try{t.start(other,std::vector<double>(7,0),std::vector<double>(7,0),.01);}catch(...){rejected=true;}
     assert(rejected&&t.id()=="test-1"&&t.active());
     assert(!t.keepalive("test-1",.16));assert(t.reason()=="lease_expired");
     assert(t.tick(.17,std::vector<double>(7,0),true).empty());}
    {ArmTrajectory t(limits,policy);t.start(p,std::vector<double>(7,0),std::vector<double>(7,0),0);
     auto actual=std::vector<double>(7,0);actual[0]=NAN;
     assert(t.tick(.01,actual,true).empty());assert(t.reason()=="feedback_invalid");}
    {auto broken=policy;broken.start_speed_deg_s=NAN;bool rejected=false;
     try{ArmTrajectory t(limits,broken);}catch(...){rejected=true;}assert(rejected);}
    std::cout<<"arm trajectory checks passed\n";
}
