#include "../windows.hpp"
#include <cassert>
#include <iostream>
int main(){
 Windows w;w.enabled={true,true};
 w.add(0,100,{{105,0},{205,0},{299,0}});w.add(1,100,{{110,1},{210,1}});
 assert(!w.pop(100));w.add(0,200,{{230,0}});w.add(1,200,{{235,1}});
 auto a=w.pop(100);assert(a&&a->start==100&&a->points.size()==2);
 w.add(0,300,{{305,0}});w.add(1,300,{{310,1}});
 auto b=w.pop(100);assert(b&&b->points.size()==5);assert(b->points.front().time==205);
 w.set_enabled({false,true});w.add(1,400,{{410,1}});w.add(1,500,{{510,1}});
 auto c=w.pop(100);assert(c&&c->points.size()==1&&c->points[0].source==1);
 bool failed=false;try{w.add(1,490,{{499,1}});}catch(...){failed=true;}assert(failed);
 Windows off;off.set_enabled({false,false});assert(!off.pop(100));
 std::cout<<"windows: passed\n";
}
