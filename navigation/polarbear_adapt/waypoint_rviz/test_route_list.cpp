#include "route_list.hpp"
#include <QApplication>
#include <stdexcept>
void require(bool v){if(!v)throw std::runtime_error("Route selection behavior failed");}
int main(int argc,char **argv){
 QApplication app(argc,argv);QListWidget list;
 auto point=[](const char *id){return QJsonObject{{"id",id},{"type","special"},{"pose",QJsonArray{1,2,0}}};};
 QJsonObject d{{"route_id","r"},{"points",QJsonArray{point("a"),point("b"),point("c")}}};
 arx_waypoints::updateRouteList(&list,{},d);require(list.currentRow()==-1);list.setCurrentRow(2);
 auto moved=d;moved["points"]=QJsonArray{point("a"),point("c"),point("b")};arx_waypoints::updateRouteList(&list,d,moved);
 require(list.currentRow()==1 && arx_waypoints::selectedPointId(&list)=="c");
 auto request=arx_waypoints::moveSelectedPoint(&list,moved,-1);require(request["id"]=="c" && request["delta"]==-1);
 auto top=d;top["points"]=QJsonArray{point("c"),point("a"),point("b")};arx_waypoints::updateRouteList(&list,moved,top);require(list.currentRow()==0);
 // Periodic identical refresh must keep the same selected point.
 arx_waypoints::updateRouteList(&list,top,top);require(arx_waypoints::selectedPointId(&list)=="c");
 auto removed=top;removed["points"]=QJsonArray{point("a"),point("b")};arx_waypoints::updateRouteList(&list,top,removed);require(list.currentRow()==-1);
 list.setCurrentRow(0);auto different=removed;different["route_id"]="different";arx_waypoints::updateRouteList(&list,removed,different);require(list.currentRow()==-1);
}
