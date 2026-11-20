#include "point_inputs.hpp"
#include <QApplication>
#include <QLineEdit>
#include <stdexcept>
#include <iostream>
void require(bool value){if(!value)throw std::runtime_error("Point input behavioral assertion failed");}
int main(int argc,char**argv){
 QApplication app(argc,argv);QWidget owner;QVBoxLayout layout(&owner);QJsonObject received;int calls=0,replacements=0;QString dragKind,dragName;
 arx_waypoints::PointInputs inputs(&layout,[&](const QJsonObject&v){received=v;++calls;},[&](const QString&k,const QString&n){dragKind=k;dragName=n;},[&]{++replacements;});
 require(owner.findChildren<QLineEdit*>().isEmpty());
 owner.resize(340,110);owner.show();app.processEvents();
 auto buttons=owner.findChildren<QPushButton*>();require(buttons.size()==3);
 inputs.replace->click();require(replacements==0);inputs.replace->setEnabled(true);inputs.replace->click();require(replacements==1 && calls==0);
 require(buttons[0]->text()=="添加当前点" && buttons[1]->text()=="拖箭头加点" && buttons[2]->text()=="替换");
 require(buttons[0]->geometry().top()==buttons[1]->geometry().top() && buttons[1]->geometry().top()==buttons[2]->geometry().top());
 require(inputs.kind->currentData().toString()=="via");
 for(auto kind:{"via","special","point_1","point_2"}){
  inputs.kind->setCurrentIndex(inputs.kind->findData(kind));inputs.capture->click();
  require(received["kind"]==kind && !received.contains("name"));
  inputs.drag->click();require(dragKind==kind && dragName.isEmpty());
  auto point=arx_waypoints::typedPoint(dragKind,dragName,1.,2.,.3);
  require(!point.contains("name") && point["pose"].toArray().size()==3);
  if(dragKind.startsWith("point_"))require(point["type"]=="task" && point["point_id"]==dragKind);
 }
 auto first=received["request_id"].toString();require(!first.isEmpty());inputs.capture->click();require(calls==5 && received["request_id"]!=first);
 require(inputs.kind->findData("stop")==-1 && inputs.kind->count()==4);
 owner.resize(340,110);owner.show();app.processEvents();require(owner.minimumSizeHint().height()<=110);
 if(argc>1)require(owner.grab().save(argv[1]));
 std::cout<<"Four unnamed current-pose/drag types, unique IDs and compact input height passed\n";
}
